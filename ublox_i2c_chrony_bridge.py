#!/usr/bin/env python3
"""
Read NMEA RMC time from a u-blox GNSS receiver over I2C/DDC and feed it to
chrony via the SOCK refclock protocol.

This bridge exists because gpsd is oriented around serial/USB GPS devices.
For a ZED-F9P connected only via I2C, we still need a time-of-day source that
chrony can pair with /dev/pps0.
"""

from __future__ import annotations

import argparse
import ctypes
import datetime as dt
import errno
import os
import socket
import time
from typing import Optional

try:
    from smbus2 import SMBus  # type: ignore
except ImportError:  # pragma: no cover - fallback for Pi images with python3-smbus only
    try:
        from smbus import SMBus  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "Neither smbus2 nor smbus is available. Install python3-smbus or python3-smbus2."
        ) from exc


UBLOX_I2C_STREAM_COUNT_MSB = 0xFD
UBLOX_I2C_STREAM_COUNT_LSB = 0xFE
UBLOX_I2C_STREAM_DATA = 0xFF
CHRONY_SOCK_MAGIC = 0x534F434B


class ChronySockSample(ctypes.Structure):
    _fields_ = [
        ("tv_sec", ctypes.c_long),
        ("tv_usec", ctypes.c_long),
        ("offset", ctypes.c_double),
        ("pulse", ctypes.c_int),
        ("leap", ctypes.c_int),
        ("_pad", ctypes.c_int),
        ("magic", ctypes.c_int),
    ]


def log(message: str) -> None:
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge u-blox I2C RMC time into chrony SOCK refclock."
    )
    parser.add_argument("--i2c-bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument(
        "--i2c-addr",
        type=lambda v: int(v, 0),
        default=0x42,
        help="u-blox I2C address (default: 0x42)",
    )
    parser.add_argument(
        "--sock-path",
        default="/run/chrony/zedf9p.sock",
        help="chrony SOCK refclock path (default: /run/chrony/zedf9p.sock)",
    )
    parser.add_argument(
        "--read-chunk-size",
        type=int,
        default=32,
        help="I2C read chunk size in bytes (default: 32)",
    )
    parser.add_argument(
        "--poll-sleep",
        type=float,
        default=0.05,
        help="Sleep between idle polls in seconds (default: 0.05)",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=10.0,
        help="Seconds between status logs (default: 10.0)",
    )
    parser.add_argument(
        "--debug-nmea",
        action="store_true",
        help="Print accepted RMC sentences for debugging",
    )
    return parser.parse_args()


def checksum_ok(sentence: str) -> bool:
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, checksum = sentence[1:].split("*", 1)
    checksum = checksum.strip()
    if len(checksum) < 2:
        return False
    value = 0
    for char in body:
        value ^= ord(char)
    try:
        return value == int(checksum[:2], 16)
    except ValueError:
        return False


def parse_rmc_utc_epoch(sentence: str) -> Optional[float]:
    """
    Parse an RMC sentence and return Unix epoch seconds, or None if invalid.
    """
    if not checksum_ok(sentence):
        return None

    body = sentence[1:].split("*", 1)[0]
    fields = body.split(",")
    if not fields:
        return None

    talker = fields[0]
    if not talker.endswith("RMC"):
        return None
    if len(fields) < 10:
        return None

    time_field = fields[1]
    status_field = fields[2]
    date_field = fields[9]

    if status_field != "A":
        return None
    if not time_field or not date_field:
        return None

    try:
        if "." in time_field:
            hhmmss, frac = time_field.split(".", 1)
        else:
            hhmmss, frac = time_field, ""
        if len(hhmmss) < 6:
            return None

        hour = int(hhmmss[0:2])
        minute = int(hhmmss[2:4])
        second = int(hhmmss[4:6])
        microsecond = int((frac + "000000")[:6]) if frac else 0

        day = int(date_field[0:2])
        month = int(date_field[2:4])
        year_2 = int(date_field[4:6])
        year = 2000 + year_2 if year_2 < 80 else 1900 + year_2

        gps_dt = dt.datetime(
            year,
            month,
            day,
            hour,
            minute,
            second,
            microsecond,
            tzinfo=dt.timezone.utc,
        )
    except (ValueError, IndexError):
        return None

    return gps_dt.timestamp()


class UbloxI2CReader:
    def __init__(self, bus_num: int, address: int, read_chunk_size: int):
        self.bus_num = bus_num
        self.address = address
        self.read_chunk_size = max(1, min(read_chunk_size, 32))
        self.bus = SMBus(bus_num)

    def bytes_available(self) -> int:
        msb = self.bus.read_byte_data(self.address, UBLOX_I2C_STREAM_COUNT_MSB)
        lsb = self.bus.read_byte_data(self.address, UBLOX_I2C_STREAM_COUNT_LSB)
        count = (msb << 8) | lsb
        if count > 8192:
            count = (lsb << 8) | msb
        return count

    def read_stream_bytes(self, max_bytes: int) -> bytes:
        remaining = max_bytes
        out = bytearray()
        while remaining > 0:
            chunk = min(self.read_chunk_size, remaining)
            out.extend(self.bus.read_i2c_block_data(self.address, UBLOX_I2C_STREAM_DATA, chunk))
            remaining -= chunk
        return bytes(out)


class ChronySockWriter:
    def __init__(self, sock_path: str):
        self.sock_path = sock_path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send_sample(self, sample_time_unix: float, ref_time_unix: float) -> None:
        tv_sec = int(sample_time_unix)
        tv_usec = int((sample_time_unix - tv_sec) * 1_000_000)
        offset = ref_time_unix - sample_time_unix

        sample = ChronySockSample(
            tv_sec=tv_sec,
            tv_usec=tv_usec,
            offset=offset,
            pulse=0,
            leap=0,
            _pad=0,
            magic=CHRONY_SOCK_MAGIC,
        )

        self.sock.sendto(bytes(sample), self.sock_path)


def extract_next_nmea_line(buffer: bytearray) -> Optional[str]:
    start = buffer.find(b"$")
    if start == -1:
        if len(buffer) > 4096:
            del buffer[:-256]
        return None

    if start > 0:
        del buffer[:start]

    newline = buffer.find(b"\n")
    if newline == -1:
        if len(buffer) > 4096:
            del buffer[:-512]
        return None

    raw_line = bytes(buffer[: newline + 1])
    del buffer[: newline + 1]

    try:
        line = raw_line.decode("ascii", errors="ignore").strip()
    except UnicodeDecodeError:
        return None

    if not line.startswith("$"):
        return None
    return line


def wait_for_chrony_socket(sock_path: str) -> None:
    log(f"Waiting for chrony socket: {sock_path}")
    while not os.path.exists(sock_path):
        time.sleep(0.5)
    log("Chrony socket is present")


def main() -> int:
    args = parse_args()

    wait_for_chrony_socket(args.sock_path)

    reader = UbloxI2CReader(
        bus_num=args.i2c_bus,
        address=args.i2c_addr,
        read_chunk_size=args.read_chunk_size,
    )
    writer = ChronySockWriter(args.sock_path)

    log(
        f"Starting u-blox I2C -> chrony bridge on bus {args.i2c_bus}, "
        f"addr 0x{args.i2c_addr:02X}, socket {args.sock_path}"
    )

    buffer = bytearray()
    samples_sent = 0
    last_status = 0.0

    while True:
        try:
            available = reader.bytes_available()
        except OSError as exc:
            log(f"I2C read failed while checking available bytes: {exc}")
            time.sleep(1.0)
            continue

        if available <= 0:
            time.sleep(args.poll_sleep)
            continue

        try:
            buffer.extend(reader.read_stream_bytes(available))
        except OSError as exc:
            log(f"I2C stream read failed: {exc}")
            time.sleep(1.0)
            continue

        while True:
            line = extract_next_nmea_line(buffer)
            if line is None:
                break

            ref_time_unix = parse_rmc_utc_epoch(line)
            if ref_time_unix is None:
                continue

            sample_time_unix = time.time()
            try:
                writer.send_sample(sample_time_unix=sample_time_unix, ref_time_unix=ref_time_unix)
                samples_sent += 1
                if args.debug_nmea:
                    log(f"Accepted RMC: {line}")
            except OSError as exc:
                if exc.errno in (errno.ENOENT, errno.ECONNREFUSED):
                    log(f"Chrony socket unavailable while sending sample: {exc}")
                else:
                    log(f"Failed to send chrony SOCK sample: {exc}")
                time.sleep(1.0)
                break

        now = time.monotonic()
        if now - last_status >= args.status_interval:
            last_status = now
            log(f"Bridge alive. chrony samples sent: {samples_sent}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("Interrupted, exiting.")
        raise SystemExit(130)
