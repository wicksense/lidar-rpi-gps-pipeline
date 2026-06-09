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
import json
import math
import os
import socket
import time
from typing import Any, Optional

try:
    from smbus2 import SMBus  # type: ignore
except ImportError:  # pragma: no cover - fallback for Pi images with python3-smbus only
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:  # pragma: no cover
        SMBus = None  # type: ignore[assignment]


UBLOX_I2C_STREAM_COUNT_MSB = 0xFD
UBLOX_I2C_STREAM_COUNT_LSB = 0xFE
UBLOX_I2C_STREAM_DATA = 0xFF
CHRONY_SOCK_MAGIC = 0x534F434B
DEFAULT_FIX_JSON_PATH = "/run/ublox_i2c_chrony_bridge/latest_fix.json"


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
    parser.add_argument(
        "--fix-json-path",
        default=DEFAULT_FIX_JSON_PATH,
        help=(
            "Path where the latest decoded GPS fix should be published as JSON "
            f"(default: {DEFAULT_FIX_JSON_PATH})"
        ),
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


def parse_nmea_utc_time(value: str) -> Optional[dt.time]:
    if not value:
        return None
    try:
        if "." in value:
            hhmmss, frac = value.split(".", 1)
        else:
            hhmmss, frac = value, ""
        if len(hhmmss) < 6:
            return None
        hour = int(hhmmss[0:2])
        minute = int(hhmmss[2:4])
        second = int(hhmmss[4:6])
        microsecond = int((frac + "000000")[:6]) if frac else 0
        return dt.time(hour, minute, second, microsecond)
    except ValueError:
        return None


def parse_nmea_date(value: str) -> Optional[dt.date]:
    if not value or len(value) < 6:
        return None
    try:
        day = int(value[0:2])
        month = int(value[2:4])
        year_2 = int(value[4:6])
        year = 2000 + year_2 if year_2 < 80 else 1900 + year_2
        return dt.date(year, month, day)
    except ValueError:
        return None


def combine_utc_date_time(gps_date: Optional[dt.date], gps_time: Optional[dt.time]) -> Optional[dt.datetime]:
    if gps_date is None or gps_time is None:
        return None
    return dt.datetime.combine(gps_date, gps_time).replace(tzinfo=dt.timezone.utc)


def parse_nmea_coordinate(value: str, hemisphere: str, *, is_latitude: bool) -> Optional[float]:
    if not value or not hemisphere:
        return None
    degrees_len = 2 if is_latitude else 3
    if len(value) <= degrees_len:
        return None
    try:
        degrees = int(value[:degrees_len])
        minutes = float(value[degrees_len:])
    except ValueError:
        return None
    decimal = degrees + (minutes / 60.0)
    hemi = hemisphere.upper()
    if hemi in {"S", "W"}:
        decimal = -decimal
    elif hemi not in {"N", "E"}:
        return None
    return decimal


def parse_gga_fix(sentence: str, current_date: Optional[dt.date]) -> Optional[dict[str, Any]]:
    if not checksum_ok(sentence):
        return None

    body = sentence[1:].split("*", 1)[0]
    fields = body.split(",")
    if not fields or not fields[0].endswith("GGA") or len(fields) < 10:
        return None

    gps_time = parse_nmea_utc_time(fields[1])
    latitude = parse_nmea_coordinate(fields[2], fields[3], is_latitude=True)
    longitude = parse_nmea_coordinate(fields[4], fields[5], is_latitude=False)
    if latitude is None or longitude is None:
        return None

    try:
        gps_quality = int(fields[6] or 0)
    except ValueError:
        gps_quality = 0
    if gps_quality <= 0:
        return None

    try:
        num_sats: Any = int(fields[7]) if fields[7] else ""
    except ValueError:
        num_sats = fields[7]

    try:
        hdop: Any = float(fields[8]) if fields[8] else ""
    except ValueError:
        hdop = fields[8]

    try:
        altitude_m = float(fields[9]) if fields[9] else float("nan")
    except ValueError:
        altitude_m = float("nan")

    gps_dt = combine_utc_date_time(current_date, gps_time)
    gps_epoch_ns = int(gps_dt.timestamp() * 1e9) if gps_dt is not None else None
    gps_utc_time = gps_time.isoformat() if gps_time is not None else ""

    return {
        "gps_epoch_ns": gps_epoch_ns,
        "gps_utc_time": gps_utc_time,
        "gps_quality": gps_quality,
        "num_sats": num_sats,
        "hdop": hdop,
        "latitude": latitude,
        "longitude": longitude,
        "altitude_m": altitude_m,
    }


def write_json_atomic(path: str, payload: dict[str, Any]) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f_out:
        json.dump(payload, f_out, indent=2)
        f_out.flush()
        os.fsync(f_out.fileno())
    os.replace(tmp_path, path)


class FixPublisher:
    def __init__(self, json_path: str):
        self.json_path = json_path
        self.last_rmc_date: Optional[dt.date] = None
        self.fix_count = 0

    def update_from_nmea(self, sentence: str) -> None:
        if not checksum_ok(sentence):
            return

        body = sentence[1:].split("*", 1)[0]
        fields = body.split(",")
        if not fields:
            return

        talker = fields[0]
        if talker.endswith("RMC") and len(fields) >= 10:
            parsed_date = parse_nmea_date(fields[9])
            if parsed_date is not None:
                self.last_rmc_date = parsed_date
            return

        if not talker.endswith("GGA"):
            return

        fix = parse_gga_fix(sentence, self.last_rmc_date)
        if fix is None:
            return

        now_utc = dt.datetime.now(dt.timezone.utc)
        self.fix_count += 1
        payload = {
            "sequence": self.fix_count,
            "source": "ublox_i2c_bridge",
            "pi_time_ns": time.time_ns(),
            "pi_time_iso": now_utc.isoformat(),
            **fix,
        }
        write_json_atomic(self.json_path, payload)


class UbloxI2CReader:
    def __init__(self, bus_num: int, address: int, read_chunk_size: int):
        if SMBus is None:
            raise RuntimeError("Neither smbus2 nor smbus is available. Install python3-smbus or python3-smbus2.")
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
    fix_publisher = FixPublisher(args.fix_json_path)

    log(
        f"Starting u-blox I2C -> chrony bridge on bus {args.i2c_bus}, "
        f"addr 0x{args.i2c_addr:02X}, socket {args.sock_path}, "
        f"fix JSON {args.fix_json_path}"
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

            try:
                fix_publisher.update_from_nmea(line)
            except OSError as exc:
                log(f"Failed to publish GPS fix JSON: {exc}")

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
            log(
                f"Bridge alive. chrony samples sent: {samples_sent}, "
                f"GPS fixes published: {fix_publisher.fix_count}"
            )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("Interrupted, exiting.")
        raise SystemExit(130)
