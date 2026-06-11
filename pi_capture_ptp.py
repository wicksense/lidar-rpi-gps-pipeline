#!/usr/bin/env python3
"""
pi_capture_ptp.py

Purpose
-------
Run this on the Raspberry Pi during data collection when the timing path is:

    GPS + PPS -> Raspberry Pi -> PTP -> Ouster

This script keeps the same field-capture responsibilities as `pi_capture_raw.py`
while adding readiness gates for the new timing architecture:

1. Read GPS fixes from the shared I2C bridge, serial, or gpsd.
2. Save those GPS fixes to a CSV log file.
3. Optionally configure the Ouster lidar mode and timestamp/PTP settings.
4. Wait for:
   - valid GPS fix
   - Pi clock synchronization (`chronyc waitsync`)
   - Ouster PTP lock via HTTP API
5. Run Ouster raw capture and save chunked LiDAR files.
6. Save a manifest JSON describing the session and timing readiness state.

Important:
- This script does NOT discipline the Pi clock itself.
- This script does NOT implement PTP itself.
- `chrony`, `ptp4l`, `phc2sys`, and any GPS/PPS bridge are expected to be
  managed outside Python.
"""

import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Optional

from ouster_cli_utils import resolve_ouster_cli_executable

try:
    import pynmea2
except ImportError:  # pragma: no cover - exercised only in minimal environments
    pynmea2 = None

try:
    import serial
except ImportError:  # pragma: no cover - exercised only in minimal environments
    serial = None
from pyproj import Transformer

# =======================
# Field Capture Settings
# =======================
OUSTER_HOST = "169.254.237.207"
LIDAR_OUTPUT_MODE = "pcap_raw"
# Leave the current lidar mode unchanged by default. Override with
# `--lidar-mode 1024x20`, `--lidar-mode 2048x10`, or `--lidar-mode 4096x5`.
LIDAR_MODE: Optional[str] = None
# PTP-based timing settings.
OUSTER_TIMESTAMP_MODE: Optional[str] = "TIME_FROM_PTP_1588"
OUSTER_PTP_PROFILE: Optional[str] = "default"
SENSOR_CONFIG_SETTLE_SEC = 10
CAPTURE_DURATION_SEC = 30
CONTINUOUS_CHUNKS = True
WAIT_FOR_GPS_FIX_BEFORE_CAPTURE = True
WAIT_FOR_PI_CLOCK_SYNC_BEFORE_CAPTURE = True
WAIT_FOR_OUSTER_PTP_LOCK_BEFORE_CAPTURE = True
GPS_INPUT_MODE = "bridge"
GPS_PORT = "/dev/gps_ublox"
GPS_BAUD = 9600
GPSD_HOST = "127.0.0.1"
GPSD_PORT = 2947
BRIDGE_FIX_JSON_PATH = "/run/ublox_i2c_chrony_bridge/latest_fix.json"
OUTPUT_DIR = "/home/urp-pi5/capture_output"
UTM_EPSG = 32614
READINESS_TIMEOUT_SEC = 180
READINESS_POLL_SEC = 2
CHRONY_MAX_CORRECTION_SEC = 0.01
CHRONY_WAITSYNC_TRIES = 1
CHRONY_WAITSYNC_INTERVAL_SEC = 1
OUSTER_API_TIMEOUT_SEC = 5
TIMING_ARCHITECTURE = "gps_pps_to_pi__chrony_to_pi_clock__ptp_to_ouster"


def log(message: str) -> None:
    """Simple timestamped logger so terminal output is easy to follow."""
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def ensure_dir(path: str) -> None:
    """Create a directory if it does not exist."""
    os.makedirs(path, exist_ok=True)


def parse_cli_args() -> argparse.Namespace:
    """Optional runtime overrides."""
    parser = argparse.ArgumentParser(
        description="Capture raw Ouster + GPS on Raspberry Pi with Pi-clock/PTP gating."
    )
    parser.add_argument(
        "--wait-for-gps-fix",
        dest="wait_for_gps_fix",
        action="store_true",
        help="Override and require GPS fix before LiDAR capture starts.",
    )
    parser.add_argument(
        "--no-wait-for-gps-fix",
        dest="wait_for_gps_fix",
        action="store_false",
        help="Override and start without waiting for GPS fix.",
    )
    parser.add_argument(
        "--wait-for-pi-clock-sync",
        dest="wait_for_pi_clock_sync",
        action="store_true",
        help="Override and require chrony-reported Pi clock sync before capture.",
    )
    parser.add_argument(
        "--no-wait-for-pi-clock-sync",
        dest="wait_for_pi_clock_sync",
        action="store_false",
        help="Override and skip chrony-based Pi clock readiness gating.",
    )
    parser.add_argument(
        "--wait-for-ouster-ptp-lock",
        dest="wait_for_ouster_ptp_lock",
        action="store_true",
        help="Override and require Ouster PTP lock before capture.",
    )
    parser.add_argument(
        "--no-wait-for-ouster-ptp-lock",
        dest="wait_for_ouster_ptp_lock",
        action="store_false",
        help="Override and skip Ouster PTP readiness gating.",
    )
    parser.add_argument(
        "--lidar-mode",
        default=None,
        help=(
            "Optional Ouster lidar mode override, for example 1024x20, 2048x10, or 4096x5. "
            "If omitted, uses the hardcoded LIDAR_MODE setting."
        ),
    )
    parser.add_argument(
        "--timestamp-mode",
        default=None,
        help=(
            "Optional Ouster timestamp mode override, for example TIME_FROM_PTP_1588. "
            "If omitted, uses the hardcoded OUSTER_TIMESTAMP_MODE setting."
        ),
    )
    parser.add_argument(
        "--ptp-profile",
        default=None,
        help=(
            "Optional Ouster PTP profile override, for example default or gptp. "
            "If omitted, uses the hardcoded OUSTER_PTP_PROFILE setting."
        ),
    )
    parser.add_argument(
        "--gps-input-mode",
        choices=["serial", "gpsd", "bridge"],
        default=None,
        help="GPS ingestion mode for this run.",
    )
    parser.add_argument("--gps-port", default=None, help="Serial GPS device path override.")
    parser.add_argument("--gps-baud", type=int, default=None, help="Serial GPS baud rate override.")
    parser.add_argument("--gpsd-host", default=None, help="gpsd host override.")
    parser.add_argument("--gpsd-port", type=int, default=None, help="gpsd port override.")
    parser.add_argument(
        "--bridge-fix-path",
        default=None,
        help="Path to the latest GPS fix JSON published by the I2C chrony bridge.",
    )
    parser.add_argument("--ouster-host", default=None, help="Ouster host/IP override.")
    parser.add_argument(
        "--readiness-timeout-sec",
        type=int,
        default=None,
        help="Timeout for each readiness gate.",
    )
    parser.set_defaults(
        wait_for_gps_fix=None,
        wait_for_pi_clock_sync=None,
        wait_for_ouster_ptp_lock=None,
    )
    return parser.parse_args()


def normalize_lidar_mode(mode: Optional[str]) -> Optional[str]:
    """Normalize lidar mode strings like 1024x20 or leave unset."""
    if mode is None:
        return None
    normalized = mode.strip().lower()
    if not normalized:
        return None
    if "x" not in normalized:
        raise ValueError(f"Invalid LIDAR_MODE '{mode}'. Expected values like 1024x20, 2048x10, or 4096x5.")
    left, right = normalized.split("x", 1)
    if not (left.isdigit() and right.isdigit()):
        raise ValueError(f"Invalid LIDAR_MODE '{mode}'. Expected values like 1024x20, 2048x10, or 4096x5.")
    return f"{int(left)}x{int(right)}"


def normalize_optional_string(value: Optional[str]) -> Optional[str]:
    """Normalize optional config strings and treat empty values as unset."""
    if value is None:
        return None
    normalized = value.strip()
    return normalized or None


def gpsd_time_to_epoch_ns(time_text: Optional[str]) -> Optional[int]:
    """Convert a gpsd ISO timestamp string to Unix epoch nanoseconds."""
    if not time_text:
        return None
    normalized = time_text.strip()
    if not normalized:
        return None
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    parsed = dt.datetime.fromisoformat(normalized)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.timezone.utc)
    return int(parsed.timestamp() * 1e9)


def gpsd_time_to_utc_text(time_text: Optional[str]) -> str:
    """Extract a time-of-day display string from a gpsd timestamp."""
    if not time_text:
        return ""
    normalized = time_text.strip()
    if not normalized:
        return ""
    if normalized.endswith("Z"):
        normalized = normalized[:-1] + "+00:00"
    parsed = dt.datetime.fromisoformat(normalized)
    return parsed.timetz().isoformat()


class BaseGpsLogger(threading.Thread):
    """Shared CSV-writing behavior for GPS logger implementations."""

    def __init__(
        self,
        out_csv_path: str,
        stop_event: threading.Event,
        epsg_out: int = 32614,
        status_log_every_sec: float = 1.0,
    ):
        super().__init__(daemon=True)
        self.out_csv_path = out_csv_path
        self.stop_event = stop_event
        self.transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg_out}", always_xy=True)
        self.status_log_every_sec = status_log_every_sec
        self.last_status_log = 0.0
        self.rows_written = 0
        self.first_fix_time_ns: Optional[int] = None
        self.latest_fix: Optional[dict[str, Any]] = None

    def _write_fix(
        self,
        writer: csv.writer,
        *,
        pi_time_ns: int,
        pi_time_iso: str,
        gps_epoch_ns: Optional[int],
        gps_utc_time: str,
        gps_quality: Any,
        num_sats: Any,
        hdop: Any,
        latitude: float,
        longitude: float,
        altitude_m: float,
        source_summary: str,
    ) -> None:
        easting, northing = self.transformer.transform(longitude, latitude)
        writer.writerow(
            [
                pi_time_ns,
                pi_time_iso,
                gps_epoch_ns if gps_epoch_ns is not None else "",
                gps_utc_time,
                gps_quality,
                num_sats,
                hdop,
                latitude,
                longitude,
                altitude_m,
                easting,
                northing,
            ]
        )
        self.rows_written += 1
        if self.first_fix_time_ns is None:
            self.first_fix_time_ns = pi_time_ns
        self.latest_fix = {
            "pi_time_ns": pi_time_ns,
            "pi_time_iso": pi_time_iso,
            "gps_epoch_ns": gps_epoch_ns,
            "gps_utc_time": gps_utc_time,
            "gps_quality": gps_quality,
            "num_sats": num_sats,
            "hdop": hdop,
            "latitude": latitude,
            "longitude": longitude,
            "altitude_m": altitude_m,
            "easting": easting,
            "northing": northing,
        }

        now = time.time()
        if now - self.last_status_log >= self.status_log_every_sec:
            self.last_status_log = now
            log(
                "GPS fix logged: "
                f"{source_summary}, lat={latitude:.6f}, lon={longitude:.6f}, e={easting:.2f}, n={northing:.2f}"
            )


class SerialGpsLogger(BaseGpsLogger):
    """Background GPS logger for raw serial NMEA input."""

    def __init__(
        self,
        serial_port: str,
        baud: int,
        out_csv_path: str,
        stop_event: threading.Event,
        epsg_out: int = 32614,
        status_log_every_sec: float = 1.0,
    ):
        super().__init__(out_csv_path=out_csv_path, stop_event=stop_event, epsg_out=epsg_out, status_log_every_sec=status_log_every_sec)
        self.serial_port = serial_port
        self.baud = baud
        self.last_rmc_date: Optional[dt.date] = None

    @staticmethod
    def _gps_epoch_ns_from_date_and_time(
        gps_date: Optional[dt.date], gps_time: Optional[dt.time]
    ) -> Optional[int]:
        if gps_date is None or gps_time is None:
            return None
        combined = dt.datetime.combine(gps_date, gps_time).replace(tzinfo=dt.timezone.utc)
        return int(combined.timestamp() * 1e9)

    def run(self) -> None:
        if pynmea2 is None or serial is None:
            log("Serial GPS logger requires `pynmea2` and `pyserial` to be installed.")
            return

        header = [
            "pi_time_ns",
            "pi_time_iso",
            "gps_epoch_ns",
            "gps_utc_time",
            "gps_quality",
            "num_sats",
            "hdop",
            "latitude",
            "longitude",
            "altitude_m",
            "easting",
            "northing",
        ]

        with open(self.out_csv_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(header)

            try:
                log(f"Opening GPS serial port: {self.serial_port} @ {self.baud}")
                with serial.Serial(self.serial_port, self.baud, timeout=1) as ser:
                    while not self.stop_event.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue

                        pi_time_ns = time.time_ns()
                        pi_time_iso = dt.datetime.now(dt.timezone.utc).isoformat()

                        try:
                            msg = pynmea2.parse(raw.decode("ascii", errors="ignore"))
                        except pynmea2.ParseError:
                            continue

                        if isinstance(msg, pynmea2.RMC):
                            if getattr(msg, "datestamp", None):
                                self.last_rmc_date = msg.datestamp
                            continue

                        if not isinstance(msg, pynmea2.GGA):
                            continue

                        gps_quality = int(getattr(msg, "gps_qual", 0) or 0)
                        if gps_quality == 0:
                            continue
                        if msg.latitude is None or msg.longitude is None:
                            continue

                        try:
                            altitude_m = float(msg.altitude) if msg.altitude not in (None, "") else float("nan")
                        except (TypeError, ValueError):
                            altitude_m = float("nan")

                        gps_epoch_ns = self._gps_epoch_ns_from_date_and_time(
                            self.last_rmc_date, getattr(msg, "timestamp", None)
                        )

                        self._write_fix(
                            writer,
                            pi_time_ns=pi_time_ns,
                            pi_time_iso=pi_time_iso,
                            gps_epoch_ns=gps_epoch_ns,
                            gps_utc_time=str(getattr(msg, "timestamp", "")),
                            gps_quality=gps_quality,
                            num_sats=getattr(msg, "num_sats", ""),
                            hdop=getattr(msg, "horizontal_dil", ""),
                            latitude=float(msg.latitude),
                            longitude=float(msg.longitude),
                            altitude_m=altitude_m,
                            source_summary=f"serial q={gps_quality}",
                        )
                        f_out.flush()

            except serial.SerialException as exc:
                log(f"GPS serial error: {exc}")
            except Exception as exc:
                log(f"Unexpected GPS serial logger error: {exc}")

        log(f"GPS logger stopped. Rows written: {self.rows_written}")


class GpsdGpsLogger(BaseGpsLogger):
    """Background GPS logger for gpsd JSON streaming."""

    def __init__(
        self,
        host: str,
        port: int,
        out_csv_path: str,
        stop_event: threading.Event,
        epsg_out: int = 32614,
        status_log_every_sec: float = 1.0,
    ):
        super().__init__(out_csv_path=out_csv_path, stop_event=stop_event, epsg_out=epsg_out, status_log_every_sec=status_log_every_sec)
        self.host = host
        self.port = port
        self.last_num_sats: Any = ""
        self.last_hdop: Any = ""

    def run(self) -> None:
        header = [
            "pi_time_ns",
            "pi_time_iso",
            "gps_epoch_ns",
            "gps_utc_time",
            "gps_quality",
            "num_sats",
            "hdop",
            "latitude",
            "longitude",
            "altitude_m",
            "easting",
            "northing",
        ]

        with open(self.out_csv_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(header)

            try:
                log(f"Connecting to gpsd at {self.host}:{self.port}")
                with socket.create_connection((self.host, self.port), timeout=5) as sock:
                    sock.sendall(b'?WATCH={"enable":true,"json":true}\n')
                    file_obj = sock.makefile("r", encoding="utf-8", newline="\n")

                    while not self.stop_event.is_set():
                        line = file_obj.readline()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        msg_class = msg.get("class")
                        if msg_class == "SKY":
                            self.last_hdop = msg.get("hdop", self.last_hdop)
                            if "uSat" in msg:
                                self.last_num_sats = msg.get("uSat", self.last_num_sats)
                            elif isinstance(msg.get("satellites"), list):
                                self.last_num_sats = sum(1 for sat in msg["satellites"] if sat.get("used"))
                            continue

                        if msg_class != "TPV":
                            continue

                        mode = int(msg.get("mode", 0) or 0)
                        latitude = msg.get("lat")
                        longitude = msg.get("lon")
                        if mode < 2 or latitude is None or longitude is None:
                            continue
                        if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)):
                            continue
                        if math.isnan(latitude) or math.isnan(longitude):
                            continue

                        altitude_value = msg.get("altMSL", msg.get("altHAE", msg.get("alt")))
                        try:
                            altitude_m = float(altitude_value) if altitude_value is not None else float("nan")
                        except (TypeError, ValueError):
                            altitude_m = float("nan")

                        time_text = msg.get("time")
                        pi_time_ns = time.time_ns()
                        pi_time_iso = dt.datetime.now(dt.timezone.utc).isoformat()

                        self._write_fix(
                            writer,
                            pi_time_ns=pi_time_ns,
                            pi_time_iso=pi_time_iso,
                            gps_epoch_ns=gpsd_time_to_epoch_ns(time_text),
                            gps_utc_time=gpsd_time_to_utc_text(time_text),
                            gps_quality=mode,
                            num_sats=self.last_num_sats,
                            hdop=self.last_hdop,
                            latitude=float(latitude),
                            longitude=float(longitude),
                            altitude_m=altitude_m,
                            source_summary=f"gpsd mode={mode}",
                        )
                        f_out.flush()

            except OSError as exc:
                log(f"gpsd connection error: {exc}")
            except Exception as exc:
                log(f"Unexpected gpsd logger error: {exc}")

        log(f"GPS logger stopped. Rows written: {self.rows_written}")


class BridgeGpsLogger(BaseGpsLogger):
    """Background GPS logger that consumes fixes published by the I2C bridge."""

    def __init__(
        self,
        fix_json_path: str,
        out_csv_path: str,
        stop_event: threading.Event,
        epsg_out: int = 32614,
        status_log_every_sec: float = 1.0,
        poll_sec: float = 0.2,
    ):
        super().__init__(
            out_csv_path=out_csv_path,
            stop_event=stop_event,
            epsg_out=epsg_out,
            status_log_every_sec=status_log_every_sec,
        )
        self.fix_json_path = fix_json_path
        self.poll_sec = poll_sec

    @staticmethod
    def _safe_float(value: Any, default: float = float("nan")) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def run(self) -> None:
        header = [
            "pi_time_ns",
            "pi_time_iso",
            "gps_epoch_ns",
            "gps_utc_time",
            "gps_quality",
            "num_sats",
            "hdop",
            "latitude",
            "longitude",
            "altitude_m",
            "easting",
            "northing",
        ]

        last_sequence: Optional[int] = None
        last_signature: Optional[tuple[Any, ...]] = None

        with open(self.out_csv_path, "w", newline="", encoding="utf-8") as f_out:
            writer = csv.writer(f_out)
            writer.writerow(header)
            log(f"Watching GPS bridge fix file: {self.fix_json_path}")
            missing_logged = False

            while not self.stop_event.is_set():
                if not os.path.exists(self.fix_json_path):
                    if not missing_logged:
                        log(f"Waiting for GPS bridge fix file to appear: {self.fix_json_path}")
                        missing_logged = True
                    time.sleep(self.poll_sec)
                    continue
                if missing_logged:
                    log("GPS bridge fix file detected.")
                    missing_logged = False

                try:
                    with open(self.fix_json_path, "r", encoding="utf-8") as f_in:
                        fix = json.load(f_in)
                except (OSError, json.JSONDecodeError):
                    time.sleep(self.poll_sec)
                    continue

                sequence = fix.get("sequence")
                signature = (
                    fix.get("gps_epoch_ns"),
                    fix.get("gps_utc_time"),
                    fix.get("latitude"),
                    fix.get("longitude"),
                    fix.get("gps_quality"),
                    fix.get("num_sats"),
                    fix.get("hdop"),
                    fix.get("altitude_m"),
                )
                if sequence is not None:
                    if sequence == last_sequence:
                        time.sleep(self.poll_sec)
                        continue
                elif signature == last_signature:
                    time.sleep(self.poll_sec)
                    continue

                latitude = fix.get("latitude")
                longitude = fix.get("longitude")
                if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)):
                    time.sleep(self.poll_sec)
                    continue
                if math.isnan(latitude) or math.isnan(longitude):
                    time.sleep(self.poll_sec)
                    continue

                pi_time_ns = int(fix.get("pi_time_ns") or time.time_ns())
                pi_time_iso = str(fix.get("pi_time_iso") or dt.datetime.now(dt.timezone.utc).isoformat())
                gps_epoch_ns = fix.get("gps_epoch_ns")
                try:
                    gps_epoch_ns = int(gps_epoch_ns) if gps_epoch_ns not in (None, "") else None
                except (TypeError, ValueError):
                    gps_epoch_ns = None

                self._write_fix(
                    writer,
                    pi_time_ns=pi_time_ns,
                    pi_time_iso=pi_time_iso,
                    gps_epoch_ns=gps_epoch_ns,
                    gps_utc_time=str(fix.get("gps_utc_time") or ""),
                    gps_quality=fix.get("gps_quality", ""),
                    num_sats=fix.get("num_sats", ""),
                    hdop=fix.get("hdop", ""),
                    latitude=float(latitude),
                    longitude=float(longitude),
                    altitude_m=self._safe_float(fix.get("altitude_m")),
                    source_summary=f"bridge q={fix.get('gps_quality', '')}",
                )
                f_out.flush()

                last_sequence = sequence if isinstance(sequence, int) else last_sequence
                last_signature = signature

                time.sleep(self.poll_sec)

        log(f"GPS logger stopped. Rows written: {self.rows_written}")


def build_gps_logger(
    *,
    gps_input_mode: str,
    stop_event: threading.Event,
    gps_csv_path: str,
    gps_port: str,
    gps_baud: int,
    gpsd_host: str,
    gpsd_port: int,
    bridge_fix_path: str,
) -> BaseGpsLogger:
    """Construct the requested GPS logger implementation."""
    if gps_input_mode == "serial":
        if pynmea2 is None or serial is None:
            raise RuntimeError("GPS serial mode requires `pynmea2` and `pyserial` to be installed.")
        return SerialGpsLogger(
            serial_port=gps_port,
            baud=gps_baud,
            out_csv_path=gps_csv_path,
            stop_event=stop_event,
            epsg_out=UTM_EPSG,
        )
    if gps_input_mode == "gpsd":
        return GpsdGpsLogger(
            host=gpsd_host,
            port=gpsd_port,
            out_csv_path=gps_csv_path,
            stop_event=stop_event,
            epsg_out=UTM_EPSG,
        )
    if gps_input_mode == "bridge":
        return BridgeGpsLogger(
            fix_json_path=bridge_fix_path,
            out_csv_path=gps_csv_path,
            stop_event=stop_event,
            epsg_out=UTM_EPSG,
        )
    raise ValueError(f"Unsupported GPS_INPUT_MODE '{gps_input_mode}'.")


def wait_for_first_gps_fix(gps_logger: BaseGpsLogger, stop_event: threading.Event) -> bool:
    """Wait for the first valid GPS fix written by the logger."""
    while not stop_event.is_set():
        if gps_logger.first_fix_time_ns is not None:
            log("GPS fix acquired. Continuing readiness checks.")
            return True
        time.sleep(0.5)
    return False


def run_command_capture(cmd: list[str]) -> dict[str, Any]:
    """Run a command and capture stdout/stderr for diagnostics."""
    result = subprocess.run(cmd, capture_output=True, text=True)
    return {
        "cmd": cmd,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }


def ouster_cli_command(*args: str) -> list[str]:
    """Build an `ouster-cli` command using the best-resolved executable."""
    return [resolve_ouster_cli_executable(), *args]


def collect_chrony_snapshot() -> dict[str, Any]:
    """Capture chrony status details for manifests and debug logs."""
    return {
        "tracking": run_command_capture(["chronyc", "tracking"]),
        "sources": run_command_capture(["chronyc", "sources", "-v"]),
    }


def wait_for_pi_clock_sync(
    *,
    stop_event: threading.Event,
    timeout_sec: int,
    poll_sec: int,
    max_correction_sec: float,
) -> tuple[bool, dict[str, Any]]:
    """Wait until chrony reports the Pi system clock is synchronized."""
    deadline = time.time() + timeout_sec
    last_snapshot = collect_chrony_snapshot()

    while not stop_event.is_set():
        waitsync = run_command_capture(
            [
                "chronyc",
                "waitsync",
                str(CHRONY_WAITSYNC_TRIES),
                str(max_correction_sec),
                "0",
                str(CHRONY_WAITSYNC_INTERVAL_SEC),
            ]
        )
        last_snapshot = {
            "waitsync": waitsync,
            **collect_chrony_snapshot(),
        }
        if waitsync["returncode"] == 0:
            log("Pi clock synchronized according to chrony.")
            return True, last_snapshot

        summary = waitsync["stdout"] or waitsync["stderr"] or "chronyc waitsync not yet satisfied"
        log(f"Waiting for Pi clock sync: {summary}")

        if time.time() >= deadline:
            break
        time.sleep(poll_sec)

    return False, last_snapshot


def ouster_api_request(
    method: str,
    host: str,
    path: str,
    payload: Optional[Any] = None,
) -> Any:
    """Call an Ouster HTTP API endpoint and decode JSON when possible."""
    url = f"http://{host}{path}"
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    request = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=OUSTER_API_TIMEOUT_SEC) as response:
            body = response.read().decode("utf-8").strip()
    except urllib.error.HTTPError as exc:
        error_body = exc.read().decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"Ouster HTTP {method} {path} failed with {exc.code}: {error_body}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Ouster HTTP {method} {path} failed: {exc}") from exc

    if not body:
        return None
    try:
        return json.loads(body)
    except json.JSONDecodeError:
        return body


def get_ouster_time_status(host: str) -> dict[str, Any]:
    """Fetch current Ouster timing state."""
    return {
        "time": ouster_api_request("GET", host, "/api/v1/time"),
        "ptp": ouster_api_request("GET", host, "/api/v1/time/ptp"),
        "timestamp_mode": ouster_api_request("GET", host, "/api/v1/sensor/config/timestamp_mode"),
        "ptp_profile": ouster_api_request("GET", host, "/api/v1/time/ptp/profile"),
    }


def summarize_ouster_ptp_status(status: dict[str, Any]) -> str:
    """Build a short human-readable summary for logs."""
    timestamp_mode = status.get("timestamp_mode")
    ptp = status.get("ptp") if isinstance(status.get("ptp"), dict) else {}
    port_state = ""
    gm_present = ""
    if isinstance(ptp, dict):
        port_state = str(ptp.get("port_data_set", {}).get("port_state", "") or "")
        gm_present = str(ptp.get("time_status_np", {}).get("gm_present", "") or "")
    return f"timestamp_mode={timestamp_mode}, port_state={port_state}, gm_present={gm_present}"


def ouster_ptp_locked(status: dict[str, Any]) -> bool:
    """Return True when Ouster appears locked to an external PTP master."""
    timestamp_mode = status.get("timestamp_mode")
    if isinstance(timestamp_mode, str) and timestamp_mode != "TIME_FROM_PTP_1588":
        return False

    ptp = status.get("ptp")
    if not isinstance(ptp, dict):
        return False

    port_state = str(ptp.get("port_data_set", {}).get("port_state", "") or "").upper()
    gm_present = ptp.get("time_status_np", {}).get("gm_present")
    grandmaster_identity = str(ptp.get("parent_data_set", {}).get("grandmaster_identity", "") or "")

    if port_state != "SLAVE":
        return False
    if gm_present is False:
        return False
    if gm_present is None and not grandmaster_identity:
        return False
    return True


def wait_for_ouster_ptp_lock(
    *,
    host: str,
    stop_event: threading.Event,
    timeout_sec: int,
    poll_sec: int,
) -> tuple[bool, dict[str, Any]]:
    """Wait until the Ouster reports a healthy PTP-locked state."""
    deadline = time.time() + timeout_sec
    last_status: dict[str, Any] = {}

    while not stop_event.is_set():
        try:
            last_status = get_ouster_time_status(host)
        except Exception as exc:
            log(f"Waiting for Ouster PTP lock: {exc}")
            last_status = {"error": str(exc)}
        else:
            if ouster_ptp_locked(last_status):
                log("Ouster PTP lock acquired.")
                return True, last_status
            log(f"Waiting for Ouster PTP lock: {summarize_ouster_ptp_status(last_status)}")

        if time.time() >= deadline:
            break
        time.sleep(poll_sec)

    return False, last_status


def configure_ouster_lidar_mode(host: str, lidar_mode: Optional[str]) -> bool:
    """Optionally apply lidar mode using `ouster-cli`."""
    if not lidar_mode:
        log("Sensor config: leaving current lidar mode unchanged.")
        return False

    cmd = [
        *ouster_cli_command(
            "source",
            host,
            "config",
            "lidar_mode",
            lidar_mode,
        ),
    ]
    log("Applying Ouster sensor config:")
    log("  " + " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.stdout.strip():
        log(f"[ouster config out] {result.stdout.strip()}")
    if result.stderr.strip():
        log(f"[ouster config err] {result.stderr.strip()}")
    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to configure Ouster lidar_mode={lidar_mode}. "
            f"`ouster-cli` exited with code {result.returncode}."
        )
    log(f"Ouster lidar mode configured: {lidar_mode}")
    return True


def configure_ouster_time_mode(
    host: str,
    *,
    timestamp_mode: Optional[str],
    ptp_profile: Optional[str],
) -> bool:
    """Optionally apply Ouster timestamp/PTP config via HTTP API."""
    changed = False
    if ptp_profile:
        log(f"Setting Ouster PTP profile via HTTP API: {ptp_profile}")
        current_profile = ouster_api_request("GET", host, "/api/v1/time/ptp/profile")
        if current_profile != ptp_profile:
            ouster_api_request("PUT", host, "/api/v1/time/ptp/profile", ptp_profile)
            changed = True
        else:
            log("Ouster PTP profile already matches requested value.")

    if timestamp_mode:
        log(f"Setting Ouster timestamp mode via HTTP API: {timestamp_mode}")
        current_mode = ouster_api_request("GET", host, "/api/v1/sensor/config/timestamp_mode")
        if current_mode != timestamp_mode:
            ouster_api_request("PUT", host, "/api/v1/sensor/config/timestamp_mode", timestamp_mode)
            changed = True
        else:
            log("Ouster timestamp mode already matches requested value.")

    if timestamp_mode or ptp_profile:
        status = get_ouster_time_status(host)
        log(f"Ouster timing after config: {summarize_ouster_ptp_status(status)}")

    return changed


def configure_ouster_sensor(
    host: str,
    *,
    lidar_mode: Optional[str],
    timestamp_mode: Optional[str],
    ptp_profile: Optional[str],
) -> None:
    """Apply requested Ouster mode/timing config before capture starts."""
    changed = False
    changed = configure_ouster_lidar_mode(host, lidar_mode) or changed
    changed = configure_ouster_time_mode(
        host,
        timestamp_mode=timestamp_mode,
        ptp_profile=ptp_profile,
    ) or changed
    if changed and SENSOR_CONFIG_SETTLE_SEC > 0:
        log(f"Waiting {SENSOR_CONFIG_SETTLE_SEC}s for sensor to settle after config change...")
        time.sleep(SENSOR_CONFIG_SETTLE_SEC)


def find_chunk_files(requested_output_path: str) -> list[str]:
    """Return files created for this chunk prefix."""
    stem = os.path.splitext(requested_output_path)[0]
    return sorted([path for path in glob.glob(stem + "*") if os.path.isfile(path)])


def run_ouster_capture(
    host: str,
    seconds: int,
    requested_output_path: str,
    stop_event: threading.Event,
    output_mode: str,
) -> int:
    """Start one ouster-cli capture and return the process exit code."""
    if output_mode == "csv":
        cmd = [
            *ouster_cli_command(
                "source",
                host,
                "slice",
                f"0s:{seconds}s",
                "save",
                requested_output_path,
            ),
        ]
    elif output_mode == "pcap_raw":
        cmd = [
            *ouster_cli_command(
                "source",
                host,
                "save_raw",
                requested_output_path,
            ),
        ]
    else:
        raise ValueError(f"Unsupported LIDAR_OUTPUT_MODE: {output_mode}")

    log("Starting ouster capture:")
    log("  " + " ".join(cmd))

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def stream(pipe: Any, label: str) -> None:
        for line in iter(pipe.readline, ""):
            if stop_event.is_set():
                break
            text = line.strip()
            if text:
                log(f"[ouster {label}] {text}")
        pipe.close()

    t_out = threading.Thread(target=stream, args=(proc.stdout, "out"), daemon=True)
    t_err = threading.Thread(target=stream, args=(proc.stderr, "err"), daemon=True)
    t_out.start()
    t_err.start()

    deadline = time.time() + seconds
    sent_sigint = False
    while proc.poll() is None:
        if stop_event.is_set():
            if not sent_sigint:
                proc.send_signal(signal.SIGINT)
                sent_sigint = True
            time.sleep(0.2)
            continue

        if output_mode == "pcap_raw" and time.time() >= deadline and not sent_sigint:
            proc.send_signal(signal.SIGINT)
            sent_sigint = True
            continue

        time.sleep(0.2)

    if proc.poll() is None:
        proc.terminate()

    return_code = proc.wait()
    log(f"ouster-cli exited with code {return_code}")
    return return_code


def build_abort_manifest(
    *,
    session_id: str,
    capture_mode: str,
    ouster_host: str,
    lidar_mode: Optional[str],
    timestamp_mode: Optional[str],
    ptp_profile: Optional[str],
    wait_for_gps_fix: bool,
    wait_for_pi_clock_sync: bool,
    wait_for_ouster_ptp_lock: bool,
    gps_input_mode: str,
    gps_port: str,
    gps_baud: int,
    gpsd_host: str,
    gpsd_port: int,
    bridge_fix_path: str,
    gps_csv_path: str,
    gps_logger: BaseGpsLogger,
    readiness: dict[str, Any],
    chunks: list[dict[str, Any]],
) -> dict[str, Any]:
    """Build a manifest payload for both aborted and completed sessions."""
    now_ns = time.time_ns()
    now_iso = dt.datetime.now(dt.timezone.utc).isoformat()
    return {
        "session_id": session_id,
        "timing_architecture": TIMING_ARCHITECTURE,
        "capture_mode": capture_mode,
        "run_start_iso_utc": now_iso,
        "run_end_iso_utc": now_iso,
        "run_start_ns": now_ns,
        "run_end_ns": now_ns,
        "ouster_host": ouster_host,
        "lidar_output_mode": LIDAR_OUTPUT_MODE,
        "lidar_mode": lidar_mode,
        "ouster_timestamp_mode": timestamp_mode,
        "ouster_ptp_profile": ptp_profile,
        "chunk_duration_sec": CAPTURE_DURATION_SEC,
        "continuous_chunks": CONTINUOUS_CHUNKS,
        "wait_for_gps_fix_before_capture": wait_for_gps_fix,
        "wait_for_pi_clock_sync_before_capture": wait_for_pi_clock_sync,
        "wait_for_ouster_ptp_lock_before_capture": wait_for_ouster_ptp_lock,
        "gps_input_mode": gps_input_mode,
        "gps_port": gps_port,
        "gps_baud": gps_baud,
        "gpsd_host": gpsd_host,
        "gpsd_port": gpsd_port,
        "bridge_fix_path": bridge_fix_path,
        "utm_epsg": UTM_EPSG,
        "gps_csv_path": gps_csv_path,
        "gps_rows_written": gps_logger.rows_written,
        "gps_first_fix_time_ns": gps_logger.first_fix_time_ns,
        "gps_latest_fix": gps_logger.latest_fix,
        "readiness": readiness,
        "chunks_captured": len(chunks),
        "chunks": chunks,
    }


def main() -> None:
    cli = parse_cli_args()
    require_gps_fix = WAIT_FOR_GPS_FIX_BEFORE_CAPTURE if cli.wait_for_gps_fix is None else cli.wait_for_gps_fix
    require_pi_clock_sync = (
        WAIT_FOR_PI_CLOCK_SYNC_BEFORE_CAPTURE
        if cli.wait_for_pi_clock_sync is None
        else cli.wait_for_pi_clock_sync
    )
    require_ouster_ptp_lock = (
        WAIT_FOR_OUSTER_PTP_LOCK_BEFORE_CAPTURE
        if cli.wait_for_ouster_ptp_lock is None
        else cli.wait_for_ouster_ptp_lock
    )
    readiness_timeout_sec = cli.readiness_timeout_sec or READINESS_TIMEOUT_SEC
    lidar_mode = normalize_lidar_mode(cli.lidar_mode if cli.lidar_mode is not None else LIDAR_MODE)
    timestamp_mode = normalize_optional_string(cli.timestamp_mode if cli.timestamp_mode is not None else OUSTER_TIMESTAMP_MODE)
    ptp_profile = normalize_optional_string(cli.ptp_profile if cli.ptp_profile is not None else OUSTER_PTP_PROFILE)
    gps_input_mode = cli.gps_input_mode or GPS_INPUT_MODE
    gps_port = cli.gps_port or GPS_PORT
    gps_baud = cli.gps_baud or GPS_BAUD
    gpsd_host = cli.gpsd_host or GPSD_HOST
    gpsd_port = cli.gpsd_port or GPSD_PORT
    bridge_fix_path = cli.bridge_fix_path or BRIDGE_FIX_JSON_PATH
    ouster_host = cli.ouster_host or OUSTER_HOST

    ensure_dir(OUTPUT_DIR)

    session_id = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    gps_csv_path = os.path.join(OUTPUT_DIR, f"raw_gps_{session_id}.csv")
    manifest_path = os.path.join(OUTPUT_DIR, f"capture_manifest_{session_id}.json")
    stop_event = threading.Event()
    chunks: list[dict[str, Any]] = []
    readiness: dict[str, Any] = {
        "gps_fix": {"required": require_gps_fix, "ready": False},
        "pi_clock_sync": {"required": require_pi_clock_sync, "ready": False},
        "ouster_ptp_lock": {"required": require_ouster_ptp_lock, "ready": False},
    }

    def handle_signal(_sig: int, _frame: Any) -> None:
        log("Shutdown requested.")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    configure_ouster_sensor(
        ouster_host,
        lidar_mode=lidar_mode,
        timestamp_mode=timestamp_mode,
        ptp_profile=ptp_profile,
    )

    gps_logger = build_gps_logger(
        gps_input_mode=gps_input_mode,
        stop_event=stop_event,
        gps_csv_path=gps_csv_path,
        gps_port=gps_port,
        gps_baud=gps_baud,
        gpsd_host=gpsd_host,
        gpsd_port=gpsd_port,
        bridge_fix_path=bridge_fix_path,
    )
    gps_logger.start()

    time.sleep(2)

    if require_gps_fix:
        log("Waiting for first GPS fix before LiDAR capture...")
        has_fix = wait_for_first_gps_fix(gps_logger=gps_logger, stop_event=stop_event)
        readiness["gps_fix"]["ready"] = has_fix
        readiness["gps_fix"]["first_fix_time_ns"] = gps_logger.first_fix_time_ns
        if not has_fix:
            log("Capture aborted before start: GPS fix was not acquired.")
            stop_event.set()
            gps_logger.join(timeout=3)
            manifest = build_abort_manifest(
                session_id=session_id,
                capture_mode="aborted_no_gps_fix",
                ouster_host=ouster_host,
                lidar_mode=lidar_mode,
                timestamp_mode=timestamp_mode,
                ptp_profile=ptp_profile,
                wait_for_gps_fix=require_gps_fix,
                wait_for_pi_clock_sync=require_pi_clock_sync,
                wait_for_ouster_ptp_lock=require_ouster_ptp_lock,
                gps_input_mode=gps_input_mode,
                gps_port=gps_port,
                gps_baud=gps_baud,
                gpsd_host=gpsd_host,
                gpsd_port=gpsd_port,
                bridge_fix_path=bridge_fix_path,
                gps_csv_path=gps_csv_path,
                gps_logger=gps_logger,
                readiness=readiness,
                chunks=chunks,
            )
            with open(manifest_path, "w", encoding="utf-8") as f_out:
                json.dump(manifest, f_out, indent=2)
            log(f"GPS CSV:     {gps_csv_path}")
            log(f"Manifest:    {manifest_path}")
            return
    else:
        readiness["gps_fix"]["ready"] = True

    if require_pi_clock_sync:
        log("Waiting for Pi clock sync from chrony before LiDAR capture...")
        synced, chrony_snapshot = wait_for_pi_clock_sync(
            stop_event=stop_event,
            timeout_sec=readiness_timeout_sec,
            poll_sec=READINESS_POLL_SEC,
            max_correction_sec=CHRONY_MAX_CORRECTION_SEC,
        )
        readiness["pi_clock_sync"]["ready"] = synced
        readiness["pi_clock_sync"]["chrony"] = chrony_snapshot
        if not synced:
            log("Capture aborted before start: Pi clock did not reach chrony sync.")
            stop_event.set()
            gps_logger.join(timeout=3)
            manifest = build_abort_manifest(
                session_id=session_id,
                capture_mode="aborted_no_pi_clock_sync",
                ouster_host=ouster_host,
                lidar_mode=lidar_mode,
                timestamp_mode=timestamp_mode,
                ptp_profile=ptp_profile,
                wait_for_gps_fix=require_gps_fix,
                wait_for_pi_clock_sync=require_pi_clock_sync,
                wait_for_ouster_ptp_lock=require_ouster_ptp_lock,
                gps_input_mode=gps_input_mode,
                gps_port=gps_port,
                gps_baud=gps_baud,
                gpsd_host=gpsd_host,
                gpsd_port=gpsd_port,
                bridge_fix_path=bridge_fix_path,
                gps_csv_path=gps_csv_path,
                gps_logger=gps_logger,
                readiness=readiness,
                chunks=chunks,
            )
            with open(manifest_path, "w", encoding="utf-8") as f_out:
                json.dump(manifest, f_out, indent=2)
            log(f"GPS CSV:     {gps_csv_path}")
            log(f"Manifest:    {manifest_path}")
            return
    else:
        readiness["pi_clock_sync"]["ready"] = True

    if require_ouster_ptp_lock:
        log("Waiting for Ouster PTP lock before LiDAR capture...")
        locked, ouster_status = wait_for_ouster_ptp_lock(
            host=ouster_host,
            stop_event=stop_event,
            timeout_sec=readiness_timeout_sec,
            poll_sec=READINESS_POLL_SEC,
        )
        readiness["ouster_ptp_lock"]["ready"] = locked
        readiness["ouster_ptp_lock"]["status"] = ouster_status
        if not locked:
            log("Capture aborted before start: Ouster PTP lock was not acquired.")
            stop_event.set()
            gps_logger.join(timeout=3)
            manifest = build_abort_manifest(
                session_id=session_id,
                capture_mode="aborted_no_ouster_ptp_lock",
                ouster_host=ouster_host,
                lidar_mode=lidar_mode,
                timestamp_mode=timestamp_mode,
                ptp_profile=ptp_profile,
                wait_for_gps_fix=require_gps_fix,
                wait_for_pi_clock_sync=require_pi_clock_sync,
                wait_for_ouster_ptp_lock=require_ouster_ptp_lock,
                gps_input_mode=gps_input_mode,
                gps_port=gps_port,
                gps_baud=gps_baud,
                gpsd_host=gpsd_host,
                gpsd_port=gpsd_port,
                bridge_fix_path=bridge_fix_path,
                gps_csv_path=gps_csv_path,
                gps_logger=gps_logger,
                readiness=readiness,
                chunks=chunks,
            )
            with open(manifest_path, "w", encoding="utf-8") as f_out:
                json.dump(manifest, f_out, indent=2)
            log(f"GPS CSV:     {gps_csv_path}")
            log(f"Manifest:    {manifest_path}")
            return
    else:
        readiness["ouster_ptp_lock"]["ready"] = True

    run_start_iso = dt.datetime.now(dt.timezone.utc).isoformat()
    run_start_ns = time.time_ns()
    chunk_index = 0

    if CONTINUOUS_CHUNKS:
        log("Capture mode: continuous chunks. Press Ctrl+C to stop.")
    else:
        log("Capture mode: single chunk.")

    while not stop_event.is_set():
        chunk_stem = f"raw_lidar_{session_id}_chunk{chunk_index:04d}"
        ext = ".pcap" if LIDAR_OUTPUT_MODE == "pcap_raw" else ".csv"
        requested_output_path = os.path.join(OUTPUT_DIR, chunk_stem + ext)

        existing_files = set(find_chunk_files(requested_output_path))
        chunk_start_iso = dt.datetime.now(dt.timezone.utc).isoformat()
        chunk_start_ns = time.time_ns()

        log(f"Starting chunk {chunk_index}: {requested_output_path}")
        rc = run_ouster_capture(
            host=ouster_host,
            seconds=CAPTURE_DURATION_SEC,
            requested_output_path=requested_output_path,
            stop_event=stop_event,
            output_mode=LIDAR_OUTPUT_MODE,
        )

        chunk_end_ns = time.time_ns()
        chunk_end_iso = dt.datetime.now(dt.timezone.utc).isoformat()
        generated_files = sorted(set(find_chunk_files(requested_output_path)) - existing_files)

        chunk_record = {
            "chunk_index": chunk_index,
            "capture_start_iso_utc": chunk_start_iso,
            "capture_end_iso_utc": chunk_end_iso,
            "capture_start_ns": chunk_start_ns,
            "capture_end_ns": chunk_end_ns,
            "duration_sec": CAPTURE_DURATION_SEC,
            "ouster_exit_code": rc,
            "lidar_output_mode": LIDAR_OUTPUT_MODE,
            "lidar_mode": lidar_mode,
            "ouster_timestamp_mode": timestamp_mode,
            "ouster_ptp_profile": ptp_profile,
            "requested_output_path": requested_output_path,
            "produced_files": generated_files,
        }

        csv_candidates = [path for path in generated_files if path.lower().endswith(".csv")]
        if csv_candidates:
            chunk_record["lidar_csv_path"] = csv_candidates[0]
        pcap_candidates = [path for path in generated_files if path.lower().endswith(".pcap")]
        if pcap_candidates:
            chunk_record["lidar_pcap_path"] = pcap_candidates[0]

        chunks.append(chunk_record)

        if stop_event.is_set():
            break

        if rc != 0:
            if LIDAR_OUTPUT_MODE == "pcap_raw" and pcap_candidates:
                log(f"Chunk ended (exit code {rc}) after timed stop. Continuing...")
            else:
                log(f"Stopping capture loop because ouster-cli returned non-zero exit code: {rc}")
                break

        if not CONTINUOUS_CHUNKS:
            break

        chunk_index += 1

    run_end_ns = time.time_ns()
    run_end_iso = dt.datetime.now(dt.timezone.utc).isoformat()

    stop_event.set()
    gps_logger.join(timeout=3)

    manifest = {
        "session_id": session_id,
        "timing_architecture": TIMING_ARCHITECTURE,
        "capture_mode": "continuous_chunks" if CONTINUOUS_CHUNKS else "single_chunk",
        "run_start_iso_utc": run_start_iso,
        "run_end_iso_utc": run_end_iso,
        "run_start_ns": run_start_ns,
        "run_end_ns": run_end_ns,
        "ouster_host": ouster_host,
        "lidar_output_mode": LIDAR_OUTPUT_MODE,
        "lidar_mode": lidar_mode,
        "ouster_timestamp_mode": timestamp_mode,
        "ouster_ptp_profile": ptp_profile,
        "chunk_duration_sec": CAPTURE_DURATION_SEC,
        "continuous_chunks": CONTINUOUS_CHUNKS,
        "wait_for_gps_fix_before_capture": require_gps_fix,
        "wait_for_pi_clock_sync_before_capture": require_pi_clock_sync,
        "wait_for_ouster_ptp_lock_before_capture": require_ouster_ptp_lock,
        "gps_input_mode": gps_input_mode,
        "gps_port": gps_port,
        "gps_baud": gps_baud,
        "gpsd_host": gpsd_host,
        "gpsd_port": gpsd_port,
        "bridge_fix_path": bridge_fix_path,
        "utm_epsg": UTM_EPSG,
        "gps_csv_path": gps_csv_path,
        "gps_rows_written": gps_logger.rows_written,
        "gps_first_fix_time_ns": gps_logger.first_fix_time_ns,
        "gps_latest_fix": gps_logger.latest_fix,
        "readiness": readiness,
        "chunks_captured": len(chunks),
        "chunks": chunks,
    }

    with open(manifest_path, "w", encoding="utf-8") as f_out:
        json.dump(manifest, f_out, indent=2)

    log("Capture complete.")
    log(f"LiDAR chunks: {len(chunks)}")
    log(f"GPS CSV:     {gps_csv_path}")
    log(f"Manifest:    {manifest_path}")


if __name__ == "__main__":
    main()
