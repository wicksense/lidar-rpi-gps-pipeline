#!/usr/bin/env python3
"""
pi_capture_raw.py

Purpose
-------
Run this on the Raspberry Pi during data collection.

This script does only the field-capture work:
1. Read GPS fixes from the u-blox receiver over serial.
2. Save those GPS fixes to a CSV log file.
3. Run one Ouster capture and save the raw LiDAR CSV.
4. Save a small "manifest" JSON file that tells you which files belong together.

Important: this script does NOT georeference point clouds on the Pi.
That heavier processing is intended for an offline machine later.
"""

import argparse
import csv
import datetime as dt
import glob
import json
import os
import signal
import subprocess
import threading
import time
from typing import Optional

import pynmea2
import serial
from pyproj import Transformer

# =======================
# Field Capture Settings
# =======================
# These values are intentionally hardcoded so non-technical users can run:
#   python3 pi_capture_raw.py
# If hardware/network settings ever change, edit this section only.
# Ouster sensor address on the local Ethernet link.
OUSTER_HOST = "169.254.237.207"
# Capture format:
# - "pcap_raw": raw UDP packets via `save_raw` (recommended for Pi reliability).
# - "csv": direct point CSV via `save` (heavier; more likely to drop scans on Pi).
LIDAR_OUTPUT_MODE = "pcap_raw"
# Each LiDAR file will contain this many seconds.
# Why 30s:
# - Shorter chunks are safer in the field (less data loss if power/network drops).
# - Longer chunks reduce file count but create bigger files and higher restart cost.
# You can increase this (for example 60 or 120) if your setup is stable.
CAPTURE_DURATION_SEC = 30
# True = keep recording chunk after chunk until Ctrl+C.
# False = record one chunk and exit.
CONTINUOUS_CHUNKS = True
# True = require at least one valid GPS fix before starting LiDAR capture.
# False = start LiDAR capture immediately (useful for indoor/LiDAR-only tests).
# For mapping/georeferencing sessions, set this to True.
WAIT_FOR_GPS_FIX_BEFORE_CAPTURE = True
# Serial device name for the u-blox GPS receiver.
GPS_PORT = "/dev/gps_ublox"
# Serial baud rate for the GPS device.
GPS_BAUD = 9600
# Folder where raw LiDAR chunks, GPS log, and manifest are written.
OUTPUT_DIR = "/home/urp-pi5/capture_output"
# Projected CRS EPSG code used to convert lat/lon to easting/northing meters.
# Example: 32614 = WGS84 / UTM Zone 14N.
UTM_EPSG = 32614


def log(message: str) -> None:
    """Simple timestamped logger so terminal output is easy to follow."""
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def ensure_dir(path: str) -> None:
    """Create a directory if it does not exist."""
    os.makedirs(path, exist_ok=True)


def parse_cli_args() -> argparse.Namespace:
    """
    Optional runtime overrides.

    Most settings stay hardcoded above; this only lets operators override
    GPS-wait behavior without editing code.
    """
    parser = argparse.ArgumentParser(description="Capture raw Ouster + GPS on Raspberry Pi.")
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
        help="Override and start LiDAR capture immediately.",
    )
    parser.set_defaults(wait_for_gps_fix=None)
    return parser.parse_args()


def wait_for_first_gps_fix(
    gps_logger: "GpsLogger",
    stop_event: threading.Event,
) -> bool:
    """
    Wait for first valid GPS fix written by GpsLogger.

    Returns:
    - True if a fix is available
    - False on shutdown request
    """
    while not stop_event.is_set():
        if gps_logger.first_fix_time_ns is not None:
            log("GPS fix acquired. Starting LiDAR capture.")
            return True
        time.sleep(0.5)
    return False


class GpsLogger(threading.Thread):
    """
    Background thread that listens to the GPS serial stream and writes one row
    per valid GGA fix.

    Why a thread?
    - GPS messages arrive continuously.
    - We want to keep collecting GPS while ouster-cli runs.
    """

    def __init__(
        self,
        serial_port: str,
        baud: int,
        out_csv_path: str,
        stop_event: threading.Event,
        epsg_out: int = 32614,
        status_log_every_sec: float = 1.0,
    ):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.baud = baud
        self.out_csv_path = out_csv_path
        self.stop_event = stop_event
        self.transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg_out}", always_xy=True)
        self.status_log_every_sec = status_log_every_sec
        self.last_status_log = 0.0
        self.rows_written = 0
        self.first_fix_time_ns: Optional[int] = None
        self.last_rmc_date: Optional[dt.date] = None

    @staticmethod
    def _gps_epoch_ns_from_date_and_time(
        gps_date: Optional[dt.date], gps_time: Optional[dt.time]
    ) -> Optional[int]:
        """
        Convert GPS UTC date+time into Unix epoch nanoseconds.

        We need both date and time:
        - GGA usually has time-of-day only
        - RMC usually has date + time

        If either piece is missing, return None.
        """
        if gps_date is None or gps_time is None:
            return None

        # Build an aware UTC datetime from date + time.
        combined = dt.datetime.combine(gps_date, gps_time).replace(tzinfo=dt.timezone.utc)
        return int(combined.timestamp() * 1e9)

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

        # Open output CSV once and append rows as fixes arrive.
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
                            # Ignore malformed lines and continue streaming.
                            continue

                        # Capture current UTC date from RMC so GGA time can become full epoch time.
                        if isinstance(msg, pynmea2.RMC):
                            if getattr(msg, "datestamp", None):
                                self.last_rmc_date = msg.datestamp
                            continue

                        # We only need GGA for position + fix quality.
                        if not isinstance(msg, pynmea2.GGA):
                            continue

                        gps_quality = int(getattr(msg, "gps_qual", 0) or 0)
                        if gps_quality == 0:
                            # 0 means invalid fix.
                            continue

                        if msg.latitude is None or msg.longitude is None:
                            continue

                        try:
                            altitude_m = float(msg.altitude) if msg.altitude not in (None, "") else float("nan")
                        except (TypeError, ValueError):
                            altitude_m = float("nan")

                        # Convert lat/lon to projected meters.
                        easting, northing = self.transformer.transform(float(msg.longitude), float(msg.latitude))
                        gps_epoch_ns = self._gps_epoch_ns_from_date_and_time(
                            self.last_rmc_date, getattr(msg, "timestamp", None)
                        )

                        writer.writerow(
                            [
                                pi_time_ns,
                                pi_time_iso,
                                gps_epoch_ns if gps_epoch_ns is not None else "",
                                str(getattr(msg, "timestamp", "")),
                                gps_quality,
                                getattr(msg, "num_sats", ""),
                                getattr(msg, "horizontal_dil", ""),
                                float(msg.latitude),
                                float(msg.longitude),
                                altitude_m,
                                easting,
                                northing,
                            ]
                        )

                        # Flush frequently so data is not lost on sudden power loss.
                        f_out.flush()

                        self.rows_written += 1
                        if self.first_fix_time_ns is None:
                            self.first_fix_time_ns = pi_time_ns

                        # Log status at a controlled rate to avoid log spam.
                        now = time.time()
                        if now - self.last_status_log >= self.status_log_every_sec:
                            self.last_status_log = now
                            log(
                                "GPS fix logged: "
                                f"q={gps_quality}, lat={float(msg.latitude):.6f}, lon={float(msg.longitude):.6f}, "
                                f"e={easting:.2f}, n={northing:.2f}"
                            )

            except serial.SerialException as e:
                log(f"GPS serial error: {e}")
            except Exception as e:
                log(f"Unexpected GPS logger error: {e}")

        log(f"GPS logger stopped. Rows written: {self.rows_written}")


def find_chunk_files(requested_output_path: str) -> list:
    """
    Return files created for this chunk prefix.
    Example: if requested path is "...chunk0000.pcap", this finds:
      - ...chunk0000.pcap
      - ...chunk0000.json (metadata, if generated)
      - ...chunk0000_*.csv (if csv mode)
    """
    stem = os.path.splitext(requested_output_path)[0]
    return sorted([p for p in glob.glob(stem + "*") if os.path.isfile(p)])


def run_ouster_capture(
    host: str,
    seconds: int,
    requested_output_path: str,
    stop_event: threading.Event,
    output_mode: str,
) -> int:
    """
    Start one ouster-cli capture.

    Returns the process exit code.
    """
    if output_mode == "csv":
        # CSV mode: ouster-cli can be bounded by a time slice directly.
        cmd = [
            "ouster-cli",
            "source",
            host,
            "slice",
            f"0s:{seconds}s",
            "save",
            requested_output_path,
        ]
    elif output_mode == "pcap_raw":
        # Raw mode: save_raw runs until interrupted; we stop it after `seconds`.
        cmd = [
            "ouster-cli",
            "source",
            host,
            "save_raw",
            requested_output_path,
        ]
    else:
        raise ValueError(f"Unsupported LIDAR_OUTPUT_MODE: {output_mode}")

    log("Starting ouster capture:")
    log("  " + " ".join(cmd))

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def stream(pipe, label: str) -> None:
        for line in iter(pipe.readline, ""):
            if stop_event.is_set():
                break
            # Keep logs concise: only print non-empty lines.
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
                # Graceful stop first so file writers can flush.
                proc.send_signal(signal.SIGINT)
                sent_sigint = True
            time.sleep(0.2)
            continue

        if output_mode == "pcap_raw" and time.time() >= deadline and not sent_sigint:
            # End this chunk on time boundary in raw mode.
            proc.send_signal(signal.SIGINT)
            sent_sigint = True
            continue

        time.sleep(0.2)

    # In rare cases, give process one more chance to exit cleanly.
    if proc.poll() is None:
        proc.terminate()

    return_code = proc.wait()
    log(f"ouster-cli exited with code {return_code}")
    return return_code


def main() -> None:
    cli = parse_cli_args()
    wait_for_gps_fix = (
        WAIT_FOR_GPS_FIX_BEFORE_CAPTURE
        if cli.wait_for_gps_fix is None
        else cli.wait_for_gps_fix
    )

    ensure_dir(OUTPUT_DIR)

    # Use one session ID so related files are easy to match later.
    session_id = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    gps_csv_path = os.path.join(OUTPUT_DIR, f"raw_gps_{session_id}.csv")
    manifest_path = os.path.join(OUTPUT_DIR, f"capture_manifest_{session_id}.json")

    stop_event = threading.Event()

    def handle_signal(_sig, _frame) -> None:
        log("Shutdown requested.")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Start GPS logging before LiDAR capture so time ranges overlap.
    gps_logger = GpsLogger(
        serial_port=GPS_PORT,
        baud=GPS_BAUD,
        out_csv_path=gps_csv_path,
        stop_event=stop_event,
        epsg_out=UTM_EPSG,
    )
    gps_logger.start()

    # Give GPS logger a short warm-up period.
    time.sleep(2)

    if wait_for_gps_fix:
        log("Waiting for first GPS fix before LiDAR capture...")
        has_fix = wait_for_first_gps_fix(gps_logger=gps_logger, stop_event=stop_event)
        if not has_fix:
            log("Capture aborted before start.")
            stop_event.set()
            gps_logger.join(timeout=3)

            run_end_ns = time.time_ns()
            run_end_iso = dt.datetime.now(dt.timezone.utc).isoformat()
            manifest = {
                "session_id": session_id,
                "capture_mode": "aborted_no_gps_fix",
                "run_start_iso_utc": run_end_iso,
                "run_end_iso_utc": run_end_iso,
                "run_start_ns": run_end_ns,
                "run_end_ns": run_end_ns,
                "ouster_host": OUSTER_HOST,
                "lidar_output_mode": LIDAR_OUTPUT_MODE,
                "chunk_duration_sec": CAPTURE_DURATION_SEC,
                "continuous_chunks": CONTINUOUS_CHUNKS,
                "wait_for_gps_fix_before_capture": wait_for_gps_fix,
                "gps_port": GPS_PORT,
                "gps_baud": GPS_BAUD,
                "utm_epsg": UTM_EPSG,
                "gps_csv_path": gps_csv_path,
                "gps_rows_written": gps_logger.rows_written,
                "gps_first_fix_time_ns": gps_logger.first_fix_time_ns,
                "chunks_captured": 0,
                "chunks": [],
            }
            with open(manifest_path, "w", encoding="utf-8") as f:
                json.dump(manifest, f, indent=2)
            log(f"GPS CSV:     {gps_csv_path}")
            log(f"Manifest:    {manifest_path}")
            return

    run_start_iso = dt.datetime.now(dt.timezone.utc).isoformat()
    run_start_ns = time.time_ns()

    chunks = []
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
            host=OUSTER_HOST,
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
            "requested_output_path": requested_output_path,
            "produced_files": generated_files,
        }

        # Backward compatibility helpers:
        # old readers looked for lidar_csv_path.
        csv_candidates = [p for p in generated_files if p.lower().endswith(".csv")]
        if csv_candidates:
            chunk_record["lidar_csv_path"] = csv_candidates[0]
        pcap_candidates = [p for p in generated_files if p.lower().endswith(".pcap")]
        if pcap_candidates:
            chunk_record["lidar_pcap_path"] = pcap_candidates[0]

        chunks.append(chunk_record)

        # If user asked to stop, end loop.
        if stop_event.is_set():
            break

        # If ouster-cli failed unexpectedly, stop instead of looping forever.
        if rc != 0:
            log(f"Stopping capture loop because ouster-cli returned non-zero exit code: {rc}")
            break

        if not CONTINUOUS_CHUNKS:
            break

        chunk_index += 1

    run_end_ns = time.time_ns()
    run_end_iso = dt.datetime.now(dt.timezone.utc).isoformat()

    # Stop GPS thread cleanly.
    stop_event.set()
    gps_logger.join(timeout=3)

    manifest = {
        "session_id": session_id,
        "capture_mode": "continuous_chunks" if CONTINUOUS_CHUNKS else "single_chunk",
        "run_start_iso_utc": run_start_iso,
        "run_end_iso_utc": run_end_iso,
        "run_start_ns": run_start_ns,
        "run_end_ns": run_end_ns,
        "ouster_host": OUSTER_HOST,
        "lidar_output_mode": LIDAR_OUTPUT_MODE,
        "chunk_duration_sec": CAPTURE_DURATION_SEC,
        "continuous_chunks": CONTINUOUS_CHUNKS,
        "wait_for_gps_fix_before_capture": wait_for_gps_fix,
        "gps_port": GPS_PORT,
        "gps_baud": GPS_BAUD,
        "utm_epsg": UTM_EPSG,
        "gps_csv_path": gps_csv_path,
        "gps_rows_written": gps_logger.rows_written,
        "gps_first_fix_time_ns": gps_logger.first_fix_time_ns,
        "chunks_captured": len(chunks),
        "chunks": chunks,
    }

    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    log("Capture complete.")
    log(f"LiDAR chunks: {len(chunks)}")
    log(f"GPS CSV:     {gps_csv_path}")
    log(f"Manifest:    {manifest_path}")


if __name__ == "__main__":
    main()
