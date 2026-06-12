#!/usr/bin/env python3
"""
pi_ptp_pcap_verify.py

Purpose
-------
Verify that a *newly recorded* raw pcap is actually using the Pi's current
PTP-served time domain.

This is meant to be usable indoors without a fresh GPS fix. It does NOT prove
GPS discipline by itself. Instead it checks a narrower but crucial question:

    "If I record a short pcap right now, do the LiDAR timestamps fall into the
    same current time domain as the Pi clock and live Ouster sensor time?"

Passing this script means:
- the Pi system clock and NIC PHC are aligned closely enough
- the Ouster reports PTP lock and sensor time close to the Pi clock
- a short freshly captured pcap contains LiDAR timestamps that fall inside that
  same live time window
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
from typing import Any, Optional

import numpy as np
from ouster.sdk import open_source

from pi_capture_ptp import (
    OUSTER_HOST,
    PTP_IFACE,
    collect_phc_alignment,
    evaluate_ouster_sensor_time_alignment,
    extract_sensor_timestamp_seconds,
    get_ouster_time_status,
    ouster_ptp_locked,
    run_ouster_capture,
    summarize_ouster_ptp_status,
)

PCAP_WINDOW_SLOP_SEC = 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify that a fresh raw pcap is using the Pi's current PTP time domain."
    )
    parser.add_argument(
        "--ouster-host",
        default=OUSTER_HOST,
        help=f"Ouster hostname/IP to query and capture from (default: {OUSTER_HOST}).",
    )
    parser.add_argument(
        "--iface",
        default=PTP_IFACE,
        help=f"PTP network interface for PHC checks (default: {PTP_IFACE}).",
    )
    parser.add_argument(
        "--seconds",
        type=int,
        default=5,
        help="Length of the short verification capture (default: 5).",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Optional directory where the verification pcap and metadata should be written.",
    )
    parser.add_argument(
        "--cleanup",
        action="store_true",
        help="Delete the temporary verification files after the check finishes.",
    )
    parser.add_argument(
        "--show-json",
        action="store_true",
        help="Print the full Ouster timing JSON before and after capture.",
    )
    return parser.parse_args()


def extract_first_scan(item: Any) -> Any:
    if item is None:
        return None

    if (
        hasattr(item, "fields")
        and hasattr(item, "field")
        and hasattr(item, "h")
        and hasattr(item, "w")
        and hasattr(item, "timestamp")
    ):
        return item

    if isinstance(item, (list, tuple)):
        for candidate in item:
            scan = extract_first_scan(candidate)
            if scan is not None:
                return scan

    if hasattr(item, "__iter__"):
        try:
            for candidate in item:
                scan = extract_first_scan(candidate)
                if scan is not None:
                    return scan
        except Exception:
            return None
    return None


def summarize_pcap_timestamps(pcap_path: str) -> dict[str, Any]:
    source = open_source(pcap_path)
    first_ts: Optional[int] = None
    last_ts: Optional[int] = None
    scan_count = 0
    valid_scan_count = 0

    for item in source:
        scan = extract_first_scan(item)
        if scan is None:
            continue
        scan_count += 1
        timestamps = np.asarray(scan.timestamp, dtype=np.int64)
        valid = timestamps[timestamps > 0]
        if valid.size == 0:
            continue
        valid_scan_count += 1
        if first_ts is None:
            first_ts = int(valid[0])
        last_ts = int(valid[-1])

    return {
        "first_lidar_timestamp_ns": first_ts,
        "last_lidar_timestamp_ns": last_ts,
        "scan_count": scan_count,
        "valid_scan_count": valid_scan_count,
        "pcap_path": pcap_path,
    }


def evaluate_pcap_time_window(
    *,
    before_sensor_sec: float,
    after_sensor_sec: float,
    first_lidar_ns: Optional[int],
    last_lidar_ns: Optional[int],
    slop_sec: float = PCAP_WINDOW_SLOP_SEC,
) -> dict[str, Any]:
    if first_lidar_ns is None or last_lidar_ns is None:
        return {
            "ready": False,
            "summary": "Could not find valid LiDAR timestamps in the verification pcap.",
            "first_delta_sec": None,
            "last_delta_sec": None,
        }

    first_sec = first_lidar_ns / 1e9
    last_sec = last_lidar_ns / 1e9
    min_allowed = before_sensor_sec - slop_sec
    max_allowed = after_sensor_sec + slop_sec
    ready = (min_allowed <= first_sec <= max_allowed) and (min_allowed <= last_sec <= max_allowed)
    first_delta = first_sec - before_sensor_sec
    last_delta = last_sec - after_sensor_sec
    summary = (
        "Fresh pcap timestamps fall inside the live Ouster/PTP time window."
        if ready
        else "Fresh pcap timestamps do not match the live Ouster/PTP time window."
    )
    return {
        "ready": ready,
        "summary": summary,
        "first_delta_sec": first_delta,
        "last_delta_sec": last_delta,
        "window_start_sec": min_allowed,
        "window_end_sec": max_allowed,
    }


def print_section(title: str) -> None:
    print(f"== {title} ==")


def main() -> int:
    args = parse_args()

    output_dir = Path(args.output_dir) if args.output_dir else Path(
        tempfile.mkdtemp(prefix="ptp_pcap_verify_")
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    requested_output_path = str(output_dir / "verify_capture.pcap")

    print("PTP pcap verification")
    print("This checks whether a *new* raw pcap is using the Pi's current PTP time domain.")
    print("It can be run indoors. Passing does not prove fresh GPS lock; it proves current PTP time usage.")
    print()

    print_section("Pi PHC")
    phc_alignment = collect_phc_alignment(args.iface)
    print(phc_alignment.get("summary", "-"))
    if phc_alignment.get("delta_seconds") is not None:
        print(f"PHC minus CLOCK_REALTIME delta: {phc_alignment['delta_seconds']:+.6f}s")
    print()

    print_section("Ouster Before Capture")
    try:
        before_status = get_ouster_time_status(args.ouster_host)
    except Exception as exc:
        print(f"error: {exc}")
        return 1
    print(summarize_ouster_ptp_status(before_status))
    before_locked = ouster_ptp_locked(before_status)
    before_alignment = evaluate_ouster_sensor_time_alignment(before_status)
    before_sensor_sec = extract_sensor_timestamp_seconds(before_status)
    print(before_alignment["summary"])
    if args.show_json:
        print(json.dumps(before_status, indent=2))
    print()

    print_section("Short Capture")
    print(f"Writing verification files to: {output_dir}")
    stop_event = threading.Event()
    capture_start_pi_ns = time.time_ns()
    rc = run_ouster_capture(
        host=args.ouster_host,
        seconds=args.seconds,
        requested_output_path=requested_output_path,
        stop_event=stop_event,
        output_mode="pcap_raw",
    )
    capture_end_pi_ns = time.time_ns()
    print(f"capture_start_pi_ns={capture_start_pi_ns}")
    print(f"capture_end_pi_ns={capture_end_pi_ns}")
    print(f"ouster-cli return code={rc}")
    print()

    print_section("Ouster After Capture")
    try:
        after_status = get_ouster_time_status(args.ouster_host)
    except Exception as exc:
        print(f"error: {exc}")
        return 1
    print(summarize_ouster_ptp_status(after_status))
    after_locked = ouster_ptp_locked(after_status)
    after_alignment = evaluate_ouster_sensor_time_alignment(after_status)
    after_sensor_sec = extract_sensor_timestamp_seconds(after_status)
    print(after_alignment["summary"])
    if args.show_json:
        print(json.dumps(after_status, indent=2))
    print()

    print_section("Pcap Timestamps")
    pcap_summary = summarize_pcap_timestamps(requested_output_path)
    print(json.dumps(pcap_summary, indent=2))
    print()

    pcap_window = evaluate_pcap_time_window(
        before_sensor_sec=before_sensor_sec if before_sensor_sec is not None else 0.0,
        after_sensor_sec=after_sensor_sec if after_sensor_sec is not None else 0.0,
        first_lidar_ns=pcap_summary["first_lidar_timestamp_ns"],
        last_lidar_ns=pcap_summary["last_lidar_timestamp_ns"],
    )
    print_section("Result")
    print(f"PHC aligned: {phc_alignment.get('ready')}")
    print(f"Ouster locked before capture: {before_locked}")
    print(f"Ouster sensor time aligned before capture: {before_alignment.get('ready')}")
    print(f"Ouster locked after capture: {after_locked}")
    print(f"Ouster sensor time aligned after capture: {after_alignment.get('ready')}")
    print(f"Pcap timestamps in live PTP window: {pcap_window.get('ready')}")
    print(pcap_window.get("summary", "-"))
    if pcap_window.get("ready") and not (before_alignment.get("ready") and after_alignment.get("ready")):
        print(
            "Important: the pcap is faithfully recording the Ouster's current live timestamps, "
            "but the Ouster's live PTP time is still not aligned to the Pi clock."
        )

    ok = bool(
        phc_alignment.get("ready")
        and before_locked
        and after_locked
        and before_alignment.get("ready")
        and after_alignment.get("ready")
        and pcap_window.get("ready")
    )

    if ok:
        print("PASS: A fresh pcap appears to be using the Pi's current PTP-served time.")
    else:
        print("FAIL: The fresh pcap does not yet look safely aligned to the Pi's current PTP time.")

    if args.cleanup:
        shutil.rmtree(output_dir, ignore_errors=True)

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
