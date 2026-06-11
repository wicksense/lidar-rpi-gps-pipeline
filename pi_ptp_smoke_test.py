#!/usr/bin/env python3
"""
pi_ptp_smoke_test.py

Purpose
-------
Quick local smoke test for Pi -> Ouster PTP behavior without requiring a live GPS fix.

This is intentionally narrower than the full field-capture path:
- It checks whether the Pi system clock appears synchronized according to chrony.
- It checks whether the Ouster is reachable over HTTP.
- It samples the Ouster PTP state a few times and reports whether the sensor
  reaches `TIME_FROM_PTP_1588` + `port_state=SLAVE`.

Important:
- Passing this test means the Ouster appears to be following the Pi over PTP.
- It does NOT prove that the Pi clock is currently GPS-disciplined.
- It is useful for indoor / bench testing where you want to isolate PTP from GPS.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Optional

from pi_capture_ptp import (
    CHRONY_MAX_CORRECTION_SEC,
    CHRONY_WAITSYNC_INTERVAL_SEC,
    CHRONY_WAITSYNC_TRIES,
    collect_chrony_snapshot,
    get_ouster_time_status,
    ouster_ptp_locked,
    run_command_capture,
    summarize_ouster_ptp_status,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Smoke-test whether the Ouster is following the Pi over PTP."
    )
    parser.add_argument(
        "--ouster-host",
        default="169.254.237.207",
        help="Ouster hostname/IP to query (default: 169.254.237.207).",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of Ouster PTP samples to collect (default: 5).",
    )
    parser.add_argument(
        "--interval-sec",
        type=float,
        default=1.0,
        help="Seconds between Ouster samples (default: 1.0).",
    )
    parser.add_argument(
        "--chrony-max-correction",
        type=float,
        default=CHRONY_MAX_CORRECTION_SEC,
        help="Correction threshold passed to chronyc waitsync (default: 0.01).",
    )
    parser.add_argument(
        "--show-json",
        action="store_true",
        help="Print the full Ouster JSON payload for each sample.",
    )
    return parser.parse_args()


def chrony_ready(max_correction_sec: float) -> tuple[bool, dict[str, Any]]:
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
    snapshot = {
        "waitsync": waitsync,
        **collect_chrony_snapshot(),
    }
    return waitsync["returncode"] == 0, snapshot


def extract_sensor_timestamp_seconds(status: dict[str, Any]) -> Optional[float]:
    time_block = status.get("time")
    if not isinstance(time_block, dict):
        return None
    sensor = time_block.get("sensor")
    if not isinstance(sensor, dict):
        return None
    timestamp = sensor.get("timestamp")
    if not isinstance(timestamp, dict):
        return None
    value = timestamp.get("time")
    if isinstance(value, (int, float)):
        return float(value)
    return None


def sensor_minus_pi_ms(status: dict[str, Any], pi_now_sec: float) -> Optional[float]:
    sensor_ts = extract_sensor_timestamp_seconds(status)
    if sensor_ts is None:
        return None
    return (sensor_ts - pi_now_sec) * 1000.0


def print_chrony_status(max_correction_sec: float) -> bool:
    ready, snapshot = chrony_ready(max_correction_sec)
    tracking = snapshot.get("tracking", {})
    waitsync = snapshot.get("waitsync", {})

    print("== Pi Clock ==")
    print(f"chrony ready: {ready}")
    if waitsync.get("stdout"):
        print(f"waitsync: {waitsync['stdout']}")
    elif waitsync.get("stderr"):
        print(f"waitsync: {waitsync['stderr']}")

    tracking_stdout = tracking.get("stdout", "")
    if tracking_stdout:
        print("tracking:")
        print(tracking_stdout)
    print()
    return ready


def print_ouster_samples(host: str, samples: int, interval_sec: float, show_json: bool) -> bool:
    print("== Ouster PTP ==")
    locked_all = True

    for index in range(samples):
        pi_now = time.time()
        try:
            status = get_ouster_time_status(host)
        except Exception as exc:
            print(f"[sample {index + 1}/{samples}] error: {exc}")
            locked_all = False
        else:
            locked = ouster_ptp_locked(status)
            delta_ms = sensor_minus_pi_ms(status, pi_now)
            delta_text = "n/a" if delta_ms is None else f"{delta_ms:+.3f} ms"
            print(
                f"[sample {index + 1}/{samples}] "
                f"{summarize_ouster_ptp_status(status)} | "
                f"sensor_minus_pi={delta_text} | locked={locked}"
            )
            if show_json:
                print(json.dumps(status, indent=2))
            locked_all = locked_all and locked

        if index + 1 < samples:
            time.sleep(interval_sec)

    print()
    return locked_all


def main() -> int:
    args = parse_args()

    print("PTP smoke test")
    print("This checks Pi clock sync + Ouster PTP lock without requiring a GPS fix.")
    print("A passing result proves Pi -> Ouster PTP behavior, not GPS discipline.")
    print()

    chrony_ok = print_chrony_status(args.chrony_max_correction)
    ouster_ok = print_ouster_samples(
        host=args.ouster_host,
        samples=args.samples,
        interval_sec=args.interval_sec,
        show_json=args.show_json,
    )

    print("== Result ==")
    print(f"Pi clock ready: {chrony_ok}")
    print(f"Ouster PTP locked across all samples: {ouster_ok}")

    if chrony_ok and ouster_ok:
        print("PASS: Ouster appears to be following the Pi over PTP.")
        return 0

    if not chrony_ok and ouster_ok:
        print("PARTIAL: Ouster reports PTP lock, but chrony does not consider the Pi clock ready.")
    elif chrony_ok and not ouster_ok:
        print("PARTIAL: Pi clock is ready, but the Ouster did not report stable PTP slave lock.")
    else:
        print("FAIL: Neither Pi clock readiness nor stable Ouster PTP lock was confirmed.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
