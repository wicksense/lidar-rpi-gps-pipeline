#!/usr/bin/env python3
"""
pi_capture_web.py

Local web control panel for Raspberry Pi field capture.

This script serves a lightweight browser UI that can be opened from a phone
connected to the Pi's hotspot or local network. It launches the existing Pi
capture scripts as background subprocesses, streams their logs, and exposes
basic readiness/preflight information without requiring VNC.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import os
import re
import signal
import subprocess
import sys
import threading
import urllib.parse
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

try:
    import pi_capture_ptp as ptp_capture
except ModuleNotFoundError as exc:
    if exc.name not in {"pyproj", "pynmea2", "serial"}:
        raise

    class _FallbackPtpCapture:
        OUSTER_HOST = "169.254.237.207"
        OUSTER_TIMESTAMP_MODE = "TIME_FROM_PTP_1588"
        OUSTER_PTP_PROFILE = "default"
        SLAM_MIN_RANGE_M = 1.0
        SLAM_MAX_RANGE_M = 150.0
        CHRONY_MAX_CORRECTION_SEC = 0.01
        CHRONY_WAITSYNC_TRIES = 1
        CHRONY_WAITSYNC_INTERVAL_SEC = 1
        OUSTER_API_TIMEOUT_SEC = 5

        @staticmethod
        def resolve_range_defaults(min_range_m: Optional[float], max_range_m: Optional[float]) -> tuple[float, float]:
            resolved_min = _FallbackPtpCapture.SLAM_MIN_RANGE_M if min_range_m is None else float(min_range_m)
            resolved_max = _FallbackPtpCapture.SLAM_MAX_RANGE_M if max_range_m is None else float(max_range_m)
            if resolved_min < 0:
                raise ValueError("Minimum range must be >= 0.")
            if resolved_max <= 0:
                raise ValueError("Maximum range must be > 0.")
            if resolved_max <= resolved_min:
                raise ValueError("Maximum range must be greater than minimum range.")
            return resolved_min, resolved_max

        @staticmethod
        def run_command_capture(cmd: list[str]) -> dict[str, Any]:
            result = subprocess.run(cmd, capture_output=True, text=True)
            return {
                "cmd": cmd,
                "returncode": result.returncode,
                "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip(),
            }

        @staticmethod
        def _ouster_api_request(method: str, host: str, path: str) -> Any:
            request = urllib.request.Request(f"http://{host}{path}", method=method)
            with urllib.request.urlopen(request, timeout=_FallbackPtpCapture.OUSTER_API_TIMEOUT_SEC) as response:
                body = response.read().decode("utf-8").strip()
            if not body:
                return None
            try:
                return json.loads(body)
            except json.JSONDecodeError:
                return body

        @staticmethod
        def get_ouster_time_status(host: str) -> dict[str, Any]:
            return {
                "time": _FallbackPtpCapture._ouster_api_request("GET", host, "/api/v1/time"),
                "ptp": _FallbackPtpCapture._ouster_api_request("GET", host, "/api/v1/time/ptp"),
                "timestamp_mode": _FallbackPtpCapture._ouster_api_request("GET", host, "/api/v1/sensor/config/timestamp_mode"),
                "ptp_profile": _FallbackPtpCapture._ouster_api_request("GET", host, "/api/v1/time/ptp/profile"),
            }

        @staticmethod
        def ouster_ptp_locked(status: dict[str, Any]) -> bool:
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

        @staticmethod
        def summarize_ouster_ptp_status(status: dict[str, Any]) -> str:
            timestamp_mode = status.get("timestamp_mode")
            ptp = status.get("ptp") if isinstance(status.get("ptp"), dict) else {}
            port_state = ""
            gm_present = ""
            if isinstance(ptp, dict):
                port_state = str(ptp.get("port_data_set", {}).get("port_state", "") or "")
                gm_present = str(ptp.get("time_status_np", {}).get("gm_present", "") or "")
            return f"timestamp_mode={timestamp_mode}, port_state={port_state}, gm_present={gm_present}"

    ptp_capture = _FallbackPtpCapture()


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
LOG_HISTORY_LIMIT = 500

DEFAULT_UI_CONFIG = {
    "capture_mode": "ptp",
    "lidar_mode": "",
    "min_range_m": ptp_capture.SLAM_MIN_RANGE_M,
    "max_range_m": ptp_capture.SLAM_MAX_RANGE_M,
    "wait_for_gps_fix": True,
    "wait_for_pi_clock_sync": True,
    "wait_for_ouster_ptp_lock": True,
    "gps_input_mode": "bridge",
    "timestamp_mode": ptp_capture.OUSTER_TIMESTAMP_MODE or "",
    "ptp_profile": ptp_capture.OUSTER_PTP_PROFILE or "",
    "ouster_host": ptp_capture.OUSTER_HOST,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve a phone-friendly web UI for Pi capture control.")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Bind address (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Port (default: {DEFAULT_PORT})")
    return parser.parse_args()


def optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def bool_field(payload: dict[str, Any], key: str, default: bool) -> bool:
    value = payload.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def optional_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    return float(value)


def build_capture_command(payload: dict[str, Any]) -> list[str]:
    capture_mode = optional_text(payload.get("capture_mode")) or "ptp"
    if capture_mode not in {"ptp", "original"}:
        raise ValueError("capture_mode must be 'ptp' or 'original'.")

    lidar_mode = optional_text(payload.get("lidar_mode"))
    min_range_m = optional_float(payload.get("min_range_m"))
    max_range_m = optional_float(payload.get("max_range_m"))
    ptp_capture.resolve_range_defaults(min_range_m, max_range_m)

    script_name = "pi_capture_ptp.py" if capture_mode == "ptp" else "pi_capture_raw.py"
    cmd = [sys.executable, os.path.join(SCRIPT_DIR, script_name)]

    wait_for_gps_fix = bool_field(payload, "wait_for_gps_fix", True)
    cmd.append("--wait-for-gps-fix" if wait_for_gps_fix else "--no-wait-for-gps-fix")

    if lidar_mode:
        cmd.extend(["--lidar-mode", lidar_mode])
    if min_range_m is not None:
        cmd.extend(["--min-range-m", str(min_range_m)])
    if max_range_m is not None:
        cmd.extend(["--max-range-m", str(max_range_m)])

    if capture_mode == "ptp":
        wait_for_pi_clock_sync = bool_field(payload, "wait_for_pi_clock_sync", True)
        wait_for_ouster_ptp_lock = bool_field(payload, "wait_for_ouster_ptp_lock", True)
        cmd.append("--wait-for-pi-clock-sync" if wait_for_pi_clock_sync else "--no-wait-for-pi-clock-sync")
        cmd.append("--wait-for-ouster-ptp-lock" if wait_for_ouster_ptp_lock else "--no-wait-for-ouster-ptp-lock")

        gps_input_mode = optional_text(payload.get("gps_input_mode")) or "bridge"
        if gps_input_mode not in {"serial", "gpsd", "bridge"}:
            raise ValueError("gps_input_mode must be 'serial', 'gpsd', or 'bridge'.")
        cmd.extend(["--gps-input-mode", gps_input_mode])

        timestamp_mode = optional_text(payload.get("timestamp_mode"))
        if timestamp_mode:
            cmd.extend(["--timestamp-mode", timestamp_mode])

        ptp_profile = optional_text(payload.get("ptp_profile"))
        if ptp_profile:
            cmd.extend(["--ptp-profile", ptp_profile])

        ouster_host = optional_text(payload.get("ouster_host"))
        if ouster_host:
            cmd.extend(["--ouster-host", ouster_host])

    return cmd


def build_preflight_snapshot(ouster_host: Optional[str]) -> dict[str, Any]:
    try:
        chrony_waitsync = ptp_capture.run_command_capture(
            [
                "chronyc",
                "waitsync",
                str(ptp_capture.CHRONY_WAITSYNC_TRIES),
                str(ptp_capture.CHRONY_MAX_CORRECTION_SEC),
                "0",
                str(ptp_capture.CHRONY_WAITSYNC_INTERVAL_SEC),
            ]
        )
        chrony_tracking = ptp_capture.run_command_capture(["chronyc", "tracking"])
    except FileNotFoundError as exc:
        chrony_waitsync = {
            "cmd": ["chronyc", "waitsync"],
            "returncode": 127,
            "stdout": "",
            "stderr": str(exc),
        }
        chrony_tracking = {
            "cmd": ["chronyc", "tracking"],
            "returncode": 127,
            "stdout": "",
            "stderr": str(exc),
        }

    chrony_ready = chrony_waitsync["returncode"] == 0
    preflight = {
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "chrony": {
            "ready": chrony_ready,
            "waitsync": chrony_waitsync,
            "tracking": chrony_tracking,
        },
        "ouster": {
            "requested_host": ouster_host,
            "reachable": False,
            "locked": False,
            "summary": "not checked",
        },
    }

    host = optional_text(ouster_host)
    if not host:
        preflight["ouster"]["summary"] = "ouster host not provided"
        return preflight

    try:
        status = ptp_capture.get_ouster_time_status(host)
    except Exception as exc:
        preflight["ouster"]["summary"] = str(exc)
        preflight["ouster"]["error"] = str(exc)
        return preflight

    preflight["ouster"]["reachable"] = True
    preflight["ouster"]["locked"] = ptp_capture.ouster_ptp_locked(status)
    preflight["ouster"]["summary"] = ptp_capture.summarize_ouster_ptp_status(status)
    preflight["ouster"]["status"] = status
    return preflight


def apply_log_line_updates(state: dict[str, Any], line: str) -> None:
    if "GPS fix acquired" in line:
        state["gps_fix_ready"] = True
    if "Pi clock synchronized according to chrony." in line:
        state["pi_clock_sync_ready"] = True
    if "Ouster PTP lock acquired." in line:
        state["ouster_ptp_lock_ready"] = True
    if "GPS fix logged:" in line:
        state["latest_gps_log"] = line
    if "GPS CSV:" in line:
        state["gps_csv_path"] = line.split("GPS CSV:", 1)[1].strip()
    if "Manifest:" in line:
        state["manifest_path"] = line.split("Manifest:", 1)[1].strip()
    if "Applying Ouster sensor config:" in line:
        state["phase"] = "configuring sensor"
    if "Waiting for first GPS fix" in line:
        state["phase"] = "waiting for gps fix"
    if "Waiting for Pi clock sync" in line:
        state["phase"] = "waiting for pi clock sync"
    if "Waiting for Ouster PTP lock" in line:
        state["phase"] = "waiting for ouster ptp lock"
    if "Capture mode: continuous chunks" in line or "Capture mode: single chunk." in line:
        state["phase"] = "capturing"

    chunk_match = re.search(r"Starting chunk (\d+):", line)
    if chunk_match:
        state["current_chunk_index"] = int(chunk_match.group(1))
        state["phase"] = "capturing"


class CaptureManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen[str]] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._next_log_id = 1
        self._run_token = 0
        self._logs: collections.deque[dict[str, Any]] = collections.deque(maxlen=LOG_HISTORY_LIMIT)
        self._state: dict[str, Any] = self._fresh_state()

    def _append_internal_log(self, text: str) -> None:
        entry = {
            "id": self._next_log_id,
            "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
            "text": text,
        }
        self._next_log_id += 1
        self._logs.append(entry)

    @staticmethod
    def _fresh_state() -> dict[str, Any]:
        return {
            "running": False,
            "capture_mode": None,
            "pid": None,
            "run_token": 0,
            "command": None,
            "config": None,
            "started_at": None,
            "finished_at": None,
            "exit_code": None,
            "stop_requested": False,
            "phase": "idle",
            "gps_fix_ready": False,
            "pi_clock_sync_ready": False,
            "ouster_ptp_lock_ready": False,
            "current_chunk_index": None,
            "latest_gps_log": None,
            "gps_csv_path": None,
            "manifest_path": None,
        }

    def _snapshot_unlocked(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._state))

    def start(self, payload: dict[str, Any]) -> dict[str, Any]:
        command = build_capture_command(payload)
        capture_mode = optional_text(payload.get("capture_mode")) or "ptp"

        with self._lock:
            if self._proc and self._proc.poll() is None:
                raise RuntimeError("A capture is already running.")

            self._logs.clear()
            self._next_log_id = 1
            self._run_token += 1
            self._state = self._fresh_state()
            self._state.update(
                {
                    "running": True,
                    "capture_mode": capture_mode,
                    "command": command,
                    "config": payload,
                    "run_token": self._run_token,
                    "started_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                    "phase": "starting",
                }
            )
            self._append_internal_log(f"[web] Starting capture: {' '.join(command)}")

            try:
                proc = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except Exception as exc:
                self._state["running"] = False
                self._state["phase"] = "failed"
                self._state["exit_code"] = -1
                self._append_internal_log(f"[web] Failed to start capture: {exc}")
                raise RuntimeError(f"Failed to start capture: {exc}") from exc

            self._proc = proc
            self._state["pid"] = proc.pid

            reader = threading.Thread(target=self._read_logs, args=(proc,), daemon=True)
            self._reader_thread = reader
            reader.start()
            snapshot = self._snapshot_unlocked()

        return snapshot

    def _read_logs(self, proc: subprocess.Popen[str]) -> None:
        assert proc.stdout is not None
        for raw_line in proc.stdout:
            line = raw_line.rstrip("\n")
            with self._lock:
                entry = {
                    "id": self._next_log_id,
                    "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
                    "text": line,
                }
                self._next_log_id += 1
                self._logs.append(entry)
                apply_log_line_updates(self._state, line)

        proc.wait()
        with self._lock:
            self._state["running"] = False
            self._state["pid"] = None
            self._state["finished_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
            self._state["exit_code"] = proc.returncode
            if self._state.get("phase") not in {"failed", "complete"}:
                self._state["phase"] = "complete" if proc.returncode == 0 else "failed"
            self._proc = None

    def stop(self) -> dict[str, Any]:
        with self._lock:
            proc = self._proc
            if not proc or proc.poll() is not None:
                self._append_internal_log("[web] Stop requested but no capture process is running.")
                snapshot = self._snapshot_unlocked()
                return snapshot
            self._state["stop_requested"] = True
            self._state["phase"] = "stopping"
            self._append_internal_log("[web] Sending SIGINT to capture process.")
            proc.send_signal(signal.SIGINT)
            snapshot = self._snapshot_unlocked()
            return snapshot

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_unlocked()

    def logs_after(self, after_id: int) -> dict[str, Any]:
        with self._lock:
            logs = [entry for entry in self._logs if entry["id"] > after_id]
            newest_id = self._logs[-1]["id"] if self._logs else after_id
            return {"entries": logs, "newest_id": newest_id, "run_token": self._state.get("run_token", 0)}


CAPTURE_MANAGER = CaptureManager()


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pi Road Capture</title>
  <style>
    :root {
      --bg: #f4efe6;
      --paper: #fffaf2;
      --ink: #18342f;
      --muted: #5d6f69;
      --accent: #0f8c78;
      --accent-strong: #0b6d5e;
      --warn: #d97706;
      --bad: #b42318;
      --line: #d8cfc1;
      --shadow: 0 14px 40px rgba(24, 52, 47, 0.12);
      --radius: 18px;
      --font: "Atkinson Hyperlegible", "Segoe UI", "Helvetica Neue", sans-serif;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: var(--font);
      color: var(--ink);
      background:
        radial-gradient(circle at top right, rgba(15, 140, 120, 0.15), transparent 28%),
        linear-gradient(180deg, #f7f1e7 0%, #f0e8da 100%);
      min-height: 100vh;
    }
    .page {
      max-width: 1100px;
      margin: 0 auto;
      padding: 16px;
    }
    .hero {
      background: var(--paper);
      border: 1px solid rgba(216, 207, 193, 0.9);
      border-radius: calc(var(--radius) + 4px);
      box-shadow: var(--shadow);
      padding: 18px 18px 16px;
      margin-bottom: 16px;
    }
    .hero h1 {
      margin: 0 0 8px;
      font-size: 1.7rem;
      letter-spacing: -0.03em;
    }
    .hero p {
      margin: 0;
      color: var(--muted);
      line-height: 1.45;
    }
    .grid {
      display: grid;
      grid-template-columns: 1.2fr 0.8fr;
      gap: 16px;
    }
    .panel {
      background: rgba(255, 250, 242, 0.96);
      border: 1px solid rgba(216, 207, 193, 0.9);
      border-radius: var(--radius);
      box-shadow: var(--shadow);
      padding: 16px;
    }
    .panel h2 {
      margin: 0 0 12px;
      font-size: 1.05rem;
      letter-spacing: -0.02em;
    }
    .form-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }
    .field {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .field.full { grid-column: 1 / -1; }
    label {
      font-size: 0.92rem;
      font-weight: 700;
    }
    .hint {
      color: var(--muted);
      font-size: 0.83rem;
      line-height: 1.35;
    }
    input, select, button {
      font: inherit;
    }
    input, select {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 12px 12px;
      background: #fff;
      color: var(--ink);
    }
    .checkbox-row {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-top: 4px;
    }
    .check {
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 10px 12px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.8);
    }
    .check input {
      width: auto;
      margin: 0;
    }
    .actions {
      display: flex;
      gap: 10px;
      margin-top: 14px;
      flex-wrap: wrap;
    }
    button {
      border: none;
      border-radius: 999px;
      padding: 12px 18px;
      font-weight: 800;
      letter-spacing: 0.01em;
      cursor: pointer;
    }
    .primary {
      background: linear-gradient(135deg, var(--accent) 0%, #14b89d 100%);
      color: white;
    }
    .secondary {
      background: white;
      color: var(--ink);
      border: 1px solid var(--line);
    }
    .danger {
      background: #fff1ef;
      color: var(--bad);
      border: 1px solid rgba(180, 35, 24, 0.2);
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 12px;
    }
    .status-card {
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 12px;
      background: rgba(255, 255, 255, 0.84);
    }
    .status-card strong {
      display: block;
      font-size: 0.8rem;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .status-value {
      font-size: 1rem;
      font-weight: 800;
      line-height: 1.3;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border-radius: 999px;
      padding: 8px 12px;
      font-size: 0.88rem;
      font-weight: 700;
      background: white;
      border: 1px solid var(--line);
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      background: var(--muted);
      flex: 0 0 auto;
    }
    .ok .dot { background: var(--accent); }
    .warn .dot { background: var(--warn); }
    .bad .dot { background: var(--bad); }
    pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      font-family: "JetBrains Mono", "SFMono-Regular", monospace;
      font-size: 0.83rem;
      line-height: 1.45;
    }
    .logbox {
      height: 420px;
      overflow: auto;
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 12px;
      background: #f9f5ee;
    }
    .mini {
      color: var(--muted);
      font-size: 0.82rem;
    }
    .subtle {
      background: rgba(15, 140, 120, 0.08);
      border-left: 4px solid rgba(15, 140, 120, 0.5);
      padding: 10px 12px;
      border-radius: 12px;
      margin-top: 10px;
      color: var(--ink);
    }
    .hidden { display: none; }
    @media (max-width: 920px) {
      .grid {
        grid-template-columns: 1fr;
      }
    }
    @media (max-width: 640px) {
      .form-grid,
      .checkbox-row,
      .status-grid {
        grid-template-columns: 1fr;
      }
      .page {
        padding: 12px;
      }
      .hero, .panel {
        padding: 14px;
      }
      .logbox {
        height: 320px;
      }
    }
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <h1>Pi Road Capture</h1>
      <p>Start and stop field capture from your phone, watch readiness gates, and monitor live logs without dealing with VNC on a small screen.</p>
    </section>

    <div class="grid">
      <section class="panel">
        <h2>Capture Setup</h2>
        <div class="form-grid">
          <div class="field">
            <label for="capture_mode">Capture Path</label>
            <select id="capture_mode">
              <option value="ptp">PTP path</option>
              <option value="original">Original path</option>
            </select>
            <div class="hint">Use PTP when the Pi clock is disciplining Ouster timing. Use Original for the older serial/PPS capture flow.</div>
          </div>

          <div class="field">
            <label for="lidar_mode">Lidar Mode</label>
            <select id="lidar_mode">
              <option value="">Leave sensor mode unchanged</option>
              <option value="1024x20">1024x20 (1024 columns, 20 Hz)</option>
              <option value="2048x10">2048x10 (2048 columns, 10 Hz)</option>
              <option value="4096x5">4096x5 (4096 columns, 5 Hz)</option>
            </select>
            <div class="hint">This is the sensor resolution x sample-rate mode. Leave it unchanged if the sensor is already configured the way you want.</div>
          </div>

          <div class="field">
            <label for="min_range_m">Min Range (m)</label>
            <input id="min_range_m" type="number" step="0.1" min="0">
            <div class="hint">Saved into the manifest as a downstream SLAM/filter default. It does not change raw packet capture.</div>
          </div>

          <div class="field">
            <label for="max_range_m">Max Range (m)</label>
            <input id="max_range_m" type="number" step="0.1" min="0.1">
            <div class="hint">Also stored in the manifest for later processing defaults.</div>
          </div>

          <div class="field full">
            <label>Readiness Gates</label>
            <div class="checkbox-row">
              <label class="check"><input id="wait_for_gps_fix" type="checkbox"> Wait for GPS fix</label>
              <label class="check ptp-only"><input id="wait_for_pi_clock_sync" type="checkbox"> Wait for Pi clock sync</label>
              <label class="check ptp-only"><input id="wait_for_ouster_ptp_lock" type="checkbox"> Wait for Ouster PTP lock</label>
            </div>
          </div>

          <div class="field ptp-only">
            <label for="gps_input_mode">GPS Input Mode</label>
            <select id="gps_input_mode">
              <option value="bridge">bridge (I2C bridge service)</option>
              <option value="serial">serial</option>
              <option value="gpsd">gpsd</option>
            </select>
          </div>

          <div class="field ptp-only">
            <label for="ouster_host">Ouster Host</label>
            <input id="ouster_host" type="text" placeholder="169.254.x.x">
          </div>

          <div class="field ptp-only">
            <label for="timestamp_mode">Timestamp Mode</label>
            <input id="timestamp_mode" type="text" placeholder="TIME_FROM_PTP_1588">
          </div>

          <div class="field ptp-only">
            <label for="ptp_profile">PTP Profile</label>
            <input id="ptp_profile" type="text" placeholder="default">
          </div>
        </div>

        <div class="subtle">
          The range fields are here so you can choose your capture-session processing defaults from the phone. Raw `.pcap` capture still records the full sensor data.
        </div>

        <div class="actions">
          <button class="primary" id="start_btn">Start Capture</button>
          <button class="danger" id="stop_btn">Stop Capture</button>
          <button class="secondary" id="refresh_btn">Refresh Status</button>
        </div>
      </section>

      <section class="panel">
        <h2>Live Status</h2>
        <div class="status-grid">
          <div class="status-card">
            <strong>Capture</strong>
            <div class="status-value" id="capture_status">Idle</div>
          </div>
          <div class="status-card">
            <strong>Phase</strong>
            <div class="status-value" id="capture_phase">idle</div>
          </div>
          <div class="status-card">
            <strong>Chunk</strong>
            <div class="status-value" id="capture_chunk">-</div>
          </div>
          <div class="status-card">
            <strong>Manifest</strong>
            <div class="status-value mini" id="manifest_path">-</div>
          </div>
        </div>

        <div class="status-grid">
          <div class="badge" id="badge_gps"><span class="dot"></span><span>GPS fix</span></div>
          <div class="badge" id="badge_clock"><span class="dot"></span><span>Pi clock sync</span></div>
          <div class="badge" id="badge_ptp"><span class="dot"></span><span>Ouster PTP</span></div>
          <div class="badge" id="badge_proc"><span class="dot"></span><span>Process</span></div>
        </div>

        <div class="field full">
          <label>Latest GPS Log</label>
          <pre id="latest_gps_log">-</pre>
        </div>

          <div class="field full">
            <label>Preflight Snapshot</label>
            <pre id="preflight_box">Loading…</pre>
          </div>
          <div class="field full">
            <label>UI Messages</label>
            <pre id="ui_message_box">-</pre>
          </div>
      </section>
    </div>

    <section class="panel" style="margin-top: 16px;">
      <h2>Live Log</h2>
      <div class="logbox" id="logbox"><pre id="logtext"></pre></div>
    </section>
  </div>

  <script>
    const DEFAULTS = __DEFAULT_CONFIG__;
    let lastLogId = 0;
    let currentRunToken = 0;

    function resetLogView() {
      lastLogId = 0;
      document.getElementById("logtext").textContent = "";
    }

    function setUiMessage(text) {
      document.getElementById("ui_message_box").textContent = text || "-";
    }

    function setDefaults() {
      document.getElementById("capture_mode").value = DEFAULTS.capture_mode;
      document.getElementById("lidar_mode").value = DEFAULTS.lidar_mode;
      document.getElementById("min_range_m").value = DEFAULTS.min_range_m;
      document.getElementById("max_range_m").value = DEFAULTS.max_range_m;
      document.getElementById("wait_for_gps_fix").checked = DEFAULTS.wait_for_gps_fix;
      document.getElementById("wait_for_pi_clock_sync").checked = DEFAULTS.wait_for_pi_clock_sync;
      document.getElementById("wait_for_ouster_ptp_lock").checked = DEFAULTS.wait_for_ouster_ptp_lock;
      document.getElementById("gps_input_mode").value = DEFAULTS.gps_input_mode;
      document.getElementById("timestamp_mode").value = DEFAULTS.timestamp_mode;
      document.getElementById("ptp_profile").value = DEFAULTS.ptp_profile;
      document.getElementById("ouster_host").value = DEFAULTS.ouster_host;
      applyModeVisibility();
    }

    function applyModeVisibility() {
      const ptp = document.getElementById("capture_mode").value === "ptp";
      document.querySelectorAll(".ptp-only").forEach(el => {
        el.classList.toggle("hidden", !ptp);
      });
    }

    function formData() {
      return {
        capture_mode: document.getElementById("capture_mode").value,
        lidar_mode: document.getElementById("lidar_mode").value,
        min_range_m: document.getElementById("min_range_m").value,
        max_range_m: document.getElementById("max_range_m").value,
        wait_for_gps_fix: document.getElementById("wait_for_gps_fix").checked,
        wait_for_pi_clock_sync: document.getElementById("wait_for_pi_clock_sync").checked,
        wait_for_ouster_ptp_lock: document.getElementById("wait_for_ouster_ptp_lock").checked,
        gps_input_mode: document.getElementById("gps_input_mode").value,
        timestamp_mode: document.getElementById("timestamp_mode").value,
        ptp_profile: document.getElementById("ptp_profile").value,
        ouster_host: document.getElementById("ouster_host").value
      };
    }

    function setBadge(id, state, labelText) {
      const badge = document.getElementById(id);
      badge.classList.remove("ok", "warn", "bad");
      if (state === true) {
        badge.classList.add("ok");
      } else if (state === false) {
        badge.classList.add("warn");
      } else {
        badge.classList.add("warn");
      }
      badge.lastElementChild.textContent = labelText;
    }

    async function startCapture() {
      setUiMessage("Starting capture...");
      try {
        const response = await fetch("/api/start", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(formData())
        });
        const data = await response.json();
        if (!response.ok) {
          setUiMessage(data.error || "Failed to start capture");
          return;
        }
        resetLogView();
        currentRunToken = data.run_token || 0;
        setUiMessage("Capture start request accepted.");
        await refreshStatus();
        await refreshLogs();
      } catch (error) {
        setUiMessage(`Start request failed: ${error}`);
      }
    }

    async function stopCapture() {
      setUiMessage("Stopping capture...");
      try {
        const response = await fetch("/api/stop", { method: "POST" });
        const data = await response.json();
        if (!response.ok) {
          setUiMessage(data.error || "Failed to stop capture");
          return;
        }
        setUiMessage("Stop request sent.");
        await refreshStatus();
        await refreshLogs();
      } catch (error) {
        setUiMessage(`Stop request failed: ${error}`);
      }
    }

    async function refreshStatus() {
      try {
        const response = await fetch("/api/status");
        const data = await response.json();
        if ((data.run_token || 0) !== currentRunToken) {
          currentRunToken = data.run_token || 0;
          resetLogView();
        }
        document.getElementById("capture_status").textContent = data.running ? "Running" : "Idle";
        document.getElementById("capture_phase").textContent = data.phase || "-";
        document.getElementById("capture_chunk").textContent = data.current_chunk_index ?? "-";
        document.getElementById("manifest_path").textContent = data.manifest_path || "-";
        document.getElementById("latest_gps_log").textContent = data.latest_gps_log || "-";

        setBadge("badge_gps", data.gps_fix_ready, "GPS fix");
        const ptpMode = document.getElementById("capture_mode").value === "ptp";
        setBadge("badge_clock", ptpMode ? data.pi_clock_sync_ready : null, ptpMode ? "Pi clock sync" : "Pi clock sync n/a");
        setBadge("badge_ptp", ptpMode ? data.ouster_ptp_lock_ready : null, ptpMode ? "Ouster PTP" : "Ouster PTP n/a");
        setBadge("badge_proc", data.running ? true : null, data.running ? "Process running" : "Process idle");
        if (data.phase === "failed" && data.exit_code !== null && data.exit_code !== undefined) {
          setUiMessage(`Capture process exited with code ${data.exit_code}. Check the live log below.`);
        }
      } catch (error) {
        setUiMessage(`Status refresh failed: ${error}`);
      }
    }

    async function refreshLogs() {
      try {
        const response = await fetch(`/api/logs?after=${lastLogId}`);
        const data = await response.json();
        if ((data.run_token || 0) !== currentRunToken) {
          currentRunToken = data.run_token || 0;
          resetLogView();
        }
        const logtext = document.getElementById("logtext");
        for (const entry of data.entries) {
          logtext.textContent += `${entry.text}\n`;
        }
        if (data.entries.length > 0 || data.newest_id !== undefined) {
          lastLogId = data.newest_id;
        }
        if (data.entries.length > 0) {
          const box = document.getElementById("logbox");
          box.scrollTop = box.scrollHeight;
        }
      } catch (error) {
        setUiMessage(`Log refresh failed: ${error}`);
      }
    }

    async function refreshPreflight() {
      try {
        const params = new URLSearchParams();
        const host = document.getElementById("ouster_host").value;
        if (host) params.set("ouster_host", host);
        const response = await fetch(`/api/preflight?${params.toString()}`);
        const data = await response.json();
        document.getElementById("preflight_box").textContent = JSON.stringify(data, null, 2);
      } catch (error) {
        document.getElementById("preflight_box").textContent = `Preflight refresh failed: ${error}`;
      }
    }

    document.getElementById("capture_mode").addEventListener("change", applyModeVisibility);
    document.getElementById("start_btn").addEventListener("click", startCapture);
    document.getElementById("stop_btn").addEventListener("click", stopCapture);
    document.getElementById("refresh_btn").addEventListener("click", async () => {
      await refreshStatus();
      await refreshPreflight();
    });

    setDefaults();
    refreshStatus();
    refreshLogs();
    refreshPreflight();
    setInterval(refreshStatus, 2000);
    setInterval(refreshLogs, 1000);
    setInterval(refreshPreflight, 5000);
  </script>
</body>
</html>
"""


class CaptureRequestHandler(BaseHTTPRequestHandler):
    server_version = "PiCaptureWeb/1.0"

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, html_text: str) -> None:
        body = html_text.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            html = HTML_TEMPLATE.replace("__DEFAULT_CONFIG__", json.dumps(DEFAULT_UI_CONFIG))
            self._send_html(html)
            return

        if parsed.path == "/api/status":
            self._send_json(HTTPStatus.OK, CAPTURE_MANAGER.snapshot())
            return

        if parsed.path == "/api/logs":
            query = urllib.parse.parse_qs(parsed.query)
            try:
                after = int(query.get("after", ["0"])[0])
            except ValueError:
                after = 0
            self._send_json(HTTPStatus.OK, CAPTURE_MANAGER.logs_after(after))
            return

        if parsed.path == "/api/preflight":
            query = urllib.parse.parse_qs(parsed.query)
            host = query.get("ouster_host", [""])[0]
            self._send_json(HTTPStatus.OK, build_preflight_snapshot(host))
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/start":
            try:
                payload = self._read_json_body()
                snapshot = CAPTURE_MANAGER.start(payload)
            except RuntimeError as exc:
                self._send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
                return
            except Exception as exc:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
                return
            self._send_json(HTTPStatus.OK, snapshot)
            return

        if parsed.path == "/api/stop":
            snapshot = CAPTURE_MANAGER.stop()
            self._send_json(HTTPStatus.OK, snapshot)
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:
        timestamp = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [web] {format % args}", flush=True)


def main() -> None:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), CaptureRequestHandler)
    print(
        f"[{dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] "
        f"Pi capture web UI listening on http://{args.host}:{args.port}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Interrupted, shutting down web server.", flush=True)
    finally:
        CAPTURE_MANAGER.stop()
        server.server_close()


if __name__ == "__main__":
    main()
