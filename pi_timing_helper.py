#!/usr/bin/env python3
"""
pi_timing_helper.py

Small privileged helper for the Pi capture web UI.

This script intentionally exposes only a tiny whitelist of timing-related
operations so the web UI can stay phone-friendly without requiring operators
to type systemctl or nmcli commands during field use.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shutil
import subprocess
import sys
import time
from typing import Any

DEFAULT_IFACE = "eth0"
LOCAL_CHRONY_SOURCE_NAMES = {"GPS", "PPS"}
PHC_READY_DELTA_SEC = 0.01
PHC_WARN_DELTA_SEC = 1.0
SERVICE_UNITS = {
    "chrony": "chrony.service",
    "bridge": "ublox-i2c-chrony-bridge.service",
    "ptp4l": "ptp4l.service",
    "phc2sys": "phc2sys.service",
}
WIFI_CONNECTION_TYPES = {"wifi", "802-11-wireless"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Privileged timing helper for the Pi capture web UI.")
    parser.add_argument("--iface", default=DEFAULT_IFACE, help=f"Network interface carrying Ouster PTP (default: {DEFAULT_IFACE})")

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="Return timing service status as JSON")
    subparsers.add_parser("wifi-status", help="Return saved and active Wi-Fi connections as JSON")

    restart_parser = subparsers.add_parser("restart-phc2sys", help="Safely restart the Pi time broadcast service")
    restart_parser.add_argument("--settle-sec", type=float, default=2.0, help="Seconds to wait after restart before re-checking status")

    wifi_switch_parser = subparsers.add_parser("switch-wifi", help="Bring up a saved NetworkManager Wi-Fi connection")
    wifi_switch_parser.add_argument("--connection", required=True, help="Saved NetworkManager connection name to activate")

    return parser.parse_args()


def run_command_capture(cmd: list[str]) -> dict[str, Any]:
    result = subprocess.run(cmd, capture_output=True, text=True)
    return {
        "cmd": cmd,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }


def service_state(unit: str) -> dict[str, Any]:
    active = run_command_capture(["systemctl", "is-active", unit])
    enabled = run_command_capture(["systemctl", "is-enabled", unit])
    active_state = (active.get("stdout") or active.get("stderr") or "unknown").strip()
    enabled_state = (enabled.get("stdout") or enabled.get("stderr") or "unknown").strip()
    return {
        "unit": unit,
        "active": active_state == "active",
        "active_state": active_state,
        "enabled_state": enabled_state,
        "active_cmd": active,
        "enabled_cmd": enabled,
    }


def parse_chrony_sources_output(text: str) -> list[dict[str, str]]:
    entries: list[dict[str, str]] = []
    for raw_line in text.splitlines():
        if len(raw_line) < 3:
            continue
        mode = raw_line[0]
        state = raw_line[1]
        if mode not in {"#", "^", "="}:
            continue
        if state not in {"*", "+", "-", "?", "x", "~"}:
            continue
        remainder = raw_line[2:].strip()
        if not remainder:
            continue
        entries.append(
            {
                "mode": mode,
                "state": state,
                "name": remainder.split()[0],
                "raw": raw_line.rstrip(),
            }
        )
    return entries


def evaluate_chrony_local_gps_pps_sync(snapshot: dict[str, Any]) -> dict[str, Any]:
    waitsync = snapshot.get("waitsync") if isinstance(snapshot.get("waitsync"), dict) else {}
    waitsync_ok = waitsync.get("returncode") == 0
    sources_result = snapshot.get("sources") if isinstance(snapshot.get("sources"), dict) else {}
    sources = parse_chrony_sources_output(str(sources_result.get("stdout") or ""))
    selected = next((entry for entry in sources if entry["state"] == "*"), None)
    local_sources = [entry for entry in sources if entry["name"] in LOCAL_CHRONY_SOURCE_NAMES]
    selected_name = selected["name"] if selected else None
    selected_is_local = selected_name in LOCAL_CHRONY_SOURCE_NAMES if selected_name else False
    ready = bool(waitsync_ok and selected_is_local)

    if not waitsync_ok:
        summary = waitsync.get("stdout") or waitsync.get("stderr") or "Pi time has not settled yet."
    elif not local_sources:
        summary = "GPS/PPS timing sources are not showing up yet."
    elif selected is None:
        summary = "A timing source is visible, but none is selected yet."
    elif not selected_is_local:
        summary = f"Pi time is still following {selected_name} instead of local GPS/PPS."
    else:
        summary = f"Pi time is locked to local {selected_name}."

    return {
        "ready": ready,
        "summary": summary,
        "waitsync_ok": waitsync_ok,
        "selected_source": selected,
        "selected_source_name": selected_name,
        "parsed_sources": sources,
    }


def collect_chrony_snapshot() -> dict[str, Any]:
    return {
        "waitsync": run_command_capture(["chronyc", "waitsync", "1", "0.01", "0", "1"]),
        "tracking": run_command_capture(["chronyc", "tracking"]),
        "sources": run_command_capture(["chronyc", "sources", "-v"]),
    }


def parse_phc_seconds(text: str) -> float:
    match = re.search(r"clock time is ([0-9]+\.[0-9]+)", text)
    if not match:
        raise ValueError(f"Could not parse PHC output: {text}")
    return float(match.group(1))


def collect_phc_alignment(iface: str) -> dict[str, Any]:
    if not shutil_which("phc_ctl"):
        return {
            "available": False,
            "ready": False,
            "summary": "PHC comparison tool is not installed.",
        }

    sys_out = run_command_capture(["phc_ctl", "CLOCK_REALTIME", "get"])
    phc_out = run_command_capture(["phc_ctl", iface, "get"])

    if sys_out["returncode"] != 0 or phc_out["returncode"] != 0:
        return {
            "available": True,
            "ready": False,
            "summary": "Could not read the Pi network hardware clock.",
            "system_clock": sys_out,
            "phc_clock": phc_out,
        }

    try:
        sys_t = parse_phc_seconds(sys_out["stdout"])
        phc_t = parse_phc_seconds(phc_out["stdout"])
    except ValueError as exc:
        return {
            "available": True,
            "ready": False,
            "summary": str(exc),
            "system_clock": sys_out,
            "phc_clock": phc_out,
        }

    delta = phc_t - sys_t
    abs_delta = abs(delta)
    if abs_delta <= PHC_READY_DELTA_SEC:
        summary = "Pi network time broadcast is aligned."
        ready = True
        severity = "ok"
    elif abs_delta <= PHC_WARN_DELTA_SEC:
        summary = "Pi network time broadcast is close, but not quite aligned yet."
        ready = False
        severity = "warn"
    else:
        summary = "Pi network time broadcast is far off and needs a restart."
        ready = False
        severity = "bad"

    return {
        "available": True,
        "ready": ready,
        "severity": severity,
        "delta_seconds": delta,
        "summary": summary,
        "system_clock": sys_out,
        "phc_clock": phc_out,
    }


def shutil_which(name: str) -> str | None:
    return shutil.which(name)


def is_wifi_connection_type(conn_type: str) -> bool:
    return conn_type.strip().lower() in WIFI_CONNECTION_TYPES


def list_wifi_connections() -> dict[str, Any]:
    if not shutil_which("nmcli"):
        return {
            "available": False,
            "summary": "NetworkManager tools are not installed.",
            "active_connection": None,
            "saved_connections": [],
        }

    active_cmd = run_command_capture(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"])
    saved_cmd = run_command_capture(["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show"])

    active_connection = None
    active_entries: list[dict[str, str]] = []
    if active_cmd["returncode"] == 0:
        for line in active_cmd["stdout"].splitlines():
            if not line.strip():
                continue
            parts = line.split(":")
            if len(parts) < 3:
                continue
            name, conn_type, device = parts[0], parts[1], parts[2]
            entry = {"name": name, "type": conn_type, "device": device}
            active_entries.append(entry)
            if is_wifi_connection_type(conn_type):
                active_connection = name

    saved_connections: list[str] = []
    if saved_cmd["returncode"] == 0:
        for line in saved_cmd["stdout"].splitlines():
            if not line.strip():
                continue
            parts = line.split(":")
            if len(parts) < 2:
                continue
            name, conn_type = parts[0], parts[1]
            if is_wifi_connection_type(conn_type):
                saved_connections.append(name)

    summary = (
        f"Active Wi-Fi: {active_connection}."
        if active_connection
        else "No active Wi-Fi connection is currently selected."
    )
    if not saved_connections:
        summary = "No saved Wi-Fi connections were found."

    return {
        "available": True,
        "summary": summary,
        "active_connection": active_connection,
        "active_entries": active_entries,
        "saved_connections": saved_connections,
        "active_cmd": active_cmd,
        "saved_cmd": saved_cmd,
    }


def switch_wifi(connection_name: str) -> dict[str, Any]:
    before = list_wifi_connections()
    if not before["available"]:
        return {
            "ok": False,
            "summary": before["summary"],
            "status": before,
        }
    if connection_name not in before["saved_connections"]:
        return {
            "ok": False,
            "summary": f"Saved Wi-Fi connection '{connection_name}' was not found.",
            "status": before,
        }

    switch_cmd = run_command_capture(["nmcli", "connection", "up", connection_name])
    time.sleep(1.0)
    after = list_wifi_connections()
    ok = switch_cmd["returncode"] == 0 and after.get("active_connection") == connection_name
    return {
        "ok": ok,
        "summary": (
            f"Switched Wi-Fi to '{connection_name}'."
            if ok
            else f"Tried to switch Wi-Fi to '{connection_name}', but it did not become active."
        ),
        "switch_cmd": switch_cmd,
        "status": after,
    }


def build_status(iface: str) -> dict[str, Any]:
    chrony_snapshot = collect_chrony_snapshot()
    chrony_evaluation = evaluate_chrony_local_gps_pps_sync(chrony_snapshot)
    services = {name: service_state(unit) for name, unit in SERVICE_UNITS.items()}
    phc_alignment = collect_phc_alignment(iface)

    gps_time_ready = bool(chrony_evaluation["ready"])
    service_stack_ready = all(
        services[name]["active"] for name in ("chrony", "bridge", "ptp4l", "phc2sys")
    )
    pi_time_broadcast_ready = bool(
        services["ptp4l"]["active"] and services["phc2sys"]["active"] and phc_alignment.get("ready") is True
    )

    if gps_time_ready and pi_time_broadcast_ready:
        summary = "Timing stack looks ready for PTP capture."
        recommended_action = "You can start a PTP capture session."
    elif not gps_time_ready:
        summary = chrony_evaluation["summary"]
        recommended_action = "Move outside for sky view and wait for GPS time to lock before starting capture."
    elif not services["phc2sys"]["active"]:
        summary = "Pi time broadcast is not running yet."
        recommended_action = "Use the Fix Time Broadcast button once GPS time is ready."
    elif not phc_alignment.get("ready", False):
        summary = phc_alignment.get("summary") or "Pi time broadcast is not aligned yet."
        recommended_action = "Use the Fix Time Broadcast button to restart the broadcast after GPS time is ready."
    elif not service_stack_ready:
        summary = "One or more background timing services are not running."
        recommended_action = "Check the service details below and restart the missing service."
    else:
        summary = "Timing stack is partially ready."
        recommended_action = "Refresh timing status after a few seconds."

    services_ready_count = sum(1 for entry in services.values() if entry["active"])
    background_services_summary = f"{services_ready_count}/{len(services)} timing services running."

    return {
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "iface": iface,
        "gps_time_ready": gps_time_ready,
        "gps_time_summary": chrony_evaluation["summary"],
        "pi_time_broadcast_ready": pi_time_broadcast_ready,
        "pi_time_broadcast_summary": phc_alignment.get("summary") or "Pi time broadcast status unavailable.",
        "background_services_ready": service_stack_ready,
        "background_services_summary": background_services_summary,
        "summary": summary,
        "recommended_action": recommended_action,
        "actions": {
            "restart_phc2sys_allowed": gps_time_ready,
            "restart_phc2sys_reason": (
                "GPS time is ready, so the Pi time broadcast can be restarted safely."
                if gps_time_ready
                else "Wait for GPS/PPS lock before restarting the Pi time broadcast."
            ),
        },
        "services": services,
        "chrony": {
            "ready": gps_time_ready,
            "evaluation": chrony_evaluation,
            "waitsync": chrony_snapshot["waitsync"],
            "tracking": chrony_snapshot["tracking"],
            "sources": chrony_snapshot["sources"],
        },
        "phc_alignment": phc_alignment,
    }


def restart_phc2sys(iface: str, settle_sec: float) -> dict[str, Any]:
    before = build_status(iface)
    if not before["gps_time_ready"]:
        return {
            "ok": False,
            "summary": "GPS time is not ready yet, so Pi time broadcast was not restarted.",
            "recommended_action": before["recommended_action"],
            "status": before,
        }

    restart_cmd = run_command_capture(["systemctl", "restart", SERVICE_UNITS["phc2sys"]])
    time.sleep(max(0.0, settle_sec))
    after = build_status(iface)
    ok = restart_cmd["returncode"] == 0 and after["pi_time_broadcast_ready"]

    return {
        "ok": ok,
        "summary": (
            "Pi time broadcast restarted successfully."
            if ok
            else "Pi time broadcast was restarted, but it still does not look ready."
        ),
        "recommended_action": after["recommended_action"],
        "restart_cmd": restart_cmd,
        "status": after,
    }


def main() -> None:
    args = parse_args()

    if args.command == "status":
        payload = build_status(args.iface)
    elif args.command == "wifi-status":
        payload = list_wifi_connections()
    elif args.command == "restart-phc2sys":
        payload = restart_phc2sys(args.iface, args.settle_sec)
    elif args.command == "switch-wifi":
        payload = switch_wifi(args.connection)
    else:
        raise SystemExit(f"Unsupported command: {args.command}")

    json.dump(payload, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
