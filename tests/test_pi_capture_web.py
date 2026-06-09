import importlib.util
from pathlib import Path


def _load_module(filename: str, module_name: str):
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / filename
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


web_capture = _load_module("pi_capture_web.py", "pi_capture_web")


def test_build_capture_command_for_ptp_mode():
    cmd = web_capture.build_capture_command(
        {
            "capture_mode": "ptp",
            "lidar_mode": "1024x20",
            "min_range_m": "2.5",
            "max_range_m": "80",
            "wait_for_gps_fix": True,
            "wait_for_pi_clock_sync": False,
            "wait_for_ouster_ptp_lock": True,
            "gps_input_mode": "gpsd",
            "timestamp_mode": "TIME_FROM_PTP_1588",
            "ptp_profile": "default",
            "ouster_host": "169.254.10.20",
        }
    )
    joined = " ".join(cmd)
    assert cmd[1].endswith("pi_capture_ptp.py")
    assert "--lidar-mode 1024x20" in joined
    assert "--min-range-m 2.5" in joined
    assert "--max-range-m 80.0" in joined
    assert "--wait-for-gps-fix" in joined
    assert "--no-wait-for-pi-clock-sync" in joined
    assert "--wait-for-ouster-ptp-lock" in joined
    assert "--gps-input-mode gpsd" in joined
    assert "--ouster-host 169.254.10.20" in joined


def test_build_capture_command_for_original_mode():
    cmd = web_capture.build_capture_command(
        {
            "capture_mode": "original",
            "lidar_mode": "",
            "min_range_m": 1.0,
            "max_range_m": 150.0,
            "wait_for_gps_fix": False,
        }
    )
    joined = " ".join(cmd)
    assert cmd[1].endswith("pi_capture_raw.py")
    assert "--no-wait-for-gps-fix" in joined
    assert "--min-range-m 1.0" in joined
    assert "--max-range-m 150.0" in joined


def test_apply_log_line_updates_tracks_readiness_and_paths():
    state = {
        "gps_fix_ready": False,
        "pi_clock_sync_ready": False,
        "ouster_ptp_lock_ready": False,
        "latest_gps_log": None,
        "gps_csv_path": None,
        "manifest_path": None,
        "phase": "starting",
        "current_chunk_index": None,
    }
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:00] GPS fix acquired. Continuing readiness checks.")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:01] Pi clock synchronized according to chrony.")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:02] Ouster PTP lock acquired.")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:03] GPS fix logged: q=4, lat=29.0")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:04] Starting chunk 3: /tmp/raw_lidar_chunk0003.pcap")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:05] GPS CSV: /tmp/raw_gps.csv")
    web_capture.apply_log_line_updates(state, "[2026-06-08 12:00:06] Manifest: /tmp/capture_manifest.json")

    assert state["gps_fix_ready"] is True
    assert state["pi_clock_sync_ready"] is True
    assert state["ouster_ptp_lock_ready"] is True
    assert state["latest_gps_log"].endswith("GPS fix logged: q=4, lat=29.0")
    assert state["current_chunk_index"] == 3
    assert state["gps_csv_path"] == "/tmp/raw_gps.csv"
    assert state["manifest_path"] == "/tmp/capture_manifest.json"
