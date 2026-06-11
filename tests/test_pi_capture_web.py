import importlib.util
import io
from pathlib import Path
from unittest import mock


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
            "lidar_resolution": "1024",
            "lidar_hz": "20",
            "min_range_m": "2.5",
            "max_range_m": "80",
            "wait_for_gps_fix": True,
            "wait_for_pi_clock_sync": False,
            "wait_for_ouster_ptp_lock": True,
            "gps_input_mode": "bridge",
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
    assert "--gps-input-mode bridge" in joined
    assert "--ouster-host 169.254.10.20" in joined


def test_build_capture_command_for_original_mode():
    cmd = web_capture.build_capture_command(
        {
            "capture_mode": "original",
            "lidar_resolution": "",
            "lidar_hz": "",
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


def test_build_capture_command_rejects_unsupported_lidar_pair():
    try:
        web_capture.build_capture_command(
            {
                "capture_mode": "ptp",
                "lidar_resolution": "1024",
                "lidar_hz": "10",
                "wait_for_gps_fix": False,
            }
        )
    except ValueError as exc:
        assert "Unsupported lidar mode combination" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unsupported lidar mode pair")


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


def test_capture_manager_start_and_stop_do_not_deadlock():
    manager = web_capture.CaptureManager()

    class _FakeProc:
        def __init__(self):
            self.pid = 4321
            self.stdout = io.StringIO("")
            self.returncode = None
            self._sent_signal = None

        def poll(self):
            return self.returncode

        def wait(self):
            self.returncode = 0
            return self.returncode

        def send_signal(self, sig):
            self._sent_signal = sig
            self.returncode = 0

    fake_proc = _FakeProc()

    class _FakeThread:
        def __init__(self, target=None, args=(), daemon=None):
            self.target = target
            self.args = args
            self.daemon = daemon

        def start(self):
            return None

    with (
        mock.patch.object(web_capture.subprocess, "Popen", return_value=fake_proc),
        mock.patch.object(web_capture.threading, "Thread", _FakeThread),
    ):
        snapshot = manager.start({"capture_mode": "original", "wait_for_gps_fix": False})
        assert snapshot["running"] is True
        assert snapshot["phase"] == "starting"
        assert snapshot["pid"] == 4321

        stop_snapshot = manager.stop()
        assert stop_snapshot["stop_requested"] is True
        assert stop_snapshot["phase"] == "stopping"
        assert fake_proc._sent_signal == web_capture.signal.SIGINT


def test_capture_manager_restart_increments_run_token_and_resets_log_ids():
    manager = web_capture.CaptureManager()

    class _FakeProc:
        next_pid = 5000

        def __init__(self):
            type(self).next_pid += 1
            self.pid = type(self).next_pid
            self.stdout = io.StringIO("")
            self.returncode = None

        def poll(self):
            return self.returncode

        def wait(self):
            self.returncode = 0
            return self.returncode

        def send_signal(self, _sig):
            self.returncode = 0

    class _FakeThread:
        def __init__(self, target=None, args=(), daemon=None):
            self.target = target
            self.args = args
            self.daemon = daemon

        def start(self):
            return None

    with (
        mock.patch.object(web_capture.subprocess, "Popen", side_effect=[_FakeProc(), _FakeProc()]),
        mock.patch.object(web_capture.threading, "Thread", _FakeThread),
    ):
        first = manager.start({"capture_mode": "original", "wait_for_gps_fix": False})
        assert first["run_token"] == 1
        first_logs = manager.logs_after(0)
        assert first_logs["run_token"] == 1
        assert first_logs["newest_id"] >= 1

        manager._proc = None
        manager._state["running"] = False
        manager._state["phase"] = "complete"

        second = manager.start({"capture_mode": "original", "wait_for_gps_fix": False})
        assert second["run_token"] == 2
        second_logs = manager.logs_after(0)
        assert second_logs["run_token"] == 2
        assert second_logs["entries"][0]["id"] == 1


def test_html_template_contains_compact_preflight_summary():
    html = web_capture.HTML_TEMPLATE
    assert "Preflight Summary" in html
    assert "preflight_clock_status" in html
    assert "preflight_ouster_status" in html
    assert "preflight_ptp_status" in html
    assert "Show detailed preflight JSON" in html
