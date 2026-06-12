import importlib.util
from pathlib import Path
from unittest import mock


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "pi_timing_helper.py"
    spec = importlib.util.spec_from_file_location("pi_timing_helper", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


timing_helper = _load_module()


def test_evaluate_chrony_local_gps_pps_sync_accepts_local_selected_source():
    snapshot = {
        "waitsync": {"returncode": 0, "stdout": "ok", "stderr": ""},
        "sources": {
            "stdout": "\n".join(
                [
                    "#+ GPS                           0   2   377    10   +0ns[   +0ns] +/-    0ns",
                    "#* PPS                           0   2   377    10   +0ns[   +0ns] +/-    0ns",
                    "^- 23.150.41.123                2   6   377    10   +1ms[  +1ms] +/-   22ms",
                ]
            )
        },
    }

    evaluation = timing_helper.evaluate_chrony_local_gps_pps_sync(snapshot)
    assert evaluation["ready"] is True
    assert evaluation["selected_source_name"] == "PPS"
    assert "locked to local PPS" in evaluation["summary"]


def test_build_status_reports_ready_when_everything_is_healthy():
    fake_services = {
        "chrony": {"active": True, "active_state": "active"},
        "bridge": {"active": True, "active_state": "active"},
        "ptp4l": {"active": True, "active_state": "active"},
        "phc2sys": {"active": True, "active_state": "active"},
    }
    fake_phc = {"available": True, "ready": True, "summary": "Pi network time broadcast is aligned."}
    fake_chrony = {
        "waitsync": {"returncode": 0, "stdout": "ok", "stderr": ""},
        "tracking": {},
        "sources": {"stdout": "#* PPS ..."},
    }
    fake_eval = {"ready": True, "summary": "Pi time is locked to local PPS."}

    with (
        mock.patch.object(timing_helper, "collect_chrony_snapshot", return_value=fake_chrony),
        mock.patch.object(timing_helper, "evaluate_chrony_local_gps_pps_sync", return_value=fake_eval),
        mock.patch.object(timing_helper, "collect_phc_alignment", return_value=fake_phc),
        mock.patch.object(timing_helper, "service_state", side_effect=lambda unit: fake_services[[k for k, v in timing_helper.SERVICE_UNITS.items() if v == unit][0]]),
    ):
        status = timing_helper.build_status("eth0")

    assert status["gps_time_ready"] is True
    assert status["pi_time_broadcast_ready"] is True
    assert status["background_services_ready"] is True
    assert "ready for PTP capture" in status["summary"]


def test_restart_phc2sys_refuses_when_gps_time_not_ready():
    before = {
        "gps_time_ready": False,
        "recommended_action": "Move outside for sky view.",
    }
    with mock.patch.object(timing_helper, "build_status", return_value=before):
        result = timing_helper.restart_phc2sys("eth0", settle_sec=0)

    assert result["ok"] is False
    assert "was not restarted" in result["summary"]
    assert result["recommended_action"] == "Move outside for sky view."


def test_list_wifi_connections_parses_active_and_saved_profiles():
    active_cmd = {
        "returncode": 0,
        "stdout": "AirRowdy:wifi:wlan0\nURP-Ouster-Link:ethernet:eth0",
        "stderr": "",
    }
    saved_cmd = {
        "returncode": 0,
        "stdout": "URP-RPI-Net:wifi\nAirRowdy:wifi\nURP-Ouster-Link:ethernet",
        "stderr": "",
    }
    with (
        mock.patch.object(timing_helper, "shutil_which", return_value="/usr/bin/nmcli"),
        mock.patch.object(timing_helper, "run_command_capture", side_effect=[active_cmd, saved_cmd]),
    ):
        status = timing_helper.list_wifi_connections()

    assert status["available"] is True
    assert status["active_connection"] == "AirRowdy"
    assert status["saved_connections"] == ["URP-RPI-Net", "AirRowdy"]


def test_switch_wifi_rejects_unknown_connection():
    before = {
        "available": True,
        "saved_connections": ["URP-RPI-Net", "AirRowdy"],
        "summary": "Active Wi-Fi: AirRowdy.",
    }
    with mock.patch.object(timing_helper, "list_wifi_connections", return_value=before):
        result = timing_helper.switch_wifi("MissingNet")

    assert result["ok"] is False
    assert "was not found" in result["summary"]
