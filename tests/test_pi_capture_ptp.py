import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest import mock


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "pi_capture_ptp.py"
    spec = importlib.util.spec_from_file_location("pi_capture_ptp", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


ptp_capture = _load_module()


def test_normalize_lidar_mode_accepts_compact_input():
    assert ptp_capture.normalize_lidar_mode(" 1024X20 ") == "1024x20"


def test_normalize_lidar_mode_rejects_invalid_input():
    try:
        ptp_capture.normalize_lidar_mode("1024")
    except ValueError as exc:
        assert "Expected values like 1024x20" in str(exc)
    else:
        raise AssertionError("Expected ValueError for invalid lidar mode")


def test_gpsd_time_to_epoch_ns_parses_utc_z_suffix():
    epoch_ns = ptp_capture.gpsd_time_to_epoch_ns("2026-05-12T13:47:29.123456Z")
    assert epoch_ns == 1778593649123456000


def test_build_gps_logger_supports_bridge_mode(tmp_path):
    logger = ptp_capture.build_gps_logger(
        gps_input_mode="bridge",
        stop_event=threading.Event(),
        gps_csv_path=str(tmp_path / "raw_gps.csv"),
        gps_port="/dev/does-not-matter",
        gps_baud=9600,
        gpsd_host="127.0.0.1",
        gpsd_port=2947,
        bridge_fix_path=str(tmp_path / "latest_fix.json"),
    )
    assert isinstance(logger, ptp_capture.BridgeGpsLogger)


def test_bridge_gps_logger_writes_fix_from_json(tmp_path):
    fix_path = tmp_path / "latest_fix.json"
    fix_path.write_text(
        json.dumps(
            {
                "sequence": 1,
                "pi_time_ns": 1781034250801505000,
                "pi_time_iso": "2026-06-09T19:44:10.801505+00:00",
                "gps_epoch_ns": 1781034250000000000,
                "gps_utc_time": "19:44:10",
                "gps_quality": 4,
                "num_sats": 18,
                "hdop": 0.6,
                "latitude": 29.42412,
                "longitude": -98.49363,
                "altitude_m": 200.5,
            }
        ),
        encoding="utf-8",
    )

    stop_event = threading.Event()
    logger = ptp_capture.BridgeGpsLogger(
        fix_json_path=str(fix_path),
        out_csv_path=str(tmp_path / "raw_gps.csv"),
        stop_event=stop_event,
        poll_sec=0.01,
        min_fresh_pi_time_ns=0,
    )
    logger.start()
    deadline = time.time() + 0.5
    while logger.rows_written < 1 and time.time() < deadline:
        time.sleep(0.01)
    stop_event.set()
    logger.join(timeout=1)

    csv_text = (tmp_path / "raw_gps.csv").read_text(encoding="utf-8")
    assert "29.42412" in csv_text
    assert "-98.49363" in csv_text
    assert logger.rows_written >= 1
    assert logger.first_fix_time_ns == 1781034250801505000


def test_bridge_gps_logger_waits_for_new_fix_after_stale_file(tmp_path):
    fix_path = tmp_path / "latest_fix.json"
    stale_pi_time_ns = 1781034250801505000
    fix_path.write_text(
        json.dumps(
            {
                "sequence": 1,
                "pi_time_ns": stale_pi_time_ns,
                "pi_time_iso": "2026-06-09T19:44:10.801505+00:00",
                "gps_epoch_ns": 1781034250000000000,
                "gps_utc_time": "19:44:10",
                "gps_quality": 4,
                "num_sats": 18,
                "hdop": 0.6,
                "latitude": 29.42412,
                "longitude": -98.49363,
                "altitude_m": 200.5,
            }
        ),
        encoding="utf-8",
    )

    stop_event = threading.Event()
    logger = ptp_capture.BridgeGpsLogger(
        fix_json_path=str(fix_path),
        out_csv_path=str(tmp_path / "raw_gps.csv"),
        stop_event=stop_event,
        poll_sec=0.01,
        min_fresh_pi_time_ns=stale_pi_time_ns + 1,
    )
    logger.start()
    time.sleep(0.05)

    assert logger.rows_written == 0
    assert logger.first_fix_time_ns is None

    fix_path.write_text(
        json.dumps(
            {
                "sequence": 2,
                "pi_time_ns": stale_pi_time_ns + 5000,
                "pi_time_iso": "2026-06-09T19:44:10.806505+00:00",
                "gps_epoch_ns": 1781034250005000000,
                "gps_utc_time": "19:44:10.005",
                "gps_quality": 4,
                "num_sats": 18,
                "hdop": 0.6,
                "latitude": 29.42413,
                "longitude": -98.49364,
                "altitude_m": 200.6,
            }
        ),
        encoding="utf-8",
    )

    deadline = time.time() + 0.5
    while logger.rows_written < 1 and time.time() < deadline:
        time.sleep(0.01)
    stop_event.set()
    logger.join(timeout=1)

    assert logger.rows_written >= 1
    assert logger.first_fix_time_ns == stale_pi_time_ns + 5000


def test_ouster_ptp_locked_accepts_slave_with_master():
    status = {
        "timestamp_mode": "TIME_FROM_PTP_1588",
        "ptp": {
            "port_data_set": {"port_state": "SLAVE"},
            "time_status_np": {"gm_present": True},
            "parent_data_set": {"grandmaster_identity": "00-11-22-33-44-55-66-77"},
        },
    }
    assert ptp_capture.ouster_ptp_locked(status) is True


def test_ouster_ptp_locked_rejects_non_ptp_mode():
    status = {
        "timestamp_mode": "TIME_FROM_INTERNAL_OSC",
        "ptp": {
            "port_data_set": {"port_state": "SLAVE"},
            "time_status_np": {"gm_present": True},
            "parent_data_set": {"grandmaster_identity": "00-11-22-33-44-55-66-77"},
        },
    }
    assert ptp_capture.ouster_ptp_locked(status) is False


def test_ouster_ptp_locked_rejects_missing_master_signal():
    status = {
        "timestamp_mode": "TIME_FROM_PTP_1588",
        "ptp": {
            "port_data_set": {"port_state": "LISTENING"},
            "time_status_np": {"gm_present": False},
            "parent_data_set": {"grandmaster_identity": ""},
        },
    }
    assert ptp_capture.ouster_ptp_locked(status) is False


def test_evaluate_ouster_sensor_time_alignment_accepts_small_delta():
    status = {
        "time": {
            "sensor": {
                "timestamp": {
                    "time": 1781280168.6174073,
                }
            }
        }
    }
    evaluation = ptp_capture.evaluate_ouster_sensor_time_alignment(status, pi_now_sec=1781280168.602482)
    assert evaluation["ready"] is True
    assert evaluation["delta_seconds"] > 0


def test_evaluate_ouster_sensor_time_alignment_rejects_large_delta():
    status = {
        "time": {
            "sensor": {
                "timestamp": {
                    "time": 1781032462.8541107,
                }
            }
        }
    }
    evaluation = ptp_capture.evaluate_ouster_sensor_time_alignment(status, pi_now_sec=1781278969.1754236)
    assert evaluation["ready"] is False
    assert "not aligned" in evaluation["summary"]


def test_choose_ptp_profile_reacquire_target_flips_default_and_gptp():
    assert ptp_capture.choose_ptp_profile_reacquire_target("default") == "gptp"
    assert ptp_capture.choose_ptp_profile_reacquire_target("gptp") == "default"
    assert ptp_capture.choose_ptp_profile_reacquire_target("custom") is None


def test_wait_for_ouster_sensor_time_alignment_recovers_stale_slave_time():
    stale_status = {
        "timestamp_mode": "TIME_FROM_PTP_1588",
        "ptp": {
            "port_data_set": {"port_state": "SLAVE"},
            "time_status_np": {"gm_present": True},
            "parent_data_set": {"grandmaster_identity": "00-11"},
        },
        "time": {"sensor": {"timestamp": {"time": 1781032462.8541107}}},
    }
    recovered_status = {
        "timestamp_mode": "TIME_FROM_PTP_1588",
        "ptp": {
            "port_data_set": {"port_state": "SLAVE"},
            "time_status_np": {"gm_present": True},
            "parent_data_set": {"grandmaster_identity": "00-11"},
        },
        "time": {"sensor": {"timestamp": {"time": 1781280168.6174073}}},
    }

    with (
        mock.patch.object(ptp_capture, "get_ouster_time_status", return_value=stale_status),
        mock.patch.object(ptp_capture, "force_ouster_ptp_reacquire", return_value=recovered_status) as reacquire,
        mock.patch.object(ptp_capture.time, "time", return_value=1781280168.602482),
    ):
        ready, result = ptp_capture.wait_for_ouster_sensor_time_alignment(
            host="192.168.50.2",
            stop_event=threading.Event(),
            timeout_sec=1,
            poll_sec=0,
            timestamp_mode="TIME_FROM_PTP_1588",
            ptp_profile="default",
        )

    assert ready is True
    assert result["ready"] is True
    reacquire.assert_called_once()


def test_collect_phc_alignment_reports_ready_when_delta_small():
    with mock.patch.object(
        ptp_capture,
        "run_command_capture",
        side_effect=[
            {"returncode": 0, "stdout": "phc_ctl[1]: clock time is 1781280168.602482101 or Fri Jun 12 11:02:48 2026", "stderr": "", "cmd": []},
            {"returncode": 0, "stdout": "phc_ctl[1]: clock time is 1781280168.617407242 or Fri Jun 12 11:02:48 2026", "stderr": "", "cmd": []},
        ],
    ):
        result = ptp_capture.collect_phc_alignment("eth0")

    assert result["ready"] is True
    assert abs(result["delta_seconds"]) < ptp_capture.PHC_ALIGNMENT_READY_DELTA_SEC


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
    evaluation = ptp_capture.evaluate_chrony_local_gps_pps_sync(snapshot)
    assert evaluation["ready"] is True
    assert evaluation["selected_source_name"] == "PPS"
    assert evaluation["local_source_states"] == {"GPS": "+", "PPS": "*"}


def test_evaluate_chrony_local_gps_pps_sync_rejects_network_selected_source():
    snapshot = {
        "waitsync": {"returncode": 0, "stdout": "ok", "stderr": ""},
        "sources": {
            "stdout": "\n".join(
                [
                    "#? GPS                           0   2     0     -   +0ns[   +0ns] +/-    0ns",
                    "#? PPS                           0   2     0     -   +0ns[   +0ns] +/-    0ns",
                    "^* 23.150.41.123                2   6   377    10   +1ms[  +1ms] +/-   22ms",
                ]
            )
        },
    }
    evaluation = ptp_capture.evaluate_chrony_local_gps_pps_sync(snapshot)
    assert evaluation["ready"] is False
    assert evaluation["selected_source_name"] == "23.150.41.123"
    assert "instead of GPS/PPS" in evaluation["summary"]


def test_main_discards_artifacts_when_waiting_for_gps_and_no_fix(tmp_path):
    class _FakeLogger:
        rows_written = 0
        first_fix_time_ns = None
        latest_fix = None

        def start(self):
            return None

        def join(self, timeout=None):
            return None

    cli = SimpleNamespace(
        wait_for_gps_fix=True,
        wait_for_pi_clock_sync=False,
        wait_for_ouster_ptp_lock=False,
        readiness_timeout_sec=None,
        lidar_mode=None,
        timestamp_mode=None,
        ptp_profile=None,
        gps_input_mode="bridge",
        gps_port=None,
        gps_baud=None,
        gpsd_host=None,
        gpsd_port=None,
        bridge_fix_path=str(tmp_path / "latest_fix.json"),
        ouster_host=None,
    )

    with (
        mock.patch.object(ptp_capture, "OUTPUT_DIR", str(tmp_path)),
        mock.patch.object(ptp_capture, "parse_cli_args", return_value=cli),
        mock.patch.object(ptp_capture, "build_gps_logger", return_value=_FakeLogger()),
        mock.patch.object(ptp_capture, "wait_for_first_gps_fix", return_value=False),
        mock.patch.object(ptp_capture, "configure_ouster_sensor", return_value=None),
        mock.patch.object(ptp_capture.signal, "signal", return_value=None),
        mock.patch.object(ptp_capture.time, "sleep", return_value=None),
    ):
        ptp_capture.main()

    assert list(tmp_path.glob("raw_gps_*.csv")) == []
    assert list(tmp_path.glob("capture_manifest_*.json")) == []
