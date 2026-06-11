import importlib.util
import json
from pathlib import Path
import threading
import time


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
