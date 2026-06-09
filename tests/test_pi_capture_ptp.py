import importlib.util
from pathlib import Path


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


def test_resolve_range_defaults_uses_script_defaults():
    assert ptp_capture.resolve_range_defaults(None, None) == (1.0, 150.0)


def test_resolve_range_defaults_rejects_inverted_values():
    try:
        ptp_capture.resolve_range_defaults(50.0, 10.0)
    except ValueError as exc:
        assert "Maximum range must be greater than minimum range" in str(exc)
    else:
        raise AssertionError("Expected ValueError for inverted range values")


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
