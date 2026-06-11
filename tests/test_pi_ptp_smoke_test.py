import importlib.util
from pathlib import Path


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "pi_ptp_smoke_test.py"
    spec = importlib.util.spec_from_file_location("pi_ptp_smoke_test", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


ptp_smoke = _load_module()


def test_extract_sensor_timestamp_seconds_reads_nested_time_value():
    status = {
        "time": {
            "sensor": {
                "timestamp": {
                    "time": 1748546500.6490452,
                }
            }
        }
    }
    assert ptp_smoke.extract_sensor_timestamp_seconds(status) == 1748546500.6490452


def test_sensor_minus_pi_ms_reports_signed_delta():
    status = {
        "time": {
            "sensor": {
                "timestamp": {
                    "time": 200.050,
                }
            }
        }
    }
    delta_ms = ptp_smoke.sensor_minus_pi_ms(status, 200.000)
    assert round(delta_ms, 3) == 50.0
