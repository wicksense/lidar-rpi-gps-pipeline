import importlib.util
import json
from pathlib import Path


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "ublox_i2c_chrony_bridge.py"
    spec = importlib.util.spec_from_file_location("ublox_i2c_chrony_bridge", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


bridge = _load_module()


def test_parse_gga_fix_decodes_coordinates_and_quality():
    sentence = "$GPGGA,194410.00,2925.4472,N,09829.6178,W,4,18,0.6,200.5,M,-24.3,M,,*59"
    fix = bridge.parse_gga_fix(sentence, bridge.dt.date(2026, 6, 9))
    assert fix is not None
    assert abs(fix["latitude"] - 29.42412) < 1e-6
    assert abs(fix["longitude"] + 98.49363) < 1e-6
    assert fix["gps_quality"] == 4
    assert fix["num_sats"] == 18


def test_fix_publisher_writes_latest_fix_json(tmp_path):
    fix_path = tmp_path / "latest_fix.json"
    publisher = bridge.FixPublisher(str(fix_path))
    publisher.update_from_nmea("$GPRMC,194410.00,A,2925.4472,N,09829.6178,W,0.1,0.0,090626,,,A*44")
    publisher.update_from_nmea("$GPGGA,194410.00,2925.4472,N,09829.6178,W,4,18,0.6,200.5,M,-24.3,M,,*59")

    payload = json.loads(fix_path.read_text(encoding="utf-8"))
    assert payload["sequence"] == 1
    assert payload["gps_quality"] == 4
    assert payload["num_sats"] == 18
    assert payload["gps_utc_time"].startswith("19:44:10")
