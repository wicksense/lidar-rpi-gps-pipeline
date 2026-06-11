import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from types import ModuleType
from unittest import mock


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "pi_capture_raw.py"
    spec = importlib.util.spec_from_file_location("pi_capture_raw", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    fake_pynmea2 = ModuleType("pynmea2")
    fake_pynmea2.ParseError = Exception
    fake_pynmea2.RMC = type("RMC", (), {})
    fake_pynmea2.GGA = type("GGA", (), {})
    fake_pynmea2.parse = lambda _text: None

    fake_serial = ModuleType("serial")
    fake_serial.SerialException = Exception
    fake_serial.Serial = object

    fake_pyproj = ModuleType("pyproj")

    class _FakeTransformer:
        @staticmethod
        def from_crs(*_args, **_kwargs):
            class _Instance:
                @staticmethod
                def transform(lon, lat):
                    return lon, lat

            return _Instance()

    fake_pyproj.Transformer = _FakeTransformer

    with mock.patch.dict(
        sys.modules,
        {
            "pynmea2": fake_pynmea2,
            "serial": fake_serial,
            "pyproj": fake_pyproj,
        },
    ):
        spec.loader.exec_module(module)
    return module


raw_capture = _load_module()


def test_main_discards_artifacts_when_waiting_for_gps_and_no_fix(tmp_path):
    class _FakeLogger:
        rows_written = 0
        first_fix_time_ns = None

        def start(self):
            return None

        def join(self, timeout=None):
            return None

    cli = SimpleNamespace(
        wait_for_gps_fix=True,
        lidar_mode=None,
    )

    with (
        mock.patch.object(raw_capture, "OUTPUT_DIR", str(tmp_path)),
        mock.patch.object(raw_capture, "parse_cli_args", return_value=cli),
        mock.patch.object(raw_capture, "GpsLogger", return_value=_FakeLogger()),
        mock.patch.object(raw_capture, "wait_for_first_gps_fix", return_value=False),
        mock.patch.object(raw_capture, "configure_ouster_sensor", return_value=None),
        mock.patch.object(raw_capture.signal, "signal", return_value=None),
        mock.patch.object(raw_capture.time, "sleep", return_value=None),
    ):
        raw_capture.main()

    assert list(tmp_path.glob("raw_gps_*.csv")) == []
    assert list(tmp_path.glob("capture_manifest_*.json")) == []
