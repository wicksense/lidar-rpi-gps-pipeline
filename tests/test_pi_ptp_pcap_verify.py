import importlib.util
from pathlib import Path


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "pi_ptp_pcap_verify.py"
    spec = importlib.util.spec_from_file_location("pi_ptp_pcap_verify", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


pcap_verify = _load_module()


def test_evaluate_pcap_time_window_accepts_current_time_domain():
    result = pcap_verify.evaluate_pcap_time_window(
        before_sensor_sec=1781280168.60,
        after_sensor_sec=1781280175.10,
        first_lidar_ns=int(1781280169.20 * 1e9),
        last_lidar_ns=int(1781280174.80 * 1e9),
        slop_sec=5.0,
    )
    assert result["ready"] is True
    assert "fall inside" in result["summary"]


def test_evaluate_pcap_time_window_rejects_old_epoch():
    result = pcap_verify.evaluate_pcap_time_window(
        before_sensor_sec=1781280168.60,
        after_sensor_sec=1781280175.10,
        first_lidar_ns=int(1781032462.85 * 1e9),
        last_lidar_ns=int(1781032468.10 * 1e9),
        slop_sec=5.0,
    )
    assert result["ready"] is False
    assert "do not match" in result["summary"]


def test_choose_capture_attempts_retries_short_windows():
    assert pcap_verify.choose_capture_attempts(5) == [5, 10]
    assert pcap_verify.choose_capture_attempts(10) == [10]


def test_wait_for_capture_file_reports_missing_path(tmp_path):
    result = pcap_verify.wait_for_capture_file(str(tmp_path / "missing.pcap"), timeout_sec=0.01)
    assert result["ready"] is False
    assert "not created" in result["summary"]


def test_wait_for_capture_file_accepts_written_file(tmp_path):
    pcap_path = tmp_path / "verify_capture.pcap"
    pcap_path.write_bytes(b"pcap")
    result = pcap_verify.wait_for_capture_file(str(pcap_path), timeout_sec=0.01)
    assert result["ready"] is True
    assert result["size_bytes"] == 4
