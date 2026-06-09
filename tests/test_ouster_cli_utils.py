import importlib.util
from pathlib import Path
from unittest import mock


def _load_module():
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "ouster_cli_utils.py"
    spec = importlib.util.spec_from_file_location("ouster_cli_utils", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


ouster_cli_utils = _load_module()


def test_resolve_ouster_cli_prefers_env_override(tmp_path):
    override = tmp_path / "ouster-cli"
    override.write_text("", encoding="utf-8")
    with mock.patch.dict(ouster_cli_utils.os.environ, {"OUSTER_CLI_BIN": str(override)}, clear=False):
        assert ouster_cli_utils.resolve_ouster_cli_executable() == str(override)


def test_resolve_ouster_cli_uses_sibling_of_python_when_path_missing(tmp_path):
    fake_python = tmp_path / "python"
    fake_python.write_text("", encoding="utf-8")
    sibling_cli = tmp_path / "ouster-cli"
    sibling_cli.write_text("", encoding="utf-8")

    with (
        mock.patch.dict(ouster_cli_utils.os.environ, {}, clear=True),
        mock.patch.object(ouster_cli_utils.shutil, "which", return_value=None),
        mock.patch.object(ouster_cli_utils.sys, "executable", str(fake_python)),
    ):
        assert ouster_cli_utils.resolve_ouster_cli_executable() == str(sibling_cli)


def test_resolve_ouster_cli_prefers_venv_sibling_before_resolved_symlink_target(tmp_path):
    venv_bin = tmp_path / ".venv" / "bin"
    venv_bin.mkdir(parents=True)
    real_bin = tmp_path / "real-bin"
    real_bin.mkdir()

    real_python = real_bin / "python3.13"
    real_python.write_text("", encoding="utf-8")
    venv_python = venv_bin / "python"
    venv_python.symlink_to(real_python)

    venv_cli = venv_bin / "ouster-cli"
    venv_cli.write_text("", encoding="utf-8")

    with (
        mock.patch.dict(ouster_cli_utils.os.environ, {}, clear=True),
        mock.patch.object(ouster_cli_utils.shutil, "which", return_value=None),
        mock.patch.object(ouster_cli_utils.sys, "executable", str(venv_python)),
    ):
        assert ouster_cli_utils.resolve_ouster_cli_executable() == str(venv_cli)
