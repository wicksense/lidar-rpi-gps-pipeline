#!/usr/bin/env python3
"""
Helpers for locating `ouster-cli` in environments where PATH may be minimal,
such as systemd services started from a Python virtual environment.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import sys


def resolve_ouster_cli_executable() -> str:
    """
    Resolve the best `ouster-cli` executable for the current runtime.

    Search order:
    1. `OUSTER_CLI_BIN` environment override
    2. current PATH via `shutil.which`
    3. sibling `ouster-cli` next to the active Python interpreter
    4. common system install locations
    """
    env_override = os.environ.get("OUSTER_CLI_BIN", "").strip()
    if env_override:
        override_path = Path(env_override)
        if override_path.exists():
            return str(override_path)
        raise RuntimeError(f"OUSTER_CLI_BIN is set but does not exist: {env_override}")

    which_path = shutil.which("ouster-cli")
    if which_path:
        return which_path

    executable_path = Path(sys.executable)
    sibling_candidates = [
        executable_path.with_name("ouster-cli"),
        executable_path.resolve().with_name("ouster-cli"),
    ]
    for sibling in sibling_candidates:
        if sibling.exists():
            return str(sibling)

    for candidate in ("/usr/local/bin/ouster-cli", "/usr/bin/ouster-cli"):
        if Path(candidate).exists():
            return candidate

    raise RuntimeError(
        "Could not find `ouster-cli`. If you are running from systemd with a virtualenv, "
        "install the Ouster SDK in that venv or set OUSTER_CLI_BIN explicitly."
    )
