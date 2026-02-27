#!/usr/bin/env python3
"""Launch desktop GUI for LiDAR RPi GPS Pipeline."""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from lidar_rpi_gps_pipeline.gui import run_gui


if __name__ == "__main__":
    try:
        run_gui()
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        raise SystemExit(1) from e
