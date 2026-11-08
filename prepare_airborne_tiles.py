#!/usr/bin/env python3
"""CLI wrapper: Prepare downloaded airborne lidar tiles (StratMap) for placement."""

import os
import sys


REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from lidar_rpi_gps_pipeline.jobs.airborne_tiles import main


if __name__ == "__main__":
    main()
