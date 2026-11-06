import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from lidar_rpi_gps_pipeline.jobs.overlap_flags import overlap_mask  # noqa: E402


def test_the_closer_pass_is_kept_and_the_other_flagged() -> None:
    rng = np.random.default_rng(0)
    xy = rng.uniform(0, 1.9, (200, 2))
    drive = np.zeros(200, dtype=np.int64)
    # One drive, two passes 300 s apart; the second saw the square from closer.
    times = np.r_[np.full(100, 10.0), np.full(100, 310.0)]
    ranges = np.r_[np.full(100, 20.0), np.full(100, 8.0)]

    flagged = overlap_mask(xy, times, ranges, drive)

    assert flagged[:100].all() and not flagged[100:].any()


def test_two_drives_and_a_square_seen_once() -> None:
    xy = np.array([[0.5, 0.5]] * 4 + [[10.5, 0.5]] * 2)
    drive = np.array([0, 0, 1, 1, 1, 1])
    times = np.array([0.0, 1.0, 0.0, 1.0, 5.0, 6.0])
    ranges = np.array([5.0, 5.0, 15.0, 15.0, 30.0, 30.0])

    flagged = overlap_mask(xy, times, ranges, drive)

    # Shared square: drive 0 is closer. Square seen by drive 1 alone: nothing to flag, however far.
    assert flagged.tolist() == [False, False, True, True, False, False]


def test_a_continuous_pass_is_not_split() -> None:
    xy = np.zeros((50, 2)) + 0.5
    times = np.arange(50, dtype=float)  # one pass, 1 s steps
    ranges = np.linspace(5, 40, 50)

    assert not overlap_mask(xy, times, ranges, np.zeros(50, dtype=np.int64)).any()
