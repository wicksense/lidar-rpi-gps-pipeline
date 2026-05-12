from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from lidar_rpi_gps_pipeline.jobs.offline_fusion import (
    _extract_session_id_from_gps_csv_path,
    _filter_raw_paths_by_session,
    _lookup_pose_columns,
    _trim_query_to_gps_range,
    choose_time_mode,
    filter_gps_outliers_by_speed,
    load_gps_for_interpolation,
)


def test_choose_time_mode_relative_start_offsets_both_series_from_lidar_start() -> None:
    lidar_ts = np.asarray([1_000, 1_050, 1_100], dtype=np.int64)
    gps_ts = np.asarray([1_020, 1_070, 1_120], dtype=np.int64)

    lidar_adj, gps_adj, mode = choose_time_mode(lidar_ts, gps_ts, "relative_start")

    assert mode == "relative_start"
    assert np.array_equal(lidar_adj, np.asarray([0, 50, 100], dtype=np.int64))
    assert np.array_equal(gps_adj, np.asarray([20, 70, 120], dtype=np.int64))


def test_choose_time_mode_auto_prefers_unix_ns_for_overlapping_epoch_series() -> None:
    lidar_ts = np.asarray(
        [1_773_758_296_000_000_000, 1_773_758_297_000_000_000],
        dtype=np.int64,
    )
    gps_ts = np.asarray(
        [1_773_758_295_500_000_000, 1_773_758_298_000_000_000],
        dtype=np.int64,
    )

    lidar_adj, gps_adj, mode = choose_time_mode(lidar_ts, gps_ts, "auto")

    assert mode == "unix_ns"
    assert np.array_equal(lidar_adj, lidar_ts)
    assert np.array_equal(gps_adj, gps_ts)


def test_load_gps_for_interpolation_drops_non_finite_rows_and_duplicate_timestamps(
    tmp_path: Path,
) -> None:
    gps_csv = tmp_path / "gps.csv"
    pd.DataFrame(
        [
            {
                "gps_epoch_ns": 300,
                "pi_time_ns": 30,
                "easting": 630_000.0,
                "northing": 3_900_000.0,
                "altitude_m": 120.0,
            },
            {
                "gps_epoch_ns": 100,
                "pi_time_ns": 10,
                "easting": 629_999.0,
                "northing": 3_899_999.0,
                "altitude_m": math.nan,
            },
            {
                "gps_epoch_ns": 300,
                "pi_time_ns": 31,
                "easting": 630_100.0,
                "northing": 3_900_100.0,
                "altitude_m": 121.0,
            },
            {
                "gps_epoch_ns": 200,
                "pi_time_ns": 20,
                "easting": 630_010.0,
                "northing": 3_900_010.0,
                "altitude_m": float("inf"),
            },
            {
                "gps_epoch_ns": 250,
                "pi_time_ns": 25,
                "easting": 630_020.0,
                "northing": 3_900_020.0,
                "altitude_m": 122.5,
            },
        ]
    ).to_csv(gps_csv, index=False)

    gps_df, gps_time_col = load_gps_for_interpolation(str(gps_csv), "auto")

    assert gps_time_col == "gps_epoch_ns"
    assert gps_df[gps_time_col].tolist() == [250, 300]
    assert gps_df["altitude_m"].tolist() == [122.5, 120.0]


def test_load_gps_for_interpolation_requires_two_finite_rows(tmp_path: Path) -> None:
    gps_csv = tmp_path / "gps.csv"
    pd.DataFrame(
        [
            {
                "gps_epoch_ns": 100,
                "pi_time_ns": 10,
                "easting": 630_000.0,
                "northing": 3_900_000.0,
                "altitude_m": math.nan,
            },
            {
                "gps_epoch_ns": 200,
                "pi_time_ns": 20,
                "easting": 630_100.0,
                "northing": 3_900_100.0,
                "altitude_m": 120.0,
            },
        ]
    ).to_csv(gps_csv, index=False)

    with pytest.raises(ValueError, match="Need at least 2 finite GPS rows"):
        load_gps_for_interpolation(str(gps_csv), "auto")


def test_trim_query_to_gps_range_trims_edges_and_returns_mask() -> None:
    query_t = np.asarray([5, 10, 20, 30, 35], dtype=np.int64)
    ref_t = np.asarray([10, 30], dtype=np.int64)

    trimmed, mask = _trim_query_to_gps_range(
        query_t,
        ref_t,
        label="gps fusion",
        min_points=2,
    )

    assert np.array_equal(trimmed, np.asarray([10, 20, 30], dtype=np.int64))
    assert np.array_equal(mask, np.asarray([False, True, True, True, False]))


def test_trim_query_to_gps_range_raises_when_too_few_points_remain() -> None:
    query_t = np.asarray([5, 10, 35], dtype=np.int64)
    ref_t = np.asarray([10, 30], dtype=np.int64)

    with pytest.raises(ValueError, match="only 1 samples remain after GPS-range trimming"):
        _trim_query_to_gps_range(
            query_t,
            ref_t,
            label="gps fusion",
            min_points=2,
        )


def test_extract_session_id_from_gps_csv_path_standard_name() -> None:
    gps_csv = "/tmp/raw_gps_20260317_094022.csv"
    assert _extract_session_id_from_gps_csv_path(gps_csv) == "20260317_094022"


def test_extract_session_id_from_gps_csv_path_fallback_pattern() -> None:
    gps_csv = "/tmp/gps_run_20260317_101058_processed.csv"
    assert _extract_session_id_from_gps_csv_path(gps_csv) == "20260317_101058"


def test_filter_raw_paths_by_session_keeps_only_requested_session() -> None:
    raw_paths = [
        "/data/raw_lidar_20260317_093703_chunk0000.pcap",
        "/data/raw_lidar_20260317_093703_chunk0001.pcap",
        "/data/raw_lidar_20260317_094022_chunk0000.pcap",
    ]
    filtered = _filter_raw_paths_by_session(raw_paths, "20260317_094022")
    assert filtered == ["/data/raw_lidar_20260317_094022_chunk0000.pcap"]


def test_filter_raw_paths_by_session_raises_when_no_match() -> None:
    raw_paths = ["/data/raw_lidar_20260317_093703_chunk0000.pcap"]
    with pytest.raises(ValueError, match="No raw LiDAR files matched session filter"):
        _filter_raw_paths_by_session(raw_paths, "20260317_094022")


def test_lookup_pose_columns_marks_large_drift_as_outlier_instead_of_raising() -> None:
    pose_timestamps_ns = np.asarray([1_000_000_000, 1_000_100_000], dtype=np.int64)
    column_poses = np.repeat(np.eye(4, dtype=np.float64)[np.newaxis, :, :], 2, axis=0)
    scan_timestamps_ns = np.asarray(
        [
            1_000_000_000,  # exact
            1_000_050_000,  # near
            1_300_000_000,  # too far (drift > 100 ms)
        ],
        dtype=np.int64,
    )

    pose_cols, inlier_mask, stats = _lookup_pose_columns(
        scan_timestamps_ns,
        pose_timestamps_ns,
        column_poses,
    )

    assert pose_cols.shape == (3, 4, 4)
    assert np.array_equal(inlier_mask, np.asarray([True, True, False]))
    assert stats["columns"] == 3
    assert stats["valid_columns"] == 3
    assert stats["inlier_columns"] == 2
    assert stats["dropped_columns"] == 1


def test_filter_gps_outliers_by_speed_drops_jump_and_keeps_recovered_track() -> None:
    t0 = 1_773_758_460_000_000_000
    gps_df = pd.DataFrame(
        [
            {"gps_epoch_ns": t0 + 0 * 1_000_000_000, "easting": 0.0, "northing": 0.0},
            {"gps_epoch_ns": t0 + 1 * 1_000_000_000, "easting": 2.0, "northing": 0.0},
            {"gps_epoch_ns": t0 + 2 * 1_000_000_000, "easting": 4.0, "northing": 0.0},
            {"gps_epoch_ns": t0 + 3 * 1_000_000_000, "easting": 400.0, "northing": 0.0},
            {"gps_epoch_ns": t0 + 4 * 1_000_000_000, "easting": 6.0, "northing": 0.0},
        ]
    )

    filtered, stats = filter_gps_outliers_by_speed(
        gps_df,
        "gps_epoch_ns",
        max_speed_mps=20.0,
    )

    assert filtered["easting"].tolist() == [0.0, 2.0, 4.0, 6.0]
    assert stats["input_rows"] == 5
    assert stats["kept_rows"] == 4
    assert stats["dropped_rows"] == 1
    assert stats["dropped_speed"] == 1
