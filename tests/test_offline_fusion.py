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
    MAP_ATTR_COLUMNS,
    _CaptureGapTracker,
    _extract_scan_column_poses,
    _extract_session_id_from_gps_csv_path,
    _filter_raw_paths_by_session,
    _find_metadata_json_candidates,
    _lookup_pose_columns,
    _new_las_header,
    _rigid_transform_matrix,
    _scan_valid_timestamp_count,
    _trajectory_csv_path,
    _trim_query_to_gps_range,
    _updated_has_any_valid_timestamps,
    _write_trajectory_csv,
    choose_time_mode,
    expand_raw_lidar_inputs,
    filter_gps_outliers_by_speed,
    load_gps_for_interpolation,
    preferred_gps_time_column_from_manifest,
)


def test_choose_time_mode_relative_start_offsets_each_series_from_its_own_start() -> None:
    lidar_ts = np.asarray([1_000, 1_050, 1_100], dtype=np.int64)
    gps_ts = np.asarray([1_020, 1_070, 1_120], dtype=np.int64)

    lidar_adj, gps_adj, mode = choose_time_mode(lidar_ts, gps_ts, "relative_start")

    assert mode == "relative_start"
    assert np.array_equal(lidar_adj, np.asarray([0, 50, 100], dtype=np.int64))
    assert np.array_equal(gps_adj, np.asarray([0, 50, 100], dtype=np.int64))


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


def test_load_gps_for_interpolation_respects_preferred_auto_columns(tmp_path: Path) -> None:
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
                "gps_epoch_ns": 400,
                "pi_time_ns": 40,
                "easting": 630_010.0,
                "northing": 3_900_010.0,
                "altitude_m": 120.5,
            },
        ]
    ).to_csv(gps_csv, index=False)

    gps_df, gps_time_col = load_gps_for_interpolation(
        str(gps_csv),
        "auto",
        preferred_auto_columns=["pi_time_ns", "gps_epoch_ns"],
    )

    assert gps_time_col == "pi_time_ns"
    assert gps_df[gps_time_col].tolist() == [30, 40]


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


def test_expand_raw_lidar_inputs_directory_sorts_split_chunks_chronologically(tmp_path: Path) -> None:
    names = [
        "raw_lidar_20260701_191429_chunk0000-2.pcap",
        "raw_lidar_20260701_191429_chunk0000-1.pcap",
        "raw_lidar_20260701_191429_chunk0000.pcap",
    ]
    for name in names:
        (tmp_path / name).write_bytes(b"")

    expanded = expand_raw_lidar_inputs([str(tmp_path)])

    assert expanded == [
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000.pcap"),
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000-1.pcap"),
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000-2.pcap"),
    ]


def test_expand_raw_lidar_inputs_glob_sorts_split_chunks_chronologically(tmp_path: Path) -> None:
    names = [
        "raw_lidar_20260701_191429_chunk0000-2.pcap",
        "raw_lidar_20260701_191429_chunk0000-1.pcap",
        "raw_lidar_20260701_191429_chunk0000.pcap",
    ]
    for name in names:
        (tmp_path / name).write_bytes(b"")

    expanded = expand_raw_lidar_inputs([str(tmp_path / "*.pcap")])

    assert expanded == [
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000.pcap"),
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000-1.pcap"),
        str(tmp_path / "raw_lidar_20260701_191429_chunk0000-2.pcap"),
    ]


class _FakeScan:
    def __init__(self, timestamps: list[int]) -> None:
        self.timestamp = timestamps
        self.fields = []
        self.h = 1
        self.w = len(timestamps)

    def field(self, _name: str) -> None:
        raise KeyError


def test_scan_valid_timestamp_count_counts_positive_entries() -> None:
    scan = _FakeScan([0, 10, 20, 0, -1])
    assert _scan_valid_timestamp_count(scan) == 2


def test_updated_has_any_valid_timestamps_ignores_empty_scans() -> None:
    assert _updated_has_any_valid_timestamps([_FakeScan([0, 0]), _FakeScan([0, 5, 0])]) is True
    assert _updated_has_any_valid_timestamps([_FakeScan([0, 0])]) is False


def test_preferred_gps_time_column_from_manifest_prefers_pi_time_for_ptp_architecture() -> None:
    manifest = {
        "timing_architecture": "gps_pps_to_pi__chrony_to_pi_clock__ptp_to_ouster",
    }
    assert preferred_gps_time_column_from_manifest(manifest) == "pi_time_ns"


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


def test_capture_gap_tracker_reports_break_between_scans() -> None:
    tracker = _CaptureGapTracker(threshold_s=0.5)

    assert tracker.observe(_FakeScan([1_000_000_000, 1_050_000_000])) is None
    # Next scan continues immediately: no gap.
    assert tracker.observe(_FakeScan([1_100_000_000, 1_150_000_000])) is None
    # Then two seconds of missing data.
    gap = tracker.observe(_FakeScan([3_150_000_000, 3_200_000_000]))

    assert gap is not None
    assert gap["gap_s"] == pytest.approx(2.0)
    summary = tracker.summary()
    assert summary["count"] == 1
    assert summary["total_s"] == pytest.approx(2.0)


def test_capture_gap_tracker_ignores_scans_without_valid_timestamps() -> None:
    tracker = _CaptureGapTracker(threshold_s=0.5)

    tracker.observe(_FakeScan([1_000_000_000]))
    # A scan the SDK invalidated carries zeroed timestamps; it must not be read as a
    # jump back to the epoch, which would otherwise register a huge bogus gap.
    assert tracker.observe(_FakeScan([0, 0])) is None
    assert tracker.observe(_FakeScan([1_100_000_000])) is None
    assert tracker.summary()["count"] == 0


def test_find_metadata_json_candidates_skips_empty_sidecar_and_uses_sibling(
    tmp_path: Path,
) -> None:
    pcap = tmp_path / "raw_lidar_20260826_094253_chunk0000.pcap"
    pcap.write_bytes(b"\x00")
    # The sidecar belonging to this chunk survived the transfer as zero bytes.
    (tmp_path / "raw_lidar_20260826_094253_chunk0000_0.json").write_text("")
    sibling = tmp_path / "raw_lidar_20260826_094253_chunk0000-1_0.json"
    sibling.write_text('{"sensor_info": {}}')

    candidates = _find_metadata_json_candidates(str(pcap))

    assert str(sibling) in candidates
    assert all(Path(p).stat().st_size > 0 for p in candidates)


def test_find_metadata_json_candidates_prefers_own_sidecar_when_populated(
    tmp_path: Path,
) -> None:
    pcap = tmp_path / "raw_lidar_20260826_094253_chunk0000.pcap"
    pcap.write_bytes(b"\x00")
    own = tmp_path / "raw_lidar_20260826_094253_chunk0000_0.json"
    own.write_text('{"sensor_info": {"own": true}}')
    (tmp_path / "raw_lidar_20260826_094253_chunk0000-1_0.json").write_text("{}")

    assert _find_metadata_json_candidates(str(pcap))[0] == str(own)


def test_write_trajectory_csv_strides_and_keeps_pose_columns(tmp_path: Path) -> None:
    count = 100
    timestamps = np.arange(count, dtype=np.int64) * 1_000_000 + 1_787_755_378_000_000_000
    poses = np.tile(np.eye(4, dtype=np.float64), (count, 1, 1))
    poses[:, 0, 3] = np.arange(count, dtype=np.float64)

    path = _write_trajectory_csv(str(tmp_path / "out.las"), timestamps, poses, max_rows=10)

    assert path == _trajectory_csv_path(str(tmp_path / "out.las"))
    frame = pd.read_csv(path)
    assert list(frame.columns) == [
        "time_ns",
        "time_s",
        "x",
        "y",
        "z",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]
    assert len(frame) == 10
    assert frame["x"].tolist() == [float(i) for i in range(0, 100, 10)]


def test_write_trajectory_csv_returns_none_on_mismatched_inputs(tmp_path: Path) -> None:
    poses = np.tile(np.eye(4, dtype=np.float64), (3, 1, 1))
    timestamps = np.asarray([1, 2], dtype=np.int64)

    assert _write_trajectory_csv(str(tmp_path / "out.las"), timestamps, poses) is None


def test_new_las_header_point_format_8_carries_nir_and_extra_dims() -> None:
    header = _new_las_header(
        32614,
        0.0,
        0.0,
        0.0,
        point_format=8,
        extra_dims=[("range_m", "f4", "Sensor-to-point range in metres")],
    )

    names = {d.name for d in header.point_format.dimensions}
    assert "nir" in names
    assert "range_m" in names
    assert "gps_time" in names


def test_map_attr_columns_match_documented_layout() -> None:
    # The replay builds this array positionally and the LAS writer reads it back the
    # same way, so the order is load-bearing.
    assert MAP_ATTR_COLUMNS == (
        "time_s",
        "reflectivity",
        "near_ir",
        "range_m",
        "beam_row",
    )


def test_rigid_transform_matrix_places_without_reshaping() -> None:
    angle = np.deg2rad(30.0)
    rotation = np.array(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    )
    transform = _rigid_transform_matrix(rotation, np.array([100.0, 200.0]), 5.0)

    assert transform.shape == (4, 4)
    assert transform[2, 2] == pytest.approx(1.0)
    assert transform[:, 3].tolist() == pytest.approx([100.0, 200.0, 5.0, 1.0])

    # Applying it to two points must preserve the distance between them, which is
    # the whole reason this backend exists.
    pts = np.array([[0.0, 0.0, 0.0, 1.0], [3.0, 4.0, 12.0, 1.0]]).T
    moved = (transform @ pts).T[:, :3]
    assert np.linalg.norm(moved[1] - moved[0]) == pytest.approx(13.0)


def test_extract_scan_column_poses_drops_invalid_columns() -> None:
    class _PoseScan:
        def __init__(self) -> None:
            self.timestamp = np.array([100, 0, 300], dtype=np.int64)
            self.pose = np.stack([np.eye(4) * k for k in (1.0, 2.0, 3.0)])

    ts, poses = _extract_scan_column_poses(_PoseScan())

    assert ts.tolist() == [100, 300]
    assert poses.shape == (2, 4, 4)
    assert poses[0][0, 0] == pytest.approx(1.0)
    assert poses[1][0, 0] == pytest.approx(3.0)


def test_extract_scan_column_poses_rejects_mismatched_layout() -> None:
    class _BadScan:
        timestamp = np.array([1, 2, 3], dtype=np.int64)
        pose = np.stack([np.eye(4), np.eye(4)])

    with pytest.raises(ValueError, match="does not match"):
        _extract_scan_column_poses(_BadScan())
