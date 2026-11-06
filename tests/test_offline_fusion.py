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
    _estimate_gps_time_offset,
    _extract_session_id_from_gps_csv_path,
    _find_loop_pairs,
    _find_nearby_pairs,
    _find_transient_pose_spikes,
    _local_map_spans,
    _path_along,
    _repair_pose_spikes,
    _filter_raw_paths_by_session,
    _find_metadata_json_candidates,
    _lookup_pose_columns,
    _new_las_header,
    _scan_column_validity,
    _apply_transform,
    _fit_vertical_tilt,
    _gravity_place_warp,
    _level_poses_on_gravity,
    _level_transform,
    _tie_gravity_corrections,
    _rigid_transform_matrix,
    _rotvec_to_matrix,
    _yaw_rotation_3d,
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
    def __init__(
        self,
        timestamps: list[int],
        frame_id: int | None = None,
        status: list[int] | None = None,
    ) -> None:
        self.timestamp = timestamps
        self.fields = []
        self.h = 1
        self.w = len(timestamps)
        # Only set when the test needs them, so the attribute-absent fallbacks
        # stay covered by the tests that pass timestamps alone.
        if frame_id is not None:
            self.frame_id = frame_id
        if status is not None:
            self.status = status

    def field(self, _name: str) -> None:
        raise KeyError


def test_scan_valid_timestamp_count_counts_positive_entries() -> None:
    scan = _FakeScan([0, 10, 20, 0, -1])
    assert _scan_valid_timestamp_count(scan) == 2


def test_updated_has_any_valid_timestamps_ignores_empty_scans() -> None:
    assert _updated_has_any_valid_timestamps([_FakeScan([0, 0]), _FakeScan([0, 5, 0])]) is True
    assert _updated_has_any_valid_timestamps([_FakeScan([0, 0])]) is False


def test_preferred_gps_time_column_from_manifest_prefers_fix_epoch_for_ptp_architecture() -> None:
    manifest = {
        "timing_architecture": "gps_pps_to_pi__chrony_to_pi_clock__ptp_to_ouster",
    }
    assert preferred_gps_time_column_from_manifest(manifest) == "gps_epoch_ns"


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


def test_scan_column_validity_prefers_the_status_field() -> None:
    # A column can hold a timestamp while the sensor still marks it invalid, so
    # status wins where it is available.
    assert _scan_column_validity(_FakeScan([1, 2, 3, 4], status=[1, 1, 0, 0])) == (2, 4)
    # With no status field, a zeroed timestamp is the signal instead.
    assert _scan_column_validity(_FakeScan([1, 0, 3, 0])) == (2, 4)


def test_capture_gap_tracker_flags_scans_missing_columns() -> None:
    tracker = _CaptureGapTracker(threshold_s=0.5)
    tracker.start_file("/data/raw_lidar_chunk0000.pcap")

    tracker.observe(_FakeScan([1_000_000_000, 1_000_050_000, 1_000_100_000], frame_id=10))
    # Two of this frame's three columns never arrived.
    tracker.observe(_FakeScan([1_000_150_000, 0, 0], frame_id=11))
    tracker.observe(_FakeScan([1_000_200_000, 1_000_250_000, 1_000_300_000], frame_id=12))

    summary = tracker.summary()
    assert summary["count"] == 0
    assert summary["scans_seen"] == 3
    assert summary["incomplete_scan_count"] == 1
    assert summary["missing_columns"] == 2

    entry = summary["incomplete_scans"][0]
    assert entry["frame_id"] == 11
    assert entry["valid_columns"] == 1
    assert entry["total_columns"] == 3
    assert entry["missing_columns"] == 2
    assert entry["at_file_boundary"] is False


def test_capture_gap_tracker_sees_dropout_that_stays_under_the_time_threshold() -> None:
    """
    The 2026-07-21 defect: 408 packets were lost in a 0.32 s hole. The hole sat
    below CAPTURE_GAP_THRESHOLD_S, nothing was reported, and the merged trajectory
    came out 28 % longer than the GPS track. Column counts have to catch it.
    """
    tracker = _CaptureGapTracker(threshold_s=0.5)
    tracker.start_file("/data/raw_lidar_chunk0000-1.pcap")

    tracker.observe(_FakeScan([1_000_000_000, 1_000_050_000], frame_id=764))
    # The SDK emits zeroed scans for frames whose packets were all lost.
    tracker.observe(_FakeScan([0, 0], frame_id=765))
    tracker.observe(_FakeScan([0, 0], frame_id=766))
    tracker.observe(_FakeScan([1_320_000_000, 1_320_050_000], frame_id=767))

    summary = tracker.summary()
    # 0.32 s is under the threshold, so the time check stays silent ...
    assert summary["count"] == 0
    # ... but the missing columns are counted exactly.
    assert summary["incomplete_scan_count"] == 2
    assert summary["missing_columns"] == 4
    assert [s["frame_id"] for s in summary["incomplete_scans"]] == [765, 766]


def test_capture_gap_tracker_labels_chunk_boundary_scans() -> None:
    tracker = _CaptureGapTracker(threshold_s=0.5)

    tracker.start_file("/data/chunk0000.pcap")
    tracker.observe(_FakeScan([1_000_000_000, 1_000_050_000], frame_id=1))
    # Last scan of the file: the rotation cut this frame in half.
    tracker.observe(_FakeScan([1_000_100_000, 0], frame_id=2))

    tracker.start_file("/data/chunk0000-1.pcap")
    # First scan of the next file carries the other half.
    tracker.observe(_FakeScan([0, 1_000_200_000], frame_id=3))
    tracker.observe(_FakeScan([1_000_250_000, 1_000_300_000], frame_id=4))

    summary = tracker.summary()
    assert summary["incomplete_scan_count"] == 2
    assert [s["at_file_boundary"] for s in summary["incomplete_scans"]] == [True, True]
    assert [s["file"] for s in summary["incomplete_scans"]] == [
        "chunk0000.pcap",
        "chunk0000-1.pcap",
    ]


def test_capture_gap_tracker_stays_quiet_on_complete_scans() -> None:
    tracker = _CaptureGapTracker(threshold_s=0.5)
    tracker.start_file("/data/chunk0000.pcap")

    for index in range(5):
        base = 1_000_000_000 + index * 50_000
        tracker.observe(_FakeScan([base, base + 25_000], frame_id=index))

    summary = tracker.summary()
    assert summary["scans_seen"] == 5
    assert summary["incomplete_scan_count"] == 0
    assert summary["missing_columns"] == 0
    assert summary["incomplete_scans"] == []
    assert summary["incomplete_scans_truncated"] is False


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
    transform = _rigid_transform_matrix(
        _yaw_rotation_3d(rotation), np.array([100.0, 200.0, 5.0])
    )

    assert transform.shape == (4, 4)
    assert transform[2, 2] == pytest.approx(1.0)
    assert transform[:, 3].tolist() == pytest.approx([100.0, 200.0, 5.0, 1.0])

    # Applying it to two points must preserve the distance between them, which is
    # the whole reason this backend exists.
    pts = np.array([[0.0, 0.0, 0.0, 1.0], [3.0, 4.0, 12.0, 1.0]]).T
    moved = (transform @ pts).T[:, :3]
    assert np.linalg.norm(moved[1] - moved[0]) == pytest.approx(13.0)


def _l_shaped_track(n: int = 400) -> np.ndarray:
    """An L, so a tilt is observable in both easting and northing."""
    leg = n // 2
    north = np.column_stack([np.zeros(leg), np.linspace(0.0, 350.0, leg)])
    east = np.column_stack([np.linspace(0.0, 700.0, leg), np.full(leg, 350.0)])
    return np.vstack([north, east])


def test_fit_vertical_tilt_recovers_a_known_tilt() -> None:
    xy = _l_shaped_track()
    # A plane tilted by a known gradient, which is what an IMU misalignment leaves
    # behind in the height residual.
    slope_x, slope_y = 0.012, -0.009
    residual = 3.0 + slope_x * (xy[:, 0] - xy[:, 0].mean()) + slope_y * (
        xy[:, 1] - xy[:, 1].mean()
    )

    rotation, method, info = _fit_vertical_tilt(
        xy, residual, min_span_m=50.0, max_tilt_deg=5.0
    )

    assert method == "tilt_fitted"
    assert info["tilt_deg"] == pytest.approx(
        math.degrees(math.atan(math.hypot(slope_x, slope_y))), abs=1e-6
    )
    # It must be a rotation: orthonormal, right-handed, distance preserving.
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)

    # Applying it to the tilted surface levels it.
    points = np.column_stack([xy[:, 0], xy[:, 1], residual])
    levelled = (rotation @ points.T).T
    spread = levelled[:, 2].max() - levelled[:, 2].min()
    assert spread < 0.02


def test_fit_vertical_tilt_preserves_distances() -> None:
    xy = _l_shaped_track()
    residual = 0.015 * (xy[:, 0] - xy[:, 0].mean())
    rotation, method, _info = _fit_vertical_tilt(
        xy, residual, min_span_m=50.0, max_tilt_deg=5.0
    )
    assert method == "tilt_fitted"

    pair = np.array([[0.0, 0.0, 0.0], [3.0, 4.0, 12.0]])
    moved = (rotation @ pair.T).T
    assert np.linalg.norm(moved[1] - moved[0]) == pytest.approx(13.0)


def test_fit_vertical_tilt_skips_a_straight_route() -> None:
    # Down one street only the along-track gradient is observable; fitting the
    # cross-track one would invent a tilt out of noise.
    xy = np.column_stack([np.linspace(0.0, 800.0, 300), np.zeros(300)])
    residual = 0.01 * xy[:, 0]

    rotation, method, info = _fit_vertical_tilt(
        xy, residual, min_span_m=50.0, max_tilt_deg=5.0
    )

    assert method == "tilt_skipped_route_too_straight"
    assert np.allclose(rotation, np.eye(3))
    assert info["tilt_span_y_m"] == pytest.approx(0.0)


def test_fit_vertical_tilt_rejects_an_implausible_tilt() -> None:
    xy = _l_shaped_track()
    # 30 degrees is a broken altitude series or a hillside, not an IMU offset.
    residual = math.tan(math.radians(30.0)) * (xy[:, 0] - xy[:, 0].mean())

    rotation, method, info = _fit_vertical_tilt(
        xy, residual, min_span_m=50.0, max_tilt_deg=5.0
    )

    assert method == "tilt_rejected_implausible"
    assert np.allclose(rotation, np.eye(3))
    assert info["tilt_deg"] == pytest.approx(30.0, abs=1e-6)


def test_level_transform_removes_the_tilt_rather_than_doubling_it() -> None:
    # The sign between "cloud height" and "GPS height" has one wrong answer that
    # still produces a perfectly valid rotation, applied the wrong way: the ramp
    # comes out twice as large instead of gone. Asserting the residual shrinks is
    # what separates the two; asserting a tilt was fitted is not.
    xy = _l_shaped_track()
    gps_z = 200.0 + 0.004 * xy[:, 1]
    slope_x, slope_y = -0.0114, -0.0112
    placed = np.column_stack(
        [
            xy[:, 0],
            xy[:, 1],
            gps_z
            + 1.5
            + slope_x * (xy[:, 0] - xy[:, 0].mean())
            + slope_y * (xy[:, 1] - xy[:, 1].mean()),
        ]
    )

    before = float(np.sqrt(np.mean((placed[:, 2] - gps_z) ** 2)))
    transform, method, info = _level_transform(
        placed, gps_z, min_span_m=50.0, max_tilt_deg=5.0
    )
    levelled = _apply_transform(transform, placed)
    after = float(np.sqrt(np.mean((levelled[:, 2] - gps_z) ** 2)))

    assert method == "tilt_fitted"
    assert info["tilt_deg"] == pytest.approx(0.914, abs=0.01)
    assert after < 0.05
    assert after < before / 10.0


def test_level_transform_keeps_the_cloud_where_it_was_horizontally() -> None:
    xy = _l_shaped_track()
    gps_z = np.full(xy.shape[0], 200.0)
    placed = np.column_stack(
        [xy[:, 0], xy[:, 1], gps_z + 0.012 * (xy[:, 0] - xy[:, 0].mean())]
    )

    transform, _method, _info = _level_transform(
        placed, gps_z, min_span_m=50.0, max_tilt_deg=5.0
    )
    levelled = _apply_transform(transform, placed)

    # Levelling is a rotation about the trajectory centre, so the horizontal
    # placement the XY fit already settled must survive it.
    horizontal_shift = np.linalg.norm(
        levelled[:, :2].mean(axis=0) - placed[:, :2].mean(axis=0)
    )
    assert horizontal_shift < 0.01


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


def _straight_drive(n: int, rate_hz: float, speed_mps: float) -> tuple[np.ndarray, np.ndarray]:
    t = (1_784_671_600 * 10**9 + np.arange(n) * (1e9 / rate_hz)).astype(np.int64)
    xyz = np.zeros((n, 3))
    xyz[:, 0] = speed_mps * (t - t[0]) / 1e9
    return t, xyz


def test_find_transient_pose_spikes_finds_an_out_and_back_excursion() -> None:
    t, xyz = _straight_drive(2000, 1000.0, 6.0)
    xyz[700:740] = [-20.0, -18.0, -1.5]  # falls back to a far-away pose for 40 ms

    windows, steps = _find_transient_pose_spikes(t, xyz)

    assert windows == [(700, 740)]
    assert steps == []


def test_find_transient_pose_spikes_ignores_honest_motion_across_a_capture_gap() -> None:
    t, xyz = _straight_drive(2000, 1000.0, 6.0)
    # 0.35 s of missing poses: consecutive samples 2.1 m apart, but at 6 m/s.
    t = np.concatenate([t[:1000], t[1350:]])
    xyz = np.concatenate([xyz[:1000], xyz[1350:]])

    windows, steps = _find_transient_pose_spikes(t, xyz)

    assert windows == []
    assert steps == []


def test_find_transient_pose_spikes_reports_a_jump_that_never_returns() -> None:
    t, xyz = _straight_drive(4000, 1000.0, 6.0)
    xyz[1500:, 1] += 25.0

    windows, steps = _find_transient_pose_spikes(t, xyz)

    assert windows == []
    assert steps == [1500]


def test_repair_pose_spikes_interpolates_translation_and_rotation() -> None:
    t = np.array([0, 10, 20, 30, 40], dtype=np.int64)
    poses = np.repeat(np.eye(4)[np.newaxis], 5, axis=0)
    poses[:, 0, 3] = [0.0, 99.0, 99.0, 99.0, 4.0]
    theta = math.radians(10.0)
    poses[4, :3, :3] = _yaw_rotation_3d(
        np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    )

    repaired = _repair_pose_spikes(t, poses, [(1, 4)])

    assert np.allclose(repaired[1:4, 0, 3], [1.0, 2.0, 3.0])
    mid_yaw = math.degrees(math.atan2(repaired[2, 1, 0], repaired[2, 0, 0]))
    assert mid_yaw == pytest.approx(5.0, abs=1e-6)
    assert np.allclose(repaired[[0, 4]], poses[[0, 4]])


def test_estimate_gps_time_offset_recovers_a_known_lag() -> None:
    # An L-shaped drive with changing speed, so a lag cannot be absorbed by the fit.
    slam_t = (np.arange(0, 120, 0.05) * 1e9).astype(np.int64)
    s = slam_t / 1e9
    speed = 4.0 + 3.0 * np.sin(s / 7.0)
    dist = np.cumsum(speed) * 0.05
    leg = dist < 300
    x = np.where(leg, dist, 300.0)
    y = np.where(leg, 0.0, dist - 300.0)
    slam_xy = np.column_stack([x, y])
    gps_t = (np.arange(1, 119) * 1e9).astype(np.int64)
    gps_e = np.interp(gps_t, slam_t, x) + 500_000.0
    gps_n = np.interp(gps_t, slam_t, y) + 3_000_000.0
    lagged = gps_t + int(0.3 * 1e9)  # stamped 0.3 s late, like pi_time_ns

    result = _estimate_gps_time_offset(slam_t, slam_xy, lagged, gps_e, gps_n)

    assert result["gps_time_offset_best_s"] == pytest.approx(-0.3, abs=0.03)
    assert result["gps_time_offset_median_at_best_m"] < 0.1
    assert result["gps_time_offset_median_at_zero_m"] > 0.5


def _straight_poses(xs: np.ndarray, y: float = 0.0) -> np.ndarray:
    poses = np.broadcast_to(np.eye(4), (xs.size, 4, 4)).copy()
    poses[:, 0, 3] = xs
    poses[:, 1, 3] = y
    return poses


def test_local_map_spans_cover_the_route_end_to_end() -> None:
    spans = _local_map_spans(_straight_poses(np.arange(0.0, 121.0)), 50.0, 10.0)

    assert spans[0] == (0, 50)
    assert spans[1] == (10, 60)
    assert spans[-1][1] == 120
    assert all(b[0] - a[0] == 10 for a, b in zip(spans, spans[1:]))


def test_find_loop_pairs_uses_the_closure_when_slam_has_drifted() -> None:
    # Out along x, then along a parallel leg that SLAM believes is 30 m to the side,
    # which is the same street: only the closure knows that.
    first = _straight_poses(np.arange(0.0, 100.0))
    link = _straight_poses(np.full(100, 100.0), 0.0)
    link[:, 1, 3] = np.arange(100.0)
    second = _straight_poses(np.arange(0.0, 100.0), 30.0)
    poses = np.concatenate([first, link, second])
    spans = [(0, 100), (100, 200), (200, 300)]
    # Earlier map frame to later map frame: the later anchor is really at the earlier one.
    closures = [(0, 2, np.eye(4))]

    pairs = _find_loop_pairs(poses, spans, closures, radius_m=10.0, min_separation_m=50.0, every_m=10.0)

    assert len(pairs) == 10
    for j, i, distance, guess in pairs:
        assert j == i - 200
        assert distance == pytest.approx(0.0, abs=1e-9)
        assert guess == pytest.approx(np.eye(4), abs=1e-9)


def test_find_loop_pairs_ignores_a_vehicle_standing_still() -> None:
    poses = _straight_poses(np.zeros(100))
    pairs = _find_loop_pairs(
        poses, [(0, 50), (50, 99)], [(0, 1, np.eye(4))], radius_m=10.0, min_separation_m=50.0, every_m=10.0
    )

    assert pairs == []


def test_a_pose_spike_does_not_make_a_revisit() -> None:
    # A straight 100 m with one key frame thrown 27 m sideways: no revisit, though the
    # spike alone adds 54 m to the distance travelled.
    poses = _straight_poses(np.arange(0.0, 100.0))
    poses[50, 1, 3] = 27.0
    along = _path_along(poses[:, :2, 3])

    assert along[-1] == pytest.approx(97.0)
    assert _find_nearby_pairs(poses, along, [], radius_m=5.0, min_separation_m=50.0, every_m=10.0) == []


def test_pairs_never_use_the_last_key_frame() -> None:
    # A route that ends where it began: out along x, back 20 m to the side, then across to
    # the start. The SDK's optimiser silently ignores a constraint on the last key frame.
    out = _straight_poses(np.arange(0.0, 100.0))
    back = _straight_poses(np.arange(100.0, 0.0, -1.0), 20.0)
    across = _straight_poses(np.zeros(20))
    across[:, 1, 3] = np.arange(19.0, -1.0, -1.0)
    poses = np.concatenate([out, back, across])
    last = poses.shape[0] - 1
    along = np.arange(poses.shape[0], dtype=np.float64)

    nearby = _find_nearby_pairs(poses, along, [], radius_m=3.0, min_separation_m=50.0, every_m=1.0)
    closed = _find_loop_pairs(
        poses, [(0, 50), (170, 220)], [(0, 1, np.linalg.inv(poses[0]) @ poses[170])],
        radius_m=3.0, min_separation_m=50.0, every_m=1.0,
    )

    assert last - 1 in [i for _, i, _, _ in nearby]
    assert last not in [i for _, i, _, _ in nearby]
    assert last not in [i for _, i, _, _ in closed]


def test_find_nearby_pairs_finds_a_return_and_skips_what_is_taken() -> None:
    # A 400 m out-and-back along x: the return passes the outward leg again.
    xs = np.concatenate([np.arange(0.0, 200.0), np.arange(200.0, 0.0, -1.0)])
    poses = _straight_poses(xs)
    along = np.arange(xs.size, dtype=np.float64)

    pairs = _find_nearby_pairs(poses, along, [], radius_m=2.0, min_separation_m=50.0, every_m=10.0)
    assert pairs
    assert all(along[i] - along[j] >= 50.0 for j, i, _, _ in pairs)
    assert all(d <= 2.0 for _, _, d, _ in pairs)

    taken = [i for _, i, _, _ in pairs]
    assert _find_nearby_pairs(poses, along, taken, radius_m=2.0, min_separation_m=50.0, every_m=10.0) == []


def _bent_straight_road(duration_s: float = 200.0, speed: float = 7.0, bend_deg_per_km: float = 3.4):
    """A level straight drive, as SLAM would report it with up bending by distance, and its accelerometer."""
    t = np.arange(0.0, duration_s, 0.05)
    bend = np.radians(bend_deg_per_km) * (speed * t) / 1000.0
    rv = np.column_stack([np.zeros_like(bend), -bend, np.zeros_like(bend)])
    rotation = _rotvec_to_matrix(rv)
    poses = np.tile(np.eye(4), (t.size, 1, 1))
    poses[:, :3, :3] = rotation
    steps = (rotation[:-1] @ np.array([speed * 0.05, 0.0, 0.0]))
    poses[1:, :3, 3] = np.cumsum(steps, axis=0)
    ts = (t * 1e9).astype(np.int64) + 1_000_000_000
    # A level road: true up is vertical, seen in the sensor's axes through the true attitude, which is identity.
    accel = np.tile(np.array([0.0, 0.0, 9.81]), (t.size, 1))
    return ts, poses, accel


def test_gravity_levelling_flattens_a_bend_and_keeps_distances() -> None:
    ts, poses, accel = _bent_straight_road()
    before = poses[:, 2, 3]
    assert abs(before).max() > 2.0

    levelled, info = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0))

    # The window averages the bend, so the middle of the route is the fair place to judge it.
    middle = levelled[ts.size // 4 : 3 * ts.size // 4, 2, 3]
    assert np.ptp(middle) < 0.1 * np.ptp(before)
    step_before = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    step_after = np.linalg.norm(np.diff(levelled[:, :3, 3], axis=0), axis=1)
    np.testing.assert_allclose(step_after, step_before, atol=1e-9)
    assert info["gravity_correction_max_deg"] > 1.0
    np.testing.assert_allclose(levelled[0, :3, 3], poses[0, :3, 3])


def test_gravity_levelling_leaves_a_level_route_alone() -> None:
    ts, poses, accel = _bent_straight_road(bend_deg_per_km=0.0)

    levelled, info = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0))

    np.testing.assert_allclose(levelled, poses, atol=1e-6)
    assert info["gravity_correction_max_deg"] < 1e-6


def test_gravity_levelling_removes_the_sensor_offset() -> None:
    ts, poses, accel = _bent_straight_road(bend_deg_per_km=0.0)
    accel[:, 0] += np.radians(0.2) * 9.81

    levelled, _ = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.2, 0.0))

    np.testing.assert_allclose(levelled, poses, atol=1e-6)


def test_gravity_place_warp_keeps_a_surface_together_where_a_correction_by_time_splits_it() -> None:
    ts, poses, accel = _bent_straight_road(bend_deg_per_km=30.0)
    levelled, _ = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0))
    # One point of the SLAM map, seen nine times over 8 s.
    patch = poses[ts.size // 2, :3, 3] + np.array([15.0, 0.0, -2.0])
    times = ts[ts.size // 2] / 1e9 + np.linspace(-4.0, 4.0, 9)
    points = np.tile(patch, (times.size, 1))

    warp, stations = _gravity_place_warp(ts, poses, levelled)
    moved = warp(points, times)
    assert stations > 100
    assert np.ptp(moved, axis=0).max() < 1e-9

    index = np.searchsorted(ts, (times * 1e9).astype(np.int64))
    correction = levelled[index, :3, :3] @ np.transpose(poses[index, :3, :3], (0, 2, 1))
    by_time = (correction @ (points - poses[index, :3, 3])[:, :, None])[:, :, 0] + levelled[index, :3, 3]
    assert np.ptp(by_time[:, 2]) > 0.02


def test_gravity_place_warp_leaves_a_level_route_alone() -> None:
    ts, poses, accel = _bent_straight_road(bend_deg_per_km=0.0)
    levelled, _ = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0))
    points = np.array([[100.0, 8.0, -2.0], [300.0, -20.0, 5.0]])

    warp, _ = _gravity_place_warp(ts, poses, levelled)

    np.testing.assert_allclose(warp(points, np.array([1014.0, 1043.0])), points, atol=1e-5)


def test_gravity_place_warp_keeps_two_passes_apart() -> None:
    """A second pass of the same place holds its own correction, so it must not borrow the first's."""
    ts, poses, _ = _bent_straight_road(duration_s=400.0, bend_deg_per_km=0.0)
    half = ts.size // 2
    # Out and back along x: the same places at two times far apart.
    poses[half:, :3, 3] = poses[half - 1 :: -1, :3, 3][: ts.size - half]
    levelled = poses.copy()
    levelled[half:, 2, 3] += 3.0
    point = poses[half // 2, :3, 3] + np.array([0.0, 5.0, 0.0])

    warp, _ = _gravity_place_warp(ts, poses, levelled)

    first = warp(point[None], np.array([ts[half // 2] / 1e9]))
    second = warp(point[None], np.array([ts[half + half // 2] / 1e9]))
    np.testing.assert_allclose(first, point[None], atol=1e-6)
    np.testing.assert_allclose(second - point, [[0.0, 0.0, 3.0]], atol=1e-6)


def _out_and_back(duration_s: float = 200.0, length_m: float = 700.0):
    """A level road driven out and back, with the accelerometer wrong by 0.2 deg on the way back."""
    t = np.arange(0.0, duration_s + 1e-9, 0.05)
    half = t.size // 2
    w = 2 * np.pi / duration_s
    poses = np.tile(np.eye(4), (t.size, 1, 1))
    poses[:, 0, 3] = length_m * (1 - np.cos(w * t)) / 2
    ts = (t * 1e9).astype(np.int64) + 1_000_000_000
    # The sensor feels the vehicle's acceleration as well as gravity.
    accel = np.column_stack([length_m * w**2 / 2 * np.cos(w * t), np.zeros(t.size), np.full(t.size, 9.81)])
    accel[half:, 0] += np.radians(0.2) * 9.81
    # Every 10 s on the way out, the moment the way back passes the same place.
    ties = [(int(ts[k]), int(ts[2 * half - k])) for k in range(200, half - 1, 200)]
    return ts, poses, accel, ties, half


def test_gravity_ties_keep_two_visits_of_a_place_together() -> None:
    ts, poses, accel, ties, half = _out_and_back()

    def gap(levelled: np.ndarray) -> float:
        return max(abs(levelled[2 * half - first_k, 2, 3] - levelled[first_k, 2, 3]) for first_k in range(200, half - 200, 200))

    alone, _ = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0))
    tied, info = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0), ties_ns=ties)

    assert gap(alone) > 0.5
    assert gap(tied) < 0.1 * gap(alone)
    assert info["gravity_ties"] == len(ties)
    assert info["gravity_tie_moved_max_m"] < 0.1


def test_gravity_ties_hold_as_well_with_corrections_of_several_degrees() -> None:
    # 1 km out and back, climbing and turning, with 0.2 deg more correction on the way
    # back. A common 7 deg correction should leave the visits as close as none does.
    n = 400
    steps = np.zeros((n - 1, 3))
    steps[: n // 2] = [3.0, 4.0, 0.3]
    steps[n // 2 :] = [-3.0, -4.0, -0.3]

    def moved(base_deg: float) -> np.ndarray:
        rv = np.zeros((n, 3))
        rv[:, :2] = np.radians(base_deg) * np.array([0.6, 0.8])
        rv[n // 2 :, 0] += np.radians(0.2)
        return _tie_gravity_corrections(rv, steps, [(k, n - 1 - k) for k in range(20, n // 2 - 1, 20)])[1]

    np.testing.assert_allclose(moved(7.0), moved(0.0), atol=0.005)


def test_gravity_ties_change_nothing_where_the_accelerometer_already_agrees() -> None:
    ts, poses, accel, ties, half = _out_and_back()
    accel[half:, 0] -= np.radians(0.2) * 9.81

    tied, _ = _level_poses_on_gravity(ts, poses, ts, accel, sensor_offset_deg=(0.0, 0.0), ties_ns=ties)

    # The road is level, so the path should stay flat; the window's edges leave a few cm over 700 m.
    assert np.abs(tied[:, 2, 3]).max() < 0.05


def test_gravity_levelling_rejects_imu_outside_the_route() -> None:
    ts, poses, accel = _bent_straight_road()
    with pytest.raises(ValueError, match="IMU"):
        _level_poses_on_gravity(ts, poses, ts + 10_000_000_000_000, accel)
