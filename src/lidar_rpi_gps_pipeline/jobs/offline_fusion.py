#!/usr/bin/env python3
"""Offline LiDAR + GPS fusion jobs (CLI + in-process API)."""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import os
import re
import shutil
import tempfile
import time
from io import StringIO
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

import laspy
import numpy as np
import pandas as pd
from pyproj import CRS


class JobCancelledError(RuntimeError):
    """Raised when a running job is cancelled by caller."""


EventEmitter = Optional[Callable[[Dict[str, Any]], None]]
CancelChecker = Optional[Callable[[], bool]]

# Column order of the per-point attribute array carried through the pose-optimizer
# raw replay and written into the exported LAS. See the accumulator in
# `_build_point_cloud_from_raw_with_poses` for why only averageable quantities
# appear here.
MAP_ATTR_COLUMNS = ("time_s", "reflectivity", "near_ir", "range_m", "beam_row")

# A break in the LiDAR stream longer than this is treated as a capture dropout.
# Scan-to-map registration cannot bridge one: at 5 m/s a one-second hole is five
# metres of unobserved motion, and once the SLAM map absorbs the resulting bad
# pose the trajectory never recovers.
CAPTURE_GAP_THRESHOLD_S = 0.5


def log(message: str) -> None:
    print(message, flush=True)


def _emit(emit_event: EventEmitter, kind: str, **payload: Any) -> None:
    if emit_event is None:
        return
    emit_event({"kind": kind, **payload})


def _log_and_emit(emit_event: EventEmitter, message: str) -> None:
    """
    Print a line and publish it as a status event without printing it twice.

    Both sinks are wanted: stdout for CLI runs, the event stream for the GUI. The
    `console_echoed` flag tells `_console_event_emitter` that stdout already carries
    this line, which matters now that the CLI supplies an emitter of its own.
    """
    log(message)
    _emit(emit_event, "status", message=message, console_echoed=True)


def _check_cancel(should_cancel: CancelChecker) -> None:
    if should_cancel is not None and should_cancel():
        raise JobCancelledError("Job cancelled by user.")


def _format_unix_utc(seconds: float) -> str:
    return datetime.datetime.utcfromtimestamp(float(seconds)).strftime("%H:%M:%S.%f")[:12]


# Marks a frame the batcher has already shown whole, so that a zeroed placeholder
# emitted for it afterwards is not mistaken for a lost frame.
_FRAME_SEEN_COMPLETE = object()


def _scan_column_validity(scan: Any) -> Optional[Tuple[int, int]]:
    """
    Return `(valid_columns, total_columns)` for one scan, or None if neither
    field can be read.

    `status` is the sensor's own per-column validity flag and is the primary
    source. The timestamp column is the fallback: a column whose UDP packet never
    arrived is left zeroed, so counting non-zero timestamps measures the same
    thing for this purpose.
    """
    try:
        status = np.asarray(scan.status).astype(np.int64)
        if status.size:
            return int(np.count_nonzero(status & 1)), int(status.size)
    except Exception:
        pass

    try:
        timestamps = np.asarray(scan.timestamp, dtype=np.int64)
    except Exception:
        return None
    if timestamps.size == 0:
        return None
    return int(np.count_nonzero(timestamps > 0)), int(timestamps.size)


class _CaptureGapTracker:
    """
    Detect breaks in the LiDAR stream as scans are fed to SLAM.

    A dropout during capture leaves a hole that scan-to-map registration cannot
    bridge: the sensor has moved metres with nothing observed in between, the
    estimated pose for the first scan after the hole is wrong, and once the SLAM map
    absorbs that pose the trajectory never recovers. The failure is invisible in the
    QA report, which only compares the trajectory against GPS after the fact, so it
    has to be reported while the scans are being read.

    Timestamps come from the sensor clock, so a gap here means packets were lost or
    the sensor stopped sending, not that files were supplied out of order.

    Incomplete scans are the second defect, and the time check above is blind to
    them. When UDP packets are dropped at capture time the affected frame still
    arrives, just with fewer columns than the sensor emitted, and its surviving
    columns still carry timestamps -- so the stream reads as continuous. On the
    2026-07-21 session a 0.32 s hole (408 packets lost, six frames gone outright)
    stayed under `CAPTURE_GAP_THRESHOLD_S`, was never reported, and left the merged
    trajectory 28 % longer than the GPS track it was built against. Column counts
    are exact and need no threshold, so they are taken for every scan.
    """

    # Listing every incomplete scan would bloat the QA report on a badly damaged
    # session. Counts and totals stay exact; only the per-scan detail is capped.
    MAX_INCOMPLETE_DETAIL = 100

    # After a dropout the SDK's scan batcher emits the same frame more than once:
    # first the fragment it holds, then fuller versions as late packets are placed,
    # and zeroed placeholders for frames that never arrived at all. Counting each
    # emission separately would multiply one damaged frame into a dozen. Damaged
    # frames are therefore held briefly and merged by id, keeping the most complete
    # view of each. The window is small because the repeats are always adjacent,
    # and because frame ids are 16-bit: a map kept for the whole session would
    # eventually merge unrelated frames after a wrap.
    RECENT_FRAME_WINDOW = 256

    def __init__(self, threshold_s: float = CAPTURE_GAP_THRESHOLD_S) -> None:
        self.threshold_s = float(threshold_s)
        self.gaps: List[Dict[str, float]] = []
        self.incomplete: List[Dict[str, Any]] = []
        self.incomplete_count = 0
        self.missing_columns = 0
        self.scans_seen = 0
        self._last_ns: Optional[int] = None
        self._file: Optional[str] = None
        self._scan_in_file = 0
        self._scans_per_file: Dict[str, int] = {}
        self._detail_truncated = False
        # Values are either a damaged-frame entry or the _FRAME_SEEN_COMPLETE mark.
        self._pending: Dict[Any, Any] = {}

    def start_file(self, path: str) -> None:
        """
        Note which raw file the scans that follow came from.

        A frame that straddles a chunk boundary is split across two files and shows
        up as incomplete in each. That is how the capture was rotated rather than a
        dropout in flight, so `summary` labels those entries separately; knowing the
        file and the position within it is what makes the distinction possible.
        """
        self._file = os.path.basename(path)
        self._scan_in_file = 0

    def _record(self, entry: Dict[str, Any]) -> None:
        self.incomplete_count += 1
        self.missing_columns += entry["missing_columns"]
        if len(self.incomplete) >= self.MAX_INCOMPLETE_DETAIL:
            self._detail_truncated = True
            return
        self.incomplete.append(entry)

    def _release_oldest(self) -> None:
        entry = self._pending.pop(next(iter(self._pending)))
        if entry is not _FRAME_SEEN_COMPLETE:
            self._record(entry)

    def _evict(self) -> None:
        while len(self._pending) > self.RECENT_FRAME_WINDOW:
            self._release_oldest()

    def _flush_pending(self) -> None:
        while self._pending:
            self._release_oldest()

    def _note_incomplete(self, scan: Any) -> None:
        validity = _scan_column_validity(scan)
        if validity is None:
            return
        valid, total = validity
        if total <= 0:
            return

        frame_id = getattr(scan, "frame_id", None)
        try:
            frame_id = None if frame_id is None else int(frame_id)
        except Exception:
            frame_id = None
        # Without a frame id there is nothing to merge on, so each scan stands
        # alone and the scan index keeps the keys distinct.
        key = (self._file, frame_id, None if frame_id is not None else self._scan_in_file)

        if valid >= total:
            # The frame is intact. It may already be held as damaged, from an
            # earlier partial emission, or the placeholder may still be coming --
            # the batcher does both. Recording the verdict covers either order.
            self._pending[key] = _FRAME_SEEN_COMPLETE
            self._evict()
            return

        held = self._pending.get(key)
        if held is _FRAME_SEEN_COMPLETE:
            return
        if held is not None:
            held["last_scan_index_in_file"] = self._scan_in_file
            if valid > held["valid_columns"]:
                held["valid_columns"] = valid
                held["missing_columns"] = total - valid
            return

        unix_s: Optional[float] = None
        try:
            timestamps = np.asarray(scan.timestamp, dtype=np.int64)
            positive = timestamps[timestamps > 0]
            if positive.size:
                unix_s = float(positive.min()) / 1e9
        except Exception:
            unix_s = None

        self._pending[key] = {
            "file": self._file,
            "scan_index_in_file": self._scan_in_file,
            "last_scan_index_in_file": self._scan_in_file,
            "frame_id": frame_id,
            "valid_columns": valid,
            "total_columns": total,
            "missing_columns": total - valid,
            "unix_s": unix_s,
        }
        self._evict()

    def observe(self, scan: Any) -> Optional[Dict[str, float]]:
        if scan is None:
            return None

        self.scans_seen += 1
        self._scan_in_file += 1
        if self._file is not None:
            self._scans_per_file[self._file] = self._scan_in_file
        # Before the timestamp check below, which returns early on a scan the SDK
        # zeroed out -- exactly the scans a dropout produces.
        self._note_incomplete(scan)

        try:
            timestamps = np.asarray(scan.timestamp, dtype=np.int64)
        except Exception:
            return None
        timestamps = timestamps[timestamps > 0]
        if timestamps.size == 0:
            return None

        first = int(timestamps.min())
        last = int(timestamps.max())
        gap: Optional[Dict[str, float]] = None
        if self._last_ns is not None:
            delta_s = (first - self._last_ns) / 1e9
            if delta_s > self.threshold_s:
                gap = {
                    "after_unix_s": self._last_ns / 1e9,
                    "before_unix_s": first / 1e9,
                    "gap_s": delta_s,
                }
                self.gaps.append(gap)
        self._last_ns = last if self._last_ns is None else max(self._last_ns, last)
        return gap

    def summary(self) -> Dict[str, Any]:
        # Damaged frames still inside the merge window have not been counted yet.
        # Flushing empties the window, so calling this twice is safe.
        self._flush_pending()

        incomplete: List[Dict[str, Any]] = []
        for entry in self.incomplete:
            item = dict(entry)
            scans_in_file = self._scans_per_file.get(item["file"])
            item["at_file_boundary"] = bool(
                item["scan_index_in_file"] == 1
                or (
                    scans_in_file is not None
                    and item["last_scan_index_in_file"] == scans_in_file
                )
            )
            incomplete.append(item)

        return {
            "threshold_s": self.threshold_s,
            "count": len(self.gaps),
            "total_s": float(sum(g["gap_s"] for g in self.gaps)),
            "gaps": self.gaps,
            "scans_seen": self.scans_seen,
            "incomplete_scan_count": self.incomplete_count,
            "missing_columns": self.missing_columns,
            "incomplete_scans": incomplete,
            "incomplete_scans_truncated": self._detail_truncated,
        }


def read_ouster_csv(path: str) -> pd.DataFrame:
    """Read Ouster CSV that may include comment lines before the real header."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    header_index = next((i for i, line in enumerate(lines) if "TIMESTAMP" in line.upper()), None)
    if header_index is None:
        raise ValueError(f"Could not find header line containing TIMESTAMP in: {path}")

    lines[header_index] = lines[header_index].lstrip("#").strip() + "\n"
    df = pd.read_csv(StringIO("".join(lines[header_index:])))
    df.columns = [c.strip().lower() for c in df.columns]
    return df


def choose_time_mode(
    lidar_ts_ns: np.ndarray,
    gps_ts_ns: np.ndarray,
    requested_mode: str,
) -> Tuple[np.ndarray, np.ndarray, str]:
    """
    Decide how to align LiDAR and GPS clocks.

    Modes:
    - unix_ns: direct nanosecond epoch matching
    - relative_start: normalize each series to its own start timestamp
    - auto: choose unix_ns if overlap looks valid, else relative_start
    """
    lidar_start = int(lidar_ts_ns[0])
    gps_start = int(gps_ts_ns[0])

    if requested_mode == "unix_ns":
        return lidar_ts_ns, gps_ts_ns, "unix_ns"

    if requested_mode == "relative_start":
        return lidar_ts_ns - lidar_start, gps_ts_ns - gps_start, "relative_start"

    overlap = (lidar_ts_ns.min() <= gps_ts_ns.max()) and (gps_ts_ns.min() <= lidar_ts_ns.max())

    ns_2000 = 946684800 * 10**9
    ns_2100 = 4102444800 * 10**9
    lidar_looks_like_epoch = ns_2000 <= np.median(lidar_ts_ns) <= ns_2100
    gps_looks_like_epoch = ns_2000 <= np.median(gps_ts_ns) <= ns_2100

    if overlap and lidar_looks_like_epoch and gps_looks_like_epoch:
        return lidar_ts_ns, gps_ts_ns, "unix_ns"

    return lidar_ts_ns - lidar_start, gps_ts_ns - gps_start, "relative_start"


def preferred_gps_time_column_from_manifest(manifest: Optional[Dict[str, Any]]) -> Optional[str]:
    """
    Prefer `gps_epoch_ns` for Pi-clock/PTP capture manifests.

    In the PTP pipeline the Pi clock is disciplined to GPS PPS, so LiDAR
    timestamps and the receiver's fix epoch are both UTC. `gps_epoch_ns` is the
    instant the position is valid for. `pi_time_ns` is when the Pi finished
    reading it over I2C, about 0.3 s later on this rig, which places every fix
    roughly 2 m behind the vehicle at urban speeds. It is not truncated: the
    receiver fixes on whole seconds at 1 Hz, and the bridge keeps any fraction.
    """
    if not isinstance(manifest, dict):
        return None
    timing_architecture = str(manifest.get("timing_architecture") or "")
    if timing_architecture == "gps_pps_to_pi__chrony_to_pi_clock__ptp_to_ouster":
        return "gps_epoch_ns"
    return None


def pick_gps_time_column(
    df: pd.DataFrame,
    requested: str,
    *,
    preferred_auto: Optional[List[str]] = None,
) -> str:
    if requested == "gps_epoch_ns":
        if "gps_epoch_ns" not in df.columns:
            raise ValueError("Requested --gps-time-column gps_epoch_ns, but column is missing.")
        return "gps_epoch_ns"
    if requested == "pi_time_ns":
        if "pi_time_ns" not in df.columns:
            raise ValueError("Requested --gps-time-column pi_time_ns, but column is missing.")
        return "pi_time_ns"

    preferred = preferred_auto or ["gps_epoch_ns", "pi_time_ns"]
    for column in preferred:
        if column in df.columns and df[column].notna().sum() >= 2:
            return column
    raise ValueError("No usable GPS time column found (need gps_epoch_ns or pi_time_ns with >=2 rows).")


def load_gps_for_interpolation(
    gps_csv: str,
    requested_time_col: str,
    *,
    preferred_auto_columns: Optional[List[str]] = None,
    apply_outlier_filter: bool = False,
    gps_outlier_max_speed_mps: float = 30.0,
) -> Tuple[pd.DataFrame, str]:
    log(f"Loading GPS CSV: {gps_csv}")
    gps_df = pd.read_csv(gps_csv)

    needed_gps_cols = ["easting", "northing", "altitude_m"]
    missing_gps = [c for c in needed_gps_cols if c not in gps_df.columns]
    if missing_gps:
        raise ValueError(f"Missing required GPS columns: {missing_gps}")

    gps_time_col = pick_gps_time_column(
        gps_df,
        requested_time_col,
        preferred_auto=preferred_auto_columns,
    )
    log(f"GPS time column: {gps_time_col}")

    gps_cols = [gps_time_col, "easting", "northing", "altitude_m"]
    if "hdop" in gps_df.columns:
        gps_cols.append("hdop")
    gps_df = gps_df[gps_cols].copy()
    gps_df = gps_df.dropna(subset=[gps_time_col, "easting", "northing", "altitude_m"])
    gps_df[gps_time_col] = gps_df[gps_time_col].astype("int64")
    gps_df["easting"] = gps_df["easting"].astype("float64")
    gps_df["northing"] = gps_df["northing"].astype("float64")
    gps_df["altitude_m"] = gps_df["altitude_m"].astype("float64")
    if "hdop" in gps_df.columns:
        gps_df["hdop"] = pd.to_numeric(gps_df["hdop"], errors="coerce").astype("float64")
    gps_df = gps_df[
        np.isfinite(gps_df["easting"])
        & np.isfinite(gps_df["northing"])
        & np.isfinite(gps_df["altitude_m"])
    ]
    gps_df = gps_df.sort_values(gps_time_col)
    gps_df = gps_df.drop_duplicates(subset=[gps_time_col])

    if apply_outlier_filter:
        gps_df, outlier_stats = filter_gps_outliers_by_speed(
            gps_df,
            gps_time_col,
            max_speed_mps=float(gps_outlier_max_speed_mps),
        )
        log(
            "GPS outlier filter: "
            f"kept {outlier_stats['kept_rows']}/{outlier_stats['input_rows']} rows, "
            f"dropped={outlier_stats['dropped_rows']} "
            f"(speed={outlier_stats['dropped_speed']}, "
            f"nonpositive_dt={outlier_stats['dropped_nonpositive_dt']}), "
            f"max_speed_mps={float(gps_outlier_max_speed_mps):.2f}"
        )

    if len(gps_df) < 2:
        raise ValueError(
            "Need at least 2 finite GPS rows to interpolate "
            "(time/easting/northing/altitude_m are required)."
        )

    return gps_df, gps_time_col


def filter_gps_outliers_by_speed(
    gps_df: pd.DataFrame,
    gps_time_col: str,
    *,
    max_speed_mps: float,
) -> Tuple[pd.DataFrame, Dict[str, int]]:
    """
    Remove implausible GPS rows using incremental speed from last accepted point.
    """
    if max_speed_mps <= 0:
        raise ValueError("gps_outlier_max_speed_mps must be > 0.")
    if len(gps_df) < 2:
        return gps_df.copy(), {"input_rows": len(gps_df), "kept_rows": len(gps_df), "dropped_rows": 0}

    t = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    e = gps_df["easting"].to_numpy(dtype=np.float64)
    n = gps_df["northing"].to_numpy(dtype=np.float64)

    keep = np.zeros(len(gps_df), dtype=bool)
    keep[0] = True
    last_kept = 0
    dropped_nonpositive_dt = 0
    dropped_speed = 0

    for idx in range(1, len(gps_df)):
        dt_ns = int(t[idx] - t[last_kept])
        if dt_ns <= 0:
            dropped_nonpositive_dt += 1
            continue

        dt_s = dt_ns * 1e-9
        step_m = float(np.hypot(e[idx] - e[last_kept], n[idx] - n[last_kept]))
        speed_mps = step_m / dt_s
        if speed_mps > max_speed_mps:
            dropped_speed += 1
            continue

        keep[idx] = True
        last_kept = idx

    filtered = gps_df.loc[keep].copy()
    stats = {
        "input_rows": int(len(gps_df)),
        "kept_rows": int(len(filtered)),
        "dropped_rows": int(len(gps_df) - len(filtered)),
        "dropped_speed": int(dropped_speed),
        "dropped_nonpositive_dt": int(dropped_nonpositive_dt),
    }
    return filtered, stats


def _gps_in_range_mask(query_t: np.ndarray, ref_t: np.ndarray) -> np.ndarray:
    if query_t.size == 0:
        return np.zeros((0,), dtype=bool)
    ref_min = int(ref_t.min())
    ref_max = int(ref_t.max())
    return (query_t >= ref_min) & (query_t <= ref_max)


def _trim_query_to_gps_range(
    query_t: np.ndarray,
    ref_t: np.ndarray,
    *,
    label: str,
    min_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Trim query timestamps to GPS-supported range.

    Returns:
      - trimmed query timestamps
      - boolean mask (same length as original query)
    """
    mask = _gps_in_range_mask(query_t, ref_t)
    kept = int(mask.sum())
    dropped = int(mask.size - kept)
    if dropped > 0:
        log(f"{label}: trimming {dropped} samples outside GPS time coverage.")
    if kept < min_points:
        ref_min = int(ref_t.min())
        ref_max = int(ref_t.max())
        raise ValueError(
            f"{label}: only {kept} samples remain after GPS-range trimming "
            f"(need >= {min_points}). gps=[{ref_min}, {ref_max}]"
        )
    return query_t[mask], mask


def _compute_path_tangent_heading(
    easting: np.ndarray,
    northing: np.ndarray,
    *,
    fallback_heading_rad: float,
) -> np.ndarray:
    """
    Compute heading (rad) from GPS trajectory tangent in EN coordinates.
    """
    if easting.size == 0:
        return np.asarray([], dtype=np.float64)
    if easting.size == 1:
        return np.asarray([fallback_heading_rad], dtype=np.float64)

    de = np.gradient(easting)
    dn = np.gradient(northing)
    motion = np.hypot(de, dn)
    heading = np.arctan2(dn, de)

    valid = motion > 1e-9
    if not np.any(valid):
        return np.full_like(easting, fallback_heading_rad, dtype=np.float64)

    first_valid = int(np.flatnonzero(valid)[0])
    heading[:first_valid] = heading[first_valid]
    for idx in range(first_valid + 1, heading.size):
        if not valid[idx]:
            heading[idx] = heading[idx - 1]
    return heading


def _rotate_local_xy_for_gps_fusion(
    x_local: np.ndarray,
    y_local: np.ndarray,
    e_interp: np.ndarray,
    n_interp: np.ndarray,
    *,
    orientation_mode: str,
    sensor_yaw_deg: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Rotate local XY before adding to GPS coordinates.

    - fixed_yaw: use one constant yaw (deg) for all points.
    - path_tangent: heading follows GPS trajectory tangent + yaw offset.
    """
    yaw_rad = float(np.deg2rad(sensor_yaw_deg))
    if orientation_mode == "fixed_yaw":
        theta = np.full_like(x_local, yaw_rad, dtype=np.float64)
    elif orientation_mode == "path_tangent":
        theta = _compute_path_tangent_heading(
            e_interp,
            n_interp,
            fallback_heading_rad=yaw_rad,
        ) + yaw_rad
    else:
        raise ValueError(
            "gps_fusion_orientation must be one of: fixed_yaw, path_tangent"
        )

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    x_rot = (cos_t * x_local) - (sin_t * y_local)
    y_rot = (sin_t * x_local) + (cos_t * y_local)
    return x_rot, y_rot


def _estimate_las_header_offsets_from_gps(
    gps_e: np.ndarray,
    gps_n: np.ndarray,
    gps_z: np.ndarray,
    *,
    sensor_offset_x: float,
    sensor_offset_y: float,
    sensor_offset_z: float,
    padding_m: float = 500.0,
) -> Tuple[float, float, float]:
    """
    Estimate stable LAS header offsets from full GPS trajectory (not first scan only).
    """
    return (
        float(np.min(gps_e) + sensor_offset_x - padding_m),
        float(np.min(gps_n) + sensor_offset_y - padding_m),
        float(np.min(gps_z) + sensor_offset_z - padding_m),
    )


def chunk_prefix(output_dir: str, lidar_csv: str) -> str:
    base = os.path.splitext(os.path.basename(lidar_csv))[0]
    return os.path.join(output_dir, f"geo_{base}")


def prefixed_output_paths(output_prefix: str, lidar_paths: List[str]) -> List[str]:
    if len(lidar_paths) == 1:
        return [output_prefix]
    return [f"{output_prefix}_{os.path.splitext(os.path.basename(p))[0]}" for p in lidar_paths]


def parse_manifest_lidar_paths(manifest: dict) -> List[str]:
    if "chunks" in manifest and isinstance(manifest["chunks"], list) and manifest["chunks"]:
        chunks: List[Tuple[int, str]] = []
        for item in manifest["chunks"]:
            if not isinstance(item, dict):
                continue
            chunk_index = item.get("chunk_index", 10**9)

            produced_files = item.get("produced_files", [])
            if isinstance(produced_files, list):
                for f in produced_files:
                    if isinstance(f, str) and f.lower().endswith(".csv"):
                        chunks.append((chunk_index, f))

            lidar_path = item.get("lidar_csv_path")
            if isinstance(lidar_path, str) and lidar_path.lower().endswith(".csv"):
                chunks.append((chunk_index, lidar_path))

        chunks.sort(key=lambda x: (x[0], x[1]))
        seen = set()
        ordered = []
        for _, p in chunks:
            if p not in seen:
                seen.add(p)
                ordered.append(p)
        return ordered

    if "lidar_csv_path" in manifest and manifest["lidar_csv_path"]:
        return [manifest["lidar_csv_path"]]

    return []


def parse_manifest_raw_paths(manifest: dict) -> List[str]:
    chunks = manifest.get("chunks", [])
    if not isinstance(chunks, list) or not chunks:
        return []

    raw_items: List[Tuple[int, str]] = []
    for item in chunks:
        if not isinstance(item, dict):
            continue
        chunk_index = item.get("chunk_index", 10**9)

        files = item.get("produced_files", [])
        if isinstance(files, list):
            for f in files:
                if isinstance(f, str) and (f.lower().endswith(".pcap") or f.lower().endswith(".osf")):
                    raw_items.append((chunk_index, f))

        for key in ("lidar_pcap_path", "requested_output_path"):
            value = item.get(key)
            if isinstance(value, str) and (value.lower().endswith(".pcap") or value.lower().endswith(".osf")):
                raw_items.append((chunk_index, value))

    raw_items.sort(key=lambda x: (x[0], x[1]))
    seen = set()
    ordered = []
    for _, p in raw_items:
        if p not in seen:
            seen.add(p)
            ordered.append(p)
    return ordered


def _is_raw_lidar_path(path: str) -> bool:
    lower = path.lower()
    return lower.endswith(".pcap") or lower.endswith(".osf")


def _raw_lidar_path_sort_key(path: str) -> Tuple[str, int, int, str]:
    """
    Sort split raw captures in chronological chunk order.

    Example desired order:
    - raw_lidar_<session>_chunk0000.pcap
    - raw_lidar_<session>_chunk0000-1.pcap
    - raw_lidar_<session>_chunk0000-2.pcap
    """
    parent = os.path.dirname(path)
    base = os.path.basename(path)
    match = re.search(r"(.*_chunk)(\d+)(?:-(\d+))?(\.[^.]+)$", base)
    if not match:
        return (parent, 10**9, 10**9, base)

    chunk_index = int(match.group(2))
    split_index = -1 if match.group(3) is None else int(match.group(3))
    prefix = match.group(1)
    suffix = match.group(4)
    normalized_base = f"{prefix}{chunk_index:04d}{suffix}"
    return (parent, chunk_index, split_index, normalized_base)


def _extract_session_id_from_gps_csv_path(gps_csv: str) -> Optional[str]:
    """
    Infer session id from a GPS CSV filename.

    Expected common pattern: raw_gps_YYYYMMDD_HHMMSS.csv
    """
    name = os.path.splitext(os.path.basename(gps_csv))[0]
    if name.startswith("raw_gps_"):
        suffix = name.replace("raw_gps_", "", 1)
        if re.fullmatch(r"\d{8}_\d{6}", suffix):
            return suffix

    match = re.search(r"(\d{8}_\d{6})", name)
    if match:
        return match.group(1)
    return None


def _filter_raw_paths_by_session(raw_paths: List[str], session_id: str) -> List[str]:
    """
    Keep only raw LiDAR files that belong to a specific session id.

    Common filename pattern: raw_lidar_<session_id>_chunkXXXX.pcap
    """
    prefix = f"raw_lidar_{session_id}_"
    filtered = []
    for path in raw_paths:
        base = os.path.basename(path)
        if base.startswith(prefix):
            filtered.append(path)

    # Fallback for non-standard names that still embed the session id.
    if not filtered:
        token = f"_{session_id}_"
        for path in raw_paths:
            base = os.path.basename(path)
            if session_id in base or token in base:
                filtered.append(path)

    if not filtered:
        raise ValueError(
            f"No raw LiDAR files matched session filter '{session_id}'. "
            "Check raw input dir and GPS/session selection."
        )
    return filtered


def expand_raw_lidar_inputs(raw_inputs: List[str]) -> List[str]:
    """
    Expand --lidar-raw inputs into concrete file paths.

    Supports explicit files, directories, and glob patterns.
    """
    expanded = []

    for item in raw_inputs:
        if os.path.isdir(item):
            for name in sorted(os.listdir(item), key=_raw_lidar_path_sort_key):
                path = os.path.join(item, name)
                if os.path.isfile(path) and _is_raw_lidar_path(path):
                    expanded.append(path)
            continue

        if any(ch in item for ch in "*?[]"):
            for path in sorted(glob.glob(item), key=_raw_lidar_path_sort_key):
                if os.path.isfile(path) and _is_raw_lidar_path(path):
                    expanded.append(path)
            continue

        if os.path.isfile(item):
            if not _is_raw_lidar_path(item):
                raise ValueError(f"--lidar-raw file is not .pcap/.osf: {item}")
            expanded.append(item)
            continue

        raise ValueError(f"--lidar-raw path does not exist: {item}")

    seen = set()
    ordered = []
    for p in expanded:
        if p not in seen:
            seen.add(p)
            ordered.append(p)

    if not ordered:
        raise ValueError(
            "No raw LiDAR files found from --lidar-raw input. "
            "Pass .pcap/.osf files, a directory containing them, or a glob pattern."
        )
    return ordered


def resolve_manifest_path(path: str, manifest_dir: str, manifest_base_dir: Optional[str]) -> str:
    candidates = [path]

    if not os.path.isabs(path):
        candidates.append(os.path.join(manifest_dir, path))

    if manifest_base_dir:
        candidates.append(os.path.join(manifest_base_dir, os.path.basename(path)))
        if not os.path.isabs(path):
            candidates.append(os.path.join(manifest_base_dir, path))

    seen = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if os.path.isfile(candidate):
            return candidate

    return candidates[0]


def resolve_manifest_path_list(
    paths: List[str],
    manifest_json: str,
    manifest_base_dir: Optional[str],
    label: str,
) -> List[str]:
    manifest_dir = os.path.dirname(os.path.abspath(manifest_json))
    resolved = [resolve_manifest_path(p, manifest_dir, manifest_base_dir) for p in paths]
    missing = [p for p in resolved if not os.path.isfile(p)]
    if missing:
        sample = "\n".join(f"- {p}" for p in missing[:5])
        raise ValueError(
            f"Could not resolve {label} path(s) from manifest:\n{sample}\n"
            "If manifest has Pi absolute paths, use --manifest-base-dir "
            "and/or pass --gps-csv explicitly."
        )
    return resolved


def _find_metadata_json_candidates(raw_path: str) -> List[str]:
    base = os.path.splitext(raw_path)[0]
    candidates = [f"{base}.json", f"{base}_0.json"]
    candidates.extend(sorted(glob.glob(f"{base}*.json")))

    # A failed transfer can leave a zero-byte sidecar beside an otherwise good
    # capture, and `open_source` then fails with "empty metadata passed in" rather
    # than looking further. Every chunk of one session carries a byte-identical
    # sensor configuration, so any non-empty `*_0.json` in the same directory is a
    # valid stand-in. Manifests are excluded by the `_0.json` suffix.
    directory = os.path.dirname(raw_path) or "."
    candidates.extend(sorted(glob.glob(os.path.join(directory, "*_0.json"))))

    ordered = []
    seen = set()
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)
        if os.path.isfile(p) and os.path.getsize(p) > 0:
            ordered.append(p)
    return ordered


def _extract_first_sensor_info(source: Any) -> Any:
    for attr in ("metadata", "metadatas", "infos", "sensor_info"):
        value = getattr(source, attr, None)
        if value is None:
            continue
        if isinstance(value, (list, tuple)):
            for item in value:
                if item is not None:
                    return item
        else:
            return value
    raise ValueError("Could not obtain sensor metadata from source.")


def _is_scan_like(item: Any) -> bool:
    return bool(
        hasattr(item, "fields")
        and hasattr(item, "field")
        and hasattr(item, "h")
        and hasattr(item, "w")
        and hasattr(item, "timestamp")
    )


def _iter_scans(item: Any) -> Iterable[Any]:
    if item is None:
        return

    if _is_scan_like(item):
        yield item
        return

    if isinstance(item, (list, tuple)):
        for candidate in item:
            if candidate is not None and _is_scan_like(candidate):
                yield candidate
        return

    if hasattr(item, "__iter__"):
        try:
            for candidate in item:
                if candidate is not None and _is_scan_like(candidate):
                    yield candidate
        except Exception:
            return


def _extract_first_scan(item: Any) -> Any:
    return next(_iter_scans(item), None)


def _scan_valid_timestamp_count(scan: Any) -> int:
    timestamps = np.asarray(scan.timestamp, dtype=np.int64)
    return int(np.count_nonzero(timestamps > 0))


def _updated_has_any_valid_timestamps(item: Any) -> bool:
    return any(_scan_valid_timestamp_count(scan) > 0 for scan in _iter_scans(item))


def _scan_has_range_field(scan: Any, ChanField: Any) -> bool:
    try:
        if ChanField.RANGE in scan.fields:
            return True
    except Exception:
        pass
    try:
        field_names = {str(f) for f in scan.fields}
        if "RANGE" in field_names or "ChanField.RANGE" in field_names:
            return True
    except Exception:
        pass
    try:
        scan.field("RANGE")
        return True
    except Exception:
        return False


def _scan_get_range_field(scan: Any, ChanField: Any) -> Any:
    try:
        return scan.field(ChanField.RANGE)
    except Exception:
        return scan.field("RANGE")


def _scan_has_reflectivity_field(scan: Any, ChanField: Any) -> bool:
    try:
        if ChanField.REFLECTIVITY in scan.fields:
            return True
    except Exception:
        pass
    try:
        field_names = {str(f) for f in scan.fields}
        if "REFLECTIVITY" in field_names or "ChanField.REFLECTIVITY" in field_names:
            return True
    except Exception:
        pass
    try:
        scan.field("REFLECTIVITY")
        return True
    except Exception:
        return False


def _scan_get_reflectivity_field(scan: Any, ChanField: Any) -> Any:
    try:
        return scan.field(ChanField.REFLECTIVITY)
    except Exception:
        return scan.field("REFLECTIVITY")


def _scan_has_near_ir_field(scan: Any, ChanField: Any) -> bool:
    try:
        if ChanField.NEAR_IR in scan.fields:
            return True
    except Exception:
        pass
    try:
        field_names = {str(f) for f in scan.fields}
        if "NEAR_IR" in field_names or "ChanField.NEAR_IR" in field_names:
            return True
    except Exception:
        pass
    try:
        scan.field("NEAR_IR")
        return True
    except Exception:
        return False


def _scan_get_near_ir_field(scan: Any, ChanField: Any) -> Any:
    """
    Ambient near-infrared image, present in the RNG/RFL/NIR sensor profiles.

    Unlike reflectivity this is passive solar radiance around 865 nm rather than a
    return from the sensor's own laser. Vegetation is far brighter in it than
    asphalt or concrete, which makes it a useful discriminator downstream. It
    depends on sun angle and shadowing, so values are comparable within one
    session but not across sessions.
    """
    try:
        return scan.field(ChanField.NEAR_IR)
    except Exception:
        return scan.field("NEAR_IR")


def _next_available_output_path(path: str) -> str:
    """
    Return a non-conflicting path by appending an incrementing suffix.

    Example:
    - geo_session.las
    - geo_session_2.las
    - geo_session_3.las
    """
    base_dir = os.path.dirname(path) or "."
    filename = os.path.basename(path)
    stem, ext = os.path.splitext(filename)
    candidate = path
    index = 2

    while os.path.exists(candidate) or os.path.exists(candidate + ".partial"):
        candidate = os.path.join(base_dir, f"{stem}_{index}{ext}")
        index += 1
    if candidate != path:
        log(f"Output path exists; using next available file: {candidate}")
    return candidate


def _resolve_output_las_path(
    args: argparse.Namespace,
    *,
    raw_paths: List[str],
    manifest_json: Optional[str],
) -> str:

    if args.output_las:
        output_las = args.output_las
    elif args.output_prefix:
        output_las = args.output_prefix + ".las"
    else:
        output_dir = args.output_dir or "."
        os.makedirs(output_dir, exist_ok=True)

        if manifest_json:
            manifest_base = os.path.splitext(os.path.basename(manifest_json))[0]
            if manifest_base.startswith("capture_manifest_"):
                manifest_base = manifest_base.replace("capture_manifest_", "", 1)
            base_name = f"geo_session_{manifest_base}.las"
        elif raw_paths:
            first_base = os.path.splitext(os.path.basename(raw_paths[0]))[0]
            base_name = f"geo_{first_base}_merged.las"
        else:
            base_name = "geo_fused_output.las"

        output_las = os.path.join(output_dir, base_name)

    out_dir = os.path.dirname(output_las) or "."
    os.makedirs(out_dir, exist_ok=True)
    return _next_available_output_path(output_las)


def _resolve_output_osf_path(
    args: argparse.Namespace,
    *,
    output_las: str,
) -> Optional[str]:
    if not getattr(args, "save_osf", False) and not getattr(args, "output_osf", None):
        return None

    if args.output_osf:
        output_osf = args.output_osf
    else:
        root, _ext = os.path.splitext(output_las)
        output_osf = root + ".osf"

    out_dir = os.path.dirname(output_osf) or "."
    os.makedirs(out_dir, exist_ok=True)
    return _next_available_output_path(output_osf)


def _temp_osf_path(output_osf: Optional[str]) -> Optional[str]:
    """
    Build a temporary OSF path that still ends with `.osf`.
    Some Ouster save paths expect OSF-like filenames and can reject arbitrary suffixes.
    """
    if not output_osf:
        return None
    root, ext = os.path.splitext(output_osf)
    ext = ext or ".osf"
    return f"{root}.partial{ext}"


def _open_raw_source(raw_path: str) -> Any:
    """Open a raw Ouster source with best-effort metadata sidecar discovery."""
    try:
        from ouster.sdk import open_source
    except Exception as e:
        raise ValueError(
            "ouster-sdk is not installed or not importable. Install with: pip install ouster-sdk"
        ) from e

    open_kwargs = {"sensor_idx": 0, "collate": False}
    if raw_path.lower().endswith(".pcap"):
        meta_candidates = _find_metadata_json_candidates(raw_path)
        if meta_candidates:
            open_kwargs["meta"] = [meta_candidates[0]]

    try:
        return open_source(raw_path, **open_kwargs)
    except TypeError:
        return open_source(raw_path)


def _source_infos(source: Any) -> List[Any]:
    infos = getattr(source, "sensor_info", None)
    if isinstance(infos, list) and infos:
        return infos
    if isinstance(infos, tuple) and infos:
        return list(infos)

    return [_extract_first_sensor_info(source)]


def _open_ouster_source(raw_path: str) -> Tuple[Any, Any, Any, Any, Any, Any]:
    """
    Open a raw Ouster source and return (source, info, infos, XYZLut, ChanField, destagger).
    """
    try:
        from ouster.sdk.core import ChanField, XYZLut, destagger
    except Exception as e:
        raise ValueError(
            "ouster-sdk core bindings are not importable. Install with: pip install ouster-sdk"
        ) from e

    source = _open_raw_source(raw_path)
    info = _extract_first_sensor_info(source)
    infos = _source_infos(source)
    xyzlut = XYZLut(info, use_extrinsics=True)
    return source, info, infos, xyzlut, ChanField, destagger


def _extract_scan_arrays(
    scan: Any,
    xyzlut: Any,
    info: Any,
    ChanField: Any,
    destagger: Any,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Flatten one scan into (timestamp_ns, x_local, y_local, z_local, reflectivity).

    All five arrays are masked by the same validity test, so they stay row-aligned.
    Reflectivity is zero-filled when the sensor profile does not carry the field.
    """
    range_img = _scan_get_range_field(scan, ChanField)
    timestamps = np.tile(np.asarray(scan.timestamp, dtype=np.int64), (scan.h, 1))
    xyz = xyzlut(range_img)

    refl_img = None
    if _scan_has_reflectivity_field(scan, ChanField):
        try:
            refl_img = np.asarray(
                _scan_get_reflectivity_field(scan, ChanField), dtype=np.float64
            )
        except Exception:
            refl_img = None

    try:
        range_img = destagger(info, range_img)
        timestamps = destagger(info, timestamps)
        xyz = destagger(info, xyz)
        if refl_img is not None:
            refl_img = destagger(info, refl_img)
    except Exception:
        pass

    timestamp_ns = timestamps.reshape(-1).astype(np.int64)
    x_local = -xyz[:, :, 0].reshape(-1).astype(np.float64)
    y_local = -xyz[:, :, 1].reshape(-1).astype(np.float64)
    z_local = xyz[:, :, 2].reshape(-1).astype(np.float64)
    if refl_img is not None:
        reflectivity = np.asarray(refl_img, dtype=np.float64).reshape(-1)
    else:
        reflectivity = np.zeros(timestamp_ns.shape, dtype=np.float64)

    valid = np.isfinite(x_local) & np.isfinite(y_local) & np.isfinite(z_local)
    valid = valid & (range_img.reshape(-1) > 0)

    return (
        timestamp_ns[valid],
        x_local[valid],
        y_local[valid],
        z_local[valid],
        reflectivity[valid],
    )


def _write_debug_converted_csv(
    csv_path: str,
    timestamp_ns: np.ndarray,
    x_local: np.ndarray,
    y_local: np.ndarray,
    z_local: np.ndarray,
) -> None:
    frame_df = pd.DataFrame(
        {
            "TIMESTAMP (ns)": timestamp_ns,
            "X1 (m)": -x_local,
            "Y1 (m)": -y_local,
            "Z1 (m)": z_local,
        }
    )
    frame_df.to_csv(csv_path, index=False)


def _new_las_header(
    utm_epsg: Optional[int],
    first_x: float,
    first_y: float,
    first_z: float,
    *,
    point_format: int = 6,
    extra_dims: Optional[List[Tuple[str, str, str]]] = None,
) -> laspy.LasHeader:
    """
    Build the LAS 1.4 header.

    `point_format` 6 carries X/Y/Z, intensity, classification and gps_time. Format 8
    adds the standard `nir` field, which is where the sensor's ambient near-infrared
    channel belongs; prefer it over an extra dimension so ordinary LAS readers
    understand the data. `extra_dims` entries are (name, laspy type, description).
    """
    header = laspy.LasHeader(point_format=point_format, version="1.4")
    header.x_scale = header.y_scale = header.z_scale = 0.001
    header.x_offset = float(first_x)
    header.y_offset = float(first_y)
    header.z_offset = float(first_z)
    if utm_epsg is not None:
        header.add_crs(CRS.from_epsg(utm_epsg))
    for name, type_str, description in extra_dims or []:
        header.add_extra_dim(
            laspy.ExtraBytesParams(name=name, type=type_str, description=description)
        )
    return header


def _check_slam_map_coverage(
    points_xy: np.ndarray,
    traj_xy: np.ndarray,
    *,
    emit_event: EventEmitter,
    context: str,
    min_span_ratio: float = 0.9,
) -> Dict[str, float]:
    """
    Warn when an extracted SLAM map covers far less ground than the trajectory.

    `SlamEngine.get_point_cloud()` returns the SLAM backend's rolling local map,
    which prunes older voxels as the sensor moves away. On long sessions that map
    can end up covering only the tail of the route while the trajectory - and
    therefore the QA report, which is computed from the trajectory - still spans the
    whole session. Without this check the shortfall is completely silent.

    A complete map should span at least as much ground as the trajectory, since the
    sensor also sees sideways, so anything below `min_span_ratio` is reported.
    """
    report: Dict[str, float] = {}
    if points_xy.size == 0 or traj_xy.shape[0] < 2:
        return report

    map_span = points_xy.max(axis=0) - points_xy.min(axis=0)
    traj_span = traj_xy.max(axis=0) - traj_xy.min(axis=0)
    ratios = [
        float(map_span[i] / traj_span[i]) if traj_span[i] > 1.0 else float("inf")
        for i in range(2)
    ]
    report = {
        "map_span_x_m": float(map_span[0]),
        "map_span_y_m": float(map_span[1]),
        "trajectory_span_x_m": float(traj_span[0]),
        "trajectory_span_y_m": float(traj_span[1]),
        "map_to_trajectory_span_ratio": float(min(ratios)),
    }

    if min(ratios) < min_span_ratio:
        message = (
            f"{context}: extracted map spans {map_span[0]:.0f}x{map_span[1]:.0f} m but the "
            f"trajectory spans {traj_span[0]:.0f}x{traj_span[1]:.0f} m. The SLAM backend only "
            "retains a rolling local map, so earlier parts of the route are missing from the "
            "point cloud even though the QA report (computed from the trajectory) still looks "
            "complete. Use --processing-mode slam_gps_anchor with --anchor-backend "
            "pose_optimizer for full-route coverage."
        )
        log(f"WARNING: {message}")
        _emit(emit_event, "status", message=f"WARNING: {message}")

    return report


def _extract_scan_pose_sample(scan: Any) -> Tuple[int, np.ndarray]:
    """
    Extract one representative timestamp and translation from a SLAM-updated scan.
    """
    timestamps = np.asarray(scan.timestamp, dtype=np.int64)
    poses = np.asarray(scan.pose, dtype=np.float64)
    if timestamps.ndim != 1 or poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("Unexpected scan timestamp/pose layout from Ouster SDK.")

    valid = np.flatnonzero(timestamps > 0)
    if valid.size == 0:
        raise ValueError("No valid scan timestamps available for SLAM trajectory sample.")

    idx = int(valid[valid.size // 2])
    ts = int(timestamps[idx])
    xyz = poses[idx, :3, 3].astype(np.float64)
    return ts, xyz


def _extract_scan_column_poses(scan: Any) -> Tuple[np.ndarray, np.ndarray]:
    """
    Every valid column pose of a SLAM-updated scan, with its timestamp.

    `_extract_scan_pose_sample` keeps one pose per scan, which is enough to fit a
    trajectory against GPS but not to replay the raw scans: dewarping needs the
    pose that was current for each individual column, because the sensor moves
    measurably during the 50 ms a frame takes.
    """
    timestamps = np.asarray(scan.timestamp, dtype=np.int64)
    poses = np.asarray(scan.pose, dtype=np.float64)
    if timestamps.ndim != 1 or poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("Unexpected scan timestamp/pose layout from Ouster SDK.")
    if poses.shape[0] != timestamps.shape[0]:
        raise ValueError("Scan pose count does not match timestamp count.")

    valid = timestamps > 0
    return timestamps[valid], poses[valid]


def _rigid_transform_matrix(
    rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    """
    Assemble the 4x4 that maps the SLAM local frame onto the georeferenced frame.

    Takes a full 3x3 rotation rather than a yaw alone, because placing a cloud
    correctly can need a rotation about a horizontal axis as well - see
    `_fit_vertical_tilt`. Composing two of these stays rigid, which is the whole
    point of the backend: no relative geometry can change no matter how many are
    multiplied together.
    """
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def _yaw_rotation_3d(rotation_xy: np.ndarray) -> np.ndarray:
    """Embed a 2x2 horizontal rotation in 3D, leaving the vertical axis alone."""
    rotation = np.eye(3, dtype=np.float64)
    rotation[:2, :2] = np.asarray(rotation_xy, dtype=np.float64)
    return rotation


def _apply_transform(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 to an Nx3 array of points."""
    points = np.asarray(points, dtype=np.float64)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def _fit_xy_rigid_transform(
    src_xy: np.ndarray,
    dst_xy: np.ndarray,
    *,
    min_motion_m: float,
) -> Tuple[np.ndarray, np.ndarray, str]:
    """
    Fit a 2D rigid transform (rotation + translation) from src to dst.
    Falls back to translation-only when trajectory motion is too small.
    """
    if src_xy.shape != dst_xy.shape or src_xy.ndim != 2 or src_xy.shape[1] != 2:
        raise ValueError("Transform fitting expects matched Nx2 source/destination arrays.")
    if src_xy.shape[0] < 2:
        raise ValueError("Need at least 2 matched samples to compute anchoring transform.")

    src_center = src_xy.mean(axis=0)
    dst_center = dst_xy.mean(axis=0)
    src_centered = src_xy - src_center
    dst_centered = dst_xy - dst_center

    motion_span = float(np.linalg.norm(src_xy.max(axis=0) - src_xy.min(axis=0)))
    if motion_span < min_motion_m:
        rotation = np.eye(2, dtype=np.float64)
        translation = dst_center - src_center
        return rotation, translation, "translation_only_low_motion"

    cov = src_centered.T @ dst_centered
    u, _s, vh = np.linalg.svd(cov)
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) < 0:
        vh[-1, :] *= -1
        rotation = vh.T @ u.T
    translation = dst_center - (rotation @ src_center)
    return rotation, translation, "rigid_xy"


POSE_SPIKE_MIN_JUMP_M = 2.0
POSE_SPIKE_MAX_SPEED_MPS = 40.0
POSE_SPIKE_MAX_DURATION_S = 2.0


def _find_transient_pose_spikes(
    timestamps_ns: np.ndarray,
    xyz: np.ndarray,
    *,
    min_jump_m: float = POSE_SPIKE_MIN_JUMP_M,
    max_speed_mps: float = POSE_SPIKE_MAX_SPEED_MPS,
    max_duration_s: float = POSE_SPIKE_MAX_DURATION_S,
) -> Tuple[List[Tuple[int, int]], List[int]]:
    """
    Find poses that leave the trajectory and come back.

    Observed on 2026-07-21 right after a 0.35 s capture dropout: for about 40 ms
    the SLAM column poses fell back to the run's starting pose, 27 m away, then
    resumed where the vehicle actually was. Points replayed through those poses
    land as a displaced ghost at plausible heights, which no height check sees.

    A jump has to be both large and impossibly fast. Large alone would flag the
    honest step across a capture gap, where 0.35 s at 6 m/s moves 2 m between
    consecutive samples; fast alone is meaningless at the column rate, where
    millimetre jitter over 50 microseconds reads as tens of metres per second.

    Returns `(windows, steps)`. Each window is a half-open index range `[a, b)` of
    bad samples bracketed by good ones. `steps` lists jumps that never came back
    within `max_duration_s`: a step change in the solution, reported but not
    repaired, because interpolating across it would invent a trajectory.
    """
    t = np.asarray(timestamps_ns, dtype=np.int64)
    xyz = np.asarray(xyz, dtype=np.float64)
    if t.size < 3:
        return [], []
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.diff(t).astype(np.float64) / 1e9
    with np.errstate(divide="ignore", invalid="ignore"):
        speed = np.where(dt > 0, step / dt, np.inf)
    jumps = np.flatnonzero((step > min_jump_m) & (speed > max_speed_mps))

    windows: List[Tuple[int, int]] = []
    steps: List[int] = []
    k = 0
    while k < jumps.size:
        out = int(jumps[k]) + 1  # first bad sample
        anchor = out - 1
        returned_at = None
        for later in jumps[k + 1 :]:
            back = int(later) + 1
            span_s = (t[back] - t[anchor]) / 1e9
            if span_s > max_duration_s:
                break
            reach = min_jump_m + max_speed_mps * span_s
            if np.linalg.norm(xyz[back] - xyz[anchor]) <= reach:
                returned_at = back
                break
        if returned_at is None:
            steps.append(out)
            k += 1
            continue
        windows.append((out, returned_at))
        k = int(np.searchsorted(jumps, returned_at - 1, side="right"))
    return windows, steps


def _rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) of a 3x3 rotation."""
    m = np.asarray(rotation, dtype=np.float64)
    trace = np.trace(m)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def _quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(dot)
    return (np.sin((1 - u) * theta) * q0 + np.sin(u * theta) * q1) / np.sin(theta)


def _repair_pose_spikes(
    timestamps_ns: np.ndarray,
    poses: np.ndarray,
    windows: List[Tuple[int, int]],
) -> np.ndarray:
    """
    Replace each spike window with poses interpolated between its two good
    neighbours: linear in translation, slerp in rotation. The points measured
    during a spike are genuine, only their poses are wrong, so they are kept
    and re-placed rather than deleted.
    """
    repaired = np.array(poses, dtype=np.float64, copy=True)
    t = np.asarray(timestamps_ns, dtype=np.int64)
    for a, b in windows:
        before, after = a - 1, b
        q0 = _rotation_to_quaternion(repaired[before, :3, :3])
        q1 = _rotation_to_quaternion(repaired[after, :3, :3])
        p0 = repaired[before, :3, 3].copy()
        p1 = repaired[after, :3, 3].copy()
        span = float(t[after] - t[before])
        for i in range(a, b):
            u = float(t[i] - t[before]) / span if span > 0 else 0.5
            repaired[i, :3, :3] = _quaternion_to_rotation(_slerp(q0, q1, u))
            repaired[i, :3, 3] = p0 + u * (p1 - p0)
    return repaired


def _estimate_gps_time_offset(
    slam_t_ns: np.ndarray,
    slam_xy: np.ndarray,
    gps_t_ns: np.ndarray,
    gps_e: np.ndarray,
    gps_n: np.ndarray,
    *,
    search_s: float = 0.6,
    step_s: float = 0.02,
) -> Dict[str, Any]:
    """
    Find the GPS time shift that lets a rigid fit match the trajectory best.

    A rigid fit cannot bend to absorb a timing error, so a lag between the two
    clocks shows up directly as residual that grows with speed. Sweeping a shift
    and refitting finds it. On a healthy capture the minimum sits near zero; on
    2026-07-21 with `pi_time_ns` it sat at -0.31 s, the I2C read latency, and
    removing it took the median residual from 1.49 m to 0.33 m.

    Only meaningful for `rigid_fit`: an optimiser that bends the trajectory onto
    GPS absorbs the lag into its shape instead of leaving it in the residual.
    """
    slam_t = np.asarray(slam_t_ns, dtype=np.int64)
    gps_t = np.asarray(gps_t_ns, dtype=np.int64)
    offsets = np.arange(-search_s, search_s + step_s / 2, step_s)
    medians = []
    for off in offsets:
        shifted = gps_t + int(round(off * 1e9))
        inside = (slam_t >= shifted[0]) & (slam_t <= shifted[-1])
        if inside.sum() < 10:
            medians.append(float("nan"))
            continue
        ref = np.column_stack(
            [np.interp(slam_t[inside], shifted, gps_e), np.interp(slam_t[inside], shifted, gps_n)]
        )
        rot, trans, _ = _fit_xy_rigid_transform(slam_xy[inside], ref, min_motion_m=0.0)
        resid = ref - (slam_xy[inside] @ rot.T + trans)
        medians.append(float(np.median(np.linalg.norm(resid, axis=1))))
    medians_arr = np.asarray(medians)
    if not np.isfinite(medians_arr).any():
        return {"gps_time_offset_best_s": None}
    best = int(np.nanargmin(medians_arr))
    best_s = float(offsets[best])
    # Parabolic refinement through the minimum and its neighbours.
    if 0 < best < len(offsets) - 1 and np.all(np.isfinite(medians_arr[best - 1 : best + 2])):
        y0, y1, y2 = medians_arr[best - 1 : best + 2]
        denom = y0 - 2 * y1 + y2
        if denom > 0:
            best_s += 0.5 * step_s * (y0 - y2) / denom
    zero = int(np.argmin(np.abs(offsets)))
    return {
        "gps_time_offset_best_s": best_s,
        "gps_time_offset_median_at_best_m": float(medians_arr[best]),
        "gps_time_offset_median_at_zero_m": float(medians_arr[zero]),
        "gps_time_offset_search_s": float(search_s),
    }


def _fit_vertical_tilt(
    xy: np.ndarray,
    height_residual: np.ndarray,
    *,
    min_span_m: float,
    max_tilt_deg: float,
) -> Tuple[np.ndarray, str, Dict[str, float]]:
    """
    Fit the rotation that takes the SLAM frame's vertical onto the local vertical.

    SLAM defines "up" from the IMU at the start of a run, and that direction is off
    by a fraction of a degree. A yaw-plus-offset placement cannot correct it,
    because none of its four numbers is a rotation about a horizontal axis, so the
    error appears as a height residual that grows linearly across the site: on a
    1031 m route, 0.9 deg of tilt put the two ends of the cloud 10.65 m apart in
    height while the ground itself moved only 7 m. Over the 165 m chunk the method
    was first tested on, the same tilt produced 2.6 m of ramp and the height offset
    absorbed most of it, which is why this went unseen.

    The tilt is fitted from the height residual alone, after the horizontal
    placement is already fixed, so GNSS altitude noise - always worse than
    horizontal - cannot pull the easting and northing fit around.

    Returns a rotation, the name of what was fitted, and the fit's diagnostics.
    """
    xy = np.asarray(xy, dtype=np.float64)
    height_residual = np.asarray(height_residual, dtype=np.float64)
    identity = np.eye(3, dtype=np.float64)

    # A gradient in two directions needs the trajectory to have travelled in two
    # directions. Down a straight street only the along-track component is
    # observable, and fitting the cross-track one anyway invents a tilt from noise.
    span_x = float(xy[:, 0].max() - xy[:, 0].min())
    span_y = float(xy[:, 1].max() - xy[:, 1].min())
    if min(span_x, span_y) < min_span_m:
        return identity, "tilt_skipped_route_too_straight", {
            "tilt_span_x_m": span_x,
            "tilt_span_y_m": span_y,
        }

    centre = xy.mean(axis=0)
    design = np.column_stack(
        [np.ones(xy.shape[0]), xy[:, 0] - centre[0], xy[:, 1] - centre[1]]
    )
    coefficients, *_ = np.linalg.lstsq(design, height_residual, rcond=None)
    _constant, slope_x, slope_y = coefficients
    gradient = float(np.hypot(slope_x, slope_y))
    tilt_deg = float(np.degrees(np.arctan(gradient)))

    diagnostics = {
        "tilt_deg": tilt_deg,
        "tilt_slope_x": float(slope_x),
        "tilt_slope_y": float(slope_y),
        "tilt_span_x_m": span_x,
        "tilt_span_y_m": span_y,
    }

    # A real IMU misalignment is a fraction of a degree. Anything large is some
    # other problem - a broken altitude series, a route on a hillside the fit has
    # mistaken for tilt - and silently rotating the cloud by it would hide that.
    if tilt_deg > max_tilt_deg:
        return identity, "tilt_rejected_implausible", diagnostics
    if gradient == 0.0:
        return identity, "tilt_zero", diagnostics

    # The fitted plane has normal (-slope_x, -slope_y, 1). Rotate that onto the
    # vertical: axis is their cross product, angle the one between them. This is a
    # true rotation, so unlike adding the plane back as a height correction - which
    # would be a shear - it leaves every distance and angle in the cloud untouched.
    normal = np.array([-slope_x, -slope_y, 1.0], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    vertical = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axis = np.cross(normal, vertical)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm == 0.0:
        return identity, "tilt_zero", diagnostics
    axis /= axis_norm
    angle = float(np.arccos(np.clip(np.dot(normal, vertical), -1.0, 1.0)))

    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    rotation = (
        identity + np.sin(angle) * cross + (1.0 - np.cos(angle)) * (cross @ cross)
    )
    return rotation, "tilt_fitted", diagnostics


def _level_transform(
    placed: np.ndarray,
    gps_z: np.ndarray,
    *,
    min_span_m: float,
    max_tilt_deg: float,
) -> Tuple[np.ndarray, str, Dict[str, float]]:
    """
    The extra transform that levels an already-placed cloud onto GPS heights.

    Takes the placed trajectory and the GPS height at the same instants, and owns
    the sign convention between them so that no caller has to get it right: the
    residual `_fit_vertical_tilt` needs is how far the cloud sits *above* truth,
    and passing it the other way round rotates the cloud the wrong way and doubles
    the error instead of removing it.
    """
    placed = np.asarray(placed, dtype=np.float64)
    gps_z = np.asarray(gps_z, dtype=np.float64)

    rotation, method, diagnostics = _fit_vertical_tilt(
        placed[:, :2],
        placed[:, 2] - gps_z,
        min_span_m=min_span_m,
        max_tilt_deg=max_tilt_deg,
    )

    # Rotate about the trajectory's own centre, so levelling cannot move the cloud
    # sideways as a side effect.
    pivot = placed.mean(axis=0)
    transform = _rigid_transform_matrix(rotation, pivot - rotation @ pivot)

    # Levelling shifts the cloud in height, so the offset is taken again against
    # the levelled trajectory rather than the tilted one.
    levelled = _apply_transform(transform, placed)
    offset = float(np.median(gps_z - levelled[:, 2]))
    transform = (
        _rigid_transform_matrix(np.eye(3), np.array([0.0, 0.0, offset])) @ transform
    )
    diagnostics["tilt_residual_offset_m"] = offset
    return transform, method, diagnostics


def _rotation_deg(rotation: np.ndarray) -> float:
    return float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))


def _parse_xyz_weights(weights: str) -> np.ndarray:
    parts = [p for p in weights.replace(" ", "").split(",") if p]
    if len(parts) != 3:
        raise ValueError("Expected 3 comma-separated weights, e.g. 0.01,0.01,0.001")
    values = np.asarray([float(p) for p in parts], dtype=np.float64)
    if np.any(values < 0):
        raise ValueError("All weights must be non-negative.")
    return values


def _xy_path_length(xy: np.ndarray) -> float:
    if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def _compute_xy_alignment_metrics(reference_xy: np.ndarray, estimated_xy: np.ndarray) -> Dict[str, float]:
    if reference_xy.shape != estimated_xy.shape:
        raise ValueError("Alignment metrics require matched Nx2 arrays.")
    if reference_xy.ndim != 2 or reference_xy.shape[1] != 2:
        raise ValueError("Alignment metrics require matched Nx2 arrays.")
    if reference_xy.shape[0] == 0:
        raise ValueError("Alignment metrics require at least one sample.")

    residual = np.linalg.norm(estimated_xy - reference_xy, axis=1)
    return {
        "samples": int(reference_xy.shape[0]),
        "xy_mean_m": float(np.mean(residual)),
        "xy_rmse_m": float(np.sqrt(np.mean(residual**2))),
        "xy_median_m": float(np.median(residual)),
        "xy_p95_m": float(np.percentile(residual, 95)),
        "xy_max_m": float(np.max(residual)),
        "reference_path_length_m": _xy_path_length(reference_xy),
        "estimated_path_length_m": _xy_path_length(estimated_xy),
        "reference_span_x_m": float(reference_xy[:, 0].max() - reference_xy[:, 0].min()),
        "reference_span_y_m": float(reference_xy[:, 1].max() - reference_xy[:, 1].min()),
        "estimated_span_x_m": float(estimated_xy[:, 0].max() - estimated_xy[:, 0].min()),
        "estimated_span_y_m": float(estimated_xy[:, 1].max() - estimated_xy[:, 1].min()),
    }


def _qa_report_path(output_las: str) -> str:
    root, _ext = os.path.splitext(output_las)
    return root + "_qa.json"


def _trajectory_csv_path(output_las: str) -> str:
    root, _ext = os.path.splitext(output_las)
    return root + "_trajectory.csv"


def _write_trajectory_csv(
    output_las: str,
    timestamps_ns: Any,
    poses: Any,
    *,
    max_rows: int = 50000,
) -> Optional[str]:
    """
    Write the optimized sensor trajectory next to the exported LAS.

    The point cloud records where every point is but not where it was observed
    from, and that viewing geometry is exactly what incidence angle, occlusion
    reasoning and density normalisation need downstream. Carrying a sensor position
    on every point would be wasteful, so the trajectory is written once and a point
    is matched to it by `gps_time`.

    Poses arrive sampled per lidar column - around 20,000 rows per second of
    capture, far finer than the trajectory actually resolves - so rows are strided
    down to at most `max_rows`. The default leaves better than per-scan resolution
    for sessions up to roughly forty minutes.
    """
    ts = np.asarray(timestamps_ns, dtype=np.int64).reshape(-1)
    p = np.asarray(poses, dtype=np.float64)
    if ts.size == 0 or p.ndim != 3 or p.shape[1:] != (4, 4) or p.shape[0] != ts.size:
        return None

    stride = max(1, int(np.ceil(ts.size / float(max_rows))))
    ts = ts[::stride]
    p = p[::stride]

    rot = p[:, :3, :3]
    # ZYX (yaw-pitch-roll) decomposition; the pipeline never needs to recover the
    # matrix bit-exactly, only to reason about where the sensor was pointing.
    yaw = np.degrees(np.arctan2(rot[:, 1, 0], rot[:, 0, 0]))
    pitch = np.degrees(
        np.arctan2(-rot[:, 2, 0], np.hypot(rot[:, 2, 1], rot[:, 2, 2]))
    )
    roll = np.degrees(np.arctan2(rot[:, 2, 1], rot[:, 2, 2]))

    frame = pd.DataFrame(
        {
            "time_ns": ts,
            "time_s": ts / 1e9,
            "x": p[:, 0, 3],
            "y": p[:, 1, 3],
            "z": p[:, 2, 3],
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        }
    )
    path = _trajectory_csv_path(output_las)
    frame.to_csv(path, index=False)
    return path


def _write_anchor_qa_report(
    output_las: str,
    *,
    backend: str,
    time_mode_used: str,
    metrics: Dict[str, float],
    extra: Optional[Dict[str, Any]] = None,
) -> Tuple[str, bool]:
    thresholds = {
        "xy_median_max_m": 5.0,
        "xy_p95_max_m": 10.0,
    }
    passed = bool(
        metrics.get("xy_median_m", float("inf")) <= thresholds["xy_median_max_m"]
        and metrics.get("xy_p95_m", float("inf")) <= thresholds["xy_p95_max_m"]
    )

    report: Dict[str, Any] = {
        "mode": "slam_gps_anchor",
        "backend": backend,
        "time_mode_used": time_mode_used,
        "pass": passed,
        "thresholds": thresholds,
        "metrics": metrics,
    }
    if extra:
        report["extra"] = extra

    qa_path = _qa_report_path(output_las)
    with open(qa_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    return qa_path, passed


def _lookup_pose_columns(
    scan_timestamps_ns: np.ndarray,
    pose_timestamps_ns: np.ndarray,
    column_poses: np.ndarray,
    *,
    max_allowed_delta_ns: int = 100_000_000,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, int]]:
    if pose_timestamps_ns.ndim != 1 or pose_timestamps_ns.size == 0:
        raise ValueError("Optimized pose timestamps are empty.")
    if column_poses.ndim != 3 or column_poses.shape[1:] != (4, 4):
        raise ValueError("Optimized column poses must be shaped Nx4x4.")
    if column_poses.shape[0] != pose_timestamps_ns.shape[0]:
        raise ValueError("Optimized pose timestamp count does not match pose matrix count.")

    ts = np.asarray(scan_timestamps_ns, dtype=np.int64)
    if ts.ndim != 1:
        raise ValueError("Scan timestamps must be 1D.")
    if ts.size == 0:
        raise ValueError("Scan has no column timestamps.")

    n = pose_timestamps_ns.shape[0]
    valid = ts > 0

    idx_right = np.searchsorted(pose_timestamps_ns, ts, side="left")
    idx_right = np.clip(idx_right, 0, n - 1)
    idx_left = np.clip(idx_right - 1, 0, n - 1)

    delta_right = np.abs(pose_timestamps_ns[idx_right] - ts)
    delta_left = np.abs(pose_timestamps_ns[idx_left] - ts)
    use_right = delta_right < delta_left
    idx = np.where(use_right, idx_right, idx_left)
    delta = np.where(use_right, delta_right, delta_left)

    exact = valid & (pose_timestamps_ns[idx_right] == ts)
    valid_count = int(valid.sum())
    if valid_count == 0:
        idx[:] = 0
        delta[:] = 0
    else:
        first_valid_idx = int(np.flatnonzero(valid)[0])
        idx[~valid] = idx[first_valid_idx]
        delta[~valid] = 0

    max_delta_ns = int(delta[valid].max()) if valid_count else 0
    inlier_mask = valid & (delta <= max_allowed_delta_ns)
    inlier_count = int(inlier_mask.sum())
    dropped_due_to_drift = int(valid_count - inlier_count)

    stats = {
        "columns": int(ts.size),
        "valid_columns": valid_count,
        "inlier_columns": inlier_count,
        "dropped_columns": dropped_due_to_drift,
        "exact_columns": int(exact.sum()),
        "nearest_columns": int(valid_count - int(exact.sum())),
        "max_delta_ns": max_delta_ns,
    }
    return column_poses[idx], inlier_mask, stats


def _build_point_cloud_from_raw_with_poses(
    raw_paths: List[str],
    *,
    pose_timestamps_ns: np.ndarray,
    column_poses: np.ndarray,
    min_range: float,
    max_range: float,
    reflectivity_filter: bool,
    min_reflectivity: int,
    map_voxel_size: float,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, int]]:
    """
    Build a global point cloud by replaying raw scans and dewarping with optimized poses.

    Returns (points, attributes, stats) where `attributes` is an Nx2 array holding
    per-point capture time in unix seconds and sensor reflectivity, aligned row-wise
    with `points`.
    """
    try:
        from ouster.sdk.core import ChanField, XYZLut, dewarp, voxel_downsample
    except Exception as e:
        raise ValueError("ouster-sdk core modules are required for raw replay map extraction.") from e

    downsample_size = float(map_voxel_size)
    points_accum = np.zeros((0, 3), dtype=np.float64)
    pending_batches: List[np.ndarray] = []
    # Per-point attributes carried alongside XYZ, in MAP_ATTR_COLUMNS order:
    # [capture time (unix seconds), reflectivity, near infrared, range (m), beam row].
    #
    # Time is stored in seconds rather than nanoseconds because the attribute array is
    # float64: epoch nanoseconds (~1.8e18) exceed the 2^53 range where float64 is exact,
    # while epoch seconds keep sub-microsecond resolution.
    #
    # `voxel_downsample` AVERAGES attributes over the points in a voxel, so only
    # quantities that survive averaging belong here. Time, reflectivity, near
    # infrared and range all do. Discrete identifiers such as a frame number do not,
    # which is why none is carried; gps_time already locates a point in time and
    # stays meaningful when averaged. Beam row averages to the typical beam that saw
    # the voxel, which is a usable proxy for viewing geometry; it is exact only when
    # downsampling is disabled with --poseopt-map-voxel-size 0.
    attrs_accum = np.zeros((0, len(MAP_ATTR_COLUMNS)), dtype=np.float64)
    pending_attrs: List[np.ndarray] = []
    scans_seen = 0

    lookup_stats = {
        "columns": 0,
        "valid_columns": 0,
        "inlier_columns": 0,
        "dropped_columns": 0,
        "exact_columns": 0,
        "nearest_columns": 0,
        "max_delta_ns": 0,
        "reflectivity_scans_missing_field": 0,
        "near_ir_scans_missing_field": 0,
    }

    def flush_batches() -> None:
        nonlocal points_accum, attrs_accum, pending_batches, pending_attrs
        if not pending_batches:
            return
        merged = np.concatenate(pending_batches, axis=0)
        merged_attrs = np.concatenate(pending_attrs, axis=0)
        pending_batches = []
        pending_attrs = []
        if points_accum.shape[0] == 0:
            points_accum = merged
            attrs_accum = merged_attrs
        else:
            points_accum = np.concatenate([points_accum, merged], axis=0)
            attrs_accum = np.concatenate([attrs_accum, merged_attrs], axis=0)
        if downsample_size > 0 and points_accum.shape[0] > 0:
            points_accum, attrs_accum = voxel_downsample(
                downsample_size,
                points_accum,
                attrs_accum,
            )

    for file_idx, raw_path in enumerate(raw_paths, start=1):
        source = None
        try:
            source = _open_raw_source(raw_path)
            info = _extract_first_sensor_info(source)
            xyzlut = XYZLut(info, use_extrinsics=True)
            _emit(
                emit_event,
                "status",
                message=(
                    f"Replaying raw with optimized poses "
                    f"({file_idx}/{len(raw_paths)}): {os.path.basename(raw_path)}"
                ),
            )

            for scan_idx, item in enumerate(source, start=1):
                _check_cancel(should_cancel)
                scans_seen += 1
                scan = _extract_first_scan(item)
                if scan is None:
                    continue
                if not _scan_has_range_field(scan, ChanField):
                    continue

                ranges = _scan_get_range_field(scan, ChanField)
                ranges_m = np.asarray(ranges, dtype=np.float64) * 0.001

                scan_col_ts = np.asarray(scan.timestamp, dtype=np.int64)
                pose_cols, pose_inlier_cols, stats = _lookup_pose_columns(
                    scan_col_ts,
                    pose_timestamps_ns,
                    column_poses,
                )
                lookup_stats["columns"] += stats["columns"]
                lookup_stats["valid_columns"] += stats["valid_columns"]
                lookup_stats["inlier_columns"] += stats["inlier_columns"]
                lookup_stats["dropped_columns"] += stats["dropped_columns"]
                lookup_stats["exact_columns"] += stats["exact_columns"]
                lookup_stats["nearest_columns"] += stats["nearest_columns"]
                lookup_stats["max_delta_ns"] = max(lookup_stats["max_delta_ns"], stats["max_delta_ns"])

                points_local = xyzlut(ranges)
                points_global = dewarp(points_local, pose_cols)

                valid = (ranges > 0) & pose_inlier_cols[np.newaxis, :]
                if min_range > 0:
                    valid = valid & (ranges_m >= min_range)
                if max_range > 0:
                    valid = valid & (ranges_m <= max_range)

                # Reflectivity is read on every scan, not only when filtering, so that it
                # can be carried into the exported LAS as the intensity field.
                if _scan_has_reflectivity_field(scan, ChanField):
                    refl = np.asarray(
                        _scan_get_reflectivity_field(scan, ChanField), dtype=np.float64
                    )
                else:
                    refl = np.zeros(np.shape(ranges), dtype=np.float64)
                    lookup_stats["reflectivity_scans_missing_field"] += 1

                # Ambient near-infrared. Absent from range-only sensor profiles, in
                # which case the channel is exported as zero rather than dropped so
                # the LAS layout stays the same across sessions.
                if _scan_has_near_ir_field(scan, ChanField):
                    near_ir = np.asarray(
                        _scan_get_near_ir_field(scan, ChanField), dtype=np.float64
                    )
                else:
                    near_ir = np.zeros(np.shape(ranges), dtype=np.float64)
                    lookup_stats["near_ir_scans_missing_field"] += 1

                if reflectivity_filter:
                    valid = valid & (refl >= float(min_reflectivity))

                selected = points_global[valid]
                if selected.size == 0:
                    continue
                scan_ts_sec = (
                    np.broadcast_to(scan_col_ts[np.newaxis, :], valid.shape)[valid] / 1e9
                )
                # Row index in the staggered range image is the beam index. Kept
                # because point density falls off as 1/range^2 and varies per beam,
                # so any density-dependent downstream step needs the viewing
                # geometry rather than the point alone.
                beam_rows = np.broadcast_to(
                    np.arange(valid.shape[0], dtype=np.float64)[:, np.newaxis], valid.shape
                )[valid]
                pending_batches.append(np.asarray(selected, dtype=np.float64))
                pending_attrs.append(
                    np.column_stack(
                        [
                            scan_ts_sec,
                            refl[valid],
                            near_ir[valid],
                            ranges_m[valid],
                            beam_rows,
                        ]
                    ).astype(np.float64)
                )

                if scan_idx % 20 == 0:
                    flush_batches()
                    _emit(
                        emit_event,
                        "scan_progress",
                        file_index=file_idx,
                        file_total=len(raw_paths),
                        scan_index=scan_idx,
                        total_scans=scans_seen,
                        total_points=int(points_accum.shape[0]),
                    )
        finally:
            close_fn = getattr(source, "close", None)
            if callable(close_fn):
                close_fn()

    flush_batches()
    return points_accum, attrs_accum, lookup_stats


def fuse_raw_to_single_las(
    raw_paths: List[str],
    gps_df: pd.DataFrame,
    gps_time_col: str,
    output_las: str,
    *,
    time_mode: str,
    utm_epsg: int,
    sensor_offset_x: float,
    sensor_offset_y: float,
    sensor_offset_z: float,
    gps_fusion_orientation: str,
    sensor_yaw_deg: float,
    reflectivity_filter: bool,
    min_reflectivity: int,
    keep_converted: bool,
    converted_csv_dir: Optional[str],
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    gps_e = gps_df["easting"].to_numpy(dtype=np.float64)
    gps_n = gps_df["northing"].to_numpy(dtype=np.float64)
    gps_z = gps_df["altitude_m"].to_numpy(dtype=np.float64)

    if keep_converted:
        if not converted_csv_dir:
            converted_csv_dir = os.path.join(os.path.dirname(output_las) or ".", "converted_csv")
        os.makedirs(converted_csv_dir, exist_ok=True)

    tmp_output = output_las + ".partial"
    if os.path.exists(tmp_output):
        os.remove(tmp_output)

    writer = None
    chosen_mode: Optional[str] = None
    gps_t_mode: Optional[np.ndarray] = None
    lidar_start_ns: Optional[int] = None
    gps_start_ns: Optional[int] = None
    total_points = 0
    total_scans = 0
    debug_csv_count = 0
    trimmed_time_samples = 0
    trimmed_scans = 0
    reflectivity_scans_missing_field = 0
    header_offsets = _estimate_las_header_offsets_from_gps(
        gps_e,
        gps_n,
        gps_z,
        sensor_offset_x=sensor_offset_x,
        sensor_offset_y=sensor_offset_y,
        sensor_offset_z=sensor_offset_z,
    )

    try:
        for file_idx, raw_path in enumerate(raw_paths, start=1):
            _check_cancel(should_cancel)

            if not os.path.isfile(raw_path):
                raise ValueError(f"Raw LiDAR file does not exist: {raw_path}")

            _emit(
                emit_event,
                "file_started",
                file_index=file_idx,
                file_total=len(raw_paths),
                path=raw_path,
            )
            log(f"Processing raw LiDAR: {raw_path}")

            source = None
            try:
                source, info, _infos, xyzlut, ChanField, destagger = _open_ouster_source(raw_path)

                for scan_idx, item in enumerate(source, start=1):
                    _check_cancel(should_cancel)

                    scan = _extract_first_scan(item)
                    if scan is None:
                        continue
                    if not _scan_has_range_field(scan, ChanField):
                        continue

                    timestamp_ns, x_local, y_local, z_local, refl_local = _extract_scan_arrays(
                        scan, xyzlut, info, ChanField, destagger
                    )
                    if timestamp_ns.size == 0:
                        continue
                    # Absolute capture time is tracked separately from `lidar_t`, which may be
                    # rebased to zero in relative_start mode; the LAS gps_time field must stay
                    # absolute.
                    abs_ts_ns = timestamp_ns

                    if keep_converted and converted_csv_dir:
                        base = os.path.splitext(os.path.basename(raw_path))[0]
                        debug_csv_path = os.path.join(converted_csv_dir, f"{base}_{scan_idx:06d}.csv")
                        _write_debug_converted_csv(debug_csv_path, timestamp_ns, x_local, y_local, z_local)
                        debug_csv_count += 1

                    if chosen_mode is None:
                        _, _, chosen_mode = choose_time_mode(timestamp_ns, gps_t_raw, time_mode)
                        log(f"Time alignment mode: {chosen_mode}")
                        _emit(emit_event, "status", message=f"Time mode selected: {chosen_mode}")

                        if chosen_mode == "relative_start":
                            lidar_start_ns = int(timestamp_ns[0])
                            gps_start_ns = int(gps_t_raw[0])
                            gps_t_mode = gps_t_raw - gps_start_ns
                        else:
                            gps_t_mode = gps_t_raw

                    if chosen_mode == "relative_start":
                        assert lidar_start_ns is not None
                        lidar_t = timestamp_ns - lidar_start_ns
                    else:
                        lidar_t = timestamp_ns

                    assert gps_t_mode is not None
                    lidar_t_trimmed, in_range = _trim_query_to_gps_range(
                        lidar_t,
                        gps_t_mode,
                        label=f"{os.path.basename(raw_path)} scan {scan_idx}",
                        min_points=0,
                    )
                    dropped_now = int(in_range.size - int(in_range.sum()))
                    if dropped_now > 0:
                        trimmed_time_samples += dropped_now
                        trimmed_scans += 1
                        x_local = x_local[in_range]
                        y_local = y_local[in_range]
                        z_local = z_local[in_range]
                        refl_local = refl_local[in_range]
                        abs_ts_ns = abs_ts_ns[in_range]
                    if lidar_t_trimmed.size == 0:
                        continue

                    e_interp = np.interp(lidar_t_trimmed, gps_t_mode, gps_e)
                    n_interp = np.interp(lidar_t_trimmed, gps_t_mode, gps_n)
                    z_interp = np.interp(lidar_t_trimmed, gps_t_mode, gps_z)

                    x_local_rot, y_local_rot = _rotate_local_xy_for_gps_fusion(
                        x_local,
                        y_local,
                        e_interp,
                        n_interp,
                        orientation_mode=gps_fusion_orientation,
                        sensor_yaw_deg=sensor_yaw_deg,
                    )

                    x_global = e_interp + x_local_rot + sensor_offset_x
                    y_global = n_interp + y_local_rot + sensor_offset_y
                    z_global = z_interp + z_local + sensor_offset_z

                    finite_global = np.isfinite(x_global) & np.isfinite(y_global) & np.isfinite(z_global)
                    if not np.any(finite_global):
                        continue
                    if not np.all(finite_global):
                        x_global = x_global[finite_global]
                        y_global = y_global[finite_global]
                        z_global = z_global[finite_global]
                        lidar_t_trimmed = lidar_t_trimmed[finite_global]
                        refl_local = refl_local[finite_global]
                        abs_ts_ns = abs_ts_ns[finite_global]

                    if reflectivity_filter:
                        if _scan_has_reflectivity_field(scan, ChanField):
                            refl_keep = refl_local >= float(min_reflectivity)
                            if not np.any(refl_keep):
                                continue
                            x_global = x_global[refl_keep]
                            y_global = y_global[refl_keep]
                            z_global = z_global[refl_keep]
                            refl_local = refl_local[refl_keep]
                            abs_ts_ns = abs_ts_ns[refl_keep]
                        else:
                            reflectivity_scans_missing_field += 1

                    if writer is None:
                        header = _new_las_header(utm_epsg, *header_offsets)
                        writer = laspy.open(tmp_output, mode="w", header=header)
                        _emit(emit_event, "status", message=f"Writing LAS: {output_las}")

                    assert writer is not None
                    point_record = laspy.ScaleAwarePointRecord.zeros(len(x_global), header=writer.header)
                    point_record.x = x_global
                    point_record.y = y_global
                    point_record.z = z_global
                    # gps_time holds absolute capture time in unix seconds; intensity holds
                    # sensor reflectivity (LAS stores it as uint16, sensor range is 0-255).
                    point_record.gps_time = abs_ts_ns.astype(np.float64) / 1e9
                    point_record.intensity = np.clip(refl_local, 0, 65535).astype(np.uint16)
                    writer.write_points(point_record)

                    total_points += len(x_global)
                    total_scans += 1

                    if scan_idx % 5 == 0:
                        _emit(
                            emit_event,
                            "scan_progress",
                            file_index=file_idx,
                            file_total=len(raw_paths),
                            scan_index=scan_idx,
                            total_scans=total_scans,
                            total_points=total_points,
                        )

                _emit(
                    emit_event,
                    "file_completed",
                    file_index=file_idx,
                    file_total=len(raw_paths),
                    path=raw_path,
                    total_points=total_points,
                )
            finally:
                close_fn = getattr(source, "close", None)
                if callable(close_fn):
                    close_fn()

        if writer is None or total_points == 0:
            raise ValueError("No valid LiDAR points were produced from raw input.")

        writer.close()
        writer = None
        os.replace(tmp_output, output_las)

        log(f"Done. Final LAS: {output_las}")
        _emit(emit_event, "status", message=f"Done. Final LAS: {output_las}")
        return {
            "mode": "raw",
            "output_las": output_las,
            "raw_files": len(raw_paths),
            "total_scans": total_scans,
            "total_points": total_points,
            "debug_csv_count": debug_csv_count,
            "time_mode_used": chosen_mode or time_mode,
            "gps_fusion_orientation": gps_fusion_orientation,
            "sensor_yaw_deg": float(sensor_yaw_deg),
            "trimmed_time_samples": int(trimmed_time_samples),
            "trimmed_scans": int(trimmed_scans),
            "reflectivity_filter": bool(reflectivity_filter),
            "min_reflectivity": int(min_reflectivity),
            "reflectivity_scans_missing_field": int(reflectivity_scans_missing_field),
        }
    except Exception:
        if writer is not None:
            writer.close()
        if os.path.exists(tmp_output):
            os.remove(tmp_output)
        raise


def slam_raw_to_single_las(
    raw_paths: List[str],
    output_las: str,
    *,
    output_osf: Optional[str],
    voxel_size: float,
    min_range: float,
    max_range: float,
    deskew_method: str,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    """
    Run Ouster SDK SLAM on raw input files and save one merged LAS map.
    """
    try:
        from ouster.sdk.mapping import SlamConfig, SlamEngine
        from ouster.sdk.osf import Writer
    except Exception as e:
        raise ValueError(
            "ouster-sdk mapping bindings are not importable. Install with: pip install ouster-sdk"
        ) from e

    if min_range < 0 or max_range <= 0:
        raise ValueError("SLAM min/max range must be positive.")
    if min_range >= max_range:
        raise ValueError("SLAM min_range must be smaller than max_range.")
    if voxel_size < 0:
        raise ValueError("SLAM voxel_size cannot be negative.")

    tmp_output = output_las + ".partial"
    if os.path.exists(tmp_output):
        os.remove(tmp_output)
    tmp_output_osf = _temp_osf_path(output_osf)
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    osf_writer = None
    skipped_osf_updates = 0
    traj_xyz_local: List[np.ndarray] = []

    try:
        for file_idx, raw_path in enumerate(raw_paths, start=1):
            _check_cancel(should_cancel)
            if not os.path.isfile(raw_path):
                raise ValueError(f"Raw LiDAR file does not exist: {raw_path}")

            _emit(
                emit_event,
                "file_started",
                file_index=file_idx,
                file_total=len(raw_paths),
                path=raw_path,
            )
            log(f"Processing raw LiDAR for SLAM: {raw_path}")

            source = None
            try:
                source = _open_raw_source(raw_path)
                if slam_engine is None:
                    config = SlamConfig()
                    config.backend = "kiss"
                    config.deskew_method = deskew_method
                    config.min_range = float(min_range)
                    config.max_range = float(max_range)
                    config.voxel_size = float(voxel_size)
                    slam_engine = SlamEngine(infos=_source_infos(source), config=config)
                    if tmp_output_osf:
                        osf_writer = Writer(tmp_output_osf, _source_infos(source))
                    _emit(
                        emit_event,
                        "status",
                        message=(
                            "SLAM configured "
                            f"(voxel={config.voxel_size}, min={config.min_range}, "
                            f"max={config.max_range}, deskew={config.deskew_method})"
                        ),
                    )

                for scan_idx, scans in enumerate(source, start=1):
                    _check_cancel(should_cancel)
                    assert slam_engine is not None
                    updated = slam_engine.update(scans)
                    if osf_writer is not None and _updated_has_any_valid_timestamps(updated):
                        osf_writer.save(updated)
                    elif osf_writer is not None:
                        skipped_osf_updates += 1
                    # One pose sample per scan is enough to compare the trajectory extent
                    # against the extracted map and detect a truncated local map.
                    pose_scan = _extract_first_scan(updated)
                    if pose_scan is not None:
                        try:
                            _ts, pose_xyz = _extract_scan_pose_sample(pose_scan)
                            traj_xyz_local.append(pose_xyz)
                        except Exception:
                            pass
                    total_scans += 1

                    if scan_idx % 10 == 0:
                        _emit(
                            emit_event,
                            "scan_progress",
                            file_index=file_idx,
                            file_total=len(raw_paths),
                            scan_index=scan_idx,
                            total_scans=total_scans,
                        )

                _emit(
                    emit_event,
                    "file_completed",
                    file_index=file_idx,
                    file_total=len(raw_paths),
                    path=raw_path,
                    total_scans=total_scans,
                )
            finally:
                close_fn = getattr(source, "close", None)
                if callable(close_fn):
                    close_fn()

        if slam_engine is None or total_scans == 0:
            raise ValueError("No scans processed for SLAM map generation.")

        if skipped_osf_updates > 0:
            _emit(
                emit_event,
                "status",
                message=(
                    f"Skipped {skipped_osf_updates} scan updates with no valid "
                    "timestamps while writing OSF."
                ),
            )

        if osf_writer is not None:
            osf_writer.close()
            osf_writer = None
            assert tmp_output_osf is not None and output_osf is not None
            os.replace(tmp_output_osf, output_osf)
            _log_and_emit(emit_event, f"Saved SLAM OSF: {output_osf}")

        _emit(emit_event, "status", message="Building final SLAM point cloud...")
        points = slam_engine.get_point_cloud()
        if points is None or len(points) == 0:
            raise ValueError("SLAM point cloud is empty.")

        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("Unexpected SLAM point cloud shape; expected Nx3 points.")

        coverage = _check_slam_map_coverage(
            points[:, :2],
            np.asarray(traj_xyz_local, dtype=np.float64)[:, :2]
            if traj_xyz_local
            else np.zeros((0, 2), dtype=np.float64),
            emit_event=emit_event,
            context="slam_map",
        )

        header = _new_las_header(None, points[:, 0].min(), points[:, 1].min(), points[:, 2].min())
        las = laspy.LasData(header)
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        _log_and_emit(emit_event, f"Done. Final SLAM LAS: {output_las}")
        return {
            "mode": "slam_map",
            "output_las": output_las,
            "raw_files": len(raw_paths),
            "total_scans": total_scans,
            "total_points": int(points.shape[0]),
            "slam_voxel_size": float(voxel_size),
            "slam_min_range": float(min_range),
            "slam_max_range": float(max_range),
            "slam_deskew_method": deskew_method,
            "output_osf": output_osf,
            **coverage,
        }
    except Exception:
        if osf_writer is not None:
            osf_writer.close()
        if os.path.exists(tmp_output):
            os.remove(tmp_output)
        if tmp_output_osf and os.path.exists(tmp_output_osf):
            os.remove(tmp_output_osf)
        raise


def slam_raw_to_gps_anchored_las(
    raw_paths: List[str],
    gps_df: pd.DataFrame,
    gps_time_col: str,
    output_las: str,
    *,
    output_osf: Optional[str],
    time_mode: str,
    utm_epsg: int,
    voxel_size: float,
    min_range: float,
    max_range: float,
    deskew_method: str,
    anchor_min_motion_m: float,
    anchor_z_mode: str,
    anchor_tilt_min_span_m: float = 50.0,
    anchor_tilt_max_deg: float = 5.0,
    reflectivity_filter: bool,
    min_reflectivity: int,
    map_voxel_size: float,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    """
    Run SLAM, then place the result in GPS coordinates with a single rigid transform.

    The transform has four degrees of freedom - yaw, easting, northing and height -
    fitted by least squares over the whole trajectory. Because it is rigid, the
    geometry SLAM produced is carried through untouched: measured on a 165 m chunk,
    surface thickness stays at the 1.0 cm pure SLAM achieves, where per-node
    optimisation against 1 m-accurate GPS smears it to 3-6 cm.

    That holds only while SLAM drift over the segment is small compared to GPS
    error. Over a few hundred metres it is; over kilometres the drift accumulates
    and a single transform will leave one end of the route misplaced, which is what
    the pose optimizer backend exists for.

    The cloud is built by replaying the raw scans against the transformed SLAM
    poses rather than from `SlamEngine.get_point_cloud()`. The engine's rolling map
    holds only coordinates, so reading it back loses reflectivity, near infrared and
    capture time, and it prunes older voxels, which silently truncated long routes.
    """
    try:
        from ouster.sdk.mapping import SlamConfig, SlamEngine
        from ouster.sdk.osf import Writer
    except Exception as e:
        raise ValueError(
            "ouster-sdk mapping bindings are not importable. Install with: pip install ouster-sdk"
        ) from e

    if min_range < 0 or max_range <= 0:
        raise ValueError("SLAM min/max range must be positive.")
    if min_range >= max_range:
        raise ValueError("SLAM min_range must be smaller than max_range.")
    if voxel_size < 0:
        raise ValueError("SLAM voxel_size cannot be negative.")
    if anchor_min_motion_m < 0:
        raise ValueError("anchor_min_motion_m cannot be negative.")
    if anchor_z_mode not in {"offset", "none", "tilt"}:
        raise ValueError("anchor_z_mode must be one of: offset, none, tilt")
    if anchor_tilt_min_span_m <= 0:
        raise ValueError("anchor_tilt_min_span_m must be positive.")
    if anchor_tilt_max_deg <= 0:
        raise ValueError("anchor_tilt_max_deg must be positive.")

    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    gps_e = gps_df["easting"].to_numpy(dtype=np.float64)
    gps_n = gps_df["northing"].to_numpy(dtype=np.float64)
    gps_z = gps_df["altitude_m"].to_numpy(dtype=np.float64)

    tmp_output = output_las + ".partial"
    if os.path.exists(tmp_output):
        os.remove(tmp_output)
    tmp_output_osf = _temp_osf_path(output_osf)
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    # One representative pose per scan drives the trajectory fit against GPS; the
    # full per-column poses are what the raw replay needs to dewarp each frame.
    column_timestamps: List[np.ndarray] = []
    column_pose_blocks: List[np.ndarray] = []
    traj_timestamps: List[int] = []
    traj_xyz_local: List[np.ndarray] = []
    osf_writer = None
    skipped_osf_updates = 0

    try:
        for file_idx, raw_path in enumerate(raw_paths, start=1):
            _check_cancel(should_cancel)
            if not os.path.isfile(raw_path):
                raise ValueError(f"Raw LiDAR file does not exist: {raw_path}")

            _emit(
                emit_event,
                "file_started",
                file_index=file_idx,
                file_total=len(raw_paths),
                path=raw_path,
            )
            log(f"Processing raw LiDAR for SLAM+GPS anchoring: {raw_path}")

            source = None
            try:
                source = _open_raw_source(raw_path)
                if slam_engine is None:
                    config = SlamConfig()
                    config.backend = "kiss"
                    config.deskew_method = deskew_method
                    config.min_range = float(min_range)
                    config.max_range = float(max_range)
                    config.voxel_size = float(voxel_size)
                    slam_engine = SlamEngine(infos=_source_infos(source), config=config)
                    if tmp_output_osf:
                        osf_writer = Writer(tmp_output_osf, _source_infos(source))
                    _emit(
                        emit_event,
                        "status",
                        message=(
                            "SLAM configured "
                            f"(voxel={config.voxel_size}, min={config.min_range}, "
                            f"max={config.max_range}, deskew={config.deskew_method})"
                        ),
                    )

                for scan_idx, scans in enumerate(source, start=1):
                    _check_cancel(should_cancel)
                    assert slam_engine is not None
                    updated = slam_engine.update(scans)
                    if osf_writer is not None and _updated_has_any_valid_timestamps(updated):
                        osf_writer.save(updated)
                    elif osf_writer is not None:
                        skipped_osf_updates += 1
                    scan = _extract_first_scan(updated)
                    if scan is not None:
                        try:
                            ts, xyz = _extract_scan_pose_sample(scan)
                            traj_timestamps.append(ts)
                            traj_xyz_local.append(xyz)
                        except Exception:
                            pass
                        try:
                            col_ts, col_poses = _extract_scan_column_poses(scan)
                            if col_ts.size:
                                column_timestamps.append(col_ts)
                                column_pose_blocks.append(col_poses)
                        except Exception:
                            pass

                    total_scans += 1
                    if scan_idx % 10 == 0:
                        _emit(
                            emit_event,
                            "scan_progress",
                            file_index=file_idx,
                            file_total=len(raw_paths),
                            scan_index=scan_idx,
                            total_scans=total_scans,
                        )

                _emit(
                    emit_event,
                    "file_completed",
                    file_index=file_idx,
                    file_total=len(raw_paths),
                    path=raw_path,
                    total_scans=total_scans,
                )
            finally:
                close_fn = getattr(source, "close", None)
                if callable(close_fn):
                    close_fn()

        if slam_engine is None or total_scans == 0:
            raise ValueError("No scans processed for SLAM map generation.")
        if len(traj_timestamps) < 2:
            raise ValueError("Not enough SLAM trajectory samples to anchor map to GPS.")

        if skipped_osf_updates > 0:
            _emit(
                emit_event,
                "status",
                message=(
                    f"Skipped {skipped_osf_updates} scan updates with no valid "
                    "timestamps while writing OSF."
                ),
            )

        if osf_writer is not None:
            osf_writer.close()
            osf_writer = None
            assert tmp_output_osf is not None and output_osf is not None
            os.replace(tmp_output_osf, output_osf)
            _log_and_emit(emit_event, f"Saved local SLAM OSF: {output_osf}")
            _emit(
                emit_event,
                "status",
                message="OSF stores local SLAM poses for rigid_fit backend (LAS is GPS-anchored output).",
            )

        if not column_pose_blocks:
            raise ValueError("No SLAM column poses were captured for the raw replay.")

        pose_ts = np.concatenate(column_timestamps)
        pose_local = np.concatenate(column_pose_blocks, axis=0)
        order = np.argsort(pose_ts, kind="stable")
        pose_ts = pose_ts[order]
        pose_local = pose_local[order]
        unique = np.concatenate([[True], np.diff(pose_ts) > 0])
        pose_ts = pose_ts[unique]
        pose_local = pose_local[unique]

        spike_windows, spike_steps = _find_transient_pose_spikes(pose_ts, pose_local[:, :3, 3])
        spike_records = []
        for a, b in spike_windows:
            jump = float(np.linalg.norm(pose_local[a:b, :3, 3] - pose_local[a - 1, :3, 3], axis=1).max())
            spike_records.append(
                {
                    "start_unix_s": float(pose_ts[a]) / 1e9,
                    "end_unix_s": float(pose_ts[b - 1]) / 1e9,
                    "duration_s": float(pose_ts[b] - pose_ts[a - 1]) / 1e9,
                    "max_jump_m": jump,
                    "column_poses": int(b - a),
                }
            )
            _log_and_emit(
                emit_event,
                "WARNING: transient pose spike at "
                f"{_format_unix_utc(float(pose_ts[a]) / 1e9)}, {b - a} column poses up to "
                f"{jump:.1f} m off; replaced by interpolation between its neighbours.",
            )
        for idx in spike_steps:
            _log_and_emit(
                emit_event,
                "WARNING: pose step at "
                f"{_format_unix_utc(float(pose_ts[idx]) / 1e9)} that does not return within "
                f"{POSE_SPIKE_MAX_DURATION_S:.0f} s; left as is. Inspect the cloud there.",
            )
        if spike_windows:
            pose_local = _repair_pose_spikes(pose_ts, pose_local, spike_windows)

        traj_t = np.asarray(traj_timestamps, dtype=np.int64)
        traj_xyz = np.asarray(traj_xyz_local, dtype=np.float64)
        if spike_windows:
            # The per-scan samples the fit uses come from the same poses; a sample
            # inside a spike is a 27 m outlier in the fit, so drop it.
            keep = np.ones(traj_t.size, dtype=bool)
            for a, b in spike_windows:
                keep &= ~((traj_t > pose_ts[a - 1]) & (traj_t < pose_ts[b]))
            traj_t = traj_t[keep]
            traj_xyz = traj_xyz[keep]

        slam_t, gps_t, chosen_time_mode = choose_time_mode(traj_t, gps_t_raw, time_mode)
        slam_t, in_range = _trim_query_to_gps_range(
            slam_t,
            gps_t,
            label="SLAM trajectory",
            min_points=2,
        )
        if not np.all(in_range):
            traj_xyz = traj_xyz[in_range]
        gps_x_interp = np.interp(slam_t, gps_t, gps_e)
        gps_y_interp = np.interp(slam_t, gps_t, gps_n)
        gps_z_interp = np.interp(slam_t, gps_t, gps_z)

        slam_xy = traj_xyz[:, :2]
        gps_xy = np.column_stack([gps_x_interp, gps_y_interp])
        rotation, translation, anchor_method = _fit_xy_rigid_transform(
            slam_xy,
            gps_xy,
            min_motion_m=anchor_min_motion_m,
        )
        if anchor_z_mode == "none":
            z_offset = 0.0
        else:
            z_offset = float(np.median(gps_z_interp - traj_xyz[:, 2]))

        # Four-parameter placement first: yaw, easting, northing, height offset.
        transform = _rigid_transform_matrix(
            _yaw_rotation_3d(rotation),
            np.array([translation[0], translation[1], z_offset], dtype=np.float64),
        )

        tilt_diagnostics: Dict[str, float] = {}
        tilt_method = "tilt_not_requested"
        if anchor_z_mode == "tilt":
            placed = _apply_transform(transform, traj_xyz)
            levelling, tilt_method, tilt_diagnostics = _level_transform(
                placed,
                gps_z_interp,
                min_span_m=anchor_tilt_min_span_m,
                max_tilt_deg=anchor_tilt_max_deg,
            )
            transform = levelling @ transform
            z_offset += tilt_diagnostics.get("tilt_residual_offset_m", 0.0)
            anchor_method = f"{anchor_method}+{tilt_method}"

        aligned_traj = _apply_transform(transform, traj_xyz)
        aligned_traj_xy = aligned_traj[:, :2]
        aligned_traj_z = aligned_traj[:, 2]

        qa_metrics = _compute_xy_alignment_metrics(gps_xy, aligned_traj_xy)
        z_residual = aligned_traj_z - gps_z_interp
        qa_metrics["z_mean_m"] = float(np.mean(z_residual))
        qa_metrics["z_rmse_m"] = float(np.sqrt(np.mean(z_residual**2)))
        qa_metrics["z_p95_abs_m"] = float(np.percentile(np.abs(z_residual), 95))

        qa_extra = {
            "anchor_method": anchor_method,
            "anchor_rotation_deg": _rotation_deg(rotation),
            "anchor_translation_x": float(translation[0]),
            "anchor_translation_y": float(translation[1]),
            "anchor_z_offset": z_offset,
            "anchor_z_mode": anchor_z_mode,
            **tilt_diagnostics,
            "pose_spikes_repaired": spike_records,
            "pose_steps_unrepaired": len(spike_steps),
        }

        time_offset = _estimate_gps_time_offset(slam_t, slam_xy, gps_t, gps_e, gps_n)
        qa_extra.update(time_offset)
        best_offset = time_offset.get("gps_time_offset_best_s")
        if best_offset is not None:
            message = (
                f"GPS time offset check: residual is smallest at {best_offset:+.2f} s "
                f"({time_offset['gps_time_offset_median_at_best_m']:.2f} m median, against "
                f"{time_offset['gps_time_offset_median_at_zero_m']:.2f} m at 0 s)"
            )
            if time_offset["gps_time_offset_median_at_best_m"] > 2.0:
                # No shift makes the two agree, so the trajectory itself is wrong -
                # a diverged SLAM run, seen with --slam-deskew-method none - and the
                # location of the minimum means nothing.
                message = (
                    "WARNING: " + message + ". No time shift brings the trajectory onto "
                    "GPS; suspect the SLAM trajectory, not the clocks."
                )
            elif abs(best_offset) > 0.1:
                message = (
                    "WARNING: " + message + ". The GPS and LiDAR clocks disagree; check "
                    "--gps-time-column before trusting the placement."
                )
            _log_and_emit(emit_event, message)

        _emit(
            emit_event,
            "status",
            message=(
                f"Anchoring transform ready "
                f"(method={anchor_method}, yaw_deg={_rotation_deg(rotation):.3f}, "
                f"time_mode={chosen_time_mode})"
            ),
        )

        # Move the SLAM poses into the georeferenced frame, then let the replay place
        # the points. Transforming poses rather than points is what keeps the result
        # rigid: every point inherits the same transform, so no relative geometry can
        # change. `transform` was composed above and already carries the tilt when one
        # was fitted.
        pose_cols = transform @ pose_local

        _emit(emit_event, "status", message="Extracting point cloud from raw replay with anchored poses...")
        map_points, map_attrs, pose_lookup = _build_point_cloud_from_raw_with_poses(
            raw_paths,
            pose_timestamps_ns=pose_ts,
            column_poses=pose_cols,
            min_range=min_range,
            max_range=max_range,
            reflectivity_filter=reflectivity_filter,
            min_reflectivity=min_reflectivity,
            map_voxel_size=map_voxel_size,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        if map_points.size == 0:
            raise ValueError("Anchored point cloud is empty.")

        # The replay covers wherever the raw scans reached, so the truncation this
        # check was added for cannot happen any more; it stays because a coverage
        # shortfall would now indicate a pose-lookup problem instead.
        aligned_traj = np.column_stack([aligned_traj_xy, aligned_traj_z])
        coverage = _check_slam_map_coverage(
            map_points[:, :2],
            aligned_traj[:, :2],
            emit_event=emit_event,
            context="slam_gps_anchor/rigid_fit",
        )
        qa_extra.update(coverage)

        header = _new_las_header(
            utm_epsg,
            map_points[:, 0].min(),
            map_points[:, 1].min(),
            map_points[:, 2].min(),
            point_format=8,
            extra_dims=[("range_m", "f4", "Sensor-to-point range in metres")],
        )
        las = laspy.LasData(header)
        las.x = map_points[:, 0]
        las.y = map_points[:, 1]
        las.z = map_points[:, 2]
        if map_attrs.shape[0] == map_points.shape[0] and map_attrs.shape[1] >= len(
            MAP_ATTR_COLUMNS
        ):
            las.gps_time = map_attrs[:, 0]
            las.intensity = np.clip(map_attrs[:, 1], 0, 65535).astype(np.uint16)
            las.nir = np.clip(map_attrs[:, 2], 0, 65535).astype(np.uint16)
            las.range_m = map_attrs[:, 3].astype(np.float32)
            las.user_data = np.clip(np.rint(map_attrs[:, 4]), 0, 255).astype(np.uint8)
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        trajectory_path = _write_trajectory_csv(output_las, pose_ts, pose_cols)

        qa_path, qa_pass = _write_anchor_qa_report(
            output_las,
            backend="rigid_fit",
            time_mode_used=chosen_time_mode,
            metrics=qa_metrics,
            extra=qa_extra,
        )
        _log_and_emit(
            emit_event,
            f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
            f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} ({qa_path})",
        )

        _log_and_emit(emit_event, f"Done. Final anchored LAS: {output_las}")
        return {
            "mode": "slam_gps_anchor",
            "output_las": output_las,
            "raw_files": len(raw_paths),
            "total_scans": total_scans,
            "total_points": int(map_points.shape[0]),
            "time_mode_used": chosen_time_mode,
            "anchor_method": anchor_method,
            "anchor_rotation_deg": _rotation_deg(rotation),
            "anchor_translation_x": float(translation[0]),
            "anchor_translation_y": float(translation[1]),
            "anchor_z_offset": z_offset,
            "slam_voxel_size": float(voxel_size),
            "slam_min_range": float(min_range),
            "slam_max_range": float(max_range),
            "slam_deskew_method": deskew_method,
            "map_voxel_size": float(map_voxel_size),
            "reflectivity_filter": bool(reflectivity_filter),
            "min_reflectivity": int(min_reflectivity),
            "reflectivity_scans_missing_field": int(pose_lookup["reflectivity_scans_missing_field"]),
            "near_ir_scans_missing_field": int(pose_lookup["near_ir_scans_missing_field"]),
            "output_trajectory_csv": trajectory_path,
            "qa_report_path": qa_path,
            "qa_pass": qa_pass,
            "qa_xy_rmse_m": qa_metrics["xy_rmse_m"],
            "qa_xy_median_m": qa_metrics["xy_median_m"],
            "qa_xy_p95_m": qa_metrics["xy_p95_m"],
            "qa_z_rmse_m": qa_metrics["z_rmse_m"],
            "output_osf": output_osf,
            "output_osf_kind": "local_slam_unanchored",
        }
    except Exception:
        if osf_writer is not None:
            osf_writer.close()
        if os.path.exists(tmp_output):
            os.remove(tmp_output)
        if tmp_output_osf and os.path.exists(tmp_output_osf):
            os.remove(tmp_output_osf)
        raise


def slam_raw_to_gps_anchored_las_pose_optimizer(
    raw_paths: List[str],
    gps_df: pd.DataFrame,
    gps_time_col: str,
    output_las: str,
    *,
    output_osf: Optional[str],
    time_mode: str,
    utm_epsg: int,
    voxel_size: float,
    min_range: float,
    max_range: float,
    deskew_method: str,
    anchor_min_motion_m: float,
    anchor_z_mode: str,
    poseopt_key_frame_distance: float,
    poseopt_constraints_every_m: float,
    poseopt_constraint_weights: str,
    poseopt_max_iterations: int,
    poseopt_map_voxel_size: float,
    reflectivity_filter: bool,
    min_reflectivity: int,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    """
    Run SLAM, optimize trajectory with GPS absolute constraints, then export anchored LAS.
    """
    try:
        from ouster.sdk.mapping import (
            AbsolutePoseConstraint,
            PoseOptimizer,
            SamplingMode,
            SlamConfig,
            SlamEngine,
            SolverConfig,
        )
        from ouster.sdk.osf import Writer
    except Exception as e:
        raise ValueError(
            "ouster-sdk mapping/osf bindings are required for pose optimizer anchoring."
        ) from e

    if min_range < 0 or max_range <= 0:
        raise ValueError("SLAM min/max range must be positive.")
    if min_range >= max_range:
        raise ValueError("SLAM min_range must be smaller than max_range.")
    if voxel_size < 0:
        raise ValueError("SLAM voxel_size cannot be negative.")
    if poseopt_constraints_every_m <= 0:
        raise ValueError("poseopt_constraints_every_m must be > 0.")
    if poseopt_key_frame_distance <= 0:
        raise ValueError("poseopt_key_frame_distance must be > 0.")
    if poseopt_max_iterations <= 0:
        raise ValueError("poseopt_max_iterations must be > 0.")
    if anchor_min_motion_m < 0:
        raise ValueError("anchor_min_motion_m cannot be negative.")
    if anchor_z_mode == "tilt":
        # `tilt` is the default because it is what rigid_fit should do. The pose
        # optimizer already solves height at every node, so there is no single tilt
        # for it to correct; treat the mode as `offset` rather than refusing to run,
        # since nobody asked for a tilt here - the default did.
        anchor_z_mode = "offset"

    weights = _parse_xyz_weights(poseopt_constraint_weights)

    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    gps_e = gps_df["easting"].to_numpy(dtype=np.float64)
    gps_n = gps_df["northing"].to_numpy(dtype=np.float64)
    gps_z = gps_df["altitude_m"].to_numpy(dtype=np.float64)

    temp_dir = tempfile.mkdtemp(prefix="lidar_poseopt_")
    osf_initial = os.path.join(temp_dir, "slam_initial.osf")
    tmp_output = output_las + ".partial"
    if os.path.exists(tmp_output):
        os.remove(tmp_output)
    tmp_output_osf = _temp_osf_path(output_osf)
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    constraints_added = 0
    chosen_time_mode = "auto"
    final_cost = None
    writer = None
    skipped_osf_updates = 0
    gap_tracker = _CaptureGapTracker()
    trajectory_path: Optional[str] = None

    try:
        for file_idx, raw_path in enumerate(raw_paths, start=1):
            _check_cancel(should_cancel)
            if not os.path.isfile(raw_path):
                raise ValueError(f"Raw LiDAR file does not exist: {raw_path}")

            _emit(
                emit_event,
                "file_started",
                file_index=file_idx,
                file_total=len(raw_paths),
                path=raw_path,
            )
            log(f"Processing raw LiDAR for SLAM+GPS pose optimization: {raw_path}")
            gap_tracker.start_file(raw_path)

            source = None
            try:
                source = _open_raw_source(raw_path)
                if slam_engine is None:
                    config = SlamConfig()
                    config.backend = "kiss"
                    config.deskew_method = deskew_method
                    config.min_range = float(min_range)
                    config.max_range = float(max_range)
                    config.voxel_size = float(voxel_size)
                    slam_engine = SlamEngine(infos=_source_infos(source), config=config)
                    writer = Writer(osf_initial, _source_infos(source))
                    _emit(
                        emit_event,
                        "status",
                        message=(
                            "SLAM configured "
                            f"(voxel={config.voxel_size}, min={config.min_range}, "
                            f"max={config.max_range}, deskew={config.deskew_method})"
                        ),
                    )

                for scan_idx, scans in enumerate(source, start=1):
                    _check_cancel(should_cancel)
                    assert slam_engine is not None

                    gap = gap_tracker.observe(_extract_first_scan(scans))
                    if gap is not None:
                        message = (
                            f"Capture gap of {gap['gap_s']:.2f} s at "
                            f"{_format_unix_utc(gap['after_unix_s'])} UTC "
                            f"({os.path.basename(raw_path)}). Scan-to-map "
                            "registration cannot bridge a hole this long; SLAM may "
                            "diverge here and not recover."
                        )
                        log(f"WARNING: {message}")
                        _emit(emit_event, "status", message=message)

                    updated = slam_engine.update(scans)
                    assert writer is not None
                    if _updated_has_any_valid_timestamps(updated):
                        writer.save(updated)
                    else:
                        skipped_osf_updates += 1
                    total_scans += 1

                    if scan_idx % 10 == 0:
                        _emit(
                            emit_event,
                            "scan_progress",
                            file_index=file_idx,
                            file_total=len(raw_paths),
                            scan_index=scan_idx,
                            total_scans=total_scans,
                        )

                _emit(
                    emit_event,
                    "file_completed",
                    file_index=file_idx,
                    file_total=len(raw_paths),
                    path=raw_path,
                    total_scans=total_scans,
                )
            finally:
                close_fn = getattr(source, "close", None)
                if callable(close_fn):
                    close_fn()

        if writer is not None:
            writer.close()
            writer = None

        if total_scans == 0:
            raise ValueError("No scans processed for SLAM map generation.")

        capture_gaps = gap_tracker.summary()
        if capture_gaps["count"] > 0:
            summary_message = (
                f"{capture_gaps['count']} capture gap(s) totalling "
                f"{capture_gaps['total_s']:.2f} s were crossed while building the SLAM "
                "trajectory. Poses after the first gap may be unusable. Consider "
                "processing the continuous stretches as separate runs and merging the "
                "georeferenced outputs, which stay in the same CRS."
            )
            log(f"WARNING: {summary_message}")
            _emit(emit_event, "status", message=summary_message)

        if capture_gaps["incomplete_scan_count"] > 0:
            details = capture_gaps["incomplete_scans"]
            at_boundary = sum(1 for item in details if item["at_file_boundary"])
            worst = min(
                details,
                key=lambda item: item["valid_columns"] / max(item["total_columns"], 1),
                default=None,
            )
            incomplete_message = (
                f"{capture_gaps['incomplete_scan_count']} frame(s) arrived "
                f"incomplete, {capture_gaps['missing_columns']} columns missing in "
                "total. That means UDP packets were lost during capture. The gap "
                "check above cannot see this on its own, because the columns that "
                "did arrive still carry timestamps and keep the stream looking "
                "continuous."
            )
            if at_boundary:
                incomplete_message += (
                    f" {at_boundary} of them are the first or last scan of a chunk "
                    "file, where a frame split across the rotation is expected."
                )
            if worst is not None:
                incomplete_message += (
                    f" Worst: frame {worst['frame_id']} with "
                    f"{worst['valid_columns']}/{worst['total_columns']} columns"
                )
                if worst["unix_s"] is not None:
                    incomplete_message += (
                        f" at {_format_unix_utc(worst['unix_s'])} UTC"
                    )
                incomplete_message += f" in {worst['file']}."
            log(f"WARNING: {incomplete_message}")
            _emit(emit_event, "status", message=incomplete_message)

        if skipped_osf_updates > 0:
            _emit(
                emit_event,
                "status",
                message=(
                    f"Skipped {skipped_osf_updates} scan updates with no valid "
                    "timestamps while building pose-optimizer OSF."
                ),
            )

        _emit(emit_event, "status", message="Running pose optimization with GPS constraints...")

        solver = SolverConfig()
        solver.key_frame_distance = float(poseopt_key_frame_distance)
        solver.fix_first_node = False
        solver.process_printout = False
        solver.max_num_iterations = int(poseopt_max_iterations)
        po = PoseOptimizer(osf_initial, solver)

        ts_cols = np.asarray(po.get_timestamps(SamplingMode.COLUMNS), dtype=np.int64)
        poses_cols = np.asarray(po.get_poses(SamplingMode.COLUMNS), dtype=np.float64)
        if ts_cols.size < 2:
            raise ValueError("Pose optimizer trajectory has insufficient samples.")

        slam_t, gps_t, chosen_time_mode = choose_time_mode(ts_cols, gps_t_raw, time_mode)
        slam_t, constraints_in_range = _trim_query_to_gps_range(
            slam_t,
            gps_t,
            label="Pose optimizer trajectory for constraints",
            min_points=2,
        )
        if not np.all(constraints_in_range):
            ts_cols = ts_cols[constraints_in_range]
            poses_cols = poses_cols[constraints_in_range]
        gps_x_interp = np.interp(slam_t, gps_t, gps_e)
        gps_y_interp = np.interp(slam_t, gps_t, gps_n)
        gps_z_interp = np.interp(slam_t, gps_t, gps_z)

        # Constraints are placed every `poseopt_constraints_every_m` of distance
        # travelled along the SLAM trajectory, so a diverged trajectory generates
        # them without limit: the distance counter clears the spacing again within a
        # handful of columns and every column earns a constraint. Left unbounded that
        # turns a run that should fail in minutes into hours of solving a hopeless
        # problem. The GPS track is the ground truth for how far the rig actually
        # went, so cap the count against it with a wide margin for legitimate jitter.
        gps_path_m = float(
            np.sum(np.hypot(np.diff(gps_x_interp), np.diff(gps_y_interp)))
        )
        max_constraints = max(
            100, int(20.0 * gps_path_m / max(float(poseopt_constraints_every_m), 0.1))
        )

        local_xy = poses_cols[:, :2, 3]
        last_xy = None
        dist_since = float("inf")
        for idx in range(len(ts_cols)):
            xy = local_xy[idx]
            if last_xy is not None:
                dist_since += float(np.linalg.norm(xy - last_xy))
            last_xy = xy
            if constraints_added > 0 and dist_since < poseopt_constraints_every_m:
                continue

            if constraints_added >= max_constraints:
                raise ValueError(
                    f"SLAM trajectory diverged: {constraints_added} GPS constraints "
                    f"were needed before {_format_unix_utc(ts_cols[idx] / 1e9)} UTC, "
                    f"against roughly {gps_path_m / max(float(poseopt_constraints_every_m), 0.1):.0f} "
                    f"expected for a {gps_path_m:.0f} m GPS track. Divergence usually "
                    "follows a capture gap; check the capture-gap warnings above and "
                    "process the continuous stretches separately."
                )

            pose = np.eye(4, dtype=np.float64)
            pose[0, 3] = float(gps_x_interp[idx])
            pose[1, 3] = float(gps_y_interp[idx])
            if anchor_z_mode == "offset":
                pose[2, 3] = float(gps_z_interp[idx])
                w = weights
            else:
                w = np.array([weights[0], weights[1], 0.0], dtype=np.float64)

            constraint = AbsolutePoseConstraint(
                timestamp=np.uint64(ts_cols[idx]),
                pose=pose,
                rotation_weight=0.0,
                translation_weight=w,
            )
            po.add_constraint(constraint)
            constraints_added += 1
            dist_since = 0.0

        if constraints_added == 0:
            raise ValueError("No GPS constraints were added for pose optimization.")

        aligned = bool(po.initialize_trajectory_alignment())
        _emit(
            emit_event,
            "status",
            message=f"Pose optimizer constraints added: {constraints_added} (aligned={aligned})",
        )

        po.solve(int(poseopt_max_iterations))
        final_cost = float(po.get_cost_value())

        if tmp_output_osf:
            po.save(tmp_output_osf)
            assert output_osf is not None
            os.replace(tmp_output_osf, output_osf)
            _log_and_emit(emit_event, f"Saved optimized OSF: {output_osf}")

        ts_opt = np.asarray(po.get_timestamps(SamplingMode.COLUMNS), dtype=np.int64)
        poses_opt = np.asarray(po.get_poses(SamplingMode.COLUMNS), dtype=np.float64)
        if ts_opt.size < 2:
            raise ValueError("Pose optimizer output trajectory has insufficient samples.")
        if poses_opt.ndim != 3 or poses_opt.shape[1:] != (4, 4):
            raise ValueError("Unexpected optimized pose shape from PoseOptimizer.")
        if poses_opt.shape[0] != ts_opt.shape[0]:
            raise ValueError("Optimized pose/timestamp count mismatch.")

        slam_t_opt, gps_t_opt, chosen_time_mode = choose_time_mode(ts_opt, gps_t_raw, time_mode)
        slam_t_opt, qa_in_range = _trim_query_to_gps_range(
            slam_t_opt,
            gps_t_opt,
            label="Pose optimizer trajectory for QA",
            min_points=2,
        )
        if not np.all(qa_in_range):
            ts_opt = ts_opt[qa_in_range]
            poses_opt = poses_opt[qa_in_range]
        gps_x_opt = np.interp(slam_t_opt, gps_t_opt, gps_e)
        gps_y_opt = np.interp(slam_t_opt, gps_t_opt, gps_n)
        gps_z_opt = np.interp(slam_t_opt, gps_t_opt, gps_z)

        optimized_xy = poses_opt[:, :2, 3]
        qa_metrics = _compute_xy_alignment_metrics(
            np.column_stack([gps_x_opt, gps_y_opt]),
            optimized_xy,
        )
        z_residual = poses_opt[:, 2, 3] - gps_z_opt
        qa_metrics["z_mean_m"] = float(np.mean(z_residual))
        qa_metrics["z_rmse_m"] = float(np.sqrt(np.mean(z_residual**2)))
        qa_metrics["z_p95_abs_m"] = float(np.percentile(np.abs(z_residual), 95))

        qa_extra = {
            "constraints_added": int(constraints_added),
            "poseopt_cost": final_cost,
            "anchor_z_mode": anchor_z_mode,
            # Recorded because the QA metrics compare the trajectory against GPS and
            # cannot see that the trajectory was built across a break in the data.
            "capture_gaps": capture_gaps,
        }

        _emit(emit_event, "status", message="Extracting point cloud from raw replay with optimized poses...")
        map_points, map_attrs, pose_lookup = _build_point_cloud_from_raw_with_poses(
            raw_paths,
            pose_timestamps_ns=ts_opt,
            column_poses=poses_opt,
            min_range=min_range,
            max_range=max_range,
            reflectivity_filter=reflectivity_filter,
            min_reflectivity=min_reflectivity,
            map_voxel_size=poseopt_map_voxel_size,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        if map_points.size == 0:
            raise ValueError("Optimized point cloud is empty.")

        # Point format 8 rather than 6 so the sensor's ambient near-infrared channel
        # lands in the standard `nir` field that ordinary LAS readers understand.
        # Range goes to an extra dimension because LAS has no standard slot for it.
        header = _new_las_header(
            utm_epsg,
            map_points[:, 0].min(),
            map_points[:, 1].min(),
            map_points[:, 2].min(),
            point_format=8,
            extra_dims=[
                ("range_m", "f4", "Sensor-to-point range in metres"),
            ],
        )
        las = laspy.LasData(header)
        las.x = map_points[:, 0]
        las.y = map_points[:, 1]
        las.z = map_points[:, 2]
        if map_attrs.shape[0] == map_points.shape[0] and map_attrs.shape[1] >= len(
            MAP_ATTR_COLUMNS
        ):
            # gps_time holds capture time in unix seconds and intensity the sensor
            # reflectivity, 0-255. nir carries the ambient near-infrared channel as
            # the SDK reports it: an 8-bit measurement already shifted left by four
            # bits, so values are multiples of 16 running 0-4080. It is stored
            # unscaled to stay lossless and to match what other Ouster tooling
            # displays. user_data holds the beam row, which fits a uint8 for every
            # current Ouster line.
            las.gps_time = map_attrs[:, 0]
            las.intensity = np.clip(map_attrs[:, 1], 0, 65535).astype(np.uint16)
            las.nir = np.clip(map_attrs[:, 2], 0, 65535).astype(np.uint16)
            las.range_m = map_attrs[:, 3].astype(np.float32)
            las.user_data = np.clip(np.rint(map_attrs[:, 4]), 0, 255).astype(np.uint8)
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        trajectory_path = _write_trajectory_csv(output_las, ts_opt, poses_opt)

        qa_path, qa_pass = _write_anchor_qa_report(
            output_las,
            backend="pose_optimizer",
            time_mode_used=chosen_time_mode,
            metrics=qa_metrics,
            extra=qa_extra,
        )
        _log_and_emit(
            emit_event,
            f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
            f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} ({qa_path})",
        )

        _log_and_emit(emit_event, f"Done. Final anchored LAS: {output_las}")
        return {
            "mode": "slam_gps_anchor",
            "anchor_backend": "pose_optimizer",
            "output_las": output_las,
            "raw_files": len(raw_paths),
            "total_scans": total_scans,
            "total_points": int(map_points.shape[0]),
            "time_mode_used": chosen_time_mode,
            "constraints_added": int(constraints_added),
            "poseopt_cost": final_cost,
            "slam_voxel_size": float(voxel_size),
            "slam_min_range": float(min_range),
            "slam_max_range": float(max_range),
            "slam_deskew_method": deskew_method,
            "qa_report_path": qa_path,
            "qa_pass": qa_pass,
            "qa_xy_rmse_m": qa_metrics["xy_rmse_m"],
            "qa_xy_median_m": qa_metrics["xy_median_m"],
            "qa_xy_p95_m": qa_metrics["xy_p95_m"],
            "qa_z_rmse_m": qa_metrics["z_rmse_m"],
            "pose_lookup_inlier_columns": int(pose_lookup["inlier_columns"]),
            "pose_lookup_dropped_columns": int(pose_lookup["dropped_columns"]),
            "pose_lookup_exact_columns": int(pose_lookup["exact_columns"]),
            "pose_lookup_nearest_columns": int(pose_lookup["nearest_columns"]),
            "pose_lookup_max_delta_ns": int(pose_lookup["max_delta_ns"]),
            "reflectivity_filter": bool(reflectivity_filter),
            "min_reflectivity": int(min_reflectivity),
            "reflectivity_scans_missing_field": int(pose_lookup["reflectivity_scans_missing_field"]),
            "near_ir_scans_missing_field": int(pose_lookup["near_ir_scans_missing_field"]),
            "output_trajectory_csv": trajectory_path,
            "capture_gaps": capture_gaps,
            "output_osf": output_osf,
            "output_osf_kind": "optimized_gps_anchored",
        }
    except Exception:
        if os.path.exists(tmp_output):
            os.remove(tmp_output)
        if tmp_output_osf and os.path.exists(tmp_output_osf):
            os.remove(tmp_output_osf)
        raise
    finally:
        close_writer = getattr(writer, "close", None)
        if callable(close_writer):
            try:
                close_writer()
            except Exception:
                pass
        shutil.rmtree(temp_dir, ignore_errors=True)


def fuse_one_csv_chunk(
    lidar_csv: str,
    gps_df: pd.DataFrame,
    gps_time_col: str,
    output_prefix: str,
    *,
    time_mode: str,
    utm_epsg: int,
    sensor_offset_x: float,
    sensor_offset_y: float,
    sensor_offset_z: float,
    gps_fusion_orientation: str,
    sensor_yaw_deg: float,
) -> Tuple[int, str]:
    log(f"Loading LiDAR CSV: {lidar_csv}")
    lidar_df = read_ouster_csv(lidar_csv)

    needed_lidar_cols = ["timestamp (ns)", "x1 (m)", "y1 (m)", "z1 (m)"]
    missing_lidar = [c for c in needed_lidar_cols if c not in lidar_df.columns]
    if missing_lidar:
        raise ValueError(f"Missing required LiDAR columns: {missing_lidar}")

    lidar_df = lidar_df[needed_lidar_cols].copy()
    lidar_df["timestamp_ns"] = lidar_df["timestamp (ns)"].astype("int64")
    lidar_df["x_local"] = -lidar_df["x1 (m)"].astype("float64")
    lidar_df["y_local"] = -lidar_df["y1 (m)"].astype("float64")
    lidar_df["z_local"] = lidar_df["z1 (m)"].astype("float64")

    lidar_t_raw = lidar_df["timestamp_ns"].to_numpy(dtype=np.int64)
    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    lidar_t, gps_t, chosen_mode = choose_time_mode(lidar_t_raw, gps_t_raw, time_mode)
    log(f"Time alignment mode for {os.path.basename(lidar_csv)}: {chosen_mode}")

    lidar_t, in_range = _trim_query_to_gps_range(
        lidar_t,
        gps_t,
        label=f"{os.path.basename(lidar_csv)}",
        min_points=1,
    )
    if not np.all(in_range):
        lidar_df = lidar_df.loc[in_range].reset_index(drop=True)

    e_interp = np.interp(lidar_t, gps_t, gps_df["easting"].to_numpy(dtype=np.float64))
    n_interp = np.interp(lidar_t, gps_t, gps_df["northing"].to_numpy(dtype=np.float64))
    z_interp = np.interp(lidar_t, gps_t, gps_df["altitude_m"].to_numpy(dtype=np.float64))

    x_local_rot, y_local_rot = _rotate_local_xy_for_gps_fusion(
        lidar_df["x_local"].to_numpy(dtype=np.float64),
        lidar_df["y_local"].to_numpy(dtype=np.float64),
        e_interp,
        n_interp,
        orientation_mode=gps_fusion_orientation,
        sensor_yaw_deg=sensor_yaw_deg,
    )

    lidar_df["x_global"] = e_interp + x_local_rot + sensor_offset_x
    lidar_df["y_global"] = n_interp + y_local_rot + sensor_offset_y
    lidar_df["z_global"] = z_interp + lidar_df["z_local"] + sensor_offset_z

    out_csv = output_prefix + ".csv"
    out_las = output_prefix + ".las"
    out_dir = os.path.dirname(out_csv) or "."
    os.makedirs(out_dir, exist_ok=True)

    log(f"Writing georeferenced CSV: {out_csv}")
    lidar_df[["timestamp_ns", "x_global", "y_global", "z_global"]].to_csv(out_csv, index=False)

    log(f"Writing LAS 1.4: {out_las}")
    header = _new_las_header(utm_epsg, lidar_df["x_global"].min(), lidar_df["y_global"].min(), lidar_df["z_global"].min())
    las = laspy.LasData(header)
    las.x = lidar_df["x_global"].to_numpy(dtype=np.float64)
    las.y = lidar_df["y_global"].to_numpy(dtype=np.float64)
    las.z = lidar_df["z_global"].to_numpy(dtype=np.float64)
    las.write(out_las)

    return len(lidar_df), chosen_mode


def fuse_csv_chunks(
    lidar_paths: List[str],
    output_prefixes: List[str],
    gps_df: pd.DataFrame,
    gps_time_col: str,
    *,
    time_mode: str,
    utm_epsg: int,
    sensor_offset_x: float,
    sensor_offset_y: float,
    sensor_offset_z: float,
    gps_fusion_orientation: str,
    sensor_yaw_deg: float,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    total_points = 0
    failures: List[Tuple[str, str]] = []

    for idx, (lidar_csv, out_prefix) in enumerate(zip(lidar_paths, output_prefixes), start=1):
        _check_cancel(should_cancel)
        _emit(emit_event, "file_started", file_index=idx, file_total=len(lidar_paths), path=lidar_csv)
        try:
            points, chosen_mode = fuse_one_csv_chunk(
                lidar_csv=lidar_csv,
                gps_df=gps_df,
                gps_time_col=gps_time_col,
                output_prefix=out_prefix,
                time_mode=time_mode,
                utm_epsg=utm_epsg,
                sensor_offset_x=sensor_offset_x,
                sensor_offset_y=sensor_offset_y,
                sensor_offset_z=sensor_offset_z,
                gps_fusion_orientation=gps_fusion_orientation,
                sensor_yaw_deg=sensor_yaw_deg,
            )
            total_points += points
            log(
                f"Finished {os.path.basename(lidar_csv)} -> {out_prefix}.csv/.las "
                f"({points} points, {chosen_mode})"
            )
            _emit(
                emit_event,
                "file_completed",
                file_index=idx,
                file_total=len(lidar_paths),
                path=lidar_csv,
                total_points=total_points,
            )
        except Exception as e:
            failures.append((lidar_csv, str(e)))
            log(f"FAILED {lidar_csv}: {e}")

    log(f"Done. LiDAR files processed: {len(lidar_paths) - len(failures)}/{len(lidar_paths)}")
    log(f"Total points processed: {total_points}")
    log(f"GPS samples used: {len(gps_df)}")

    if failures:
        raise SystemExit(1)

    return {
        "mode": "csv",
        "csv_files": len(lidar_paths),
        "total_points": total_points,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline LiDAR processing (GPS fusion, SLAM map, SLAM+GPS anchor).")
    parser.add_argument(
        "--processing-mode",
        choices=["gps_fusion", "slam_map", "slam_gps_anchor"],
        default="gps_fusion",
        help="Processing mode for raw/manifest workflows.",
    )
    parser.add_argument("--manifest-json", help="Path to capture_manifest_*.json from pi_capture_raw.py.")
    parser.add_argument(
        "--manifest-base-dir",
        help=(
            "Base directory used to remap manifest file paths on offline machines. "
            "If manifest has Pi absolute paths, files are resolved by basename here."
        ),
    )
    parser.add_argument("--lidar-csv", nargs="+", help="One or more LiDAR CSV files (advanced mode).")
    parser.add_argument(
        "--lidar-raw",
        nargs="+",
        help="Raw LiDAR input(s): one or more .pcap/.osf files, directories, or glob patterns.",
    )
    parser.add_argument(
        "--lidar-session-filter",
        help=(
            "Optional raw session id filter for --lidar-raw directory/glob inputs "
            "(example: 20260317_094022). "
            "If omitted and --gps-csv name contains a session id, that id is used automatically."
        ),
    )
    parser.add_argument("--gps-csv", help="Path to raw GPS CSV from pi_capture_raw.py.")
    parser.add_argument(
        "--converted-csv-dir",
        help="Directory for optional debug converted CSV outputs when --keep-converted is enabled.",
    )
    parser.add_argument(
        "--keep-converted",
        action="store_true",
        help="Keep per-scan converted CSV debug files (can generate many files).",
    )
    parser.add_argument("--output-las", help="Final output LAS file path (preferred for raw/manifest mode).")
    parser.add_argument(
        "--output-osf",
        help="Optional output OSF path for SLAM modes (playback/debug).",
    )
    parser.add_argument(
        "--save-osf",
        action="store_true",
        help="Also save OSF next to LAS in SLAM modes.",
    )
    parser.add_argument(
        "--output-prefix",
        help="Output prefix. Raw mode writes <prefix>.las. CSV mode writes per-file <prefix>*.csv/.las.",
    )
    parser.add_argument(
        "--output-dir",
        help="Output directory. Raw mode writes one merged LAS. CSV mode writes per-file outputs.",
    )
    parser.add_argument(
        "--time-mode",
        choices=["auto", "unix_ns", "relative_start"],
        default="auto",
        help="How to align LiDAR/GPS timestamps (gps_fusion and slam_gps_anchor modes).",
    )
    parser.add_argument(
        "--gps-time-column",
        choices=["auto", "gps_epoch_ns", "pi_time_ns"],
        default="auto",
        help=(
            "Which GPS timestamp column to use. "
            "'gps_epoch_ns' is best when available. "
            "'auto' prefers gps_epoch_ns, then falls back to pi_time_ns "
            "(gps_fusion and slam_gps_anchor modes)."
        ),
    )
    parser.add_argument(
        "--reflectivity-filter",
        choices=["on", "off"],
        default="off",
        help=(
            "Enable/disable reflectivity threshold filtering in raw replay point export paths "
            "(gps_fusion and both slam_gps_anchor backends)."
        ),
    )
    parser.add_argument(
        "--min-reflectivity",
        type=int,
        default=2,
        help="Minimum reflectivity value kept when --reflectivity-filter is on.",
    )
    parser.add_argument(
        "--gps-outlier-filter",
        choices=["on", "off"],
        default="on",
        help=(
            "Enable/disable GPS outlier filtering before interpolation in GPS-dependent modes. "
            "When enabled, points with implausible speed jump from the last kept point are dropped."
        ),
    )
    parser.add_argument(
        "--gps-outlier-max-speed-mps",
        type=float,
        default=30.0,
        help="Maximum allowed GPS speed (m/s) used by outlier filtering.",
    )
    parser.add_argument(
        "--utm-epsg",
        type=int,
        default=32614,
        help="CRS EPSG used for x/y output and LAS metadata.",
    )
    parser.add_argument("--sensor-offset-x", type=float, default=0.0, help="Sensor offset X in meters.")
    parser.add_argument("--sensor-offset-y", type=float, default=0.0, help="Sensor offset Y in meters.")
    parser.add_argument("--sensor-offset-z", type=float, default=0.0, help="Sensor offset Z in meters.")
    parser.add_argument(
        "--gps-fusion-orientation",
        choices=["fixed_yaw", "path_tangent"],
        default="path_tangent",
        help=(
            "How to orient local LiDAR XY in gps_fusion mode. "
            "'fixed_yaw' uses constant --sensor-yaw-deg. "
            "'path_tangent' uses GPS trajectory direction plus --sensor-yaw-deg."
        ),
    )
    parser.add_argument(
        "--sensor-yaw-deg",
        type=float,
        default=0.0,
        help="Additional yaw offset (degrees) applied in gps_fusion orientation handling.",
    )
    parser.add_argument(
        "--slam-voxel-size",
        type=float,
        default=0.0,
        help=(
            "SLAM voxel size in meters. "
            "Set 0 for auto-estimate from first scan."
        ),
    )
    parser.add_argument("--slam-min-range", type=float, default=1.0, help="SLAM min valid range in meters.")
    parser.add_argument("--slam-max-range", type=float, default=150.0, help="SLAM max valid range in meters.")
    parser.add_argument(
        "--slam-deskew-method",
        choices=["auto", "none", "constant_velocity", "imu_deskew"],
        default="auto",
        help="SLAM deskew method.",
    )
    parser.add_argument(
        "--anchor-min-motion-m",
        type=float,
        default=5.0,
        help=(
            "Minimum SLAM trajectory motion needed to estimate XY rotation for GPS anchoring. "
            "Below this threshold, translation-only anchoring is used."
        ),
    )
    parser.add_argument(
        "--anchor-z-mode",
        choices=["offset", "none", "tilt"],
        default="tilt",
        help=(
            "How to anchor Z for SLAM+GPS mode. 'offset' shifts the cloud by a "
            "constant height; 'tilt' also levels it, correcting the SLAM frame's "
            "vertical against GPS (rigid_fit only, and still a rigid transform)."
        ),
    )
    parser.add_argument(
        "--anchor-tilt-min-span-m",
        type=float,
        default=50.0,
        help=(
            "Minimum trajectory extent in both easting and northing before a tilt "
            "is fitted. A straight route cannot observe the cross-track component."
        ),
    )
    parser.add_argument(
        "--anchor-tilt-max-deg",
        type=float,
        default=5.0,
        help="Reject a fitted tilt larger than this rather than applying it.",
    )
    parser.add_argument(
        "--anchor-backend",
        choices=["pose_optimizer", "rigid_fit"],
        default="pose_optimizer",
        help="Backend used for SLAM+GPS anchoring.",
    )
    parser.add_argument(
        "--poseopt-key-frame-distance",
        type=float,
        default=1.0,
        help="Pose optimizer key frame distance (meters).",
    )
    parser.add_argument(
        "--poseopt-constraints-every-m",
        type=float,
        default=10.0,
        help="Add GPS absolute constraints approximately every N meters.",
    )
    parser.add_argument(
        "--poseopt-constraint-weights",
        default="0.01,0.01,0.001",
        help="GPS absolute constraint translation weights as WX,WY,WZ.",
    )
    parser.add_argument(
        "--poseopt-max-iterations",
        type=int,
        default=50,
        help="Maximum pose optimizer iterations.",
    )
    parser.add_argument(
        "--poseopt-map-voxel-size",
        type=float,
        default=0.5,
        help=(
            "Voxel size for downsampling the exported map before LAS export "
            "(0 disables). Applies to both slam_gps_anchor backends."
        ),
    )
    return parser


def _resolve_inputs_from_manifest(args: argparse.Namespace) -> Tuple[List[str], List[str], Optional[str]]:
    with open(args.manifest_json, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    raw_paths = parse_manifest_raw_paths(manifest)
    lidar_paths = parse_manifest_lidar_paths(manifest)

    if raw_paths:
        raw_paths = resolve_manifest_path_list(
            raw_paths,
            args.manifest_json,
            args.manifest_base_dir,
            "raw LiDAR",
        )
    elif lidar_paths:
        lidar_paths = resolve_manifest_path_list(
            lidar_paths,
            args.manifest_json,
            args.manifest_base_dir,
            "LiDAR CSV",
        )
    else:
        raise ValueError("No LiDAR paths found in manifest.")

    gps_csv = args.gps_csv or manifest.get("gps_csv_path")
    if gps_csv and not args.gps_csv:
        manifest_dir = os.path.dirname(os.path.abspath(args.manifest_json))
        gps_csv = resolve_manifest_path(gps_csv, manifest_dir, args.manifest_base_dir)
    if gps_csv and not os.path.isfile(gps_csv):
        raise ValueError(
            f"Could not resolve GPS CSV from manifest: {gps_csv}\n"
            "Use --gps-csv explicitly and/or set --manifest-base-dir."
        )

    return raw_paths, lidar_paths, gps_csv


def _resolve_inputs_direct(args: argparse.Namespace) -> Tuple[List[str], List[str], Optional[str]]:
    if args.lidar_csv and args.lidar_raw:
        raise ValueError("Provide either --lidar-csv or --lidar-raw, not both.")
    if not args.lidar_csv and not args.lidar_raw:
        raise ValueError("Provide --lidar-csv or --lidar-raw (or use --manifest-json).")

    raw_paths: List[str] = []
    lidar_paths: List[str] = []
    if args.lidar_raw:
        raw_paths = expand_raw_lidar_inputs(args.lidar_raw)
    else:
        lidar_paths = sorted(args.lidar_csv)

    return raw_paths, lidar_paths, args.gps_csv


def run_from_args(
    args: argparse.Namespace,
    *,
    emit_event: EventEmitter = None,
    should_cancel: CancelChecker = None,
) -> Dict[str, Any]:
    _check_cancel(should_cancel)
    manifest: Optional[Dict[str, Any]] = None

    if args.manifest_json and (args.lidar_csv or args.lidar_raw):
        raise ValueError("Use either --manifest-json or direct LiDAR inputs, not both.")

    if args.manifest_json:
        with open(args.manifest_json, "r", encoding="utf-8") as f:
            manifest = json.load(f)
        raw_paths, lidar_paths, gps_csv = _resolve_inputs_from_manifest(args)
    else:
        raw_paths, lidar_paths, gps_csv = _resolve_inputs_direct(args)

    preferred_gps_time_columns: Optional[List[str]] = None
    if args.gps_time_column == "auto":
        preferred = preferred_gps_time_column_from_manifest(manifest)
        if preferred:
            preferred_gps_time_columns = [preferred, "gps_epoch_ns", "pi_time_ns"]
            log(
                f"GPS time column preference (manifest): auto -> prefer {preferred} "
                f"for timing architecture {manifest.get('timing_architecture')}"
            )
            _emit(
                emit_event,
                "status",
                message=(
                    "GPS time column preference from manifest: "
                    f"{preferred} first for PTP/Pi-clock capture"
                ),
            )

    if raw_paths:
        requested_filter = (getattr(args, "lidar_session_filter", "") or "").strip()
        inferred_filter = None
        if not requested_filter and gps_csv:
            inferred_filter = _extract_session_id_from_gps_csv_path(gps_csv)
        session_filter = requested_filter or inferred_filter
        if session_filter:
            original_count = len(raw_paths)
            raw_paths = _filter_raw_paths_by_session(raw_paths, session_filter)
            source = "explicit" if requested_filter else "auto_from_gps_csv"
            message = (
                f"Raw session filter ({source}): {session_filter} "
                f"-> {len(raw_paths)}/{original_count} file(s)"
            )
            log(message)
            _emit(emit_event, "status", message=message)

    _check_cancel(should_cancel)

    mode = args.processing_mode
    if mode == "gps_fusion" and (args.save_osf or args.output_osf):
        raise ValueError("--save-osf/--output-osf are only supported in slam_map and slam_gps_anchor modes.")

    if mode == "slam_map":
        if args.reflectivity_filter == "on":
            _emit(
                emit_event,
                "status",
                message="Reflectivity filter is not applied in slam_map mode.",
            )
        if lidar_paths and not raw_paths:
            raise ValueError("SLAM mode requires raw LiDAR (.pcap/.osf), not pre-converted CSV input.")
        if not raw_paths:
            raise ValueError("SLAM mode requires --lidar-raw or a manifest with raw LiDAR paths.")

        output_las = _resolve_output_las_path(
            args,
            raw_paths=raw_paths,
            manifest_json=args.manifest_json,
        )
        _emit(emit_event, "status", message=f"Output LAS: {output_las}")
        output_osf = _resolve_output_osf_path(args, output_las=output_las)
        if output_osf:
            _emit(emit_event, "status", message=f"Output OSF: {output_osf}")

        summary = slam_raw_to_single_las(
            raw_paths=raw_paths,
            output_las=output_las,
            output_osf=output_osf,
            voxel_size=args.slam_voxel_size,
            min_range=args.slam_min_range,
            max_range=args.slam_max_range,
            deskew_method=args.slam_deskew_method,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        return summary

    if mode == "slam_gps_anchor":
        if lidar_paths and not raw_paths:
            raise ValueError("SLAM+GPS anchor mode requires raw LiDAR (.pcap/.osf), not pre-converted CSV input.")
        if not raw_paths:
            raise ValueError("SLAM+GPS anchor mode requires --lidar-raw or a manifest with raw LiDAR paths.")
        if not gps_csv:
            raise ValueError(
                "SLAM+GPS anchor mode requires GPS CSV. Provide --gps-csv or use manifest with gps_csv_path."
            )

        _emit(emit_event, "status", message=f"Loading GPS: {gps_csv}")
        gps_df, gps_time_col = load_gps_for_interpolation(
            gps_csv,
            args.gps_time_column,
            preferred_auto_columns=preferred_gps_time_columns,
            apply_outlier_filter=(args.gps_outlier_filter == "on"),
            gps_outlier_max_speed_mps=args.gps_outlier_max_speed_mps,
        )
        _check_cancel(should_cancel)

        output_las = _resolve_output_las_path(
            args,
            raw_paths=raw_paths,
            manifest_json=args.manifest_json,
        )
        _emit(emit_event, "status", message=f"Output LAS: {output_las}")
        output_osf = _resolve_output_osf_path(args, output_las=output_las)
        if output_osf:
            _emit(emit_event, "status", message=f"Output OSF: {output_osf}")

        if args.anchor_backend == "pose_optimizer":
            summary = slam_raw_to_gps_anchored_las_pose_optimizer(
                raw_paths=raw_paths,
                gps_df=gps_df,
                gps_time_col=gps_time_col,
                output_las=output_las,
                output_osf=output_osf,
                time_mode=args.time_mode,
                utm_epsg=args.utm_epsg,
                voxel_size=args.slam_voxel_size,
                min_range=args.slam_min_range,
                max_range=args.slam_max_range,
                deskew_method=args.slam_deskew_method,
                anchor_min_motion_m=args.anchor_min_motion_m,
                anchor_z_mode=args.anchor_z_mode,
                poseopt_key_frame_distance=args.poseopt_key_frame_distance,
                poseopt_constraints_every_m=args.poseopt_constraints_every_m,
                poseopt_constraint_weights=args.poseopt_constraint_weights,
                poseopt_max_iterations=args.poseopt_max_iterations,
                poseopt_map_voxel_size=args.poseopt_map_voxel_size,
                reflectivity_filter=(args.reflectivity_filter == "on"),
                min_reflectivity=int(args.min_reflectivity),
                emit_event=emit_event,
                should_cancel=should_cancel,
            )
        else:
            summary = slam_raw_to_gps_anchored_las(
                raw_paths=raw_paths,
                gps_df=gps_df,
                gps_time_col=gps_time_col,
                output_las=output_las,
                output_osf=output_osf,
                time_mode=args.time_mode,
                utm_epsg=args.utm_epsg,
                voxel_size=args.slam_voxel_size,
                min_range=args.slam_min_range,
                max_range=args.slam_max_range,
                deskew_method=args.slam_deskew_method,
                anchor_min_motion_m=args.anchor_min_motion_m,
                anchor_z_mode=args.anchor_z_mode,
                anchor_tilt_min_span_m=args.anchor_tilt_min_span_m,
                anchor_tilt_max_deg=args.anchor_tilt_max_deg,
                reflectivity_filter=(args.reflectivity_filter == "on"),
                min_reflectivity=args.min_reflectivity,
                map_voxel_size=args.poseopt_map_voxel_size,
                emit_event=emit_event,
                should_cancel=should_cancel,
            )
            summary["anchor_backend"] = "rigid_fit"

        summary["gps_samples"] = len(gps_df)
        return summary

    if mode != "gps_fusion":
        raise ValueError(f"Unsupported processing mode: {mode}")

    if not gps_csv:
        raise ValueError(
            "GPS fusion mode requires GPS CSV. Provide --gps-csv or use manifest with gps_csv_path."
        )

    _emit(emit_event, "status", message=f"Loading GPS: {gps_csv}")
    gps_df, gps_time_col = load_gps_for_interpolation(
        gps_csv,
        args.gps_time_column,
        preferred_auto_columns=preferred_gps_time_columns,
        apply_outlier_filter=(args.gps_outlier_filter == "on"),
        gps_outlier_max_speed_mps=args.gps_outlier_max_speed_mps,
    )

    _check_cancel(should_cancel)

    if raw_paths:
        output_las = _resolve_output_las_path(
            args,
            raw_paths=raw_paths,
            manifest_json=args.manifest_json,
        )
        _emit(emit_event, "status", message=f"Output LAS: {output_las}")

        summary = fuse_raw_to_single_las(
            raw_paths=raw_paths,
            gps_df=gps_df,
            gps_time_col=gps_time_col,
            output_las=output_las,
            time_mode=args.time_mode,
            utm_epsg=args.utm_epsg,
            sensor_offset_x=args.sensor_offset_x,
            sensor_offset_y=args.sensor_offset_y,
            sensor_offset_z=args.sensor_offset_z,
            gps_fusion_orientation=args.gps_fusion_orientation,
            sensor_yaw_deg=args.sensor_yaw_deg,
            reflectivity_filter=(args.reflectivity_filter == "on"),
            min_reflectivity=int(args.min_reflectivity),
            keep_converted=bool(args.keep_converted),
            converted_csv_dir=args.converted_csv_dir,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        summary["gps_samples"] = len(gps_df)
        return summary

    if not lidar_paths:
        raise ValueError("No LiDAR inputs found.")

    if args.reflectivity_filter == "on":
        _emit(
            emit_event,
            "status",
            message="Reflectivity filter is not applied for pre-converted CSV inputs.",
        )

    if args.output_prefix:
        output_prefixes = prefixed_output_paths(args.output_prefix, lidar_paths)
        if len(lidar_paths) > 1:
            log("Multiple LiDAR CSV files found; using suffixed output names from --output-prefix.")
    else:
        if args.manifest_json:
            default_dir = os.path.join(os.path.dirname(os.path.abspath(args.manifest_json)), "fused_output")
            output_dir = args.output_dir or default_dir
        else:
            output_dir = args.output_dir or "."
        output_prefixes = [chunk_prefix(output_dir, p) for p in lidar_paths]

    summary = fuse_csv_chunks(
        lidar_paths=lidar_paths,
        output_prefixes=output_prefixes,
        gps_df=gps_df,
        gps_time_col=gps_time_col,
        time_mode=args.time_mode,
        utm_epsg=args.utm_epsg,
        sensor_offset_x=args.sensor_offset_x,
        sensor_offset_y=args.sensor_offset_y,
        sensor_offset_z=args.sensor_offset_z,
        gps_fusion_orientation=args.gps_fusion_orientation,
        sensor_yaw_deg=args.sensor_yaw_deg,
        emit_event=emit_event,
        should_cancel=should_cancel,
    )
    summary["gps_samples"] = len(gps_df)
    return summary


def run_offline_job(
    config: Dict[str, Any],
    emit_event: EventEmitter = None,
    should_cancel: CancelChecker = None,
) -> Dict[str, Any]:
    """
    Run offline fusion job from config dictionary.

    Intended for GUI/backend integrations.
    """
    parser = build_arg_parser()
    args = parser.parse_args([])

    for key, value in config.items():
        if not hasattr(args, key):
            raise ValueError(f"Unknown offline job config key: {key}")
        setattr(args, key, value)

    _emit(emit_event, "job_started", config=config)
    try:
        summary = run_from_args(args, emit_event=emit_event, should_cancel=should_cancel)
    except JobCancelledError as e:
        _emit(emit_event, "job_cancelled", reason=str(e))
        raise
    except SystemExit as e:
        code = int(e.code) if isinstance(e.code, int) else 1
        _emit(emit_event, "job_failed", exit_code=code)
        raise
    except Exception as e:
        _emit(emit_event, "job_error", error=str(e))
        raise

    _emit(emit_event, "job_completed", summary=summary)
    return summary


def _console_event_emitter(min_interval_s: float = 30.0) -> EventEmitter:
    """
    Print progress events for CLI runs.

    `run_from_args` emits progress throughout, but the CLI passed no emitter, so a
    full session showed nothing between "Processing raw LiDAR" and the final QA line
    - over an hour of silence on both the SLAM and replay phases, with no way to
    tell a working run from a hung one. Scan progress is throttled by wall time
    because it fires every ten scans, which is twice a second.
    """
    state = {"last_progress": 0.0}

    def emit(event: Dict[str, Any]) -> None:
        kind = event.get("kind")
        if kind == "status":
            if event.get("console_echoed"):
                return
            log(str(event.get("message", "")))
        elif kind == "file_started":
            log(
                f"[{event.get('file_index')}/{event.get('file_total')}] "
                f"{os.path.basename(str(event.get('path', '')))}"
            )
        elif kind == "scan_progress":
            now = time.monotonic()
            if now - state["last_progress"] < min_interval_s:
                return
            state["last_progress"] = now
            parts = [
                f"  [{event.get('file_index')}/{event.get('file_total')}]",
                f"scan {event.get('scan_index')}",
                f"total {event.get('total_scans')}",
            ]
            points = event.get("total_points")
            if points is not None:
                parts.append(f"points {int(points):,}")
            log(" ".join(parts))

    return emit


def main(argv: Optional[List[str]] = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    run_from_args(args, emit_event=_console_event_emitter())


if __name__ == "__main__":
    main()
