#!/usr/bin/env python3
"""Offline LiDAR + GPS fusion jobs (CLI + in-process API)."""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import tempfile
from io import StringIO
from typing import Any, Callable, Dict, List, Optional, Tuple

import laspy
import numpy as np
import pandas as pd
from pyproj import CRS


class JobCancelledError(RuntimeError):
    """Raised when a running job is cancelled by caller."""


EventEmitter = Optional[Callable[[Dict[str, Any]], None]]
CancelChecker = Optional[Callable[[], bool]]


def log(message: str) -> None:
    print(message, flush=True)


def _emit(emit_event: EventEmitter, kind: str, **payload: Any) -> None:
    if emit_event is None:
        return
    emit_event({"kind": kind, **payload})


def _check_cancel(should_cancel: CancelChecker) -> None:
    if should_cancel is not None and should_cancel():
        raise JobCancelledError("Job cancelled by user.")


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
    - relative_start: normalize both series to start at 0
    - auto: choose unix_ns if overlap looks valid, else relative_start
    """
    if requested_mode == "unix_ns":
        return lidar_ts_ns, gps_ts_ns, "unix_ns"

    if requested_mode == "relative_start":
        return lidar_ts_ns - lidar_ts_ns[0], gps_ts_ns - gps_ts_ns[0], "relative_start"

    overlap = (lidar_ts_ns.min() <= gps_ts_ns.max()) and (gps_ts_ns.min() <= lidar_ts_ns.max())

    ns_2000 = 946684800 * 10**9
    ns_2100 = 4102444800 * 10**9
    lidar_looks_like_epoch = ns_2000 <= np.median(lidar_ts_ns) <= ns_2100
    gps_looks_like_epoch = ns_2000 <= np.median(gps_ts_ns) <= ns_2100

    if overlap and lidar_looks_like_epoch and gps_looks_like_epoch:
        return lidar_ts_ns, gps_ts_ns, "unix_ns"

    return lidar_ts_ns - lidar_ts_ns[0], gps_ts_ns - gps_ts_ns[0], "relative_start"


def pick_gps_time_column(df: pd.DataFrame, requested: str) -> str:
    if requested == "gps_epoch_ns":
        if "gps_epoch_ns" not in df.columns:
            raise ValueError("Requested --gps-time-column gps_epoch_ns, but column is missing.")
        return "gps_epoch_ns"
    if requested == "pi_time_ns":
        if "pi_time_ns" not in df.columns:
            raise ValueError("Requested --gps-time-column pi_time_ns, but column is missing.")
        return "pi_time_ns"

    if "gps_epoch_ns" in df.columns and df["gps_epoch_ns"].notna().sum() >= 2:
        return "gps_epoch_ns"
    if "pi_time_ns" in df.columns and df["pi_time_ns"].notna().sum() >= 2:
        return "pi_time_ns"
    raise ValueError("No usable GPS time column found (need gps_epoch_ns or pi_time_ns with >=2 rows).")


def load_gps_for_interpolation(gps_csv: str, requested_time_col: str) -> Tuple[pd.DataFrame, str]:
    log(f"Loading GPS CSV: {gps_csv}")
    gps_df = pd.read_csv(gps_csv)

    needed_gps_cols = ["easting", "northing", "altitude_m"]
    missing_gps = [c for c in needed_gps_cols if c not in gps_df.columns]
    if missing_gps:
        raise ValueError(f"Missing required GPS columns: {missing_gps}")

    gps_time_col = pick_gps_time_column(gps_df, requested_time_col)
    log(f"GPS time column: {gps_time_col}")

    gps_df = gps_df[[gps_time_col, "easting", "northing", "altitude_m"]].copy()
    gps_df = gps_df.dropna(subset=[gps_time_col, "easting", "northing"])
    gps_df[gps_time_col] = gps_df[gps_time_col].astype("int64")
    gps_df["easting"] = gps_df["easting"].astype("float64")
    gps_df["northing"] = gps_df["northing"].astype("float64")
    gps_df["altitude_m"] = gps_df["altitude_m"].astype("float64")
    gps_df = gps_df.sort_values(gps_time_col)
    gps_df = gps_df.drop_duplicates(subset=[gps_time_col])

    if len(gps_df) < 2:
        raise ValueError("Need at least 2 GPS rows to interpolate.")

    return gps_df, gps_time_col


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


def expand_raw_lidar_inputs(raw_inputs: List[str]) -> List[str]:
    """
    Expand --lidar-raw inputs into concrete file paths.

    Supports explicit files, directories, and glob patterns.
    """
    expanded = []

    for item in raw_inputs:
        if os.path.isdir(item):
            for name in sorted(os.listdir(item)):
                path = os.path.join(item, name)
                if os.path.isfile(path) and _is_raw_lidar_path(path):
                    expanded.append(path)
            continue

        if any(ch in item for ch in "*?[]"):
            for path in sorted(glob.glob(item)):
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

    ordered = []
    seen = set()
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)
        if os.path.isfile(p):
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


def _extract_first_scan(item: Any) -> Any:
    if item is None:
        return None

    if (
        hasattr(item, "fields")
        and hasattr(item, "field")
        and hasattr(item, "h")
        and hasattr(item, "w")
        and hasattr(item, "timestamp")
    ):
        return item

    if isinstance(item, (list, tuple)):
        for candidate in item:
            if candidate is None:
                continue
            if (
                hasattr(candidate, "fields")
                and hasattr(candidate, "field")
                and hasattr(candidate, "h")
                and hasattr(candidate, "w")
                and hasattr(candidate, "timestamp")
            ):
                return candidate

    if hasattr(item, "__iter__"):
        try:
            for candidate in item:
                if candidate is None:
                    continue
                if (
                    hasattr(candidate, "fields")
                    and hasattr(candidate, "field")
                    and hasattr(candidate, "h")
                    and hasattr(candidate, "w")
                    and hasattr(candidate, "timestamp")
                ):
                    return candidate
        except Exception:
            return None
    return None


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


def _open_ouster_source(raw_path: str) -> Tuple[Any, Any, Any, Any, Any]:
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
    xyzlut = XYZLut(info)
    return source, info, infos, xyzlut, ChanField, destagger


def _extract_scan_arrays(
    scan: Any,
    xyzlut: Any,
    info: Any,
    ChanField: Any,
    destagger: Any,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    range_img = _scan_get_range_field(scan, ChanField)
    timestamps = np.tile(np.asarray(scan.timestamp, dtype=np.int64), (scan.h, 1))
    xyz = xyzlut(range_img)

    try:
        range_img = destagger(info, range_img)
        timestamps = destagger(info, timestamps)
        xyz = destagger(info, xyz)
    except Exception:
        pass

    timestamp_ns = timestamps.reshape(-1).astype(np.int64)
    x_local = -xyz[:, :, 0].reshape(-1).astype(np.float64)
    y_local = -xyz[:, :, 1].reshape(-1).astype(np.float64)
    z_local = xyz[:, :, 2].reshape(-1).astype(np.float64)

    valid = np.isfinite(x_local) & np.isfinite(y_local) & np.isfinite(z_local)
    valid = valid & (range_img.reshape(-1) > 0)

    return timestamp_ns[valid], x_local[valid], y_local[valid], z_local[valid]


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
) -> laspy.LasHeader:
    header = laspy.LasHeader(point_format=6, version="1.4")
    header.x_scale = header.y_scale = header.z_scale = 0.001
    header.x_offset = float(first_x)
    header.y_offset = float(first_y)
    header.z_offset = float(first_z)
    if utm_epsg is not None:
        header.add_crs(CRS.from_epsg(utm_epsg))
    return header


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
) -> Tuple[np.ndarray, Dict[str, int]]:
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
    if valid_count and max_delta_ns > max_allowed_delta_ns:
        raise ValueError(
            "Optimized pose lookup drift too large: "
            f"max delta {max_delta_ns} ns exceeds allowed {max_allowed_delta_ns} ns."
        )

    stats = {
        "columns": int(ts.size),
        "valid_columns": valid_count,
        "exact_columns": int(exact.sum()),
        "nearest_columns": int(valid_count - int(exact.sum())),
        "max_delta_ns": max_delta_ns,
    }
    return column_poses[idx], stats


def _build_point_cloud_from_raw_with_poses(
    raw_paths: List[str],
    *,
    pose_timestamps_ns: np.ndarray,
    column_poses: np.ndarray,
    min_range: float,
    max_range: float,
    map_voxel_size: float,
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Tuple[np.ndarray, Dict[str, int]]:
    """
    Build a global point cloud by replaying raw scans and dewarping with optimized poses.
    """
    try:
        from ouster.sdk.core import ChanField, XYZLut, dewarp, voxel_downsample
    except Exception as e:
        raise ValueError("ouster-sdk core modules are required for raw replay map extraction.") from e

    downsample_size = float(map_voxel_size)
    points_accum = np.zeros((0, 3), dtype=np.float64)
    pending_batches: List[np.ndarray] = []
    scans_seen = 0

    lookup_stats = {
        "columns": 0,
        "valid_columns": 0,
        "exact_columns": 0,
        "nearest_columns": 0,
        "max_delta_ns": 0,
    }

    def flush_batches() -> None:
        nonlocal points_accum, pending_batches
        if not pending_batches:
            return
        merged = np.concatenate(pending_batches, axis=0)
        pending_batches = []
        if points_accum.size == 0:
            points_accum = merged
        else:
            points_accum = np.concatenate([points_accum, merged], axis=0)
        if downsample_size > 0 and points_accum.shape[0] > 0:
            points_accum, _ = voxel_downsample(
                downsample_size,
                points_accum,
                np.zeros((points_accum.shape[0], 0), dtype=np.float64),
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

                pose_cols, stats = _lookup_pose_columns(
                    np.asarray(scan.timestamp, dtype=np.int64),
                    pose_timestamps_ns,
                    column_poses,
                )
                lookup_stats["columns"] += stats["columns"]
                lookup_stats["valid_columns"] += stats["valid_columns"]
                lookup_stats["exact_columns"] += stats["exact_columns"]
                lookup_stats["nearest_columns"] += stats["nearest_columns"]
                lookup_stats["max_delta_ns"] = max(lookup_stats["max_delta_ns"], stats["max_delta_ns"])

                points_local = xyzlut(ranges)
                points_global = dewarp(points_local, pose_cols)

                valid = ranges > 0
                if min_range > 0:
                    valid = valid & (ranges_m >= min_range)
                if max_range > 0:
                    valid = valid & (ranges_m <= max_range)

                selected = points_global[valid]
                if selected.size == 0:
                    continue
                pending_batches.append(np.asarray(selected, dtype=np.float64))

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
    return points_accum, lookup_stats


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
    total_points = 0
    total_scans = 0
    debug_csv_count = 0

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

                    timestamp_ns, x_local, y_local, z_local = _extract_scan_arrays(
                        scan, xyzlut, info, ChanField, destagger
                    )
                    if timestamp_ns.size == 0:
                        continue

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
                            gps_t_mode = gps_t_raw - gps_t_raw[0]
                        else:
                            gps_t_mode = gps_t_raw

                    if chosen_mode == "relative_start":
                        assert lidar_start_ns is not None
                        lidar_t = timestamp_ns - lidar_start_ns
                    else:
                        lidar_t = timestamp_ns

                    assert gps_t_mode is not None
                    e_interp = np.interp(lidar_t, gps_t_mode, gps_e)
                    n_interp = np.interp(lidar_t, gps_t_mode, gps_n)
                    z_interp = np.interp(lidar_t, gps_t_mode, gps_z)

                    x_global = e_interp + x_local + sensor_offset_x
                    y_global = n_interp + y_local + sensor_offset_y
                    z_global = z_interp + z_local + sensor_offset_z

                    if writer is None:
                        header = _new_las_header(utm_epsg, x_global.min(), y_global.min(), z_global.min())
                        writer = laspy.open(tmp_output, mode="w", header=header)
                        _emit(emit_event, "status", message=f"Writing LAS: {output_las}")

                    assert writer is not None
                    point_record = laspy.ScaleAwarePointRecord.zeros(len(x_global), header=writer.header)
                    point_record.x = x_global
                    point_record.y = y_global
                    point_record.z = z_global
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
    tmp_output_osf = output_osf + ".partial" if output_osf else None
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    osf_writer = None

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
                    if osf_writer is not None:
                        osf_writer.save(updated)
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

        if osf_writer is not None:
            osf_writer.close()
            osf_writer = None
            assert tmp_output_osf is not None and output_osf is not None
            os.replace(tmp_output_osf, output_osf)
            _emit(emit_event, "status", message=f"Saved SLAM OSF: {output_osf}")
            log(f"Saved SLAM OSF: {output_osf}")

        _emit(emit_event, "status", message="Building final SLAM point cloud...")
        points = slam_engine.get_point_cloud()
        if points is None or len(points) == 0:
            raise ValueError("SLAM point cloud is empty.")

        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("Unexpected SLAM point cloud shape; expected Nx3 points.")

        header = _new_las_header(None, points[:, 0].min(), points[:, 1].min(), points[:, 2].min())
        las = laspy.LasData(header)
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        _emit(emit_event, "status", message=f"Done. Final SLAM LAS: {output_las}")
        log(f"Done. Final SLAM LAS: {output_las}")
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
    emit_event: EventEmitter,
    should_cancel: CancelChecker,
) -> Dict[str, Any]:
    """
    Run SLAM and anchor resulting map to GPS coordinates.
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
    if anchor_z_mode not in {"offset", "none"}:
        raise ValueError("anchor_z_mode must be one of: offset, none")

    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    gps_e = gps_df["easting"].to_numpy(dtype=np.float64)
    gps_n = gps_df["northing"].to_numpy(dtype=np.float64)
    gps_z = gps_df["altitude_m"].to_numpy(dtype=np.float64)

    tmp_output = output_las + ".partial"
    if os.path.exists(tmp_output):
        os.remove(tmp_output)
    tmp_output_osf = output_osf + ".partial" if output_osf else None
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    traj_timestamps: List[int] = []
    traj_xyz_local: List[np.ndarray] = []
    osf_writer = None

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
                    if osf_writer is not None:
                        osf_writer.save(updated)
                    scan = _extract_first_scan(updated)
                    if scan is not None:
                        try:
                            ts, xyz = _extract_scan_pose_sample(scan)
                            traj_timestamps.append(ts)
                            traj_xyz_local.append(xyz)
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

        if osf_writer is not None:
            osf_writer.close()
            osf_writer = None
            assert tmp_output_osf is not None and output_osf is not None
            os.replace(tmp_output_osf, output_osf)
            _emit(emit_event, "status", message=f"Saved local SLAM OSF: {output_osf}")
            _emit(
                emit_event,
                "status",
                message="OSF stores local SLAM poses for rigid_fit backend (LAS is GPS-anchored output).",
            )
            log(f"Saved local SLAM OSF: {output_osf}")

        traj_t = np.asarray(traj_timestamps, dtype=np.int64)
        traj_xyz = np.asarray(traj_xyz_local, dtype=np.float64)

        slam_t, gps_t, chosen_time_mode = choose_time_mode(traj_t, gps_t_raw, time_mode)
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
        aligned_traj_xy = (rotation @ slam_xy.T).T + translation

        if anchor_z_mode == "offset":
            z_offset = float(np.median(gps_z_interp - traj_xyz[:, 2]))
        else:
            z_offset = 0.0
        aligned_traj_z = traj_xyz[:, 2] + z_offset

        qa_metrics = _compute_xy_alignment_metrics(gps_xy, aligned_traj_xy)
        z_residual = aligned_traj_z - gps_z_interp
        qa_metrics["z_mean_m"] = float(np.mean(z_residual))
        qa_metrics["z_rmse_m"] = float(np.sqrt(np.mean(z_residual**2)))
        qa_metrics["z_p95_abs_m"] = float(np.percentile(np.abs(z_residual), 95))

        qa_path, qa_pass = _write_anchor_qa_report(
            output_las,
            backend="rigid_fit",
            time_mode_used=chosen_time_mode,
            metrics=qa_metrics,
            extra={
                "anchor_method": anchor_method,
                "anchor_rotation_deg": _rotation_deg(rotation),
                "anchor_translation_x": float(translation[0]),
                "anchor_translation_y": float(translation[1]),
                "anchor_z_offset": z_offset,
            },
        )
        _emit(
            emit_event,
            "status",
            message=(
                f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
                f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} "
                f"({qa_path})"
            ),
        )
        log(
            f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
            f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} ({qa_path})"
        )

        _emit(
            emit_event,
            "status",
            message=(
                f"Anchoring transform ready "
                f"(method={anchor_method}, yaw_deg={_rotation_deg(rotation):.3f}, "
                f"time_mode={chosen_time_mode})"
            ),
        )

        points_local = slam_engine.get_point_cloud()
        if points_local is None or len(points_local) == 0:
            raise ValueError("SLAM point cloud is empty.")

        points_local = np.asarray(points_local, dtype=np.float64)
        if points_local.ndim != 2 or points_local.shape[1] < 3:
            raise ValueError("Unexpected SLAM point cloud shape; expected Nx3 points.")

        xy_local = points_local[:, :2]
        xy_global = (rotation @ xy_local.T).T + translation
        z_global = points_local[:, 2] + z_offset

        header = _new_las_header(utm_epsg, xy_global[:, 0].min(), xy_global[:, 1].min(), z_global.min())
        las = laspy.LasData(header)
        las.x = xy_global[:, 0]
        las.y = xy_global[:, 1]
        las.z = z_global
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        _emit(emit_event, "status", message=f"Done. Final anchored LAS: {output_las}")
        log(f"Done. Final anchored LAS: {output_las}")
        return {
            "mode": "slam_gps_anchor",
            "output_las": output_las,
            "raw_files": len(raw_paths),
            "total_scans": total_scans,
            "total_points": int(points_local.shape[0]),
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
    tmp_output_osf = output_osf + ".partial" if output_osf else None
    if tmp_output_osf and os.path.exists(tmp_output_osf):
        os.remove(tmp_output_osf)

    slam_engine = None
    total_scans = 0
    constraints_added = 0
    chosen_time_mode = "auto"
    final_cost = None
    writer = None

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
                    updated = slam_engine.update(scans)
                    assert writer is not None
                    writer.save(updated)
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

        if total_scans == 0:
            raise ValueError("No scans processed for SLAM map generation.")

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
        gps_x_interp = np.interp(slam_t, gps_t, gps_e)
        gps_y_interp = np.interp(slam_t, gps_t, gps_n)
        gps_z_interp = np.interp(slam_t, gps_t, gps_z)

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
            _emit(emit_event, "status", message=f"Saved optimized OSF: {output_osf}")
            log(f"Saved optimized OSF: {output_osf}")

        ts_opt = np.asarray(po.get_timestamps(SamplingMode.COLUMNS), dtype=np.int64)
        poses_opt = np.asarray(po.get_poses(SamplingMode.COLUMNS), dtype=np.float64)
        if ts_opt.size < 2:
            raise ValueError("Pose optimizer output trajectory has insufficient samples.")
        if poses_opt.ndim != 3 or poses_opt.shape[1:] != (4, 4):
            raise ValueError("Unexpected optimized pose shape from PoseOptimizer.")
        if poses_opt.shape[0] != ts_opt.shape[0]:
            raise ValueError("Optimized pose/timestamp count mismatch.")

        slam_t_opt, gps_t_opt, chosen_time_mode = choose_time_mode(ts_opt, gps_t_raw, time_mode)
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

        qa_path, qa_pass = _write_anchor_qa_report(
            output_las,
            backend="pose_optimizer",
            time_mode_used=chosen_time_mode,
            metrics=qa_metrics,
            extra={
                "constraints_added": int(constraints_added),
                "poseopt_cost": final_cost,
                "anchor_z_mode": anchor_z_mode,
            },
        )
        _emit(
            emit_event,
            "status",
            message=(
                f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
                f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} "
                f"({qa_path})"
            ),
        )
        log(
            f"Anchor QA: median={qa_metrics['xy_median_m']:.2f} m, "
            f"p95={qa_metrics['xy_p95_m']:.2f} m, pass={qa_pass} ({qa_path})"
        )

        _emit(emit_event, "status", message="Extracting point cloud from raw replay with optimized poses...")
        map_points, pose_lookup = _build_point_cloud_from_raw_with_poses(
            raw_paths,
            pose_timestamps_ns=ts_opt,
            column_poses=poses_opt,
            min_range=min_range,
            max_range=max_range,
            map_voxel_size=poseopt_map_voxel_size,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        if map_points.size == 0:
            raise ValueError("Optimized point cloud is empty.")

        header = _new_las_header(
            utm_epsg,
            map_points[:, 0].min(),
            map_points[:, 1].min(),
            map_points[:, 2].min(),
        )
        las = laspy.LasData(header)
        las.x = map_points[:, 0]
        las.y = map_points[:, 1]
        las.z = map_points[:, 2]
        las.write(tmp_output)
        os.replace(tmp_output, output_las)

        _emit(emit_event, "status", message=f"Done. Final anchored LAS: {output_las}")
        log(f"Done. Final anchored LAS: {output_las}")
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
            "pose_lookup_exact_columns": int(pose_lookup["exact_columns"]),
            "pose_lookup_nearest_columns": int(pose_lookup["nearest_columns"]),
            "pose_lookup_max_delta_ns": int(pose_lookup["max_delta_ns"]),
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

    e_interp = np.interp(lidar_t, gps_t, gps_df["easting"].to_numpy(dtype=np.float64))
    n_interp = np.interp(lidar_t, gps_t, gps_df["northing"].to_numpy(dtype=np.float64))
    z_interp = np.interp(lidar_t, gps_t, gps_df["altitude_m"].to_numpy(dtype=np.float64))

    lidar_df["x_global"] = e_interp + lidar_df["x_local"] + sensor_offset_x
    lidar_df["y_global"] = n_interp + lidar_df["y_local"] + sensor_offset_y
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
        "--utm-epsg",
        type=int,
        default=32614,
        help="CRS EPSG used for x/y output and LAS metadata.",
    )
    parser.add_argument("--sensor-offset-x", type=float, default=0.0, help="Sensor offset X in meters.")
    parser.add_argument("--sensor-offset-y", type=float, default=0.0, help="Sensor offset Y in meters.")
    parser.add_argument("--sensor-offset-z", type=float, default=0.0, help="Sensor offset Z in meters.")
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
        choices=["offset", "none"],
        default="offset",
        help="How to anchor Z for SLAM+GPS mode.",
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
        help="Voxel size for downsampling optimized map before LAS export (0 disables).",
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

    if args.manifest_json and (args.lidar_csv or args.lidar_raw):
        raise ValueError("Use either --manifest-json or direct LiDAR inputs, not both.")

    if args.manifest_json:
        raw_paths, lidar_paths, gps_csv = _resolve_inputs_from_manifest(args)
    else:
        raw_paths, lidar_paths, gps_csv = _resolve_inputs_direct(args)

    _check_cancel(should_cancel)

    mode = args.processing_mode
    if mode == "gps_fusion" and (args.save_osf or args.output_osf):
        raise ValueError("--save-osf/--output-osf are only supported in slam_map and slam_gps_anchor modes.")

    if mode == "slam_map":
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
        gps_df, gps_time_col = load_gps_for_interpolation(gps_csv, args.gps_time_column)
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
    gps_df, gps_time_col = load_gps_for_interpolation(gps_csv, args.gps_time_column)

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
            keep_converted=bool(args.keep_converted),
            converted_csv_dir=args.converted_csv_dir,
            emit_event=emit_event,
            should_cancel=should_cancel,
        )
        summary["gps_samples"] = len(gps_df)
        return summary

    if not lidar_paths:
        raise ValueError("No LiDAR inputs found.")

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


def main(argv: Optional[List[str]] = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    run_from_args(args)


if __name__ == "__main__":
    main()
