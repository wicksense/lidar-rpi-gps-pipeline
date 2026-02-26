#!/usr/bin/env python3
"""
offline_fuse_lidar_gps.py

Purpose
-------
Run this on an offline workstation/server after field collection.

Supported workflows:
1. Single chunk: one LiDAR CSV + one GPS CSV
2. Batch chunks: multiple LiDAR CSV files + one GPS CSV
3. Manifest mode: read chunk list and GPS CSV from capture_manifest_*.json
"""

import argparse
import glob
import json
import os
from io import StringIO
from typing import List, Tuple

import laspy
import numpy as np
import pandas as pd
from pyproj import CRS


def log(message: str) -> None:
    print(message, flush=True)


def read_ouster_csv(path: str) -> pd.DataFrame:
    """Read Ouster CSV that may include comment lines before the real header."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    header_index = next((i for i, line in enumerate(lines) if "TIMESTAMP" in line.upper()), None)
    if header_index is None:
        raise ValueError(f"Could not find header line containing TIMESTAMP in: {path}")

    # Strip leading "#" from header line if present.
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

    # auto mode
    overlap = (lidar_ts_ns.min() <= gps_ts_ns.max()) and (gps_ts_ns.min() <= lidar_ts_ns.max())

    # Rough sanity check for epoch nanoseconds between years 2000 and 2100.
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

    # auto mode
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


def fuse_one_chunk(
    lidar_csv: str,
    gps_df: pd.DataFrame,
    gps_time_col: str,
    output_prefix: str,
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

    # Keep only needed columns and cast types explicitly for reliability.
    lidar_df = lidar_df[needed_lidar_cols].copy()
    lidar_df["timestamp_ns"] = lidar_df["timestamp (ns)"].astype("int64")
    lidar_df["x_local"] = -lidar_df["x1 (m)"].astype("float64")
    lidar_df["y_local"] = -lidar_df["y1 (m)"].astype("float64")
    lidar_df["z_local"] = lidar_df["z1 (m)"].astype("float64")

    lidar_t_raw = lidar_df["timestamp_ns"].to_numpy(dtype=np.int64)
    gps_t_raw = gps_df[gps_time_col].to_numpy(dtype=np.int64)
    lidar_t, gps_t, chosen_mode = choose_time_mode(lidar_t_raw, gps_t_raw, time_mode)
    log(f"Time alignment mode for {os.path.basename(lidar_csv)}: {chosen_mode}")

    # np.interp does linear interpolation and clamps values outside range to edge values.
    e_interp = np.interp(lidar_t, gps_t, gps_df["easting"].to_numpy(dtype=np.float64))
    n_interp = np.interp(lidar_t, gps_t, gps_df["northing"].to_numpy(dtype=np.float64))
    z_interp = np.interp(lidar_t, gps_t, gps_df["altitude_m"].to_numpy(dtype=np.float64))

    lidar_df["easting_interp"] = e_interp
    lidar_df["northing_interp"] = n_interp
    lidar_df["altitude_interp"] = z_interp

    lidar_df["x_global"] = lidar_df["easting_interp"] + lidar_df["x_local"] + sensor_offset_x
    lidar_df["y_global"] = lidar_df["northing_interp"] + lidar_df["y_local"] + sensor_offset_y
    lidar_df["z_global"] = lidar_df["altitude_interp"] + lidar_df["z_local"] + sensor_offset_z

    out_csv = output_prefix + ".csv"
    out_las = output_prefix + ".las"
    out_dir = os.path.dirname(out_csv) or "."
    os.makedirs(out_dir, exist_ok=True)

    log(f"Writing georeferenced CSV: {out_csv}")
    lidar_df[["timestamp_ns", "x_global", "y_global", "z_global"]].to_csv(out_csv, index=False)

    log(f"Writing LAS 1.4: {out_las}")
    header = laspy.LasHeader(point_format=6, version="1.4")
    header.x_scale = header.y_scale = header.z_scale = 0.001
    header.x_offset = float(lidar_df["x_global"].min())
    header.y_offset = float(lidar_df["y_global"].min())
    header.z_offset = float(lidar_df["z_global"].min())
    header.add_crs(CRS.from_epsg(utm_epsg))

    las = laspy.LasData(header)
    las.x = lidar_df["x_global"].to_numpy(dtype=np.float64)
    las.y = lidar_df["y_global"].to_numpy(dtype=np.float64)
    las.z = lidar_df["z_global"].to_numpy(dtype=np.float64)
    las.write(out_las)

    return len(lidar_df), chosen_mode


def chunk_prefix(output_dir: str, lidar_csv: str) -> str:
    base = os.path.splitext(os.path.basename(lidar_csv))[0]
    return os.path.join(output_dir, f"geo_{base}")


def prefixed_output_paths(output_prefix: str, lidar_paths: List[str]) -> List[str]:
    """
    Build output prefixes from a user-provided base prefix.

    - If one LiDAR input: use the prefix directly.
    - If multiple LiDAR inputs: append the LiDAR basename to keep outputs unique.
    """
    if len(lidar_paths) == 1:
        return [output_prefix]
    return [
        f"{output_prefix}_{os.path.splitext(os.path.basename(p))[0]}"
        for p in lidar_paths
    ]


def parse_manifest_lidar_paths(manifest: dict) -> List[str]:
    if "chunks" in manifest and isinstance(manifest["chunks"], list) and manifest["chunks"]:
        chunks = []
        for item in manifest["chunks"]:
            if not isinstance(item, dict):
                continue
            chunk_index = item.get("chunk_index", 10**9)

            # New manifest style: produced_files contains all files created for a chunk.
            produced_files = item.get("produced_files", [])
            if isinstance(produced_files, list):
                for f in produced_files:
                    if isinstance(f, str) and f.lower().endswith(".csv"):
                        chunks.append((chunk_index, f))

            # Backward compatibility: older manifests had one lidar_csv_path.
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

    # Backward compatibility with older single-file manifest shape.
    if "lidar_csv_path" in manifest and manifest["lidar_csv_path"]:
        return [manifest["lidar_csv_path"]]

    return []


def parse_manifest_raw_paths(manifest: dict) -> List[str]:
    """
    Return raw LiDAR paths (.pcap/.osf) listed in manifest, ordered by chunk index.
    """
    chunks = manifest.get("chunks", [])
    if not isinstance(chunks, list) or not chunks:
        return []

    raw_items = []
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


def _find_metadata_json_candidates(raw_path: str) -> List[str]:
    """
    Return likely metadata json paths for a raw capture.
    """
    base = os.path.splitext(raw_path)[0]
    candidates = [
        f"{base}.json",
        f"{base}_0.json",
    ]
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


def _extract_first_sensor_info(source):
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


def _extract_first_scan(item):
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
            pass
    return None


def convert_raw_inputs_to_csv(raw_paths: List[str], converted_csv_dir: str) -> List[str]:
    """
    Convert raw LiDAR captures (.pcap/.osf) to CSV files using ouster-sdk Python APIs.
    Returns one or more generated CSV paths per raw input.
    """
    if not raw_paths:
        return []

    try:
        from ouster.sdk import open_source
        from ouster.sdk.core import ChanField, XYZLut, destagger
    except Exception as e:
        raise ValueError(
            "ouster-sdk is not installed or not importable. "
            "Install it with: pip install ouster-sdk"
        ) from e

    os.makedirs(converted_csv_dir, exist_ok=True)
    converted_csvs = []

    for raw_path in raw_paths:
        if not os.path.isfile(raw_path):
            raise ValueError(f"Raw LiDAR file does not exist: {raw_path}")

        base = os.path.splitext(os.path.basename(raw_path))[0]
        new_csvs = []

        open_kwargs = {"sensor_idx": 0, "collate": False}
        # For pcap files, provide sidecar metadata if present to avoid autodiscovery misses.
        if raw_path.lower().endswith(".pcap"):
            meta_candidates = _find_metadata_json_candidates(raw_path)
            if meta_candidates:
                open_kwargs["meta"] = [meta_candidates[0]]

        log(f"Converting raw LiDAR to CSV with ouster-sdk: {raw_path}")
        source = None
        try:
            try:
                source = open_source(raw_path, **open_kwargs)
            except TypeError:
                # Older/newer SDK signatures may not accept these kwargs.
                source = open_source(raw_path)
            info = _extract_first_sensor_info(source)
            xyzlut = XYZLut(info)

            for idx, item in enumerate(source):
                scan = _extract_first_scan(item)
                if scan is None:
                    continue
                if ChanField.RANGE not in scan.fields:
                    continue

                timestamps = np.tile(np.asarray(scan.timestamp, dtype=np.int64), (scan.h, 1))
                xyz = xyzlut(scan.field(ChanField.RANGE))

                # Match the destaggered convention used by Ouster conversion examples.
                try:
                    timestamps = destagger(info, timestamps)
                    xyz = destagger(info, xyz)
                except Exception:
                    pass

                frame_df = pd.DataFrame(
                    {
                        "TIMESTAMP (ns)": timestamps.reshape(-1).astype(np.int64),
                        "X1 (m)": xyz[:, :, 0].reshape(-1).astype(np.float64),
                        "Y1 (m)": xyz[:, :, 1].reshape(-1).astype(np.float64),
                        "Z1 (m)": xyz[:, :, 2].reshape(-1).astype(np.float64),
                    }
                )

                csv_path = os.path.join(converted_csv_dir, f"{base}_{idx}.csv")
                frame_df.to_csv(csv_path, index=False)
                new_csvs.append(csv_path)
        except Exception as e:
            raise ValueError(f"ouster-sdk conversion failed for {raw_path}: {e}") from e
        finally:
            close_fn = getattr(source, "close", None)
            if callable(close_fn):
                close_fn()

        if not new_csvs:
            raise ValueError(f"Could not produce converted CSV outputs for raw file: {raw_path}")

        converted_csvs.extend(new_csvs)

    # Preserve order while removing duplicates.
    seen = set()
    ordered = []
    for p in converted_csvs:
        if p not in seen:
            seen.add(p)
            ordered.append(p)
    return ordered


def cleanup_converted_csvs(csv_paths: List[str]) -> None:
    """
    Remove converted CSV files from this run.
    Cleanup warnings are logged but do not fail the whole run.
    """
    if not csv_paths:
        return

    removed = 0
    failed = []
    for path in sorted(set(csv_paths)):
        if not os.path.isfile(path):
            continue
        try:
            os.remove(path)
            removed += 1
        except OSError as e:
            failed.append((path, str(e)))

    log(f"Cleanup converted CSVs: removed {removed} files.")
    for path, err in failed:
        log(f"Cleanup warning: could not remove {path}: {err}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline LiDAR + GPS timestamp-based georeferencing.")
    parser.add_argument("--manifest-json", help="Path to capture_manifest_*.json from pi_capture_raw.py.")
    parser.add_argument("--lidar-csv", nargs="+", help="One or more LiDAR CSV files.")
    parser.add_argument("--lidar-raw", nargs="+", help="One or more raw LiDAR files (.pcap/.osf).")
    parser.add_argument("--gps-csv", help="Path to raw GPS CSV from pi_capture_raw.py.")
    parser.add_argument(
        "--converted-csv-dir",
        help=(
            "Directory for converted CSV files when --lidar-raw is used. "
            "By default, converted CSVs are deleted after a successful run."
        ),
    )
    parser.add_argument(
        "--keep-converted",
        action="store_true",
        help="Keep converted CSV files after successful run (default is to delete).",
    )
    parser.add_argument(
        "--output-prefix",
        help="Single output prefix. Use only with one LiDAR CSV.",
    )
    parser.add_argument(
        "--output-dir",
        help="Output directory for multi-chunk runs. Writes geo_<chunkname>.csv/.las",
    )
    parser.add_argument(
        "--time-mode",
        choices=["auto", "unix_ns", "relative_start"],
        default="auto",
        help="How to align LiDAR/GPS timestamps.",
    )
    parser.add_argument(
        "--gps-time-column",
        choices=["auto", "gps_epoch_ns", "pi_time_ns"],
        default="auto",
        help=(
            "Which GPS timestamp column to use. "
            "'gps_epoch_ns' is best when available. "
            "'auto' prefers gps_epoch_ns, then falls back to pi_time_ns."
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
    args = parser.parse_args()
    converted_csvs_to_cleanup: List[str] = []
    used_raw_conversion = False

    if args.manifest_json and (args.lidar_csv or args.lidar_raw):
        raise ValueError("Use either --manifest-json or direct LiDAR inputs, not both.")

    if args.manifest_json:
        with open(args.manifest_json, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        lidar_paths = parse_manifest_lidar_paths(manifest)
        if not lidar_paths:
            raw_paths = parse_manifest_raw_paths(manifest)
            if raw_paths:
                converted_dir = args.converted_csv_dir or os.path.join(
                    os.path.dirname(os.path.abspath(args.manifest_json)),
                    "converted_csv",
                )
                lidar_paths = convert_raw_inputs_to_csv(raw_paths, converted_dir)
                converted_csvs_to_cleanup = list(lidar_paths)
                used_raw_conversion = True

        if not lidar_paths:
            raise ValueError("No LiDAR paths found in manifest.")

        gps_csv = args.gps_csv or manifest.get("gps_csv_path")
        if not gps_csv:
            raise ValueError("GPS CSV not found. Provide --gps-csv or include gps_csv_path in manifest.")

        if args.output_prefix:
            output_prefixes = prefixed_output_paths(args.output_prefix, lidar_paths)
            if len(lidar_paths) > 1:
                log(
                    "Multiple LiDAR CSV files found; using suffixed output names from --output-prefix."
                )
        else:
            default_dir = os.path.join(os.path.dirname(os.path.abspath(args.manifest_json)), "fused_output")
            output_dir = args.output_dir or default_dir
            output_prefixes = [chunk_prefix(output_dir, p) for p in lidar_paths]
    else:
        if args.lidar_csv and args.lidar_raw:
            raise ValueError("Provide either --lidar-csv or --lidar-raw, not both.")
        if not args.lidar_csv and not args.lidar_raw:
            raise ValueError("Provide --lidar-csv or --lidar-raw (or use --manifest-json).")
        if not args.gps_csv:
            raise ValueError("Provide --gps-csv (or use --manifest-json with gps_csv_path).")

        if args.lidar_raw:
            converted_dir = args.converted_csv_dir or os.path.join(os.getcwd(), "converted_csv")
            lidar_paths = convert_raw_inputs_to_csv(sorted(args.lidar_raw), converted_dir)
            converted_csvs_to_cleanup = list(lidar_paths)
            used_raw_conversion = True
        else:
            lidar_paths = sorted(args.lidar_csv)
        gps_csv = args.gps_csv

        if args.output_prefix:
            output_prefixes = prefixed_output_paths(args.output_prefix, lidar_paths)
            if len(lidar_paths) > 1:
                log(
                    "Multiple LiDAR CSV files found; using suffixed output names from --output-prefix."
                )
        else:
            output_dir = args.output_dir or "."
            output_prefixes = [chunk_prefix(output_dir, p) for p in lidar_paths]

    gps_df, gps_time_col = load_gps_for_interpolation(gps_csv, args.gps_time_column)

    total_points = 0
    failures = []
    for lidar_csv, out_prefix in zip(lidar_paths, output_prefixes):
        try:
            points, chosen_mode = fuse_one_chunk(
                lidar_csv=lidar_csv,
                gps_df=gps_df,
                gps_time_col=gps_time_col,
                output_prefix=out_prefix,
                time_mode=args.time_mode,
                utm_epsg=args.utm_epsg,
                sensor_offset_x=args.sensor_offset_x,
                sensor_offset_y=args.sensor_offset_y,
                sensor_offset_z=args.sensor_offset_z,
            )
            total_points += points
            log(f"Finished {os.path.basename(lidar_csv)} -> {out_prefix}.csv/.las ({points} points, {chosen_mode})")
        except Exception as e:
            failures.append((lidar_csv, str(e)))
            log(f"FAILED {lidar_csv}: {e}")

    log(f"Done. LiDAR files processed: {len(lidar_paths) - len(failures)}/{len(lidar_paths)}")
    log(f"Total points processed: {total_points}")
    log(f"GPS samples used: {len(gps_df)}")

    if failures:
        raise SystemExit(1)

    if used_raw_conversion:
        if args.keep_converted:
            log("Keeping converted CSV files (--keep-converted).")
        else:
            cleanup_converted_csvs(converted_csvs_to_cleanup)


if __name__ == "__main__":
    main()
