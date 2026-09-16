#!/usr/bin/env python3
"""Coverage and height-distribution acceptance checks for an exported cloud.

Neither number is in the QA report, because both need the exported points rather
than the trajectory. They are the two acceptance checks that catch what a passing
QA cannot see:

- **Coverage.** A cloud can cover only part of a route while QA still reports a
  pass, because QA compares trajectories and a trajectory is complete even when
  the map built along it is not.
- **Height distribution.** Poses twisted by over-weighted constraints deposit
  points far from any real surface, and so does a transient pose spike. Both show
  up as points implausibly far above or below local ground and nowhere else.

Ground is estimated per XY cell as a low percentile of that cell's heights, which
follows terrain without needing a ground classifier and without assuming the site
is flat.

The GPS track is clipped to the cloud's own `gps_time` span before coverage is
measured. Without that, a cloud built from part of a session reports the rest of
the session as missing, which is true and useless.

Run this on the cropped deliverable, not on the raw export: the uncropped file is
not what is being delivered, and its unconverged head is exactly the sort of thing
the height check is meant to flag.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import laspy
import numpy as np
import pandas as pd


CHUNK_POINTS = 2_000_000

# Cell keys pack an (ix, iy) pair into one integer. The multiplier only has to
# exceed the number of cells any real route spans in y.
CELL_KEY_STRIDE = 1_000_003


def load_gps(path: Path, time_column: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (time_s, easting/northing) from a capture GPS CSV, sorted by time."""
    frame = pd.read_csv(path, usecols=[time_column, "easting", "northing"]).dropna()
    frame = frame.sort_values(time_column)
    times = frame[time_column].to_numpy(dtype=np.float64)
    if np.nanmedian(np.abs(times)) > 1e12:
        times = times / 1e9
    if times.size < 2:
        raise ValueError(f"GPS file {path} holds fewer than two usable rows.")
    return times, frame[["easting", "northing"]].to_numpy(dtype=np.float64)


def cell_keys(x: np.ndarray, y: np.ndarray, cell_m: float) -> np.ndarray:
    return (
        np.floor(x / cell_m).astype(np.int64) * CELL_KEY_STRIDE
        + np.floor(y / cell_m).astype(np.int64)
    )


def resample_track(xy: np.ndarray, step_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Positions every `step_m` along the GPS polyline, with distance along it."""
    segment = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cumulative = np.concatenate([[0.0], np.cumsum(segment)])
    grid = np.arange(0.0, float(cumulative[-1]), step_m)
    track = np.column_stack(
        [
            np.interp(grid, cumulative, xy[:, 0]),
            np.interp(grid, cumulative, xy[:, 1]),
        ]
    )
    return track, grid


def scan_cloud(
    las_path: Path, cell_m: float, ground_percentile: float, chunk_points: int
) -> tuple[int, tuple[float, float], set[int], np.ndarray, np.ndarray]:
    """One pass over the cloud: time span, occupied cells, per-cell ground height."""
    occupied: set[int] = set()
    heights: dict[int, list[np.ndarray]] = {}
    total = 0
    span_lo, span_hi = np.inf, -np.inf

    with laspy.open(str(las_path)) as reader:
        for points in reader.chunk_iterator(chunk_points):
            x = np.asarray(points.x, dtype=np.float64)
            y = np.asarray(points.y, dtype=np.float64)
            z = np.asarray(points.z, dtype=np.float64)
            t = np.asarray(points.gps_time, dtype=np.float64)
            total += x.size
            span_lo = min(span_lo, float(t.min()))
            span_hi = max(span_hi, float(t.max()))

            key = cell_keys(x, y, cell_m)
            order = np.argsort(key, kind="stable")
            key_sorted, z_sorted = key[order], z[order]
            unique, starts = np.unique(key_sorted, return_index=True)
            ends = np.append(starts[1:], key_sorted.size)
            for k, start, end in zip(unique, starts, ends):
                heights.setdefault(int(k), []).append(z_sorted[start:end])
            occupied.update(unique.tolist())

    if total == 0:
        raise ValueError(f"{las_path} holds no points.")

    ground_keys = np.array(sorted(heights), dtype=np.int64)
    ground_z = np.array(
        [
            float(np.percentile(np.concatenate(heights[int(k)]), ground_percentile))
            for k in ground_keys
        ],
        dtype=np.float64,
    )
    return total, (span_lo, span_hi), occupied, ground_keys, ground_z


def count_out_of_band(
    las_path: Path,
    cell_m: float,
    ground_keys: np.ndarray,
    ground_z: np.ndarray,
    below_m: float,
    above_m: float,
    chunk_points: int,
) -> tuple[int, int]:
    """Second pass: points too far below or above their cell's ground height."""
    below = above = 0
    with laspy.open(str(las_path)) as reader:
        for points in reader.chunk_iterator(chunk_points):
            x = np.asarray(points.x, dtype=np.float64)
            y = np.asarray(points.y, dtype=np.float64)
            z = np.asarray(points.z, dtype=np.float64)
            key = cell_keys(x, y, cell_m)
            height_above_ground = z - ground_z[np.searchsorted(ground_keys, key)]
            below += int(np.count_nonzero(height_above_ground < -below_m))
            above += int(np.count_nonzero(height_above_ground > above_m))
    return below, above


def measure_coverage(
    track: np.ndarray, occupied: set[int], cell_m: float, radius_m: float
) -> np.ndarray:
    """True for each track sample that has cloud within `radius_m`."""
    reach = int(np.ceil(radius_m / cell_m))
    covered = np.zeros(track.shape[0], dtype=bool)
    for i, (tx, ty) in enumerate(track):
        cx = int(np.floor(tx / cell_m))
        cy = int(np.floor(ty / cell_m))
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                if (cx + dx) * CELL_KEY_STRIDE + (cy + dy) in occupied:
                    covered[i] = True
                    break
            if covered[i]:
                break
    return covered


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Coverage and height-distribution checks for an exported cloud.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--las", required=True, type=Path, help="Cloud to check.")
    parser.add_argument(
        "--gps-csv", required=True, type=Path, help="Capture GPS CSV with easting/northing."
    )
    parser.add_argument("--gps-time-column", default="pi_time_ns")
    parser.add_argument(
        "--track-step-m", type=float, default=10.0, help="Spacing of coverage samples."
    )
    parser.add_argument(
        "--coverage-radius-m",
        type=float,
        default=10.0,
        help="A track sample is covered if cloud falls within this distance.",
    )
    parser.add_argument("--ground-cell-m", type=float, default=5.0)
    parser.add_argument(
        "--ground-percentile",
        type=float,
        default=2.0,
        help="Percentile of a cell's heights taken as its ground level.",
    )
    parser.add_argument("--below-m", type=float, default=10.0)
    parser.add_argument("--above-m", type=float, default=30.0)
    parser.add_argument(
        "--max-out-of-band-fraction",
        type=float,
        default=0.01,
        help="Fail if more than this fraction of points sit out of band.",
    )
    parser.add_argument(
        "--min-coverage-fraction",
        type=float,
        default=1.0,
        help="Fail if less than this fraction of the track is covered.",
    )
    parser.add_argument("--chunk-points", type=int, default=CHUNK_POINTS)
    parser.add_argument("--report-json", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if not args.las.is_file():
        print(f"error: no such LAS: {args.las}", file=sys.stderr)
        return 2

    gps_t, gps_xy = load_gps(args.gps_csv, args.gps_time_column)
    total, (span_lo, span_hi), occupied, ground_keys, ground_z = scan_cloud(
        args.las, args.ground_cell_m, args.ground_percentile, args.chunk_points
    )

    inside = (gps_t >= span_lo) & (gps_t <= span_hi)
    if int(inside.sum()) < 2:
        print(
            "error: the cloud's gps_time span does not overlap the GPS track.",
            file=sys.stderr,
        )
        return 2
    track, along = resample_track(gps_xy[inside], args.track_step_m)
    covered = measure_coverage(
        track, occupied, args.ground_cell_m, args.coverage_radius_m
    )

    below, above = count_out_of_band(
        args.las,
        args.ground_cell_m,
        ground_keys,
        ground_z,
        args.below_m,
        args.above_m,
        args.chunk_points,
    )
    out_of_band = below + above

    coverage_fraction = float(covered.mean())
    out_of_band_fraction = out_of_band / total

    print(f"Points: {total:,}")
    print(
        f"Cloud spans {span_hi - span_lo:.1f} s; GPS track clipped to "
        f"{int(inside.sum())} fixes over that window."
    )
    print()
    print(
        f"Coverage: {int(covered.sum())}/{covered.size} track samples "
        f"({100.0 * coverage_fraction:.2f}%) at {args.track_step_m:.0f} m spacing, "
        f"radius {args.coverage_radius_m:.0f} m"
    )
    if not covered.all():
        gaps = along[~covered]
        print(f"  uncovered between {gaps.min():.0f} m and {gaps.max():.0f} m along the route")
    print(
        f"Height distribution: {out_of_band:,} points out of band "
        f"({100.0 * out_of_band_fraction:.4f}%)"
    )
    print(f"  more than {args.below_m:.0f} m below local ground: {below:,}")
    print(f"  more than {args.above_m:.0f} m above local ground: {above:,}")

    failures = []
    if coverage_fraction < args.min_coverage_fraction:
        failures.append(
            f"coverage {100.0 * coverage_fraction:.2f}% below the required "
            f"{100.0 * args.min_coverage_fraction:.2f}%"
        )
    if out_of_band_fraction > args.max_out_of_band_fraction:
        failures.append(
            f"out-of-band fraction {100.0 * out_of_band_fraction:.4f}% above the "
            f"allowed {100.0 * args.max_out_of_band_fraction:.4f}%"
        )

    if args.report_json:
        args.report_json.write_text(
            json.dumps(
                {
                    "las": str(args.las),
                    "points": total,
                    "coverage_fraction": coverage_fraction,
                    "track_samples": int(covered.size),
                    "out_of_band_points": out_of_band,
                    "out_of_band_fraction": out_of_band_fraction,
                    "below_ground_points": below,
                    "above_ground_points": above,
                    "passed": not failures,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"\nWrote {args.report_json}")

    if failures:
        print("\nFAILED: " + "; ".join(failures), file=sys.stderr)
        return 1
    print("\nBoth checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
