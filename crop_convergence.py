#!/usr/bin/env python3
"""Crop the unconverged head of an anchored point cloud.

SLAM needs a short stretch of motion before scan-to-map registration settles.
Until it does, the trajectory wanders relative to GPS and every point placed
with those poses lands in the wrong spot. The residual is large at the start of
a run and drops to a steady level once registration locks on; the points from
before that moment are the ones worth removing, because they are the only ones
whose error is a property of the run rather than of GPS.

This script finds that moment from the trajectory CSV the offline fusion job
writes next to its LAS, then drops every point recorded before it. Detection is
deliberately coarse and auditable: bin the run in time, take the median residual
per bin, and cut at the first bin from which every later bin sits at the steady
level. A bin median is unmoved by a handful of outlier columns, so a transient
pose spike inside an otherwise converged stretch will not be mistaken for a
convergence boundary.

The LAS `gps_time` field and the trajectory CSV `time_s` column are the same
clock - absolute capture time in unix seconds - so the cut transfers from one to
the other directly.

Typical use, with the expected answer asserted so a surprising detection stops
the run rather than silently producing a differently-cropped deliverable:

    python crop_convergence.py \\
        --las out/geo_..._merged.las \\
        --gps-csv RawData/raw_gps_20260721_170623.csv \\
        --expect-seconds 10 --expect-tolerance 5
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import laspy
import numpy as np
import pandas as pd


# Points are streamed through in blocks of this size so that peak memory stays
# flat regardless of how many points the cloud holds.
CHUNK_POINTS = 2_000_000

# Steady state is estimated from the tail of the run, far enough past the start
# that an unconverged head cannot inflate it. How far is a calibration knob: too
# small and the head raises the steady level, hiding the very thing being looked
# for; too large and a quiet stretch at the end of one route sets a level the
# rest of the route never meets.
DEFAULT_STEADY_STATE_TAIL_FRACTION = 0.5


@dataclass
class ConvergenceResult:
    """Everything the detector concluded, in the units it concluded it in."""

    bin_seconds: float
    bin_start_s: np.ndarray
    bin_median_m: np.ndarray
    bin_samples: np.ndarray
    steady_state_m: float
    threshold_m: float
    converged_bin: int
    crop_seconds: float
    run_start_unix_s: float
    crop_at_unix_s: float
    unsettled_after_crop_s: list

    def summary(self) -> dict:
        data = asdict(self)
        for key in ("bin_start_s", "bin_median_m", "bin_samples"):
            data[key] = np.asarray(data[key]).tolist()
        return data


def _trajectory_csv_for(las_path: Path) -> Path:
    """The offline fusion job writes `<las root>_trajectory.csv` beside the LAS."""
    root, _ext = os.path.splitext(str(las_path))
    return Path(root + "_trajectory.csv")


def load_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (time_s, xy) from a trajectory CSV, sorted by time."""
    frame = pd.read_csv(path, usecols=["time_s", "x", "y"])
    frame = frame.dropna().sort_values("time_s")
    times = frame["time_s"].to_numpy(dtype=np.float64)
    xy = frame[["x", "y"]].to_numpy(dtype=np.float64)
    if times.size < 2:
        raise ValueError(f"Trajectory {path} holds fewer than two usable rows.")
    return times, xy


def load_gps(path: Path, time_column: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (time_s, easting/northing) from a capture GPS CSV, sorted by time.

    The capture writes GPS timestamps in nanoseconds; anything that large is
    rescaled so the two clocks can be compared in seconds.
    """
    frame = pd.read_csv(path, usecols=[time_column, "easting", "northing"])
    frame = frame.dropna().sort_values(time_column)
    times = frame[time_column].to_numpy(dtype=np.float64)
    if np.nanmedian(np.abs(times)) > 1e12:
        times = times / 1e9
    xy = frame[["easting", "northing"]].to_numpy(dtype=np.float64)
    if times.size < 2:
        raise ValueError(f"GPS file {path} holds fewer than two usable rows.")
    return times, xy


def trajectory_residual(
    traj_t: np.ndarray,
    traj_xy: np.ndarray,
    gps_t: np.ndarray,
    gps_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Horizontal distance from each trajectory sample to GPS at the same instant.

    GPS is linearly interpolated onto the trajectory clock, which is what the
    QA report in the fusion job does, so the numbers here are comparable to it.
    Trajectory samples outside the GPS window are dropped rather than
    extrapolated.
    """
    inside = (traj_t >= gps_t[0]) & (traj_t <= gps_t[-1])
    if not np.any(inside):
        raise ValueError("Trajectory and GPS do not overlap in time.")
    times = traj_t[inside]
    gps_x = np.interp(times, gps_t, gps_xy[:, 0])
    gps_y = np.interp(times, gps_t, gps_xy[:, 1])
    residual = np.hypot(traj_xy[inside, 0] - gps_x, traj_xy[inside, 1] - gps_y)
    return times, residual


def detect_convergence(
    times: np.ndarray,
    residual: np.ndarray,
    bin_seconds: float,
    threshold_factor: float,
    threshold_floor_m: float,
    min_bin_samples: int,
    max_crop_seconds: float,
    stay_bins: int,
    steady_state_tail_fraction: float,
) -> ConvergenceResult:
    """Find the first time bin from which the residual holds at steady state.

    The steady level is the median of the bin medians over the tail of the run,
    and a bin counts as settled when its median sits below
    `max(steady * threshold_factor, steady + threshold_floor_m)`. Two forms of
    tolerance are combined because either alone misbehaves at an extreme: a pure
    ratio is unusably tight when the steady level is already near zero, and a
    pure absolute margin is unusably loose when it is high.

    Two constraints keep the answer meaning "convergence" rather than "the
    quietest stretch of the run". The search covers only the first
    `max_crop_seconds`, because SLAM convergence is by definition a property of
    the start of a run - a residual excursion in the middle is the route passing
    a building or turning a corner, and cropping up to it would throw away good
    data. And a candidate bin has to be followed by `stay_bins` settled bins, so
    a single quiet bin inside an unconverged head does not end the search. The
    persistence window is deliberately short: requiring the residual to stay
    settled for the whole remainder of the run would push the answer to the end
    of any route that has a GPS excursion in it.
    """
    t0 = float(times[0])
    elapsed = times - t0
    edges = np.arange(0.0, float(elapsed[-1]) + bin_seconds, bin_seconds)
    index = np.clip(np.searchsorted(edges, elapsed, side="right") - 1, 0, len(edges) - 1)

    starts, medians, counts = [], [], []
    for i in range(len(edges)):
        selected = index == i
        n = int(selected.sum())
        if n < min_bin_samples:
            continue
        starts.append(float(edges[i]))
        medians.append(float(np.median(residual[selected])))
        counts.append(n)

    if len(medians) < 3:
        raise ValueError(
            f"Only {len(medians)} usable time bins at --bin-seconds {bin_seconds}; "
            "the run is too short for convergence detection."
        )

    bin_start = np.asarray(starts, dtype=np.float64)
    bin_median = np.asarray(medians, dtype=np.float64)
    bin_samples = np.asarray(counts, dtype=np.int64)

    tail_from = int(len(bin_median) * steady_state_tail_fraction)
    steady = float(np.median(bin_median[tail_from:]))
    threshold = max(steady * threshold_factor, steady + threshold_floor_m)

    settled = bin_median <= threshold
    searchable = int(np.count_nonzero(bin_start <= max_crop_seconds))

    converged = -1
    for i in range(searchable):
        window = settled[i : i + 1 + stay_bins]
        if window.size and bool(np.all(window)):
            converged = i
            break
    if converged < 0:
        raise ValueError(
            f"No bin in the first {max_crop_seconds:.0f} s is followed by "
            f"{stay_bins} settled bins (steady state {steady:.3f} m, threshold "
            f"{threshold:.3f} m). The run does not converge the way this detector "
            "assumes; inspect the residual before cropping."
        )

    crop_seconds = float(bin_start[converged])
    unsettled_after = [
        float(start)
        for start, ok in zip(bin_start[converged:], settled[converged:])
        if not ok
    ]
    return ConvergenceResult(
        bin_seconds=bin_seconds,
        bin_start_s=bin_start,
        bin_median_m=bin_median,
        bin_samples=bin_samples,
        steady_state_m=steady,
        threshold_m=threshold,
        converged_bin=converged,
        crop_seconds=crop_seconds,
        run_start_unix_s=t0,
        crop_at_unix_s=t0 + crop_seconds,
        unsettled_after_crop_s=unsettled_after,
    )


def crop_las(source: Path, destination: Path, cut_unix_s: float) -> tuple[int, int]:
    """Copy `source` to `destination` without points recorded before `cut_unix_s`.

    Returns (points read, points kept). The header - and with it the CRS, the
    scales and offsets, and the extra dimensions - is carried over unchanged, so
    the cropped file stays interchangeable with the one it came from.
    """
    kept = 0
    total = 0
    temporary = destination.with_suffix(destination.suffix + ".partial")
    with laspy.open(str(source)) as reader:
        header = reader.header
        with laspy.open(str(temporary), mode="w", header=header) as writer:
            for points in reader.chunk_iterator(CHUNK_POINTS):
                total += len(points)
                keep = np.asarray(points.gps_time) >= cut_unix_s
                if not np.any(keep):
                    continue
                selected = points[keep]
                writer.write_points(selected)
                kept += len(selected)
    if kept == 0:
        temporary.unlink(missing_ok=True)
        raise ValueError(
            "Cropping would remove every point; refusing to write an empty cloud."
        )
    os.replace(temporary, destination)
    return total, kept


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Crop the unconverged head of an anchored point cloud.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--las", required=True, type=Path, help="Exported LAS to crop.")
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        help="Trajectory CSV. Defaults to <las root>_trajectory.csv.",
    )
    parser.add_argument(
        "--gps-csv", required=True, type=Path, help="Capture GPS CSV with easting/northing."
    )
    parser.add_argument(
        "--gps-time-column",
        default="pi_time_ns",
        help="GPS timestamp column; must be the clock the run was processed against.",
    )
    parser.add_argument("--output", type=Path, help="Defaults to <las root>_cropped.las.")
    parser.add_argument(
        "--bin-seconds",
        type=float,
        default=10.0,
        help="Width of the time bins the residual is summarised in.",
    )
    parser.add_argument(
        "--crop-seconds",
        type=float,
        help="Skip detection and cut this many seconds from the start of the run.",
    )
    parser.add_argument(
        "--threshold-factor",
        type=float,
        default=1.5,
        help="A bin counts as settled below steady state times this factor.",
    )
    parser.add_argument(
        "--threshold-floor-m",
        type=float,
        default=0.10,
        help="Absolute margin above steady state, used when the ratio is too tight.",
    )
    parser.add_argument(
        "--min-bin-samples",
        type=int,
        default=50,
        help="Bins with fewer trajectory samples than this are ignored.",
    )
    parser.add_argument(
        "--max-crop-seconds",
        type=float,
        default=60.0,
        help="Only the start of the run is searched; nothing later can be the answer.",
    )
    parser.add_argument(
        "--stay-bins",
        type=int,
        default=2,
        help="How many further settled bins must follow a candidate bin.",
    )
    parser.add_argument(
        "--steady-state-tail-fraction",
        type=float,
        default=DEFAULT_STEADY_STATE_TAIL_FRACTION,
        help="Steady state is the median of the bin medians past this point in the run.",
    )
    parser.add_argument(
        "--max-post-crop-excursion-bins",
        type=int,
        default=0,
        help=(
            "Refuse to write when more than this many bins after the cut still sit "
            "above the threshold. Raise it only for a route whose excursions have "
            "been identified as GPS-side and accepted."
        ),
    )
    parser.add_argument(
        "--expect-seconds",
        type=float,
        help="Assert the detected crop is near this value; refuse to write if not.",
    )
    parser.add_argument(
        "--expect-tolerance",
        type=float,
        default=5.0,
        help="How far the detection may sit from --expect-seconds.",
    )
    parser.add_argument(
        "--report-json", type=Path, help="Write the detection summary here as JSON."
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Detect and report without writing a LAS."
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    las_path: Path = args.las
    if not las_path.is_file():
        print(f"error: no such LAS: {las_path}", file=sys.stderr)
        return 2

    trajectory_path = args.trajectory_csv or _trajectory_csv_for(las_path)
    if not trajectory_path.is_file():
        print(f"error: no such trajectory CSV: {trajectory_path}", file=sys.stderr)
        return 2

    traj_t, traj_xy = load_trajectory(trajectory_path)
    gps_t, gps_xy = load_gps(args.gps_csv, args.gps_time_column)
    times, residual = trajectory_residual(traj_t, traj_xy, gps_t, gps_xy)

    try:
        result = detect_convergence(
            times,
            residual,
            bin_seconds=args.bin_seconds,
            threshold_factor=args.threshold_factor,
            threshold_floor_m=args.threshold_floor_m,
            min_bin_samples=args.min_bin_samples,
            max_crop_seconds=args.max_crop_seconds,
            stay_bins=args.stay_bins,
            steady_state_tail_fraction=args.steady_state_tail_fraction,
        )
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 3

    print(f"Trajectory samples: {times.size:,} over {times[-1] - times[0]:.1f} s")
    print(f"Steady-state median residual: {result.steady_state_m:.3f} m")
    print(f"Converged threshold: {result.threshold_m:.3f} m")
    print()
    print(f"{'bin start (s)':>14} {'median (m)':>12} {'samples':>10}  state")
    for start, median, count in zip(
        result.bin_start_s, result.bin_median_m, result.bin_samples
    ):
        state = "settled" if median <= result.threshold_m else "UNSETTLED"
        print(f"{start:>14.1f} {median:>12.3f} {count:>10,}  {state}")
    print()
    print(f"Detected convergence at {result.crop_seconds:.1f} s into the run.")
    excursions = result.unsettled_after_crop_s
    if excursions:
        marks = ", ".join(f"{t:.0f} s" for t in excursions)
        print(
            f"Note: {len(excursions)} bin(s) after the cut still sit above the threshold "
            f"({marks}). Excursions away from the start of a run are not convergence - "
            "expect GPS, not SLAM - and cropping does not address them."
        )

    # The two guards are checked in order of how fundamental the problem is. A
    # detection that lands somewhere unexpected means the detector and the data
    # disagree, and nothing downstream of that is worth reporting yet.
    if args.expect_seconds is not None:
        drift = abs(result.crop_seconds - args.expect_seconds)
        if drift > args.expect_tolerance:
            print(
                f"error: detection ({result.crop_seconds:.1f} s) is {drift:.1f} s from "
                f"the expected {args.expect_seconds:.1f} s, more than the "
                f"{args.expect_tolerance:.1f} s tolerance. Nothing was written; "
                "check the run before cropping.",
                file=sys.stderr,
            )
            return 3

    # A dry run exists to be looked at, so it reports excursions and stops short of
    # treating them as a failure.
    if not args.dry_run and len(excursions) > args.max_post_crop_excursion_bins:
        print(
            f"error: {len(excursions)} bin(s) after the cut exceed the threshold, more "
            f"than the {args.max_post_crop_excursion_bins} allowed. Nothing was "
            "written. Identify what those excursions are before accepting the run, "
            "then raise --max-post-crop-excursion-bins deliberately if they are "
            "GPS-side and acceptable.",
            file=sys.stderr,
        )
        return 4

    crop_seconds = result.crop_seconds
    source = "detected"
    if args.crop_seconds is not None:
        crop_seconds = float(args.crop_seconds)
        source = "manual override"
        print(f"Using {source}: {crop_seconds:.1f} s")

    cut_at = result.run_start_unix_s + crop_seconds

    if args.report_json is not None:
        summary = result.summary()
        summary["applied_crop_seconds"] = crop_seconds
        summary["applied_crop_source"] = source
        args.report_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"Detection summary: {args.report_json}")

    if args.dry_run:
        print("Dry run: no LAS written.")
        return 0

    if crop_seconds <= 0.0:
        print("Nothing to crop; the run is settled from its first bin.")
        return 0

    root, ext = os.path.splitext(str(las_path))
    output = args.output or Path(f"{root}_cropped{ext}")
    total, kept = crop_las(las_path, output, cut_at)
    removed = total - kept

    print()
    print(f"Cropped {crop_seconds:.1f} s from the start of the run ({source}).")
    print(f"Points read:    {total:,}")
    print(f"Points removed: {removed:,} ({100.0 * removed / total:.2f}%)")
    print(f"Points kept:    {kept:,}")
    print(f"Wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
