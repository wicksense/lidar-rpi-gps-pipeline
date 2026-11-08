#!/usr/bin/env python3
"""
Mark points of streets covered more than once, so that each place is used from one copy.

A street can be covered twice by one drive (it comes back) or by two drives. Both copies
stay in the clouds, a few decimetres apart at best, and a tree seen twice can be counted
twice. This keeps, in every `cell_m` square, the one pass that saw it from closest and
sets the LAS overlap flag on every other pass's points there. Nothing is deleted:
classification skips flagged points, and they remain for anyone who wants the far side
of a crown.

- A pass is one drive's points in a square whose capture times have no gap longer than
  `gap_s`; a drive that comes back after more than `gap_s` makes a second pass.
- Closest is the smallest mean `range_m`, the distance from the sensor at capture.
- Run after `drive_alignment`: before it, the passes of different drives are metres apart.

Writes each input cloud, flags set, with its sidecar files to the output folder and
`overlap_flags.json` there; the inputs are not touched. With `--unflagged-dir`, also
writes copies holding only the unflagged points, for tools that read every point (the
classification pipeline); `--unflagged-only` makes just those copies from flagged clouds.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from typing import Dict, List, Optional, Sequence

import laspy
import numpy as np

CELL_M = 2.0
GAP_S = 60.0
SIDECARS = ("_trajectory.csv", "_qa.json", "_placement.json")


def log(message: str) -> None:
    print(message, flush=True)


def overlap_mask(xy: np.ndarray, times: np.ndarray, ranges: np.ndarray, drive: np.ndarray,
                 cell_m: float = CELL_M, gap_s: float = GAP_S) -> np.ndarray:
    """True for points to flag: in their square, they belong to a pass other than the one seen from closest."""
    cell = np.floor(xy / cell_m).astype(np.int64)
    order = np.lexsort((times, drive, cell[:, 1], cell[:, 0]))
    c0, c1, d, t = cell[order, 0], cell[order, 1], drive[order], times[order]
    new_cell = np.r_[True, (np.diff(c0) != 0) | (np.diff(c1) != 0)]
    new_pass = new_cell | np.r_[True, (np.diff(d) != 0) | (np.diff(t) > gap_s)]
    pass_id = np.cumsum(new_pass) - 1
    cell_id = np.cumsum(new_cell) - 1
    n_pass = pass_id[-1] + 1
    mean_range = np.bincount(pass_id, weights=ranges[order], minlength=n_pass) / np.bincount(pass_id, minlength=n_pass)
    pass_cell = cell_id[new_pass]
    # The pass with the smallest mean range in each square.
    by = np.lexsort((mean_range, pass_cell))
    first = np.r_[True, np.diff(pass_cell[by]) != 0]
    winner = np.empty(cell_id[-1] + 1, dtype=np.int64)
    winner[pass_cell[by][first]] = by[first]
    flagged = np.empty(len(order), dtype=bool)
    flagged[order] = pass_id != winner[cell_id]
    return flagged


def flag_overlaps(las_paths: Sequence[str], out_dir: str, *, cell_m: float = CELL_M, gap_s: float = GAP_S) -> Dict[str, object]:
    t0 = time.time()
    if os.path.abspath(out_dir) in {os.path.abspath(os.path.dirname(p)) for p in las_paths}:
        raise ValueError("The output folder must differ from the inputs' folders; the inputs are not overwritten.")
    os.makedirs(out_dir, exist_ok=True)
    xy, times, ranges, drive, sizes = [], [], [], [], []
    for k, p in enumerate(las_paths):
        las = laspy.read(p)
        xy.append(np.c_[np.asarray(las.x), np.asarray(las.y)])
        times.append(np.asarray(las.gps_time, dtype=np.float64))
        ranges.append(np.asarray(las.range_m, dtype=np.float64))
        drive.append(np.full(len(las.x), k, dtype=np.int64))
        sizes.append(len(las.x))
        log(f"{os.path.basename(p)}: {sizes[-1]:,} points")
        del las
    flagged = overlap_mask(np.vstack(xy), np.concatenate(times), np.concatenate(ranges), np.concatenate(drive), cell_m, gap_s)
    del xy, times, ranges, drive
    report: Dict[str, object] = dict(inputs=list(las_paths), cell_m=cell_m, gap_s=gap_s, drives={})
    start = 0
    for k, p in enumerate(las_paths):
        f = flagged[start : start + sizes[k]]
        start += sizes[k]
        las = laspy.read(p)
        las.overlap = f
        out_las = os.path.join(out_dir, os.path.basename(p))
        las.write(out_las)
        for suffix in SIDECARS:
            side = p[:-4] + suffix
            if os.path.exists(side):
                shutil.copy2(side, os.path.join(out_dir, os.path.basename(side)))
        name = os.path.splitext(os.path.basename(p))[0]
        report["drives"][name] = dict(output_las=out_las, points=int(sizes[k]), flagged=int(f.sum()),
                                      flagged_fraction=float(f.mean()))
        log(f"{name}: {f.mean():.1%} of points flagged as overlap")
    report["elapsed_s"] = round(time.time() - t0, 1)
    with open(os.path.join(out_dir, "overlap_flags.json"), "w") as fh:
        json.dump(report, fh, indent=1)
    return report


def write_unflagged(las_paths: Sequence[str], out_dir: str) -> Dict[str, int]:
    """Copy each flagged cloud with its overlap-flagged points left out, for tools that cannot skip them."""
    if os.path.abspath(out_dir) in {os.path.abspath(os.path.dirname(p)) for p in las_paths}:
        raise ValueError("The output folder must differ from the inputs' folders; the inputs are not overwritten.")
    os.makedirs(out_dir, exist_ok=True)
    kept = {}
    for p in las_paths:
        las = laspy.read(p)
        keep = ~np.asarray(las.overlap, dtype=bool)
        las.points = las.points[keep]
        las.write(os.path.join(out_dir, os.path.basename(p)))
        for suffix in SIDECARS:
            side = p[:-4] + suffix
            if os.path.exists(side):
                shutil.copy2(side, os.path.join(out_dir, os.path.basename(side)))
        name = os.path.splitext(os.path.basename(p))[0]
        kept[name] = int(keep.sum())
        log(f"{name}: {keep.sum():,} of {len(keep):,} points kept")
        del las
    return kept


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--las", required=True, nargs="+", help="aligned clouds, with gps_time and range_m")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--cell-m", type=float, default=CELL_M, help="size of the squares in which one pass is kept")
    ap.add_argument("--gap-s", type=float, default=GAP_S, help="time gap that starts a new pass of the same drive")
    ap.add_argument("--unflagged-dir", help="also write copies without the flagged points here")
    ap.add_argument("--unflagged-only", action="store_true", help="the inputs are flagged already: only write the copies")
    a = ap.parse_args(argv)
    flagged = a.las
    if not a.unflagged_only:
        report = flag_overlaps(a.las, a.output_dir, cell_m=a.cell_m, gap_s=a.gap_s)
        flagged = [d["output_las"] for d in report["drives"].values()]
    if a.unflagged_dir or a.unflagged_only:
        write_unflagged(flagged, a.unflagged_dir or a.output_dir)


if __name__ == "__main__":
    main()
