#!/usr/bin/env python3
"""
Process a set of recorded drives from raw capture to finished, checked clouds, in one command.

Each drive folder holds one session as copied from the Pi: `raw_lidar_<S>_chunk0000.pcap`,
any `raw_lidar_<S>_chunk0000-<n>.pcap` that follow it, and `raw_gps_<S>.csv`, where `<S>` is
the folder name (`YYYYMMDD_hhmmss`). The steps, as in TUNING.md:

1. fusion: SLAM with loop closure, rigid fit to GPS, gravity levelling, export voxel 0.1 m;
2. placement on the airborne lidar tiles, one rigid shift per drive;
3. alignment between drives that share streets (a drive that shares none is left as placed);
4. overlap flags on streets covered more than once;
5. acceptance checks (limits in VALIDATION.md).

Output folders under `--output-dir`: `fused/<S>/`, `placed/`, `aligned/`, `final/` (the clouds
to use, overlap flagged), `no_overlap/` (the same without the flagged points, for the
classification pipeline) and `checks/` (CHECKS.md). A step whose output is already there is skipped, so a
stopped run continues where it stopped; delete a step's folder to redo it.

The steps that use all drives together (alignment, flags, checks) belong to one set of drives, recorded in
`drives.txt`. Run again with a different set in the same output folder, for instance with a drive added, and those
folders are removed and redone; fusion and placement of the drives already done are kept.

A drive that cannot be placed on the airborne lidar (outside the tiles given, or no building or road to fit to) is
left out with its reason in the log and in `skipped_drives.txt`; the other drives are finished, and the command then
exits with code 2.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import List, Optional

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))

FUSION_ARGS = [
    "--processing-mode", "slam_gps_anchor", "--anchor-backend", "rigid_fit", "--anchor-z-mode", "offset",
    "--gravity-level", "--slam-voxel-size", "0.25", "--slam-min-range", "2.5",
    "--poseopt-map-voxel-size", "0.1", "--utm-epsg", "32614",
]


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def run(args: List[str], log_path: str, threads: int) -> None:
    env = dict(os.environ, OMP_NUM_THREADS=str(threads))
    with open(log_path, "w") as fh:
        code = subprocess.call([sys.executable, "-u"] + args, stdout=fh, stderr=subprocess.STDOUT, env=env, cwd=REPO_ROOT)
    if code != 0:
        raise RuntimeError(f"{os.path.basename(args[0])} failed (exit {code}); see {log_path}")


def session_of(drive_dir: str) -> str:
    s = os.path.basename(os.path.normpath(drive_dir))
    if not re.fullmatch(r"\d{8}_\d{6}", s):
        raise ValueError(f"{drive_dir}: the folder name must be the session, YYYYMMDD_hhmmss")
    return s


def capture_epoch(sessions: List[str]) -> float:
    """Capture date as a decimal year, from the first session's date."""
    d = dt.datetime.strptime(sessions[0][:8], "%Y%m%d")
    days = 366 if d.year % 4 == 0 and (d.year % 100 != 0 or d.year % 400 == 0) else 365
    return round(d.year + (d.timetuple().tm_yday - 1) / days, 2)


def raw_inputs(drive_dir: str, s: str) -> tuple:
    first = os.path.join(drive_dir, f"raw_lidar_{s}_chunk0000.pcap")
    rest = glob.glob(os.path.join(drive_dir, f"raw_lidar_{s}_chunk0000-*.pcap"))
    rest = sorted((p for p in rest if re.search(r"chunk0000-\d+\.pcap$", p)), key=lambda p: int(re.search(r"-(\d+)\.pcap$", p).group(1)))
    gps = os.path.join(drive_dir, f"raw_gps_{s}.csv")
    for p in (first, gps):
        if not os.path.exists(p):
            raise FileNotFoundError(p)
    return [first] + rest, gps


def stem(s: str) -> str:
    return f"geo_raw_lidar_{s}_chunk0000_merged"


def fuse(drive_dir: str, out: str, threads: int) -> None:
    s = session_of(drive_dir)
    d = os.path.join(out, "fused", s)
    if os.path.exists(os.path.join(d, stem(s) + "_qa.json")):
        log(f"{s}: fused already")
        return
    os.makedirs(d, exist_ok=True)
    raws, gps = raw_inputs(drive_dir, s)
    log(f"{s}: fusion of {len(raws)} pcap file(s)")
    run(["offline_fuse_lidar_gps.py"] + FUSION_ARGS + ["--lidar-raw"] + raws + ["--gps-csv", gps, "--output-dir", d],
        os.path.join(d, "run.log"), threads)
    log(f"{s}: fused")


def place(s: str, out: str, tiles: List[str], epoch: float, threads: int) -> Optional[str]:
    """Places one drive; returns None, or why it could not be placed (the last line of its log)."""
    d = os.path.join(out, "placed")
    if os.path.exists(os.path.join(d, stem(s) + "_placement.json")):
        log(f"{s}: placed already")
        return None
    os.makedirs(d, exist_ok=True)
    src = os.path.join(out, "fused", s, stem(s))
    log_path = os.path.join(d, f"{s}.log")
    try:
        run(["place_on_airborne.py", "--las", src + ".las", "--trajectory", src + "_trajectory.csv", "--tiles"] + tiles
            + ["--epoch", str(epoch), "--output-dir", d], log_path, threads)
    except RuntimeError:
        with open(log_path) as fh:
            lines = [ln.strip() for ln in fh if ln.strip()]
        reason = lines[-1] if lines else "no output"
        log(f"{s}: NOT PLACED, left out: {reason}")
        return reason
    log(f"{s}: placed")
    return None


COMBINED_STEPS = ("aligned", "final", "no_overlap", "checks")


def drives_of_previous_run(out: str) -> Optional[List[str]]:
    """The drives the all-drive steps in `out` were made from: `drives.txt`, or for a folder made before it existed, the
    drives in `final/`. None when those steps have not run."""
    record = os.path.join(out, "drives.txt")
    if os.path.exists(record):
        with open(record) as fh:
            return sorted(ln.strip() for ln in fh if ln.strip())
    final = os.path.join(out, "final")
    if os.path.isdir(final):
        found = [re.fullmatch(r"geo_raw_lidar_(\d{8}_\d{6})_chunk0000_merged\.las", f) for f in os.listdir(final)]
        return sorted(m.group(1) for m in found if m)
    return None


def start_drive_set(out: str, sessions: List[str]) -> None:
    """Removes the all-drive steps when they were made from other drives, and records the drives they are now for."""
    previous = drives_of_previous_run(out)
    if previous is not None and previous != sorted(sessions):
        log(f"drives changed ({', '.join(previous) or 'none'} -> {', '.join(sorted(sessions))}): "
            f"redoing alignment, flags and checks")
        for name in COMBINED_STEPS:
            shutil.rmtree(os.path.join(out, name), ignore_errors=True)
        if os.path.exists(os.path.join(out, "flag_overlap.log")):
            os.remove(os.path.join(out, "flag_overlap.log"))
    with open(os.path.join(out, "drives.txt"), "w") as fh:
        fh.write("\n".join(sorted(sessions)) + "\n")


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drives", required=True, nargs="+", help="one folder per drive, named YYYYMMDD_hhmmss")
    ap.add_argument("--tiles", required=True, nargs="+", help="airborne lidar tiles (StratMap 2021), or folders holding them, covering every drive")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--epoch", type=float, help="capture date as a decimal year; by default from the session date")
    ap.add_argument("--jobs", type=int, default=4, help="drives fused or placed at the same time")
    ap.add_argument("--threads", type=int, default=16, help="threads for the steps that use all drives")
    a = ap.parse_args(argv)

    out = os.path.abspath(a.output_dir)
    os.makedirs(out, exist_ok=True)
    sessions = [session_of(d) for d in a.drives]
    epoch = a.epoch or capture_epoch(sessions)
    per_job = max(1, a.threads // a.jobs)
    t0 = time.time()
    log(f"{len(sessions)} drive(s), epoch {epoch}, output {out}")

    with ThreadPoolExecutor(a.jobs) as pool:
        list(pool.map(lambda d: fuse(d, out, per_job), a.drives))
    with ThreadPoolExecutor(a.jobs) as pool:
        reasons = list(pool.map(lambda s: place(s, out, a.tiles, epoch, per_job), sessions))
    skipped = {s: r for s, r in zip(sessions, reasons) if r}
    skipped_path = os.path.join(out, "skipped_drives.txt")
    if skipped:
        with open(skipped_path, "w") as fh:
            fh.write("".join(f"{s}: not placed on the airborne lidar: {r} (see placed/{s}.log)\n" for s, r in skipped.items()))
    elif os.path.exists(skipped_path):
        os.remove(skipped_path)
    sessions = [s for s in sessions if s not in skipped]
    if not sessions:
        raise SystemExit("No drive could be placed on the airborne lidar; see placed/*.log")
    start_drive_set(out, sessions)

    placed = [os.path.join(out, "placed", stem(s) + ".las") for s in sessions]
    aligned = os.path.join(out, "aligned")
    if len(sessions) == 1:
        log("one drive: no alignment between drives")
        aligned_las = placed
    else:
        if not os.path.exists(os.path.join(aligned, "drive_alignment.json")):
            os.makedirs(aligned, exist_ok=True)
            run(["align_drives.py", "--las"] + placed + ["--output-dir", aligned], os.path.join(aligned, "run.log"), a.threads)
        log("aligned")
        aligned_las = [os.path.join(aligned, stem(s) + ".las") for s in sessions]

    final = os.path.join(out, "final")
    if not os.path.exists(os.path.join(final, "overlap_flags.json")):
        os.makedirs(final, exist_ok=True)
        run(["flag_overlap.py", "--las"] + aligned_las + ["--output-dir", final, "--unflagged-dir", os.path.join(out, "no_overlap")],
            os.path.join(out, "flag_overlap.log"), a.threads)
        if len(sessions) > 1:
            shutil.copy2(os.path.join(aligned, "drive_alignment.json"), os.path.join(final, "drive_alignment.json"))
    log("overlap flags set")

    checks = os.path.join(out, "checks")
    if not os.path.exists(os.path.join(checks, "CHECKS.md")):
        os.makedirs(checks, exist_ok=True)
        run(["check_drives.py", "--las"] + [os.path.join(final, stem(s) + ".las") for s in sessions] + ["--output-dir", checks],
            os.path.join(checks, "run.log"), a.threads)
    log(f"checks written to {os.path.join(checks, 'CHECKS.md')}; finished in {(time.time() - t0) / 3600:.1f} h")
    if skipped:
        log(f"{len(skipped)} drive(s) left out, see {skipped_path}: {', '.join(skipped)}")
        sys.exit(2)


if __name__ == "__main__":
    main()
