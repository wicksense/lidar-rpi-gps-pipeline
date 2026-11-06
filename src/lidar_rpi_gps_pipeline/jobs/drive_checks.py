#!/usr/bin/env python3
"""
Acceptance checks for a set of finished drives, written as one report with a verdict per check.

Each check uses something the processing did not fit to:

- Ground thickness (per drive): spread of points about the local plane on flat ground,
  from `measure_surface_thickness.py`. A blurred cloud is thick.
- Height on airborne lidar (per drive): road minus airborne bare earth on route tiles the
  placement was not fitted on, from `<stem>_placement.json` beside the source cloud.
- Repeat passes (per drive): where the drive passes a place twice, at least 60 s apart, the
  translation of the second pass onto the first, by point-to-plane ICP on surfaces in each
  50 m square. Measures whether the drive agrees with itself.
- Between drives (per pair): fixed things only, so that vehicles on the carriageway do not
  count. Points 0.5-3 m above ground and more than 5 m from both routes (walls, kerbs,
  poles, trunks, low canopy) are drawn as 0.2 m top-view images, and the shift in every
  shared 25 m square is the peak of their cross-correlation within ±3 m, kept only when
  the peak is clear. Independent of the 3-D registration `drive_alignment` uses.

Writes `checks.json` and `CHECKS.md` to the output folder.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from typing import Dict, List, Optional, Sequence

import laspy
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# Acceptance limits, as in VALIDATION.md: set for trees, not for survey.
LIMITS = dict(
    ground_thickness_cm=3.0,          # median, flat ground
    held_out_height_m=0.3,            # median |road - airborne bare earth| on tiles the placement did not see
    repeat_height_m=0.1,              # median, a drive's two passes of a place
    repeat_horizontal_m=0.5,
    between_drives_m=0.5,             # median, fixed things, per pair of drives
)
MIN_CLEAR_SQUARES = 3                 # a pair with fewer clear squares is reported but not judged

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def log(message: str) -> None:
    print(message, flush=True)


# --- between drives, fixed things --------------------------------------------------------------------------------

FIXED_SQUARE_M, FIXED_CELL_M, FIXED_SEARCH_M, ROAD_M, MIN_CELLS = 25.0, 0.2, 3.0, 5.0, 150


def _occupancy(p: np.ndarray, x0: float, y0: float, n: int) -> np.ndarray:
    ij = np.floor((p[:, :2] - [x0, y0]) / FIXED_CELL_M).astype(int)
    ok = (ij >= 0).all(axis=1) & (ij < n).all(axis=1)
    im = np.zeros((n, n))
    im[ij[ok, 0], ij[ok, 1]] = 1.0
    return im


def image_shift(pa: np.ndarray, pb: np.ndarray, key: Sequence[int]) -> Optional[np.ndarray]:
    """Shift (E, N) of b onto a in one square from the cross-correlation of their top views, or None when unclear."""
    margin = FIXED_SEARCH_M + 1.0
    n = int(round((FIXED_SQUARE_M + 2 * margin) / FIXED_CELL_M))
    x0, y0 = key[0] * FIXED_SQUARE_M - margin, key[1] * FIXED_SQUARE_M - margin
    ia, ib = _occupancy(pa, x0, y0, n), _occupancy(pb, x0, y0, n)
    if ia.sum() < MIN_CELLS or ib.sum() < MIN_CELLS:
        return None
    c = np.fft.fftshift(np.real(np.fft.ifft2(np.fft.fft2(ia) * np.conj(np.fft.fft2(ib)))))
    mid, r = n // 2, int(FIXED_SEARCH_M / FIXED_CELL_M)
    win = c[mid - r : mid + r + 1, mid - r : mid + r + 1]
    i, j = np.unravel_index(np.argmax(win), win.shape)
    peak = win[i, j]
    yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
    far = np.hypot(yy - (i - r), xx - (j - r)) * FIXED_CELL_M > 0.6
    if peak <= 0 or peak < 1.3 * win[far].max():
        return None

    def sub(m: float, lo: float, hi: float) -> float:
        d = lo - 2 * m + hi
        return 0.0 if d == 0 else 0.5 * (lo - hi) / d

    di = sub(peak, win[i - 1, j], win[i + 1, j]) if 0 < i < 2 * r else 0.0
    dj = sub(peak, win[i, j - 1], win[i, j + 1]) if 0 < j < 2 * r else 0.0
    return np.array([(i - r + di) * FIXED_CELL_M, (j - r + dj) * FIXED_CELL_M])


def _fixed_things(p: np.ndarray, roads: cKDTree) -> np.ndarray:
    z0 = np.percentile(p[:, 2], 2)
    q = p[(p[:, 2] > z0 + 0.5) & (p[:, 2] < z0 + 3.0)]
    d, _ = roads.query(q[:, :2], distance_upper_bound=ROAD_M)
    return q[~np.isfinite(d)]


def between_drives(clouds: Dict[str, np.ndarray], routes: Dict[str, np.ndarray], min_shared: int = 10) -> Dict[str, Dict]:
    names = sorted(clouds)
    keys = {s: np.floor(clouds[s][:, :2] / FIXED_SQUARE_M).astype(int) for s in names}
    out = {}
    for x in range(len(names)):
        for y in range(x + 1, len(names)):
            A, B = names[x], names[y]
            common = set(map(tuple, np.unique(keys[A], axis=0))) & set(map(tuple, np.unique(keys[B], axis=0)))
            if len(common) < min_shared:
                continue
            roads = cKDTree(np.vstack([routes[A], routes[B]]))
            shifts = []
            for k in sorted(common):
                pa = _fixed_things(clouds[A][(keys[A] == k).all(axis=1)], roads)
                pb = _fixed_things(clouds[B][(keys[B] == k).all(axis=1)], roads)
                t = image_shift(pa, pb, k)
                if t is not None:
                    shifts.append(t)
            r: Dict[str, object] = dict(shared_squares=len(common), clear_squares=len(shifts))
            if shifts:
                h = np.hypot(*np.array(shifts).T)
                r.update(median_m=float(np.median(h)), within_0p3=float(np.mean(h < 0.3)), within_0p5=float(np.mean(h < 0.5)))
            r["verdict"] = ("not judged" if len(shifts) < MIN_CLEAR_SQUARES
                            else "pass" if r["median_m"] <= LIMITS["between_drives_m"] else "fail")
            out[f"{A} / {B}"] = r
            log(f"between {A} / {B}: {r}")
    return out


# --- a drive's own repeat passes -----------------------------------------------------------------------------------

REPEAT_SQUARE_M, REPEAT_GAP_S, REPEAT_VOXEL_M, REPEAT_MIN_POINTS = 50.0, 60.0, 0.2, 1500


def _planar_surfaces(p: np.ndarray, k: int = 16):
    _, nn = cKDTree(p).query(p, k=min(k, len(p)))
    nb = p[nn] - p[nn].mean(axis=1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb))
    planar = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-9) > 0.5
    return planar, v[:, :, 0]


def plane_icp_translation(a: np.ndarray, b: np.ndarray) -> Optional[np.ndarray]:
    """Translation of b onto a by point-to-plane ICP on planar points only."""
    if len(a) < REPEAT_MIN_POINTS or len(b) < REPEAT_MIN_POINTS:
        return None
    pa, na = _planar_surfaces(a)
    pb, _ = _planar_surfaces(b)
    a, na, b = a[pa], na[pa], b[pb]
    if len(a) < REPEAT_MIN_POINTS or len(b) < REPEAT_MIN_POINTS:
        return None
    tree = cKDTree(a)
    t = np.zeros(3)
    for gate in (3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.6, 0.6, 0.4, 0.4, 0.3, 0.3):
        d, i = tree.query(b + t, distance_upper_bound=gate)
        m = np.isfinite(d)
        if m.sum() < REPEAT_MIN_POINTS / 2:
            return None
        n = na[i[m]]
        e = np.sum(n * (a[i[m]] - (b[m] + t)), axis=1)
        t = t + np.linalg.lstsq(n.T @ n + 1e-6 * np.eye(3), n.T @ e, rcond=None)[0]
    return None if np.linalg.norm(t) > 8 else t


def repeat_passes(points: np.ndarray, times: np.ndarray, route: np.ndarray) -> Dict[str, object]:
    d, _ = cKDTree(route).query(points[:, :2], distance_upper_bound=25.0)
    p, t = points[np.isfinite(d)], times[np.isfinite(d)]
    keys = np.floor(p / REPEAT_VOXEL_M).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    p, t = p[first], t[first]
    g = np.floor(p[:, :2] / REPEAT_SQUARE_M).astype(int)
    rows = []
    for k in np.unique(g, axis=0):
        m = (g == k).all(axis=1)
        tk = np.sort(t[m])
        gaps = np.diff(tk)
        if gaps.size == 0 or gaps.max() < REPEAT_GAP_S:
            continue
        cut = tk[int(np.argmax(gaps))]
        r = plane_icp_translation(p[m][t[m] <= cut], p[m][t[m] > cut])
        if r is not None:
            rows.append(r)
    if not rows:
        return dict(squares=0, verdict="not judged")
    r = np.array(rows)
    h, z = np.hypot(r[:, 0], r[:, 1]), np.abs(r[:, 2])
    out = dict(squares=len(r), horizontal_median_m=float(np.median(h)), horizontal_p95_m=float(np.percentile(h, 95)),
               height_median_m=float(np.median(z)), height_p95_m=float(np.percentile(z, 95)))
    ok = out["height_median_m"] <= LIMITS["repeat_height_m"] and out["horizontal_median_m"] <= LIMITS["repeat_horizontal_m"]
    out["verdict"] = "not judged" if len(r) < MIN_CLEAR_SQUARES else "pass" if ok else "fail"
    return out


# --- per drive -----------------------------------------------------------------------------------------------------

def ground_thickness(las_path: str, out_json: str) -> Dict[str, object]:
    subprocess.run([sys.executable, os.path.join(REPO_ROOT, "measure_surface_thickness.py"), "--las", las_path,
                    "--report-json", out_json], check=True, stdout=subprocess.DEVNULL)
    with open(out_json) as f:
        rep = json.load(f)
    ground = next(r for r in rep["results"] if r["label"].startswith("ground"))
    cm = ground["median_m"] * 100
    return dict(ground_median_cm=cm, verdict="pass" if cm <= LIMITS["ground_thickness_cm"] else "fail")


def held_out_height(placement_json: str) -> Dict[str, object]:
    if not os.path.exists(placement_json):
        return dict(verdict="not judged", note="no placement report")
    with open(placement_json) as f:
        rep = json.load(f)
    h = rep.get("held_out", {}).get("height_after")
    if not h:
        return dict(verdict="not judged", note="placement was run without the held-out check")
    return dict(abs_median_m=h["abs_median_m"], abs_p95_m=h["abs_p95_m"], tiles=rep["held_out"]["tiles"],
                verdict="pass" if h["abs_median_m"] <= LIMITS["held_out_height_m"] else "fail")


def check_drives(las_paths: Sequence[str], out_dir: str, placement_dirs: Sequence[str] = ()) -> Dict[str, object]:
    os.makedirs(out_dir, exist_ok=True)
    report: Dict[str, object] = dict(limits=LIMITS, min_clear_squares=MIN_CLEAR_SQUARES, drives={}, between={})
    clouds, routes = {}, {}
    for path in las_paths:
        name = os.path.splitext(os.path.basename(path))[0]
        log(f"checking {name}")
        las = laspy.read(path)
        p = np.c_[las.x, las.y, las.z]
        route = pd.read_csv(path[:-4] + "_trajectory.csv")[["x", "y"]].to_numpy()[::20]
        drive: Dict[str, object] = dict(las=path)
        drive["ground_thickness"] = ground_thickness(path, os.path.join(out_dir, f"{name}_thickness.json"))
        placement = next((os.path.join(d, f"{name}_placement.json") for d in list(placement_dirs) + [os.path.dirname(path)]
                          if os.path.exists(os.path.join(d, f"{name}_placement.json"))), "")
        drive["held_out_height"] = held_out_height(placement)
        drive["repeat_passes"] = repeat_passes(p, np.asarray(las.gps_time, dtype=np.float64), route)
        report["drives"][name] = drive
        log(f"  {json.dumps({k: v for k, v in drive.items() if k != 'las'})}")
        clouds[name], routes[name] = p, route
    report["between"] = between_drives(clouds, routes)
    with open(os.path.join(out_dir, "checks.json"), "w") as f:
        json.dump(report, f, indent=1)
    with open(os.path.join(out_dir, "CHECKS.md"), "w", encoding="utf-8") as f:
        f.write(render_markdown(report))
    return report


def render_markdown(report: Dict[str, object]) -> str:
    L = report["limits"]
    lines = ["# Acceptance checks", "",
             "Each check uses something the processing was not fitted to; see `drive_checks.py` for how each is made.", "",
             "## Per drive", "",
             f"| drive | ground thickness (limit {L['ground_thickness_cm']} cm) | height on airborne, held out (limit {L['held_out_height_m']} m) "
             f"| repeat passes, height / horizontal (limits {L['repeat_height_m']} / {L['repeat_horizontal_m']} m) |",
             "|---|---|---|---|"]

    def cell(v: Dict, text: str) -> str:
        return f"{text} **{v['verdict']}**" if "verdict" in v else text

    for name, d in report["drives"].items():
        g, h, r = d["ground_thickness"], d["held_out_height"], d["repeat_passes"]
        gt = cell(g, f"{g['ground_median_cm']:.2f} cm")
        ht = cell(h, f"{h['abs_median_m']:.2f} m" if "abs_median_m" in h else h.get("note", ""))
        rt = cell(r, f"{r['height_median_m']:.3f} / {r['horizontal_median_m']:.2f} m ({r['squares']} squares)" if r.get("squares") else "no repeat passes")
        lines.append(f"| {name} | {gt} | {ht} | {rt} |")
    lines += ["", "## Between drives, fixed things only", "",
              f"Median horizontal disagreement where two drives cover the same streets (limit {L['between_drives_m']} m; "
              f"pairs with fewer than {report['min_clear_squares']} clear squares are not judged).", "",
              "| drives | median | within 0.5 m | clear / shared squares | verdict |", "|---|---|---|---|---|"]
    for pair, r in report["between"].items():
        med = f"{r['median_m']:.2f} m" if "median_m" in r else "-"
        w = f"{r['within_0p5']:.0%}" if "within_0p5" in r else "-"
        lines.append(f"| {pair} | {med} | {w} | {r['clear_squares']} / {r['shared_squares']} | {r['verdict']} |")
    return "\n".join(lines) + "\n"


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--las", required=True, nargs="+", help="finished clouds, each with its _trajectory.csv beside it")
    ap.add_argument("--placement-dir", nargs="*", default=[], help="folders holding <stem>_placement.json, if not beside the clouds")
    ap.add_argument("--output-dir", required=True)
    a = ap.parse_args(argv)
    check_drives(a.las, a.output_dir, a.placement_dir)


if __name__ == "__main__":
    main()
