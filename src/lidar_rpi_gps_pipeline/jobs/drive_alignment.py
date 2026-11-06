#!/usr/bin/env python3
"""
Pull drives that cover the same streets together horizontally.

Each drive is placed on the map on its own (GPS, then `airborne_placement`), and the
horizontal part of that placement is good to 1-3 m, differently for each drive and
changing along each route. Where two drives cover the same street the same tree then
appears twice. This moves every drive by an east/north shift that changes smoothly
along its route so that they agree where they overlap. Heights are not touched.

- Unknowns: per drive, a shift at a node every `node_m` metres of its own route,
  interpolated linearly between nodes.
- Measurements: in every `square_m` square two drives share, the translation of one
  onto the other by generalized ICP on all points near both routes, so that trunks
  and poles hold the direction along the street, which walls and road do not.
- Solve: least squares over all drives together, with each measurement, neighbouring
  nodes changing by `smooth_m` or less, and a weak pull (`prior_m`) towards no shift,
  which keeps the group where the airborne placement put it.
- By place: each point takes the shift of the nearest part of its own route among the
  route within `reach_s` of the point's time. Shifting by the vehicle's position when a
  point was seen would give one object a different shift per sighting and smear it.
- Revisits: where one drive passes a place twice, both passes take the same shift, so
  the drive is not pulled apart from itself.
- Robust: squares that disagree with the solution by much more than `robust_m` are
  weighted down and the solve repeated. Where two drives overlap only at a bare
  junction, neighbouring squares can contradict each other by metres (cars, too few
  fixed things); weighted alike they leave the pair about 1 m apart (2026-10-28,
  101705/102349).

A quarter of the shared squares, chosen by place, are left out of the solve and
scored before and after, as a check on squares the solve did not see.

Writes `<stem>.las` and `<stem>_trajectory.csv` per drive to the output folder and
`drive_alignment.json` there; the inputs are not touched.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from typing import Dict, List, Optional, Sequence, Tuple

import laspy
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

SQUARE_M = 25.0
NODE_M = 50.0
SMOOTH_M = 0.5
PRIOR_M = 3.0
MEASUREMENT_M = 0.15
RADIUS_M = 50.0
REACH_S = 60.0
SELF_TIE_M = 0.05
REVISIT_NEAR_M = 5.0
HOLD_EVERY = 4
VOXEL_M = 0.2
MIN_POINTS = 1500
MIN_FITNESS = 0.3
MAX_SHIFT_M = 8.0
ROBUST_M = 0.3
ROBUST_ITERATIONS = 20


def log(message: str) -> None:
    print(message, flush=True)


def _voxel_first(points: np.ndarray, size: float) -> np.ndarray:
    """Index of the first point in each `size` cube."""
    keys = np.floor(points / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return np.sort(first)


def _covariances(points: np.ndarray, k: int = 20, epsilon: float = 1e-3) -> np.ndarray:
    """Per-point covariance of its k neighbours, regularised as in generalized ICP: the smallest direction set to
    `epsilon`, the other two to 1, so each point counts as a small piece of plane."""
    _, nn = cKDTree(points).query(points, k=min(k, len(points)))
    nb = points[nn] - points[nn].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / nn.shape[1]
    _, v = np.linalg.eigh(cov)
    d = np.array([epsilon, 1.0, 1.0])
    return np.einsum("nij,j,nkj->nik", v, d, v)


def gicp_translation(a: np.ndarray, b: np.ndarray) -> Optional[np.ndarray]:
    """Translation that moves `b` onto `a`, by generalized ICP (plane to plane), coarse to fine. None when the two do
    not overlap well enough or the answer is implausibly large."""
    if len(a) < MIN_POINTS or len(b) < MIN_POINTS:
        return None
    centre = a.mean(axis=0)
    a, b = a - centre, b - centre
    ca, cb = _covariances(a), _covariances(b)
    tree = cKDTree(a)
    t = np.zeros(3)
    for gate in (2.0, 1.0, 0.5, 0.3):
        for _ in range(30):
            d, j = tree.query(b + t, distance_upper_bound=gate)
            ok = np.isfinite(d)
            if ok.sum() < MIN_POINTS / 2:
                return None
            M = np.linalg.inv(ca[j[ok]] + cb[ok])
            r = a[j[ok]] - (b[ok] + t)
            step = np.linalg.solve(M.sum(axis=0), np.einsum("nij,nj->i", M, r))
            t = t + step
            if np.linalg.norm(step) < 1e-4:
                break
    if ok.mean() < MIN_FITNESS or np.linalg.norm(t) > MAX_SHIFT_M:
        return None
    return t


class Drive:
    """One drive's cloud and route, and the shift along that route."""

    def __init__(self, las_path: str, node_m: float, radius_m: float):
        self.las_path = las_path
        self.trajectory_path = las_path[:-4] + "_trajectory.csv"
        self.name = os.path.splitext(os.path.basename(las_path))[0]
        tr = pd.read_csv(self.trajectory_path)
        self.x, self.y, self.t = tr.x.to_numpy(), tr.y.to_numpy(), tr.time_s.to_numpy()
        self.arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))]
        self.node_m = node_m
        self.n_nodes = int(self.arc[-1] // node_m) + 2
        las = laspy.read(las_path)
        p = np.c_[las.x, las.y, las.z]
        times = np.asarray(las.gps_time, dtype=np.float64)
        d, _ = cKDTree(np.c_[self.x, self.y][::20]).query(p[:, :2], distance_upper_bound=radius_m)
        near = np.isfinite(d)
        p, times = p[near], times[near]
        keep = _voxel_first(p, VOXEL_M)
        self.points, self.point_times = p[keep], times[keep]
        keep_st = np.r_[True, np.diff(np.floor(self.arc / 0.5)) > 0]
        self._station_xy = np.c_[self.x, self.y][keep_st]
        self._station_t, self._station_arc = self.t[keep_st], self.arc[keep_st]
        self._stations = cKDTree(self._station_xy)
        self.point_arc = self.place_arc(self.points[:, :2], self.point_times)

    def place_arc(self, xy: np.ndarray, times: np.ndarray, reach_s: float = REACH_S) -> np.ndarray:
        """Route position nearest each point, among the route within `reach_s` of the point's time; the vehicle's
        position at that time where none is."""
        out = np.interp(times, self.t, self.arc)
        k = min(32, len(self._station_xy))
        for s in range(0, len(times), 1_000_000):
            sl = slice(s, s + 1_000_000)
            _, j = self._stations.query(xy[sl], k=k)
            j = j.reshape(len(j), -1)
            ok = np.abs(self._station_t[j] - times[sl, None]) <= reach_s
            first = np.argmax(ok, axis=1)
            nearest = self._station_arc[j[np.arange(len(first)), first]]
            out[sl] = np.where(ok.any(axis=1), nearest, out[sl])
        return out

    def weights(self, arc: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        u = arc / self.node_m
        i = np.clip(np.floor(u).astype(int), 0, self.n_nodes - 2)
        return i, np.clip(u - i, 0.0, 1.0)

    def shift_at(self, nodes: np.ndarray, arc: np.ndarray) -> np.ndarray:
        i, f = self.weights(arc)
        return nodes[i] * (1 - f)[:, None] + nodes[i + 1] * f[:, None]

    def revisits(self, near_m: float = REVISIT_NEAR_M, reach_s: float = REACH_S) -> List[Tuple[float, float]]:
        """Route positions where the drive comes back to a place at least `reach_s` later, one per 10 m."""
        keep = np.r_[True, np.diff(np.floor(self.arc / 10.0)) > 0]
        xy, t, a = np.c_[self.x, self.y][keep], self.t[keep], self.arc[keep]
        tree = cKDTree(xy)
        out = []
        for i in range(len(xy)):
            for j in tree.query_ball_point(xy[i], near_m):
                if t[j] - t[i] > reach_s:
                    out.append((float(a[i]), float(a[j])))
                    break
        return out


def _held_out(key: Sequence[int]) -> bool:
    return (key[0] * 7 + key[1] * 13) % HOLD_EVERY == 0


def measure(drives: List[Drive], shifts: Optional[List[np.ndarray]] = None, square_m: float = SQUARE_M) -> List[Dict]:
    """One row per square shared by two drives: the translation of the second onto the first."""
    rows = []
    for x in range(len(drives)):
        for y in range(x + 1, len(drives)):
            A, B = drives[x], drives[y]
            pa = A.points if shifts is None else A.points + np.c_[shifts[x], np.zeros(len(shifts[x]))]
            pb = B.points if shifts is None else B.points + np.c_[shifts[y], np.zeros(len(shifts[y]))]
            ga, gb = np.floor(pa[:, :2] / square_m).astype(int), np.floor(pb[:, :2] / square_m).astype(int)
            common = set(map(tuple, np.unique(ga, axis=0))) & set(map(tuple, np.unique(gb, axis=0)))
            n = 0
            for k in sorted(common):
                ma, mb = (ga == k).all(axis=1), (gb == k).all(axis=1)
                t = gicp_translation(pa[ma], pb[mb])
                if t is None:
                    continue
                rows.append(dict(a=x, b=y, key=[int(k[0]), int(k[1])], t=[float(v) for v in t],
                                 arc_a=float(np.median(A.point_arc[ma])), arc_b=float(np.median(B.point_arc[mb]))))
                n += 1
            if common:
                log(f"  {A.name} / {B.name}: {len(common)} shared squares, {n} measured")
    return rows


def summary(rows: List[Dict], held: bool) -> Dict[str, Optional[float]]:
    h = np.array([np.hypot(*r["t"][:2]) for r in rows if _held_out(r["key"]) == held])
    if not len(h):
        return dict(squares=0)
    return dict(squares=int(len(h)), horizontal_median_m=float(np.median(h)), horizontal_p95_m=float(np.percentile(h, 95)),
                within_0p3=float(np.mean(h <= 0.3)))


def solve(drives: List[Drive], rows: List[Dict], smooth_m: float, prior_m: float,
          robust_m: Optional[float] = ROBUST_M) -> Tuple[List[np.ndarray], int]:
    """Node shifts of every drive, east and north. Returns them and the number of revisit ties.

    With `robust_m`, squares are reweighted by a Cauchy kernel on their horizontal misfit, re-solving until the weights
    settle: a square that disagrees with its neighbours by much more than `robust_m` (cars, a bare junction, a wrong
    ICP minimum) stops pulling the solution towards it. Each row's final weight is written into it as `weight`."""
    offsets = np.cumsum([0] + [d.n_nodes * 2 for d in drives])
    eqs: List[Tuple[Dict[int, float], float, float]] = []
    owner: List[int] = []

    def node_terms(d: int, arc: float, axis: int, sign: float, e: Dict[int, float]) -> None:
        i, f = drives[d].weights(np.array([arc]))
        for node, w in ((i[0], 1 - f[0]), (i[0] + 1, f[0])):
            c = int(offsets[d] + node * 2 + axis)
            e[c] = e.get(c, 0.0) + sign * w

    used = [r for r in rows if not _held_out(r["key"])]
    for k, r in enumerate(used):
        for axis in (0, 1):
            e: Dict[int, float] = {}
            # shift_b - shift_a = t: b moved by t lands on a.
            node_terms(r["b"], r["arc_b"], axis, 1.0, e)
            node_terms(r["a"], r["arc_a"], axis, -1.0, e)
            eqs.append((e, r["t"][axis], MEASUREMENT_M))
            owner.append(k)
    ties = 0
    for d, drive in enumerate(drives):
        for a1, a2 in drive.revisits():
            for axis in (0, 1):
                e = {}
                node_terms(d, a2, axis, 1.0, e)
                node_terms(d, a1, axis, -1.0, e)
                eqs.append((e, 0.0, SELF_TIE_M))
            ties += 1
        for n in range(drive.n_nodes):
            for axis in (0, 1):
                c = int(offsets[d] + n * 2 + axis)
                eqs.append(({c: 1.0}, 0.0, prior_m))
                if n + 1 < drive.n_nodes:
                    eqs.append(({c + 2: 1.0, c: -1.0}, 0.0, smooth_m))
    M = np.zeros((len(eqs), int(offsets[-1])))
    b = np.zeros(len(eqs))
    for i, (e, value, sigma) in enumerate(eqs):
        for c, v in e.items():
            M[i, c] += v / sigma
        b[i] = value / sigma
    n_meas = len(owner)
    owner_a = np.array(owner, dtype=int)
    w = np.ones(len(used))
    for _ in range(ROBUST_ITERATIONS if robust_m else 1):
        scale = np.r_[np.sqrt(w[owner_a]), np.ones(len(eqs) - n_meas)]
        x = np.linalg.lstsq(M * scale[:, None], b * scale, rcond=None)[0]
        if not robust_m:
            break
        # Misfit per square in metres, both axes together.
        r2 = np.zeros(len(used))
        np.add.at(r2, owner_a, ((M[:n_meas] @ x - b[:n_meas]) * MEASUREMENT_M) ** 2)
        new_w = 1.0 / (1.0 + r2 / robust_m**2)
        settled = np.max(np.abs(new_w - w)) < 1e-3
        w = new_w
        if settled:
            break
    for r, wk in zip(used, w):
        r["weight"] = float(wk)
    return [x[offsets[d] : offsets[d + 1]].reshape(-1, 2) for d in range(len(drives))], ties


def write_aligned(drive: Drive, nodes: np.ndarray, out_dir: str) -> Dict[str, object]:
    las = laspy.read(drive.las_path)
    xy = np.c_[np.asarray(las.x), np.asarray(las.y)]
    shift = drive.shift_at(nodes, drive.place_arc(xy, np.asarray(las.gps_time, dtype=np.float64)))
    las.x = xy[:, 0] + shift[:, 0]
    las.y = xy[:, 1] + shift[:, 1]
    out_las = os.path.join(out_dir, os.path.basename(drive.las_path))
    las.write(out_las)
    tr = pd.read_csv(drive.trajectory_path)
    s = drive.shift_at(nodes, drive.arc)
    tr["x"] += s[:, 0]
    tr["y"] += s[:, 1]
    out_trajectory = os.path.join(out_dir, os.path.basename(drive.trajectory_path))
    tr.to_csv(out_trajectory, index=False)
    # The fusion QA and the placement report stay with the cloud: they record how it was built and placed.
    for suffix in ("_qa.json", "_placement.json"):
        record = drive.las_path[:-4] + suffix
        if os.path.exists(record):
            shutil.copy2(record, os.path.join(out_dir, os.path.basename(record)))
    m = np.hypot(*nodes.T)
    return dict(output_las=out_las, output_trajectory_csv=out_trajectory, shift_median_m=float(np.median(m)),
                shift_max_m=float(m.max()))


def align_drives(las_paths: Sequence[str], out_dir: str, *, node_m: float = NODE_M, smooth_m: float = SMOOTH_M,
                 prior_m: float = PRIOR_M, radius_m: float = RADIUS_M,
                 robust_m: Optional[float] = ROBUST_M) -> Dict[str, object]:
    t0 = time.time()
    if os.path.abspath(out_dir) in {os.path.abspath(os.path.dirname(p)) for p in las_paths}:
        raise ValueError("The output folder must differ from the inputs' folders; the inputs are not overwritten.")
    os.makedirs(out_dir, exist_ok=True)
    drives = []
    for p in las_paths:
        drives.append(Drive(p, node_m, radius_m))
        log(f"{drives[-1].name}: {len(drives[-1].points):,} points within {radius_m:.0f} m, route {drives[-1].arc[-1]:.0f} m")
    log("measuring shared squares")
    before = measure(drives)
    if not before:
        raise ValueError("No two drives share a square that could be measured; nothing to align.")
    nodes, ties = solve(drives, before, smooth_m, prior_m, robust_m)
    log(f"solved: {len(before)} squares, {ties} revisit ties within drives")
    after = measure(drives, [d.shift_at(n, d.point_arc) for d, n in zip(drives, nodes)])
    report: Dict[str, object] = dict(
        inputs=list(las_paths), square_m=SQUARE_M, node_m=node_m, smooth_m=smooth_m, prior_m=prior_m,
        measurement_m=MEASUREMENT_M, radius_m=radius_m, reach_s=REACH_S, self_tie_m=SELF_TIE_M, revisit_ties=ties,
        robust_m=robust_m,
        before=dict(fitted=summary(before, False), held_out=summary(before, True)),
        after=dict(fitted=summary(after, False), held_out=summary(after, True)),
        drives={}, rows_before=before, rows_after=after,
    )
    log(f"held-out squares: {report['before']['held_out']} -> {report['after']['held_out']}")
    for d, n in zip(drives, nodes):
        report["drives"][d.name] = dict(nodes=n.tolist(), **write_aligned(d, n, out_dir))
        log(f"{d.name}: shift median {report['drives'][d.name]['shift_median_m']:.2f} m, max {report['drives'][d.name]['shift_max_m']:.2f} m")
    report["elapsed_s"] = round(time.time() - t0, 1)
    with open(os.path.join(out_dir, "drive_alignment.json"), "w") as f:
        json.dump(report, f, indent=1)
    return report


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--las", required=True, nargs="+", help="placed clouds, each with its _trajectory.csv beside it")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--node-m", type=float, default=NODE_M, help="spacing of shift nodes along each route")
    ap.add_argument("--smooth-m", type=float, default=SMOOTH_M, help="allowed change of shift between neighbouring nodes")
    ap.add_argument("--prior-m", type=float, default=PRIOR_M, help="pull of every node towards no shift")
    ap.add_argument("--radius-m", type=float, default=RADIUS_M, help="points used, distance from their own route")
    ap.add_argument("--robust-m", type=float, default=ROBUST_M,
                    help="misfit at which a square's weight halves; 0 weighs every square alike")
    a = ap.parse_args(argv)
    align_drives(a.las, a.output_dir, node_m=a.node_m, smooth_m=a.smooth_m, prior_m=a.prior_m, radius_m=a.radius_m,
                 robust_m=a.robust_m or None)


if __name__ == "__main__":
    main()
