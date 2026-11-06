#!/usr/bin/env python3
"""
Place one finished drive on airborne lidar, then check the placement on ground it did not see.

A `slam_gps_anchor` cloud sits where GPS put it: within a metre or two horizontally,
and its height follows the receiver's. Airborne lidar of the same streets is an
independent survey with height good to centimetres, so the cloud is moved onto it
by one rigid transform that does not bend it:

- Height: the offset of our road surface from the airborne bare earth, per 1 m cell
  along the route, taken robustly (Huber).
- Horizontal: the shift that puts our facades (tall, thin columns of points) on the
  edge of the airborne building footprints, by grid search.

A cloud levelled with `--gravity-level` needs no rotation, and fitting one here
would follow the airborne data's own small tilts, so by default none is fitted
(`--level-plane` adds one).

The airborne tiles are in NAD83(2011) / NAVD88. The cloud is in the GPS frame:
WGS84 as realised by the receiver (ITRF2014 at the capture epoch), UTM, heights above
EGM96. The tiles are moved into the cloud's frame with PROJ, once at the site centre
(constant to millimetres over a few km), so the output stays in the cloud's frame.

Check, written to the report: the route is cut into tiles; each is left out (with a
buffer), the transform is fitted on the rest and the tile is scored on height and
facade distance. A placement that only fits the data it saw shows up here.

Writes `<stem>.las`, `<stem>_trajectory.csv` and `<stem>_placement.json` to the
output folder; the input is not touched.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import laspy
import numpy as np
import pandas as pd
from scipy import ndimage
from scipy.spatial import cKDTree

# Facade raster and its search.
FOOTPRINT_CELL_M = 0.25
FACADE_TRUNCATE_M = 1.5
SHIFT_SEARCH_M = 5.0
# Leave-one-tile-out check.
CHECK_TILE_M = 150.0
CHECK_BUFFER_M = 10.0
CHECK_MIN_CELLS = 300
# Corridors about the route.
REFERENCE_CORRIDOR_M = 40.0
CLOUD_CORRIDOR_M = 30.0
ROAD_CORRIDOR_M = 8.0
GROUND_CORRIDOR_M = 11.0


def log(message: str) -> None:
    print(message, flush=True)


def datum_shift(easting: float, northing: float, epoch: float) -> np.ndarray:
    """(dE, dN, dZ) that moves an airborne point at (easting, northing), NAD83(2011) UTM 14N / NAVD88,
    into the cloud's frame: WGS84 UTM 14N (ITRF2014 at `epoch`) / EGM96."""
    import pyproj
    from pyproj import Transformer

    pyproj.network.set_network_enabled(True)
    lon, lat = Transformer.from_crs("EPSG:6343", "EPSG:6318", always_xy=True).transform(easting, northing)
    _, _, h_nad = Transformer.from_crs("EPSG:6318+5703", "EPSG:6319", always_xy=True).transform(lon, lat, 0.0)
    lon2, lat2, h_itrf, _ = Transformer.from_crs("EPSG:6319", "EPSG:7912", always_xy=True).transform(lon, lat, h_nad, epoch)
    _, _, h_egm = Transformer.from_crs("EPSG:4979", "EPSG:4326+5773", always_xy=True).transform(lon2, lat2, h_itrf)
    e2, n2 = Transformer.from_crs("EPSG:4326", "EPSG:32614", always_xy=True).transform(lon2, lat2)
    return np.array([e2 - easting, n2 - northing, h_egm])


def read_corridor(
    paths: Sequence[str], track: cKDTree, radius: float, shift: np.ndarray = np.zeros(3), with_class: bool = False
):
    """Stream LAS/LAZ files, keeping points within `radius` of the track, moved by `shift`."""
    out, classes = [], []
    lo, hi = track.mins - radius, track.maxes + radius
    for path in paths:
        with laspy.open(path) as f:
            for chunk in f.chunk_iterator(5_000_000):
                x, y = np.asarray(chunk.x) + shift[0], np.asarray(chunk.y) + shift[1]
                box = (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
                if not box.any():
                    continue
                p = np.column_stack([x[box], y[box], np.asarray(chunk.z)[box] + shift[2]])
                d, _ = track.query(p[:, :2], distance_upper_bound=radius)
                near = np.isfinite(d)
                out.append(p[near])
                if with_class:
                    classes.append(np.asarray(chunk.classification)[box][near])
    if not out:
        raise ValueError(f"No points of {len(paths)} file(s) lie within {radius} m of the route.")
    points = np.concatenate(out)
    return (points, np.concatenate(classes)) if with_class else points


def corridor_mask(xy: np.ndarray, track_xy: np.ndarray, radius: float) -> np.ndarray:
    d, _ = cKDTree(track_xy).query(xy, distance_upper_bound=radius)
    return np.isfinite(d)


def _shift(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = dx, dy, dz
    return T


def _about(centre: np.ndarray, R: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = centre - R @ centre
    return T


def _apply(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    return p @ T[:3, :3].T + T[:3, 3]


def _level_rotation(a: float, b: float) -> np.ndarray:
    """Rotation taking the normal of z = a x + b y to straight up."""
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    v = np.cross(n, [0.0, 0.0, 1.0])
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - n[2]) / s**2)


def _huber_plane(x, y, z, centre, delta: float = 0.15, iterations: int = 25) -> np.ndarray:
    """Huber plane z = a (x - cx) + b (y - cy) + c; returns (a, b, c)."""
    A = np.c_[x - centre[0], y - centre[1], np.ones(len(x))]
    s = np.array([0.0, 0.0, np.median(z)])
    for _ in range(iterations):
        r = np.abs(z - A @ s)
        w = np.where(r <= delta, 1.0, delta / np.maximum(r, 1e-9))
        s = np.linalg.lstsq(A * w[:, None] ** 0.5, z * w**0.5, rcond=None)[0]
    return s


class AirborneReference:
    """The airborne survey along a route, moved into the cloud's frame."""

    def __init__(self, tile_paths: Sequence[str], track_xy: np.ndarray, epoch: float):
        centre = track_xy.mean(axis=0)
        self.shift = datum_shift(float(centre[0]), float(centre[1]), epoch)
        log(f"airborne to cloud frame: dE {self.shift[0]:+.3f} dN {self.shift[1]:+.3f} dZ {self.shift[2]:+.3f} m")
        points, classes = read_corridor(
            tile_paths, cKDTree(track_xy), REFERENCE_CORRIDOR_M, shift=self.shift, with_class=True
        )
        ground, building = points[classes == 2], points[classes == 6]
        log(f"airborne ground {len(ground):,}, building {len(building):,} points")
        if len(ground) == 0:
            raise ValueError("The airborne tiles hold no ground (class 2) points along the route.")
        # Bare earth per 1 m cell, flat cells only.
        g = ground[corridor_mask(ground[:, :2], track_xy, GROUND_CORRIDOR_M)]
        df = pd.DataFrame({"i": np.floor(g[:, 0]).astype(np.int64), "j": np.floor(g[:, 1]).astype(np.int64), "z": g[:, 2]})
        stats = df.groupby(["i", "j"]).z.agg(["mean", "count", "min", "max"])
        self.ground_cells = stats[(stats["count"] >= 2) & (stats["max"] - stats["min"] <= 0.15)][["mean"]]
        # Building footprint, its edge, and the distance to that edge.
        self.origin = track_xy.min(axis=0) - 50.0
        self.shape = tuple(np.ceil((track_xy.max(axis=0) + 50.0 - self.origin) / FOOTPRINT_CELL_M).astype(int) + 1)
        ij = np.floor((building[:, :2] - self.origin) / FOOTPRINT_CELL_M).astype(int)
        ok = (ij >= 0).all(axis=1) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] < self.shape[1])
        footprint = np.zeros(self.shape, bool)
        footprint[ij[ok, 0], ij[ok, 1]] = True
        footprint = ndimage.binary_closing(footprint, iterations=2)
        footprint = ndimage.binary_fill_holes(footprint)
        footprint = ndimage.binary_opening(footprint, iterations=2)
        edge = footprint & ~ndimage.binary_erosion(footprint)
        self.edge_distance_raster = ndimage.distance_transform_edt(~edge) * FOOTPRINT_CELL_M
        self.near_building = ndimage.distance_transform_edt(~footprint) * FOOTPRINT_CELL_M <= 6.0

    def edge_distance(self, xy: np.ndarray) -> np.ndarray:
        q = (xy - self.origin) / FOOTPRINT_CELL_M - 0.5
        return ndimage.map_coordinates(self.edge_distance_raster, [q[:, 0], q[:, 1]], order=1, mode="nearest")


def road_cells(points: np.ndarray, ref: AirborneReference, track_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Cell centres and (our road, 5th height percentile, minus airborne bare earth) per 1 m cell."""
    r = points[corridor_mask(points[:, :2], track_xy, ROAD_CORRIDOR_M)]
    df = pd.DataFrame({"i": np.floor(r[:, 0]).astype(np.int64), "j": np.floor(r[:, 1]).astype(np.int64), "z": r[:, 2]})
    g = df.groupby(["i", "j"]).z
    cells = pd.DataFrame({"p5": g.quantile(0.05), "n": g.size()})
    cells = cells[cells["n"] >= 20].join(ref.ground_cells, how="inner").reset_index()
    return np.c_[cells.i + 0.5, cells.j + 0.5], (cells.p5 - cells["mean"]).to_numpy()


def facade_cells(points: np.ndarray, ref: AirborneReference) -> np.ndarray:
    """Centres of tall, thin columns of points near an airborne building footprint."""
    ij = np.floor((points[:, :2] - ref.origin) / FOOTPRINT_CELL_M).astype(int)
    ok = (ij >= 0).all(axis=1) & (ij[:, 0] < ref.shape[0]) & (ij[:, 1] < ref.shape[1])
    df = pd.DataFrame({"i": ij[ok, 0], "j": ij[ok, 1], "z": points[ok, 2]})
    g = df.groupby(["i", "j"]).z
    st = pd.DataFrame({"lo": g.quantile(0.05), "hi": g.quantile(0.95), "n": g.size()}).reset_index()
    st = st[(st.hi - st.lo > 2.5) & (st.n >= 30)]
    i, j = st.i.to_numpy(), st.j.to_numpy()
    keep = ref.near_building[i, j]
    return ref.origin + (np.c_[i[keep], j[keep]] + 0.5) * FOOTPRINT_CELL_M


def fit_horizontal_shift(ref: AirborneReference, facades: np.ndarray, rng: np.random.Generator) -> Tuple[float, float]:
    """Shift putting the facades on the footprint edges: 0.25 m grid over ±5 m, then 0.05 m about the best."""
    if len(facades) == 0:
        raise ValueError("No facade cells near airborne buildings; the horizontal shift cannot be fitted.")

    def score(xy: np.ndarray) -> float:
        return float(np.minimum(ref.edge_distance(xy), FACADE_TRUNCATE_M).mean())

    sample = facades[rng.choice(len(facades), min(len(facades), 6000), replace=False)]
    steps = np.arange(-SHIFT_SEARCH_M, SHIFT_SEARCH_M + 0.01, 0.25)
    best = min(((score(sample + [dx, dy]), dx, dy) for dx in steps for dy in steps))
    fine = np.arange(-0.3, 0.301, 0.05)
    best = min(((score(facades + [dx, dy]), dx, dy) for dx in best[1] + fine for dy in best[2] + fine))
    return float(best[1]), float(best[2])


def fit_placement(
    points: np.ndarray,
    ref: AirborneReference,
    track_xy: np.ndarray,
    excluded: Callable[[np.ndarray], np.ndarray],
    rng: np.random.Generator,
    level_plane: bool = False,
) -> Dict[str, object]:
    """Fit on everything `excluded` leaves. Returns the 4x4 transform and what went into it."""

    def height(T: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        xy, dz = road_cells(_apply(T, points), ref, track_xy)
        keep = ~excluded(xy)
        if keep.sum() < 50:
            raise ValueError(f"Only {int(keep.sum())} road cells match airborne bare earth; too few to fit height.")
        centre = xy[keep].mean(axis=0)
        a, b, _ = _huber_plane(xy[keep, 0], xy[keep, 1], dz[keep], centre)
        L = T
        if level_plane:
            L = _about(np.r_[centre, np.median(points[:, 2])], _level_rotation(a, b)) @ T
            xy, dz = road_cells(_apply(L, points), ref, track_xy)
            keep = ~excluded(xy)
        offset = float(_huber_plane(xy[keep, 0], xy[keep, 1], dz[keep], centre)[2])
        plane = dict(slope_x=float(a), slope_y=float(b), deg=float(np.degrees(np.arctan(np.hypot(a, b)))), cells=int(keep.sum()))
        return _shift(dz=-offset) @ L, plane

    L, plane = height(np.eye(4))
    facades = facade_cells(_apply(L, points), ref)
    dx, dy = fit_horizontal_shift(ref, facades[~excluded(facades)], rng)
    T, plane_after = height(_shift(dx, dy) @ L)
    return dict(T=T, dE=dx, dN=dy, plane=plane, plane_after_shift=plane_after, facades=facades)


def held_out_check(points: np.ndarray, ref: AirborneReference, track_xy: np.ndarray, rng: np.random.Generator,
                   level_plane: bool = False) -> Dict[str, object]:
    """Leave each route tile out, fit on the rest, score the tile. Heights: road minus bare earth; walls: facade distance
    to the footprint edge."""
    xy0, dz0 = road_cells(points, ref, track_xy)
    facades0 = facade_cells(points, ref)
    lo = track_xy.min(axis=0) - 30.0

    def tile_of(xy: np.ndarray) -> np.ndarray:
        return np.floor((xy - lo) / CHECK_TILE_M).astype(int)

    road_tiles, facade_tiles = tile_of(xy0), tile_of(facades0)
    tiles = [
        t for t in np.unique(road_tiles, axis=0)
        if (road_tiles == t).all(axis=1).sum() >= CHECK_MIN_CELLS and (facade_tiles == t).all(axis=1).sum() >= CHECK_MIN_CELLS
    ]
    pools: Dict[str, List[np.ndarray]] = {"height_before": [], "height_after": [], "wall_before": [], "wall_after": []}
    folds = []
    for t in tiles:
        box_lo, box_hi = lo + t * CHECK_TILE_M, lo + (t + 1) * CHECK_TILE_M

        def inside(xy: np.ndarray, margin: float = 0.0) -> np.ndarray:
            return ((xy >= box_lo - margin) & (xy < box_hi + margin)).all(axis=1)

        fit = fit_placement(points, ref, track_xy, lambda xy: inside(xy, CHECK_BUFFER_M), rng, level_plane)
        xy, dz = road_cells(_apply(fit["T"], points), ref, track_xy)
        f = fit["facades"]
        walls = ref.edge_distance(f[inside(f)] + [fit["dE"], fit["dN"]])
        pools["height_before"].append(dz0[inside(xy0)])
        pools["height_after"].append(dz[inside(xy)])
        pools["wall_before"].append(ref.edge_distance(facades0[inside(facades0)]))
        pools["wall_after"].append(walls)
        folds.append(dict(tile=t.tolist(), dE=fit["dE"], dN=fit["dN"], height_abs_median_m=float(np.median(np.abs(dz[inside(xy)]))),
                          wall_median_m=float(np.median(walls))))
        log(f"held-out tile {t.tolist()}: fitted dE {fit['dE']:+.2f} dN {fit['dN']:+.2f}, "
            f"height |d| {folds[-1]['height_abs_median_m']:.2f} m, wall {folds[-1]['wall_median_m']:.2f} m")
    out: Dict[str, object] = dict(tiles=len(tiles), folds=folds)
    for name, v in pools.items():
        if v:
            x = np.concatenate(v)
            out[name] = dict(n=int(len(x)), median_m=float(np.median(x)), abs_median_m=float(np.median(np.abs(x))),
                             abs_p95_m=float(np.percentile(np.abs(x), 95)), within_0p5=float(np.mean(np.abs(x) <= 0.5)))
    return out


def write_placed(las_path: str, trajectory_path: str, T: np.ndarray, out_dir: str, stem: str) -> Dict[str, object]:
    """Write the cloud and trajectory moved by the rigid transform T. Stops if the written cloud's shape changed."""
    from scipy.spatial.transform import Rotation

    R, t = T[:3, :3], T[:3, 3]
    if abs(np.linalg.det(R) - 1) > 1e-9 or np.abs(R @ R.T - np.eye(3)).max() > 1e-9:
        raise ValueError("The placement is not a rigid transform.")
    os.makedirs(out_dir, exist_ok=True)
    las = laspy.read(las_path)
    before = np.c_[las.x, las.y, las.z]
    after = before @ R.T + t
    las.x, las.y, las.z = after[:, 0], after[:, 1], after[:, 2]
    out_las = os.path.join(out_dir, f"{stem}.las")
    tmp = out_las[:-4] + ".tmp.las"
    las.write(tmp)
    back = laspy.read(tmp)
    written = np.c_[back.x, back.y, back.z]
    rng = np.random.default_rng(0)
    n = len(written)
    i, j = rng.integers(0, n, 1_000_000), rng.integers(0, n, 1_000_000)
    worst = float(np.abs(np.linalg.norm(before[i] - before[j], axis=1) - np.linalg.norm(written[i] - written[j], axis=1)).max())
    if worst > 0.005:
        os.remove(tmp)
        raise ValueError(f"Placing changed the cloud's shape by up to {worst:.3f} m.")
    os.replace(tmp, out_las)

    tr = pd.read_csv(trajectory_path)
    tr[["x", "y", "z"]] = tr[["x", "y", "z"]].to_numpy() @ R.T + t
    attitude = Rotation.from_euler("ZYX", tr[["yaw_deg", "pitch_deg", "roll_deg"]].to_numpy(), degrees=True).as_matrix()
    yaw, pitch, roll = Rotation.from_matrix(R @ attitude).as_euler("ZYX", degrees=True).T
    tr["yaw_deg"], tr["pitch_deg"], tr["roll_deg"] = yaw, pitch, roll
    out_trajectory = os.path.join(out_dir, f"{stem}_trajectory.csv")
    tr.to_csv(out_trajectory, index=False)
    # The fusion QA stays with the cloud: it is the only record of how the cloud was built.
    qa = las_path[:-4] + "_qa.json"
    if os.path.exists(qa):
        shutil.copy2(qa, os.path.join(out_dir, f"{stem}_qa.json"))
    return dict(output_las=out_las, output_trajectory_csv=out_trajectory, points=int(n), worst_pair_distance_change_m=worst)


def place_on_airborne(
    las_path: str,
    trajectory_path: str,
    tile_paths: Sequence[str],
    epoch: float,
    out_dir: str,
    *,
    level_plane: bool = False,
    check: bool = True,
) -> Dict[str, object]:
    """Fit, check and write the placement of one drive. Returns the report, also written as `<stem>_placement.json`."""
    t0 = time.time()
    rng = np.random.default_rng(0)
    stem = os.path.splitext(os.path.basename(las_path))[0]
    track = pd.read_csv(trajectory_path)[["x", "y"]].to_numpy()[::20]
    ref = AirborneReference(tile_paths, track, epoch)
    points = read_corridor([las_path], cKDTree(track), CLOUD_CORRIDOR_M)
    log(f"cloud points within {CLOUD_CORRIDOR_M:.0f} m of the route: {len(points):,}")
    fit = fit_placement(points, ref, track, lambda xy: np.zeros(len(xy), bool), rng, level_plane)
    T = fit["T"]
    log(f"placement: dE {fit['dE']:+.2f} dN {fit['dN']:+.2f} dZ {T[2, 3]:+.2f} m")
    report: Dict[str, object] = dict(
        source_las=las_path, source_trajectory=trajectory_path, tiles=list(tile_paths), epoch=epoch,
        airborne_to_cloud_frame_m=ref.shift.tolist(), level_plane=level_plane,
        transform_4x4=T.tolist(), shift_dE_m=fit["dE"], shift_dN_m=fit["dN"], shift_dZ_m=float(T[2, 3]),
        road_plane=fit["plane"], facade_cells=int(len(fit["facades"])),
    )
    if check:
        report["held_out"] = held_out_check(points, ref, track, rng, level_plane)
    report.update(write_placed(las_path, trajectory_path, T, out_dir, stem))
    report["elapsed_s"] = round(time.time() - t0, 1)
    with open(os.path.join(out_dir, f"{stem}_placement.json"), "w") as f:
        json.dump(report, f, indent=1)
    return report


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--las", required=True, help="cloud written by offline_fuse_lidar_gps.py")
    ap.add_argument("--trajectory", required=True, help="its _trajectory.csv")
    ap.add_argument("--tiles", required=True, nargs="+", help="airborne LAS/LAZ tiles, or folders holding them, NAD83(2011) UTM 14N / NAVD88 metres")
    ap.add_argument("--epoch", type=float, required=True, help="capture date as a decimal year, e.g. 2026.82")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--level-plane", action="store_true", help="also remove a tilt fitted to the airborne ground (not for gravity-levelled clouds)")
    ap.add_argument("--no-check", action="store_true", help="skip the leave-one-tile-out check")
    a = ap.parse_args(argv)
    tiles: List[str] = []
    for t in a.tiles:
        tiles += sorted(glob.glob(os.path.join(t, "*.la[sz]"))) if os.path.isdir(t) else [t]
    report = place_on_airborne(a.las, a.trajectory, tiles, a.epoch, a.output_dir, level_plane=a.level_plane, check=not a.no_check)
    log(json.dumps({k: v for k, v in report.items() if k not in ("transform_4x4", "held_out", "tiles")}, indent=1))


if __name__ == "__main__":
    main()
