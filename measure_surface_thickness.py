#!/usr/bin/env python3
"""Measure how sharply a point cloud resolves flat surfaces.

A road or a wall is physically a sheet. If every scan that saw it was placed
correctly, the points land in a layer a centimetre or so thick; if the poses
that placed them disagree, the same wall gets written several times slightly
apart and the layer thickens. That thickening is invisible to a trajectory-based
QA report, which never inspects the points, so it is measured separately here.

The measurement is per-neighbourhood. For each sampled point, take its `k`
nearest neighbours, fit the least-squares plane through them, and record the RMS
distance from those neighbours to that plane. That value is the square root of
the smallest eigenvalue of the neighbourhood's covariance matrix, and computing
it that way needs no explicit plane fit and no assumption about orientation -
which matters, because a cloud in a SLAM local frame has no horizontal ground to
align to.

Thickness is only meaningful where the neighbourhood is a surface, so
neighbourhoods are ranked by planarity, `(l1 - l0) / l2`, and only the most
planar fraction is kept. Inside a tree crown the number would describe the crown,
not the registration.

The same measurement expressed as a PDAL pipeline - `filters.eigenvalues` and
`filters.covariancefeatures`, both at knn 20 - is available via
`--emit-pdal-pipeline`, for reproducing these numbers on a machine that has
PDAL. PDAL is not required to run this script.

Points are processed in map tiles with an overlapping halo, so peak memory
depends on tile size rather than cloud size and neighbourhoods stay correct
across tile boundaries.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import laspy
import numpy as np
from scipy.spatial import cKDTree


# A neighbourhood spanning more than this is too sparse to describe a surface;
# at that spread the plane fit is reporting the gap between scan lines.
DEFAULT_MAX_NEIGHBOURHOOD_RADIUS_M = 1.0

# Surfaces are classified by how far the fitted plane normal tilts from vertical.
GROUND_NORMAL_Z_MIN = 0.9
WALL_NORMAL_Z_MAX = 0.3


@dataclass
class ThicknessStats:
    label: str
    count: int
    median_m: float
    p95_m: float
    p99_m: float
    mean_m: float

    def line(self) -> str:
        return (
            f"{self.label:<22} n={self.count:>9,}  "
            f"median {self.median_m * 100:>6.2f} cm  "
            f"p95 {self.p95_m * 100:>7.2f} cm  "
            f"p99 {self.p99_m * 100:>7.2f} cm"
        )


def _stats(label: str, values: np.ndarray) -> Optional[ThicknessStats]:
    if values.size == 0:
        return None
    return ThicknessStats(
        label=label,
        count=int(values.size),
        median_m=float(np.median(values)),
        p95_m=float(np.percentile(values, 95)),
        p99_m=float(np.percentile(values, 99)),
        mean_m=float(np.mean(values)),
    )


def pdal_pipeline(las_path: str, knn: int) -> str:
    """The equivalent PDAL pipeline, for reproducing this on a PDAL install."""
    pipeline = [
        las_path,
        {"type": "filters.covariancefeatures", "knn": knn, "feature_set": "Dimensionality"},
        {"type": "filters.eigenvalues", "knn": knn},
        {
            "type": "filters.expression",
            "expression": "Planarity > 0.75",
            "_comment": "Stand-in for keeping the most planar quartile.",
        },
        {
            "type": "filters.python",
            "_comment": "Thickness is sqrt(Eigenvalue0); report its median and p95.",
        },
    ]
    return json.dumps({"pipeline": pipeline}, indent=2)


def read_xy_z(path: Path, chunk_points: int) -> np.ndarray:
    """Load XYZ relative to the cloud's own corner, as float32.

    The shift is not an optimisation, it is required for the measurement to mean
    anything. These clouds are in UTM, where a northing is around 3.25e6; float32
    carries 24 bits of mantissa, so one representable step at that magnitude is
    roughly 0.4 m. Storing a northing directly as float32 therefore snaps every
    point onto a grid far coarser than the centimetre-scale spread being measured,
    which collapses neighbourhoods onto degenerate planes and reports a thickness
    of exactly zero.

    Subtracting the header minimum first puts every coordinate in the range 0 to a
    few thousand metres, where a float32 step is under a tenth of a millimetre.
    Thickness is invariant under translation, so the shift changes nothing else.
    """
    blocks: list[np.ndarray] = []
    with laspy.open(str(path)) as reader:
        origin = np.asarray(reader.header.mins, dtype=np.float64)
        for points in reader.chunk_iterator(chunk_points):
            block = np.column_stack(
                [
                    np.asarray(points.x, dtype=np.float64),
                    np.asarray(points.y, dtype=np.float64),
                    np.asarray(points.z, dtype=np.float64),
                ]
            )
            blocks.append((block - origin).astype(np.float32))
    if not blocks:
        raise ValueError(f"{path} holds no points.")
    return np.concatenate(blocks, axis=0)


def _tiles(
    xyz: np.ndarray, tile_size: float
) -> Iterator[tuple[np.ndarray, np.ndarray, tuple[float, float, float, float]]]:
    """Yield (indices in tile, indices in tile plus halo bounds, tile bounds).

    Points are bucketed into a regular XY grid once, then each occupied cell is
    served together with its eight neighbours. The caller trims that 3x3 block
    to the halo it wants.
    """
    min_x = float(xyz[:, 0].min())
    min_y = float(xyz[:, 1].min())
    ix = np.floor((xyz[:, 0] - min_x) / tile_size).astype(np.int64)
    iy = np.floor((xyz[:, 1] - min_y) / tile_size).astype(np.int64)
    n_y = int(iy.max()) + 1
    cell = ix * n_y + iy

    order = np.argsort(cell, kind="stable")
    sorted_cell = cell[order]
    unique_cells, starts = np.unique(sorted_cell, return_index=True)
    ends = np.append(starts[1:], sorted_cell.size)
    lookup = {int(c): (int(s), int(e)) for c, s, e in zip(unique_cells, starts, ends)}

    for c in unique_cells:
        c = int(c)
        cx, cy = divmod(c, n_y)
        start, end = lookup[c]
        inside = order[start:end]

        neighbour_blocks = [inside]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                key = (cx + dx) * n_y + (cy + dy)
                if key in lookup:
                    s, e = lookup[key]
                    neighbour_blocks.append(order[s:e])
        context = np.concatenate(neighbour_blocks)

        bounds = (
            min_x + cx * tile_size,
            min_x + (cx + 1) * tile_size,
            min_y + cy * tile_size,
            min_y + (cy + 1) * tile_size,
        )
        yield inside, context, bounds


def measure(
    xyz: np.ndarray,
    knn: int,
    tile_size: float,
    halo_m: float,
    max_query_points: int,
    max_neighbourhood_radius_m: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (thickness, planarity, |normal z|) for sampled neighbourhoods."""
    total = xyz.shape[0]
    sample_rate = min(1.0, max_query_points / float(total))

    thickness_all: list[np.ndarray] = []
    planarity_all: list[np.ndarray] = []
    normal_z_all: list[np.ndarray] = []

    for inside, context, bounds in _tiles(xyz, tile_size):
        if inside.size < knn:
            continue
        x0, x1, y0, y1 = bounds
        ctx_xyz = xyz[context]
        keep = (
            (ctx_xyz[:, 0] >= x0 - halo_m)
            & (ctx_xyz[:, 0] <= x1 + halo_m)
            & (ctx_xyz[:, 1] >= y0 - halo_m)
            & (ctx_xyz[:, 1] <= y1 + halo_m)
        )
        ctx_xyz = ctx_xyz[keep].astype(np.float64, copy=False)
        if ctx_xyz.shape[0] < knn:
            continue

        queries = inside
        if sample_rate < 1.0:
            take = max(1, int(round(inside.size * sample_rate)))
            queries = rng.choice(inside, size=take, replace=False)
        query_xyz = xyz[queries].astype(np.float64, copy=False)

        tree = cKDTree(ctx_xyz)
        distances, indices = tree.query(query_xyz, k=knn, workers=-1)

        # A neighbourhood that had to reach far to find k points is not a
        # surface sample; it is a sparse patch, and its plane fit is noise.
        compact = distances[:, -1] <= max_neighbourhood_radius_m
        if not np.any(compact):
            continue
        indices = indices[compact]

        neighbourhoods = ctx_xyz[indices]
        centred = neighbourhoods - neighbourhoods.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centred, centred) / knn
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.clip(eigenvalues, 0.0, None)

        l0, l1, l2 = eigenvalues[:, 0], eigenvalues[:, 1], eigenvalues[:, 2]
        usable = l2 > 0
        if not np.any(usable):
            continue

        thickness_all.append(np.sqrt(l0[usable]))
        planarity_all.append((l1[usable] - l0[usable]) / l2[usable])
        normal_z_all.append(np.abs(eigenvectors[usable, 2, 0]))

    if not thickness_all:
        raise ValueError(
            "No usable neighbourhoods were found. The cloud may be too sparse for "
            f"knn {knn} within {max_neighbourhood_radius_m} m."
        )
    return (
        np.concatenate(thickness_all),
        np.concatenate(planarity_all),
        np.concatenate(normal_z_all),
    )


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure surface thickness on flat surfaces in a LAS cloud.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--las", required=True, type=Path, help="LAS or LAZ to measure.")
    parser.add_argument("--knn", type=int, default=20, help="Neighbours per neighbourhood.")
    parser.add_argument(
        "--planar-quantile",
        type=float,
        default=0.75,
        help="Keep neighbourhoods above this planarity quantile (0.75 = top quarter).",
    )
    parser.add_argument(
        "--max-query-points",
        type=int,
        default=2_000_000,
        help="Neighbourhoods to sample. Sampling changes precision, not the answer.",
    )
    parser.add_argument("--tile-size", type=float, default=50.0, help="Tile edge in metres.")
    parser.add_argument(
        "--halo-m",
        type=float,
        default=1.0,
        help="Context kept around each tile so edge neighbourhoods stay correct.",
    )
    parser.add_argument(
        "--max-neighbourhood-radius-m",
        type=float,
        default=DEFAULT_MAX_NEIGHBOURHOOD_RADIUS_M,
        help="Reject neighbourhoods whose furthest neighbour is beyond this.",
    )
    parser.add_argument(
        "--chunk-points", type=int, default=2_000_000, help="LAS read block size."
    )
    parser.add_argument("--seed", type=int, default=0, help="Sampling seed.")
    parser.add_argument("--report-json", type=Path, help="Write the results here as JSON.")
    parser.add_argument(
        "--emit-pdal-pipeline",
        action="store_true",
        help="Print the equivalent PDAL pipeline and exit.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    if args.emit_pdal_pipeline:
        print(pdal_pipeline(str(args.las), args.knn))
        return 0

    if not args.las.is_file():
        print(f"error: no such LAS: {args.las}", file=sys.stderr)
        return 2

    xyz = read_xy_z(args.las, args.chunk_points)
    print(f"Loaded {xyz.shape[0]:,} points from {args.las.name}")

    rng = np.random.default_rng(args.seed)
    thickness, planarity, normal_z = measure(
        xyz,
        knn=args.knn,
        tile_size=args.tile_size,
        halo_m=args.halo_m,
        max_query_points=args.max_query_points,
        max_neighbourhood_radius_m=args.max_neighbourhood_radius_m,
        rng=rng,
    )
    print(f"Measured {thickness.size:,} neighbourhoods at knn {args.knn}")

    cutoff = float(np.quantile(planarity, args.planar_quantile))
    planar = planarity >= cutoff
    print(
        f"Planarity cutoff at the {args.planar_quantile:.2f} quantile: {cutoff:.4f} "
        f"({int(planar.sum()):,} neighbourhoods kept)"
    )
    print()

    results = []
    overall = _stats("all planar", thickness[planar])
    ground = _stats(
        "ground-like (|nz|>=0.9)", thickness[planar & (normal_z >= GROUND_NORMAL_Z_MIN)]
    )
    wall = _stats(
        "wall-like (|nz|<=0.3)", thickness[planar & (normal_z <= WALL_NORMAL_Z_MAX)]
    )
    for stat in (overall, ground, wall):
        if stat is not None:
            results.append(stat)
            print(stat.line())

    if args.report_json is not None:
        payload = {
            "las": str(args.las),
            "knn": args.knn,
            "planar_quantile": args.planar_quantile,
            "planarity_cutoff": cutoff,
            "neighbourhoods_measured": int(thickness.size),
            "results": [
                {
                    "label": r.label,
                    "count": r.count,
                    "median_m": r.median_m,
                    "p95_m": r.p95_m,
                    "p99_m": r.p99_m,
                    "mean_m": r.mean_m,
                }
                for r in results
            ],
        }
        args.report_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nWrote {args.report_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
