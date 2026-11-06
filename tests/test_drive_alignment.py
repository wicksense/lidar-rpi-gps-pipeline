import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from lidar_rpi_gps_pipeline.jobs.drive_alignment import Drive, _held_out, gicp_translation, solve  # noqa: E402
from lidar_rpi_gps_pipeline.jobs.drive_checks import image_shift  # noqa: E402


def _street(rng: np.random.Generator, length: float = 25.0) -> np.ndarray:
    """A north-south street: ground, a wall on each side, and poles every 6 m. Walls alone say nothing about north."""
    n = 6000
    ground = np.c_[rng.uniform(-8, 8, n), rng.uniform(0, length, n), rng.normal(0, 0.01, n)]
    walls = []
    for x in (-8.0, 8.0):
        walls.append(np.c_[np.full(n, x) + rng.normal(0, 0.01, n), rng.uniform(0, length, n), rng.uniform(0, 4, n)])
    poles = []
    for y in np.arange(2.0, length, 6.0):
        a = rng.uniform(0, 2 * np.pi, 400)
        poles.append(np.c_[-5 + 0.15 * np.cos(a), y + 0.15 * np.sin(a), rng.uniform(0, 3, 400)])
    return np.vstack([ground, *walls, *poles])


def test_gicp_recovers_a_shift_along_a_street() -> None:
    rng = np.random.default_rng(0)
    a = _street(rng)
    true = np.array([0.3, 1.2, 0.0])
    b = _street(np.random.default_rng(1)) - true

    t = gicp_translation(a, b)

    assert t is not None
    np.testing.assert_allclose(t[:2], true[:2], atol=0.1)


def test_image_shift_recovers_a_shift_of_fixed_things() -> None:
    rng = np.random.default_rng(0)
    scene = _street(rng)
    scene = scene[scene[:, 2] > 0.5]
    # Shrubs and trunks at irregular places, as on a real street; walls and evenly spaced poles alone leave the
    # direction along the street ambiguous, and the check then rightly declines to answer.
    blobs = [np.c_[rng.normal(cx, 0.5, 300), rng.normal(cy, 0.5, 300), rng.uniform(0.5, 3, 300)]
             for cx, cy in rng.uniform([-20, 2], [-2, 23], (12, 2))]
    scene = np.vstack([scene, *blobs])
    true = np.array([0.6, -1.4])

    t = image_shift(scene, scene - np.r_[true, 0.0], (-1, 0))

    assert t is not None
    np.testing.assert_allclose(t, true, atol=0.15)


class _Route(Drive):
    """A drive with only a route, for the solver."""

    def __init__(self, length: float, node_m: float = 50.0, loop: bool = False):
        self.name = "route"
        s = np.arange(0.0, length, 1.0)
        if loop:
            half = s[: len(s) // 2]
            y = np.r_[half, half[::-1]]
        else:
            y = s
        self.x, self.y, self.t = np.zeros(len(s)), y, s
        self.arc = s
        self.node_m = node_m
        self.n_nodes = int(self.arc[-1] // node_m) + 2


def _rows(a_arcs, true_shift, step=25.0):
    """Squares along both routes measuring a constant shift of the second drive onto the first."""
    rows = []
    for k, arc in enumerate(a_arcs):
        key = [0, int(arc // step) + 1000 * k]
        rows.append(dict(a=0, b=1, key=key, t=[true_shift[0], true_shift[1], 0.0], arc_a=arc, arc_b=arc))
    return rows


def test_solve_shares_a_constant_offset_between_two_drives() -> None:
    drives = [_Route(500.0), _Route(500.0)]
    rows = _rows(np.arange(10.0, 490.0, 10.0), (0.0, 2.0))

    nodes, _ = solve(drives, rows, smooth_m=0.5, prior_m=3.0)

    # shift_b - shift_a reproduces the measured offset everywhere the drives overlap.
    used = [r for r in rows if not _held_out(r["key"])]
    for r in used:
        sa = drives[0].shift_at(nodes[0], np.array([r["arc_a"]]))[0]
        sb = drives[1].shift_at(nodes[1], np.array([r["arc_b"]]))[0]
        np.testing.assert_allclose(sb - sa, r["t"][:2], atol=0.05)


def test_solve_gives_both_passes_of_a_drive_the_same_shift() -> None:
    # Drive 0 goes out and back along one street; drive 1 covers only the way out, offset by 1.5 m. Without the revisit
    # ties the way back would keep its old place and the drive would split in two.
    out_and_back = _Route(1000.0, loop=True)
    other = _Route(500.0)
    rows = _rows(np.arange(10.0, 490.0, 10.0), (0.0, 1.5))

    nodes, ties = solve([out_and_back, other], rows, smooth_m=0.5, prior_m=3.0)

    assert ties > 0
    way_out = out_and_back.shift_at(nodes[0], np.array([200.0]))[0]
    way_back = out_and_back.shift_at(nodes[0], np.array([799.0]))[0]
    np.testing.assert_allclose(way_out, way_back, atol=0.1)
