import os
import sys
import zipfile

import laspy
import numpy as np
from pyproj import CRS, Transformer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from lidar_rpi_gps_pipeline.jobs.airborne_tiles import prepare_tiles  # noqa: E402

US_FT = 1200.0 / 3937.0


def stratmap_like_tile(path: str) -> tuple:
    """A tile as StratMap delivers it: Texas South Central, US survey feet, mixed classes."""
    to_sp = Transformer.from_crs(6343, 6588, always_xy=True)
    e, n = np.array([549000.0, 549010.0, 549020.0]), np.array([3255000.0, 3255010.0, 3255020.0])
    x, y = to_sp.transform(e, n)
    h = laspy.LasHeader(point_format=6, version="1.4")
    h.offsets, h.scales = [float(x.min()), float(y.min()), 0.0], [0.01, 0.01, 0.01]
    h.add_crs(CRS.from_epsg(6588))
    las = laspy.LasData(h)
    las.x, las.y, las.z = x, y, np.array([600.0, 650.0, 700.0])  # feet
    las.classification = np.array([2, 5, 6])
    las.write(path)
    return e, n


def test_feet_tile_becomes_utm_metres_with_ground_and_buildings_only(tmp_path) -> None:
    src = str(tmp_path / "stratmap21-50cm_test.laz")
    e, n = stratmap_like_tile(src)

    out = prepare_tiles([src], str(tmp_path / "out"))

    las = laspy.read(out[0])
    assert os.path.basename(out[0]) == "stratmap21-50cm_test_utm14_m.laz"
    assert las.classification.tolist() == [2, 6]
    assert np.allclose(las.x, e[[0, 2]], atol=0.02) and np.allclose(las.y, n[[0, 2]], atol=0.02)
    assert np.allclose(las.z, np.array([600.0, 700.0]) * US_FT, atol=0.01)
    assert las.header.parse_crs().to_epsg() == 6343


def test_tiles_are_read_from_the_downloaded_zip(tmp_path) -> None:
    src = str(tmp_path / "stratmap21-50cm_zipped.laz")
    stratmap_like_tile(src)
    with zipfile.ZipFile(tmp_path / "quad_lpc.zip", "w") as z:
        z.write(src, "quad/stratmap21-50cm_zipped.laz")
    os.remove(src)

    out = prepare_tiles([str(tmp_path / "quad_lpc.zip")], str(tmp_path / "out"))

    assert [os.path.basename(p) for p in out] == ["stratmap21-50cm_zipped_utm14_m.laz"]
    assert sorted(os.listdir(tmp_path / "out")) == ["stratmap21-50cm_zipped_utm14_m.laz"]
