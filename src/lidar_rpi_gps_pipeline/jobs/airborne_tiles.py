#!/usr/bin/env python3
"""
Prepare downloaded airborne lidar tiles for `place_on_airborne`: UTM 14N metres, ground and buildings only.

Placement reads NAD83(2011) / UTM zone 14N in metres (EPSG:6343) with NAVD88 heights in metres,
and uses only ground (class 2) and building (class 6) points. Texas StratMap tiles come in
NAD83(2011) / Texas South Central in US survey feet (EPSG:6588), heights in US survey feet.
This converts them:

- horizontal: from the tile's own CRS (read from its header, or `--from-epsg`) to EPSG:6343;
- height: from the CRS's vertical unit (or, without a vertical CRS, the horizontal unit) to
  metres; the vertical datum (NAVD88) is kept;
- classes 2 and 6 kept, everything else dropped.

Inputs are LAS/LAZ files, folders holding them, or the zips as downloaded from the TxGIO
DataHub (their tiles are read straight from the zip). Each tile is written as
`<tile>_utm14_m.laz` to the output folder; a tile already there is skipped.
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import tempfile
import zipfile
from typing import Iterator, List, Optional, Sequence, Tuple

import laspy
import numpy as np
from pyproj import CRS, Transformer

TARGET_EPSG = 6343
KEEP_CLASSES = (2, 6)


def log(message: str) -> None:
    print(message, flush=True)


def tiles_in(inputs: Sequence[str], scratch: str) -> Iterator[Tuple[str, str]]:
    """(tile name, readable path) for every LAS/LAZ in the inputs; zip members are extracted one at a time."""
    for item in inputs:
        if os.path.isdir(item):
            for p in sorted(glob.glob(os.path.join(item, "*.la[sz]")) + glob.glob(os.path.join(item, "*.zip"))):
                yield from tiles_in([p], scratch)
        elif item.lower().endswith(".zip"):
            with zipfile.ZipFile(item) as z:
                for member in sorted(m for m in z.namelist() if m.lower().endswith((".las", ".laz"))):
                    path = z.extract(member, scratch)
                    yield os.path.splitext(os.path.basename(member))[0], path
                    os.remove(path)
        else:
            yield os.path.splitext(os.path.basename(item))[0], item


def source_crs(header: laspy.LasHeader, from_epsg: Optional[int]) -> Tuple[CRS, float]:
    """Horizontal CRS of the tile, and the factor that turns its heights into metres."""
    crs = CRS.from_epsg(from_epsg) if from_epsg else header.parse_crs()
    if crs is None:
        raise ValueError("The tile carries no CRS; give --from-epsg.")
    horizontal, vertical = crs, None
    if crs.is_compound:
        horizontal, vertical = crs.sub_crs_list[0], crs.sub_crs_list[1]
    unit_crs = vertical if vertical is not None else horizontal
    return horizontal, float(unit_crs.axis_info[-1].unit_conversion_factor)


def convert_tile(path: str, out_path: str, from_epsg: Optional[int] = None) -> int:
    las = laspy.read(path)
    horizontal, to_m = source_crs(las.header, from_epsg)
    keep = np.isin(np.asarray(las.classification), KEEP_CLASSES)
    to_utm = Transformer.from_crs(horizontal, CRS.from_epsg(TARGET_EPSG), always_xy=True)
    x, y = to_utm.transform(np.asarray(las.x)[keep], np.asarray(las.y)[keep])
    z = np.asarray(las.z)[keep] * to_m
    header = laspy.LasHeader(point_format=6, version="1.4")
    if keep.any():
        header.offsets = [float(np.floor(x.min())), float(np.floor(y.min())), float(np.floor(z.min()))]
    header.scales = [0.01, 0.01, 0.01]
    header.add_crs(CRS.from_epsg(TARGET_EPSG))
    out = laspy.LasData(header)
    out.x, out.y, out.z = x, y, z
    out.classification = np.asarray(las.classification)[keep]
    tmp = out_path + ".part"
    out.write(tmp, do_compress=True)
    os.replace(tmp, out_path)
    return int(keep.sum())


def prepare_tiles(inputs: Sequence[str], out_dir: str, from_epsg: Optional[int] = None) -> List[str]:
    os.makedirs(out_dir, exist_ok=True)
    written = []
    scratch = tempfile.mkdtemp(dir=out_dir, prefix="_unzip_")
    try:
        for name, path in tiles_in(inputs, scratch):
            out_path = os.path.join(out_dir, f"{name}_utm14_m.laz")
            if not os.path.exists(out_path):
                n = convert_tile(path, out_path, from_epsg)
                log(f"{name}: {n:,} ground and building points")
            else:
                log(f"{name}: done already")
            written.append(out_path)
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    if not written:
        raise ValueError("No LAS/LAZ tiles found in the inputs.")
    return written


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(), formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, nargs="+", help="tiles, folders, or the zips as downloaded")
    ap.add_argument("--output-dir", required=True, help="the folder to give place_on_airborne / process_drives as --tiles")
    ap.add_argument("--from-epsg", type=int, help="the tiles' CRS, if their headers do not carry one (StratMap: 6588)")
    a = ap.parse_args(argv)
    prepare_tiles(a.input, a.output_dir, a.from_epsg)


if __name__ == "__main__":
    main()
