# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project Easy Navigation (EasyNav in short)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Copernicus DEM GLO-30 (30 m, public AWS Open Data, no API key) access.

Chosen over blendergis's default (OpenTopography SRTM, which requires a free
but registered API key) specifically so navmap_tools works without any
account setup. DEM tiles are 1x1 degree Cloud-Optimized GeoTIFFs at a fixed,
predictable S3 key; navmap_tools downloads whole covering tile(s) (not
windowed HTTP range reads) and caches them on disk -- simpler and more robust
than partial reads, and the one-time per-degree-tile download is cheap once
cached (see easynav_gis_tool.md for the full rationale).
"""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np

import tifffile

from .cache import DiskCache
from .net import get_to_file
from .projection import BBox

_BUCKET_URL = 'https://copernicus-dem-30m.s3.amazonaws.com'
_TIMEOUT_S = 120


def tile_key(lat_floor: int, lon_floor: int) -> str:
    """Return the Copernicus DEM S3 object-key prefix for a 1x1 degree tile."""
    ns = 'N' if lat_floor >= 0 else 'S'
    ew = 'E' if lon_floor >= 0 else 'W'
    return (
        f'Copernicus_DSM_COG_10_{ns}{abs(lat_floor):02d}_00_'
        f'{ew}{abs(lon_floor):03d}_00_DEM'
    )


def tile_url(lat_floor: int, lon_floor: int) -> str:
    """Return the full HTTPS URL of the DEM GeoTIFF for a 1x1 degree tile."""
    key = tile_key(lat_floor, lon_floor)
    return f'{_BUCKET_URL}/{key}/{key}.tif'


def _download(url: str, dest) -> None:
    if not get_to_file(url, dest, _TIMEOUT_S, allow_404=True):
        raise FileNotFoundError(f'no DEM tile at {url} (likely open ocean)')


def fetch_tile_path(lat_floor: int, lon_floor: int, cache: DiskCache, force: bool = False):
    """Download (or reuse from cache) the DEM tile covering (lat_floor, lon_floor)."""
    url = tile_url(lat_floor, lon_floor)
    key = f'dem/copernicus30/{tile_key(lat_floor, lon_floor)}'
    return cache.get_or_fetch(key, '.tif', lambda dest: _download(url, dest), force=force)


@dataclass
class DemGrid:
    """A rectangular elevation grid in plain WGS84 degrees, north edge at row 0."""

    elevation: np.ndarray
    west: float
    north: float
    pixel_size_lon: float
    pixel_size_lat: float

    def sample(self, lon: float, lat: float) -> float:
        """Bilinearly sample elevation (meters) at (lon, lat); clamps to the grid edge."""
        rows, cols = self.elevation.shape
        col = (lon - self.west) / self.pixel_size_lon
        row = (self.north - lat) / self.pixel_size_lat
        col = min(max(col, 0.0), cols - 1.0)
        row = min(max(row, 0.0), rows - 1.0)
        c0, r0 = int(math.floor(col)), int(math.floor(row))
        c1, r1 = min(c0 + 1, cols - 1), min(r0 + 1, rows - 1)
        fc, fr = col - c0, row - r0
        v00 = self.elevation[r0, c0]
        v01 = self.elevation[r0, c1]
        v10 = self.elevation[r1, c0]
        v11 = self.elevation[r1, c1]
        top = v00 * (1 - fc) + v01 * fc
        bot = v10 * (1 - fc) + v11 * fc
        return float(top * (1 - fr) + bot * fr)


def _read_tile_array(path) -> Tuple[np.ndarray, float, float, float, float]:
    tif = tifffile.TiffFile(str(path))
    meta = tif.geotiff_metadata
    arr = tif.pages[0].asarray().astype(np.float32)
    sx, sy, _ = meta['ModelPixelScale']
    _, _, _, ox, oy, _ = meta['ModelTiepoint']
    return arr, ox, oy, sx, sy


def load_dem_grid(bbox: BBox, cache: DiskCache, force: bool = False) -> DemGrid:
    """
    Load (downloading/caching as needed) a DEM grid covering `bbox`.

    Mosaics every 1x1 degree tile touching the bbox into a single grid so
    `DemGrid.sample()` never has to reason about tile boundaries. Tiles with
    no DEM coverage (open ocean) are skipped, contributing zero elevation.

    Raises RuntimeError if the bbox straddles a latitude band where the
    Copernicus grid's column count changes (this only happens very close to
    the poles) -- an edge case not worth silently mosaicking incorrectly.
    """
    lat0, lat1 = int(math.floor(bbox.south)), int(math.floor(bbox.north))
    lon0, lon1 = int(math.floor(bbox.west)), int(math.floor(bbox.east))

    tiles = {}
    for lat_floor in range(lat0, lat1 + 1):
        for lon_floor in range(lon0, lon1 + 1):
            try:
                path = fetch_tile_path(lat_floor, lon_floor, cache, force=force)
            except FileNotFoundError:
                continue
            tiles[(lat_floor, lon_floor)] = _read_tile_array(path)

    if not tiles:
        raise RuntimeError(f'no DEM coverage found for bbox {bbox}')

    shapes = {arr.shape for arr, _, _, _, _ in tiles.values()}
    if len(shapes) > 1:
        raise RuntimeError(
            'the requested area straddles a DEM tile latitude band with a '
            f'different grid resolution ({shapes}); reduce --size or move '
            '--center away from the tile boundary'
        )
    rows_per_tile, cols_per_tile = next(iter(shapes))
    _, _, _, sx, sy = next(iter(tiles.values()))

    mosaic_west = float(lon0)
    mosaic_north = float(lat1 + 1)
    total_rows = (lat1 - lat0 + 1) * rows_per_tile
    total_cols = (lon1 - lon0 + 1) * cols_per_tile
    mosaic = np.zeros((total_rows, total_cols), dtype=np.float32)

    for (lat_floor, lon_floor), (arr, _ox, _oy, _sx, _sy) in tiles.items():
        row_off = (lat1 - lat_floor) * rows_per_tile
        col_off = (lon_floor - lon0) * cols_per_tile
        mosaic[row_off:row_off + rows_per_tile, col_off:col_off + cols_per_tile] = arr

    return DemGrid(
        elevation=mosaic,
        west=mosaic_west,
        north=mosaic_north,
        pixel_size_lon=sx,
        pixel_size_lat=sy,
    )
