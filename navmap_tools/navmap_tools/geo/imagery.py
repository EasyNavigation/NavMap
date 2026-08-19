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
ESRI World Imagery XYZ tile fetch + mosaic (no API key required).

Uses the same keyless ArcGIS Online "World_Imagery" service blendergis lists
under its SOURCES["ESRI"]["AERIAL"] entry (core/basemaps/servicesDefs.py).
Tiles are Web Mercator (EPSG:3857) 256x256 slippy-map tiles; navmap_tools
mosaics the tiles covering a bbox and exposes point sampling in (lon, lat) so
callers never need to reason about the Web Mercator tile grid directly.
"""

from concurrent.futures import as_completed, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import math
import sys
from typing import Tuple

import numpy as np

from PIL import Image

from .cache import DiskCache
from .net import get_to_file
from .projection import BBox

_URL_TEMPLATE = (
    'https://server.arcgisonline.com/ArcGIS/rest/services/'
    'World_Imagery/MapServer/tile/{z}/{y}/{x}'
)
_TILE_SIZE = 256
_TIMEOUT_S = 60
# Generous: at zoom 23 and ~40 deg latitude this covers roughly a 500-600 m
# square (tile count grows ~4x per zoom level and with latitude via
# native_resolution_m_per_px's cos(lat) term). Fetched concurrently (see
# _MAX_WORKERS below) so this stays practical despite the tile count.
_MAX_TILES = 30000
_MAX_WORKERS = 16
# The true ceiling for this service. Not every region has real imagery
# detail this fine (some areas re-serve upsampled, not sharper, tiles past
# ~19-20), but defaulting to the max means every run gets the best available
# detail rather than silently leaving resolution on the table; --zoom can
# still be lowered explicitly for faster runs over larger areas.
DEFAULT_ZOOM = 23
# Lowest zoom that automatic fallback (find_available_zoom, below) will try
# before giving up: below this, imagery is too coarse to be worth using at
# all for a robotics-scale terrain.
_MIN_FALLBACK_ZOOM = 10

# Esri serves this exact, byte-for-byte-identical tile (a flat grey square
# reading "Map data not yet available") for coordinates it has no imagery
# for at the requested zoom, instead of a 404 -- discovered when a real
# generated world showed that message repeated across every mesh cell after
# raising the default zoom to 23. It downloads and decodes as a perfectly
# valid JPEG, so nothing in the normal fetch path notices anything wrong;
# only comparing the fetched bytes against this known placeholder catches
# it. See easynav_gis_tool.md for the full diagnosis.
_PLACEHOLDER_MD5_HASHES = frozenset({
    'f27d9de7f80c13501f470595e327aa6d',
})


def _is_placeholder_tile(path) -> bool:
    """Return True if the tile at `path` is Esri's known "no data" placeholder image."""
    digest = hashlib.md5(open(path, 'rb').read()).hexdigest()
    return digest in _PLACEHOLDER_MD5_HASHES


def find_available_zoom(
    lon: float, lat: float, cache: DiskCache, max_zoom: int,
    min_zoom: int = _MIN_FALLBACK_ZOOM, force: bool = False,
) -> int:
    """
    Probe (lon, lat) from `max_zoom` downward; return the first zoom with real imagery.

    Requesting the highest zoom unconditionally is only useful where Esri
    actually has that much detail; many rural/remote areas don't, and
    silently mosaic-ing their "not yet available" placeholder tile as if it
    were real imagery is worse than just using a lower zoom that does have
    real coverage. Falls back to `min_zoom` if nothing better is found.
    """
    for zoom in range(max_zoom, min_zoom - 1, -1):
        tx, ty = (int(math.floor(v)) for v in lonlat_to_tile(lon, lat, zoom))
        path = _fetch_tile_path(zoom, tx, ty, cache, force=force)
        if not _is_placeholder_tile(path):
            return zoom
    return min_zoom


# Web Mercator ground resolution at zoom 0, equator: Earth's equatorial
# circumference / tile size in pixels (40075016.686 m / 256 px).
_EQUATOR_METERS_PER_PIXEL_AT_ZOOM0 = 156543.03392804097


def native_resolution_m_per_px(zoom: int, lat_deg: float) -> float:
    """
    Ground resolution (meters/pixel) of Web Mercator tiles at `zoom`/`lat_deg`.

    Used to size the baked output texture to actually use the detail being
    downloaded: fetching high-zoom tiles is wasted if the final texture PNG
    is baked at a fixed low pixel count regardless of `--zoom`.
    """
    return (
        _EQUATOR_METERS_PER_PIXEL_AT_ZOOM0
        * math.cos(math.radians(lat_deg))
        / (2.0 ** zoom)
    )


def lonlat_to_tile(lon: float, lat: float, zoom: int) -> Tuple[float, float]:
    """Convert (lon, lat) to fractional Web Mercator tile coordinates at `zoom`."""
    lat = min(max(lat, -85.05112878), 85.05112878)
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def _fetch_tile_path(z: int, x: int, y: int, cache: DiskCache, force: bool = False):
    url = _URL_TEMPLATE.format(z=z, y=y, x=x)
    key = f'imagery/esri/{z}/{x}/{y}'
    return cache.get_or_fetch(
        key, '.jpg', lambda dest: get_to_file(url, dest, _TIMEOUT_S), force=force)


@dataclass
class ImageryMosaic:
    """An RGB image mosaic covering a bbox, in Web Mercator (EPSG:3857) tile space."""

    image: np.ndarray  # shape (H, W, 3), uint8
    zoom: int
    tile_x0: int
    tile_y0: int

    def sample(self, lon: float, lat: float) -> Tuple[int, int, int]:
        """Bilinearly sample an RGB color at (lon, lat); clamps to the mosaic edge."""
        gx, gy = lonlat_to_tile(lon, lat, self.zoom)
        px = (gx - self.tile_x0) * _TILE_SIZE
        py = (gy - self.tile_y0) * _TILE_SIZE
        h, w, _ = self.image.shape
        px = min(max(px, 0.0), w - 1.0)
        py = min(max(py, 0.0), h - 1.0)
        x0, y0 = int(math.floor(px)), int(math.floor(py))
        x1, y1 = min(x0 + 1, w - 1), min(y0 + 1, h - 1)
        fx, fy = px - x0, py - y0
        c00 = self.image[y0, x0].astype(np.float32)
        c01 = self.image[y0, x1].astype(np.float32)
        c10 = self.image[y1, x0].astype(np.float32)
        c11 = self.image[y1, x1].astype(np.float32)
        top = c00 * (1 - fx) + c01 * fx
        bot = c10 * (1 - fx) + c11 * fx
        rgb = top * (1 - fy) + bot * fy
        r, g, b = (int(round(v)) for v in rgb)
        return r, g, b


def load_imagery_mosaic(
    bbox: BBox, cache: DiskCache, zoom: int = DEFAULT_ZOOM, force: bool = False,
    auto_fallback: bool = True,
) -> ImageryMosaic:
    """
    Fetch/mosaic (caching each tile) the imagery covering `bbox` at `zoom`.

    If `auto_fallback` (the default), first probes `bbox`'s center and steps
    `zoom` down until it finds real imagery instead of Esri's "not yet
    available" placeholder (see `find_available_zoom`) -- the mosaic is then
    fetched at that adjusted zoom, and `ImageryMosaic.zoom` reflects it.
    """
    if not 0 <= zoom <= 23:
        raise ValueError(f'zoom out of range [0, 23]: {zoom}')

    # Bound the request at the *requested* zoom before doing any network
    # activity at all (including the fallback probe below): an absurdly
    # large bbox should fail fast regardless of what fallback might later
    # pick, and tile count only shrinks if fallback lowers the zoom.
    def _tile_range(z):
        x0f, y0f = lonlat_to_tile(bbox.west, bbox.north, z)
        x1f, y1f = lonlat_to_tile(bbox.east, bbox.south, z)
        return (
            int(math.floor(x0f)), int(math.floor(y0f)),
            int(math.floor(x1f)), int(math.floor(y1f)),
        )

    tx0, ty0, tx1, ty1 = _tile_range(zoom)
    n_tiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
    if n_tiles > _MAX_TILES:
        raise ValueError(
            f'imagery request would need {n_tiles} tiles at zoom {zoom}; '
            'reduce --size or --zoom'
        )

    if auto_fallback:
        center_lon = (bbox.west + bbox.east) / 2.0
        center_lat = (bbox.south + bbox.north) / 2.0
        effective_zoom = find_available_zoom(center_lon, center_lat, cache, zoom, force=force)
        if effective_zoom != zoom:
            print(
                f'[navmap_gis_tool] no imagery at zoom {zoom} here; '
                f'using zoom {effective_zoom} instead', file=sys.stderr)
            zoom = effective_zoom
            tx0, ty0, tx1, ty1 = _tile_range(zoom)
            n_tiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)

    cols = (tx1 - tx0 + 1) * _TILE_SIZE
    rows = (ty1 - ty0 + 1) * _TILE_SIZE
    mosaic = np.zeros((rows, cols, 3), dtype=np.uint8)

    def _fetch_and_place(tx: int, ty: int) -> None:
        path = _fetch_tile_path(zoom, tx, ty, cache, force=force)
        with Image.open(path) as im:
            tile_rgb = np.asarray(im.convert('RGB'))
        row_off = (ty - ty0) * _TILE_SIZE
        col_off = (tx - tx0) * _TILE_SIZE
        # Disjoint slice per (tx, ty): safe to write concurrently, no lock needed.
        mosaic[row_off:row_off + _TILE_SIZE, col_off:col_off + _TILE_SIZE] = tile_rgb

    tile_coords = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]
    with ThreadPoolExecutor(max_workers=_MAX_WORKERS) as pool:
        futures = [pool.submit(_fetch_and_place, tx, ty) for tx, ty in tile_coords]
        done = 0
        for future in as_completed(futures):
            future.result()  # re-raises any exception from the worker
            done += 1
            if n_tiles >= 500 and done % max(1, n_tiles // 20) == 0:
                print(f'[navmap_gis_tool] imagery: {done}/{n_tiles} tiles', file=sys.stderr)

    return ImageryMosaic(image=mosaic, zoom=zoom, tile_x0=tx0, tile_y0=ty0)
