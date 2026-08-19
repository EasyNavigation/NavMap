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
PNOA (IGN Spain national aerial orthophoto) WMS access -- no API key required.

Esri World Imagery (geo/imagery.py) has patchy real coverage in rural areas
(see find_available_zoom's docstring and easynav_gis_tool.md for a concrete
case where it fell back to a placeholder tile). PNOA is Spain's own national
aerial survey, flown specifically to cover the whole country including rural
land, at up to 25 cm/px native resolution -- for areas within Spain it is
usually the sharper, more complete option. Published CC BY 4.0, no fees, no
API key (`Fees`/`AccessConstraints` in the service's own GetCapabilities).

Unlike Esri's pre-tiled XYZ pyramid, this is a WMS service: each request asks
for an arbitrary (bbox, pixel size) image directly, capped by the server at
4096x4096 px per call (its own `MaxWidth`/`MaxHeight`), so a large area is
split into a grid of such requests and mosaicked, mirroring geo/imagery.py's
concurrent-fetch approach.
"""

from concurrent.futures import as_completed, ThreadPoolExecutor
from dataclasses import dataclass
import math
import sys
from typing import Tuple

import numpy as np

from PIL import Image

from pyproj import Transformer

from .cache import DiskCache
from .net import get_to_file
from .projection import BBox

_WMS_URL = 'https://www.ign.es/wms-inspire/pnoa-ma'
# The actual raster imagery layer. OI.MosaicElement (also offered by this
# service) looks like imagery in its GetCapabilities title ("Mosaico") but
# is a vector index of tile footprints + acquisition-date text labels, not
# pixels -- confirmed by requesting it and getting a mostly-blank image with
# a magenta date label instead of a photo, per its own style Abstract
# ("El atributo fecha ... se representa mediante una etiqueta de texto").
_LAYER = 'OI.OrthoimageCoverage'
_TIMEOUT_S = 60
# The server's own MaxWidth/MaxHeight (GetCapabilities); requesting more
# than this in one call fails.
_MAX_WMS_PX = 4096
_MAX_WORKERS = 16
# Bounds total mosaic size the same way geo/imagery.py bounds tile count:
# fails fast with a clear message instead of an impractically large fetch.
_MAX_TOTAL_PIXELS = 16384
# PNOA's stated native resolution is "0.25 m or 0.50 m depending on the
# zone" -- requesting the finer of the two is safe even where only 0.50 m
# is really available (the server just resamples up, still valid imagery).
DEFAULT_RESOLUTION_M = 0.25

_TO_WEB_MERCATOR = Transformer.from_crs('EPSG:4326', 'EPSG:3857', always_xy=True)


@dataclass
class PnoaMosaic:
    """An RGB image mosaic covering a bbox, in EPSG:3857 (Web Mercator) meters."""

    image: np.ndarray  # shape (H, W, 3), uint8
    origin_x: float  # EPSG:3857 meters, west edge
    origin_y: float  # EPSG:3857 meters, north edge
    pixel_size_m: float

    def sample(self, lon: float, lat: float) -> Tuple[int, int, int]:
        """Bilinearly sample an RGB color at (lon, lat); clamps to the mosaic edge."""
        mx, my = _TO_WEB_MERCATOR.transform(lon, lat)
        px = (mx - self.origin_x) / self.pixel_size_m
        py = (self.origin_y - my) / self.pixel_size_m
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


def _fetch_chunk(
    x0: float, y0: float, x1: float, y1: float, width_px: int, height_px: int,
    cache: DiskCache, force: bool = False,
):
    url = (
        f'{_WMS_URL}?SERVICE=WMS&REQUEST=GetMap&VERSION=1.3.0&LAYERS={_LAYER}'
        f'&STYLES=&CRS=EPSG:3857&BBOX={x0:.3f},{y0:.3f},{x1:.3f},{y1:.3f}'
        f'&WIDTH={width_px}&HEIGHT={height_px}&FORMAT=image/jpeg'
    )
    key = f'imagery/pnoa/{x0:.3f}_{y0:.3f}_{x1:.3f}_{y1:.3f}_{width_px}x{height_px}'
    return cache.get_or_fetch(
        key, '.jpg', lambda dest: get_to_file(url, dest, _TIMEOUT_S), force=force)


def load_pnoa_mosaic(
    bbox: BBox, cache: DiskCache, resolution_m_per_px: float = DEFAULT_RESOLUTION_M,
    force: bool = False,
) -> PnoaMosaic:
    """Fetch/mosaic (caching each WMS chunk) PNOA imagery covering `bbox`."""
    if resolution_m_per_px <= 0:
        raise ValueError(f'resolution_m_per_px must be positive: {resolution_m_per_px}')

    x0, y0 = _TO_WEB_MERCATOR.transform(bbox.west, bbox.south)
    x1, y1 = _TO_WEB_MERCATOR.transform(bbox.east, bbox.north)

    total_w = max(1, round((x1 - x0) / resolution_m_per_px))
    total_h = max(1, round((y1 - y0) / resolution_m_per_px))
    if total_w > _MAX_TOTAL_PIXELS or total_h > _MAX_TOTAL_PIXELS:
        raise ValueError(
            f'PNOA request would need a {total_w}x{total_h} px mosaic; '
            'reduce --size or use a coarser resolution'
        )

    n_cols = math.ceil(total_w / _MAX_WMS_PX)
    n_rows = math.ceil(total_h / _MAX_WMS_PX)
    chunk_w = math.ceil(total_w / n_cols)
    chunk_h = math.ceil(total_h / n_rows)
    chunk_size_x_m = chunk_w * resolution_m_per_px
    chunk_size_y_m = chunk_h * resolution_m_per_px

    mosaic = np.zeros((total_h, total_w, 3), dtype=np.uint8)
    n_chunks = n_cols * n_rows

    def _fetch_and_place(row: int, col: int) -> None:
        cx0 = x0 + col * chunk_size_x_m
        cx1 = min(x0 + (col + 1) * chunk_size_x_m, x1)
        # Row 0 is the northmost strip (top of the mosaic image).
        cy1 = y1 - row * chunk_size_y_m
        cy0 = max(y1 - (row + 1) * chunk_size_y_m, y0)
        w_px = max(1, round((cx1 - cx0) / resolution_m_per_px))
        h_px = max(1, round((cy1 - cy0) / resolution_m_per_px))
        path = _fetch_chunk(cx0, cy0, cx1, cy1, w_px, h_px, cache, force=force)
        with Image.open(path) as im:
            chunk_rgb = np.asarray(im.convert('RGB'))
        row_off = round((y1 - cy1) / resolution_m_per_px)
        col_off = round((cx0 - x0) / resolution_m_per_px)
        h, w, _ = chunk_rgb.shape
        # Disjoint slice per (row, col): safe to write concurrently, no lock needed.
        mosaic[row_off:row_off + h, col_off:col_off + w] = chunk_rgb[
            :min(h, total_h - row_off), :min(w, total_w - col_off)]

    coords = [(row, col) for row in range(n_rows) for col in range(n_cols)]
    with ThreadPoolExecutor(max_workers=_MAX_WORKERS) as pool:
        futures = [pool.submit(_fetch_and_place, row, col) for row, col in coords]
        done = 0
        for future in as_completed(futures):
            future.result()  # re-raises any exception from the worker
            done += 1
            if n_chunks >= 20 and done % max(1, n_chunks // 20) == 0:
                print(f'[navmap_gis_tool] imagery: {done}/{n_chunks} chunks', file=sys.stderr)

    return PnoaMosaic(image=mosaic, origin_x=x0, origin_y=y1, pixel_size_m=resolution_m_per_px)
