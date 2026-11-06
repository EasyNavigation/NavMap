# Copyright 2026 Intelligent Robotics Lab
#
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
IGN Spain terrain model (MDT, from PNOA-LiDAR) through its WCS service -- no API key.

5 m grid, orthometric heights, CC BY 4.0, Spain only (the coverage is ETRS89 /
UTM 30N for the whole peninsula, so the zone is forced for every point).
Roughly six times finer than Copernicus 30 m, which is why it is worth having
next to it for Spanish sites.

The area is requested in square blocks of at most `max_block_m` metres (one
WCS GetCoverage each, as an ArcGrid / ASC body) and every block is cached on
disk. `IgnDem.sample` has the interface of `geo.dem.DemGrid.sample` so both
can feed the same pipeline.
"""

from dataclasses import dataclass, field
import math
from typing import List, Tuple
from urllib.parse import quote

import numpy as np

from pyproj import Transformer

from .cache import DiskCache
from .net import get_to_file
from .projection import BBox

DEFAULT_ENDPOINT = 'https://servicios.idee.es/wcs-inspire/mdt'
DEFAULT_COVERAGE = 'Elevacion25830_5'
DEFAULT_CELL_M = 5.0
_MAX_BLOCK_M = 2000.0
_TIMEOUT_S = 120
_METERS_PER_DEGREE = 111320.0

_TO_UTM30 = Transformer.from_crs('EPSG:4326', 'EPSG:25830', always_xy=True)


@dataclass
class IgnBlock:
    """One ArcGrid block: row 0 is the northern one, values are at the cell centres."""

    x_min: float
    y_min: float
    cell: float
    values: np.ndarray  # (nrows, ncols) float32, NaN where the service has no data

    def sample_array(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Bilinear heights at UTM (x, y) arrays, and which of them this block could answer."""
        nrows, ncols = self.values.shape
        fx = (x - self.x_min) / self.cell - 0.5
        fy = ((self.y_min + nrows * self.cell) - y) / self.cell - 0.5
        inside = (fx >= -0.5) & (fy >= -0.5) & (fx <= ncols - 0.5) & (fy <= nrows - 0.5)
        cx = np.clip(fx, 0.0, ncols - 1.0)
        cy = np.clip(fy, 0.0, nrows - 1.0)
        i0 = np.minimum(cx.astype(int), max(ncols - 2, 0))
        j0 = np.minimum(cy.astype(int), max(nrows - 2, 0))
        i1 = np.minimum(i0 + 1, ncols - 1)
        j1 = np.minimum(j0 + 1, nrows - 1)
        tx, ty = cx - i0, cy - j0
        corners = (
            (self.values[j0, i0], (1 - tx) * (1 - ty)),
            (self.values[j0, i1], tx * (1 - ty)),
            (self.values[j1, i0], (1 - tx) * ty),
            (self.values[j1, i1], tx * ty),
        )
        acc = np.zeros(x.shape, dtype=np.float64)
        ok = inside.copy()
        for value, weight in corners:
            matters = weight >= 1e-9  # a cell without data only matters if it contributes
            ok &= ~(matters & np.isnan(value))
            acc += np.where(matters, weight * np.nan_to_num(value), 0.0)
        return acc, ok


def parse_asc(body: str) -> IgnBlock:
    """Parse the (possibly multipart) body of a GetCoverage response in ArcGrid format."""
    start = body.find('ncols')
    if start < 0:
        raise ValueError('not an ArcGrid body: no "ncols"')
    tokens = body[start:].split()
    header = {}
    pos = 0
    while pos + 1 < len(tokens):
        key = tokens[pos].lower()
        if key not in (
            'ncols', 'nrows', 'xllcorner', 'xllcenter', 'yllcorner', 'yllcenter',
            'cellsize', 'nodata_value',
        ):
            break
        header[key] = float(tokens[pos + 1])
        pos += 2
    ncols, nrows = int(header.get('ncols', 0)), int(header.get('nrows', 0))
    cell = header.get('cellsize', 0.0)
    has_x = 'xllcorner' in header or 'xllcenter' in header
    has_y = 'yllcorner' in header or 'yllcenter' in header
    if ncols < 1 or nrows < 1 or cell <= 0 or not (has_x and has_y):
        raise ValueError('bad ArcGrid header')
    x_min = header['xllcorner'] if 'xllcorner' in header else header['xllcenter'] - cell / 2
    y_min = header['yllcorner'] if 'yllcorner' in header else header['yllcenter'] - cell / 2
    nodata = header.get('nodata_value', -9999.0)
    count = ncols * nrows
    raw = np.array(tokens[pos:pos + count], dtype=np.float64)
    if raw.size != count:
        raise ValueError(f'ArcGrid body holds {raw.size} values, header says {count}')
    values = raw.astype(np.float32).reshape(nrows, ncols)
    values[np.abs(raw.reshape(nrows, ncols) - nodata) < 0.5] = np.nan
    return IgnBlock(x_min=x_min, y_min=y_min, cell=cell, values=values)


@dataclass
class IgnDem:
    """The IGN terrain model over an area, as a set of blocks."""

    blocks: List[IgnBlock] = field(default_factory=list)
    cell_m: float = DEFAULT_CELL_M

    @property
    def pixel_size_lat(self) -> float:
        """Spacing of the data in degrees of latitude (what the CLI compares with)."""
        return self.cell_m / _METERS_PER_DEGREE

    def sample_array(self, lon, lat) -> np.ndarray:
        """Heights (m above sea level) at arrays of lon, lat; it raises where there is no data."""
        lon = np.asarray(lon, dtype=np.float64)
        lat = np.asarray(lat, dtype=np.float64)
        x, y = _TO_UTM30.transform(lon, lat)
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        out = np.zeros(x.shape, dtype=np.float64)
        todo = np.ones(x.shape, dtype=bool)
        for block in self.blocks:
            if not todo.any():
                break
            value, ok = block.sample_array(x, y)
            take = todo & ok
            out[take] = value[take]
            todo &= ~take
        if todo.any():
            k = int(np.argmax(todo))
            raise RuntimeError(
                f'IGN MDT has no data at lat {lat.flat[k]:.6f}, lon {lon.flat[k]:.6f} '
                '(the service only covers Spain)')
        return out

    def sample(self, lon: float, lat: float) -> float:
        """Height at one point (the interface of `geo.dem.DemGrid.sample`)."""
        return float(self.sample_array(np.array([lon]), np.array([lat]))[0])


def _wcs_url(endpoint: str, coverage: str, x0: float, y0: float, x1: float, y1: float) -> str:
    return (
        f'{endpoint}?service=WCS&version=2.0.1&request=GetCoverage'
        f'&coverageId={quote(coverage, safe="")}'
        f'&subset={quote(f"x({x0:.0f},{x1:.0f})", safe="")}'
        f'&subset={quote(f"y({y0:.0f},{y1:.0f})", safe="")}'
        f'&format={quote("application/asc", safe="")}'
    )


def load_ign_dem(
    bbox: BBox, cache: DiskCache, force: bool = False,
    endpoint: str = DEFAULT_ENDPOINT, coverage: str = DEFAULT_COVERAGE,
    cell_m: float = DEFAULT_CELL_M, max_block_m: float = _MAX_BLOCK_M,
) -> IgnDem:
    """Load (downloading and caching each block as needed) the IGN MDT covering `bbox`."""
    xs, ys = _TO_UTM30.transform(
        [bbox.west, bbox.west, bbox.east, bbox.east],
        [bbox.south, bbox.north, bbox.south, bbox.north])
    margin = 4.0 * cell_m
    xmin = math.floor((min(xs) - margin) / cell_m) * cell_m
    ymin = math.floor((min(ys) - margin) / cell_m) * cell_m
    xmax = math.ceil((max(xs) + margin) / cell_m) * cell_m
    ymax = math.ceil((max(ys) + margin) / cell_m) * cell_m

    dem = IgnDem(cell_m=cell_m)
    y = ymin
    while y < ymax:
        x = xmin
        while x < xmax:
            x1, y1 = min(x + max_block_m, xmax), min(y + max_block_m, ymax)
            url = _wcs_url(endpoint, coverage, x, y, x1, y1)
            key = f'dem/ign/{coverage}/{x:.0f}_{y:.0f}_{x1:.0f}_{y1:.0f}'
            path = cache.get_or_fetch(
                key, '.asc', lambda dest, url=url: get_to_file(url, dest, _TIMEOUT_S),
                force=force)
            dem.blocks.append(parse_asc(path.read_text(errors='replace')))
            x += max_block_m
        y += max_block_m
    return dem
