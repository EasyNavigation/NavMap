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
The local height-grid file that bigmap_maps_manager reads (BigMap's `DemGrid`).

A square grid of heights in the local ENU frame of a GPS origin (x east, y north,
the origin an exact node), with z relative to a reference altitude, and
optionally an imagery colour per node. This module builds it from anything with
a `sample_array(lon, lat)` (or `sample(lon, lat)`) and writes the binary format
of BigMap's `DemGrid::save` (bigmap_maps_manager), so the C++ side loads it unchanged:

    char   magic[8] = "LMDEM006"
    int32  side              (odd)
    int32  source            (DemSource)
    double resolution, radius, origin_lat, origin_lon, reference_altitude,
           requested_reference (NaN: automatic), data_resolution
    float32 z[side * side]   (row-major, index = j * side + i)
    int32  imagery_zoom      (0: no colours)
    uint8  rgb[side * side * 3]   (only if imagery_zoom != 0)
    optional, to the end of the file (older files end above):
    char   tag[8] = "CELLS" and three zero bytes
    uint8  cell_valid[(side - 1) * (side - 1)]   (cell (i, j) between nodes i..i+1, j..j+1,
           row-major; 1 = ground, 0 = a hole: a building, a wall)

Native little endian, as written by the C++ side on x86 and arm.
"""

from dataclasses import dataclass, field
import enum
import math
from pathlib import Path
import struct
from typing import Callable, Optional

import numpy as np

from pyproj import Transformer

MAGIC = b'LMDEM006'
CELLS_TAG = b'CELLS\0\0\0'
_MAX_SIDE = 20000   # what DemGrid::load accepts


class DemSource(enum.IntEnum):
    """Where the heights come from (BigMap's DemGrid::Source)."""

    SYNTHETIC = 1
    IGN = 2
    MESH = 3
    TERRAIN_TILES = 4
    COPERNICUS = 5


_GEODETIC = 'EPSG:4979'   # lon, lat, height on WGS84
_ECEF = 'EPSG:4978'


class EnuProjection:
    """Local tangent plane (east, north, up) at a geodetic origin, on WGS84."""

    def __init__(self, lat0: float, lon0: float, alt0: float = 0.0):
        """Fix the origin (degrees, degrees, metres)."""
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self._to_ecef = Transformer.from_crs(_GEODETIC, _ECEF, always_xy=True)
        self._to_geodetic = Transformer.from_crs(_ECEF, _GEODETIC, always_xy=True)
        lat, lon = math.radians(lat0), math.radians(lon0)
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        # rows: east, north, up (in ECEF)
        self._rot = np.array([
            [-so, co, 0.0],
            [-sl * co, -sl * so, cl],
            [cl * co, cl * so, sl],
        ])
        self._origin = np.array(self._to_ecef.transform(lon0, lat0, alt0))

    def to_geodetic(self, east, north, up=0.0):
        """Arrays of east, north, up (m) -> (lat, lon, alt) arrays."""
        east = np.asarray(east, dtype=np.float64)
        north = np.asarray(north, dtype=np.float64)
        up = np.broadcast_to(np.asarray(up, dtype=np.float64), east.shape)
        enu = np.stack([east, north, up])
        ecef = self._origin[:, None] + self._rot.T @ enu.reshape(3, -1)
        lon, lat, alt = self._to_geodetic.transform(ecef[0], ecef[1], ecef[2])
        shape = east.shape
        return (np.asarray(lat).reshape(shape), np.asarray(lon).reshape(shape),
                np.asarray(alt).reshape(shape))

    def to_enu(self, lat, lon, alt=0.0):
        """Arrays of lat, lon (degrees) and alt (m) -> east, north, up arrays."""
        lat = np.asarray(lat, dtype=np.float64)
        lon = np.asarray(lon, dtype=np.float64)
        alt = np.broadcast_to(np.asarray(alt, dtype=np.float64), lat.shape)
        x, y, z = self._to_ecef.transform(lon, lat, alt)
        ecef = np.stack([np.ravel(x), np.ravel(y), np.ravel(z)])
        enu = self._rot @ (ecef - self._origin[:, None])
        return tuple(enu[k].reshape(lat.shape) for k in range(3))


@dataclass
class LocalDem:
    """A square height grid in the local ENU frame of a GPS origin."""

    side: int
    resolution: float
    data_resolution: float
    radius: float
    origin_lat: float
    origin_lon: float
    reference_altitude: float
    requested_reference: float          # NaN: automatic
    source: int
    z: np.ndarray                       # (side, side) float32, [j, i]
    imagery_zoom: int = 0
    rgb: Optional[np.ndarray] = field(default=None)   # (side, side, 3) uint8
    # (side - 1, side - 1) uint8 [j, i], 1 = ground; None: every cell has ground
    cell_valid: Optional[np.ndarray] = field(default=None)

    def x(self, i):
        """East coordinate (m) of node column(s) `i`."""
        return (np.asarray(i) - (self.side - 1) // 2) * self.resolution

    def y(self, j):
        """North coordinate (m) of node row(s) `j`."""
        return (np.asarray(j) - (self.side - 1) // 2) * self.resolution

    def height_at(self, x: float, y: float) -> float:
        """Return the height (m, from the reference altitude), bilinear, clamped to the grid."""
        c = (self.side - 1) / 2.0
        u = min(max(x / self.resolution + c, 0.0), self.side - 1.0)
        v = min(max(y / self.resolution + c, 0.0), self.side - 1.0)
        i = min(int(u), self.side - 2)
        j = min(int(v), self.side - 2)
        tu, tv = u - i, v - j
        z = self.z
        return float((1 - tv) * ((1 - tu) * z[j, i] + tu * z[j, i + 1]) +
                     tv * ((1 - tu) * z[j + 1, i] + tu * z[j + 1, i + 1]))

    def matches(
        self, lat: float, lon: float, radius: float, resolution: float, source: int,
        requested_reference: Optional[float] = None, data_resolution: float = 0.0,
    ) -> bool:
        """Whether this grid was made with this geometry and source (cache validity)."""
        if requested_reference is None:
            ref_ok = math.isnan(self.requested_reference)
        else:
            ref_ok = abs(requested_reference - self.requested_reference) < 1e-6
        mine = self.data_resolution if self.data_resolution > 0 else self.resolution
        want = data_resolution if data_resolution > 0 else resolution
        return (abs(lat - self.origin_lat) < 1e-9 and abs(lon - self.origin_lon) < 1e-9 and
                abs(radius - self.radius) < 1e-6 and abs(resolution - self.resolution) < 1e-9 and
                ref_ok and int(source) == self.source and abs(want - mine) < 1e-9)


def _catmull_rom(p0, p1, p2, p3, t):
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t)


def _bicubic(z: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Catmull-Rom sample of a (side, side) grid at fractional node coordinates, clamped."""
    side = z.shape[0]
    u = np.clip(u, 0.0, side - 1.0)
    v = np.clip(v, 0.0, side - 1.0)
    i = np.minimum(u.astype(int), side - 2)
    j = np.minimum(v.astype(int), side - 2)
    tu, tv = u - i, v - j

    def at(x, y):
        return z[np.clip(y, 0, side - 1), np.clip(x, 0, side - 1)].astype(np.float64)

    cols = []
    for k in range(4):
        y = j - 1 + k
        cols.append(_catmull_rom(at(i - 1, y), at(i, y), at(i + 1, y), at(i + 2, y), tu))
    return _catmull_rom(cols[0], cols[1], cols[2], cols[3], tv)


def _sample(sampler, lon: np.ndarray, lat: np.ndarray) -> np.ndarray:
    if hasattr(sampler, 'sample_array'):
        return np.asarray(sampler.sample_array(lon, lat), dtype=np.float64)
    return np.array(
        [sampler.sample(float(a), float(b)) for a, b in zip(lon.ravel(), lat.ravel())],
        dtype=np.float64).reshape(lon.shape)


def build_local_dem(
    sampler, lat0: float, lon0: float, radius: float, resolution: float, source: int,
    reference_altitude: Optional[float] = None, data_resolution: float = 0.0,
    progress: Optional[Callable[[int, int], None]] = None,
) -> LocalDem:
    """
    Sample `sampler` (heights above sea level at lon, lat) into a local grid.

    As DemGrid::build: `radius` is the half side; the grid has an odd number of nodes
    with the origin on a node. If `data_resolution` is above `resolution` the source is
    queried only every `data_resolution` metres (what its data supports) and bicubically
    interpolated to the finer grid. z is relative to `reference_altitude`, by default
    the height at the origin.
    """
    if radius <= 0.0 or resolution <= 0.0:
        raise ValueError('radius and resolution must be positive')
    dres = data_resolution if data_resolution > resolution else resolution
    half = int(math.ceil(radius / resolution))
    side = 2 * half + 1
    if side > _MAX_SIDE:
        raise ValueError(f'a grid of {side} x {side} nodes is over the limit of {_MAX_SIDE}')
    half_d = int(math.ceil(radius / dres))
    side_d = 2 * half_d + 1

    proj = EnuProjection(lat0, lon0)
    idx = (np.arange(side_d) - half_d) * dres
    east, north = np.meshgrid(idx, idx)
    lat, lon, _ = proj.to_geodetic(east, north, 0.0)
    elev = np.empty((side_d, side_d), dtype=np.float64)
    rows_per_chunk = max(1, 4096 // side_d)
    for j in range(0, side_d, rows_per_chunk):
        e = min(side_d, j + rows_per_chunk)
        elev[j:e] = _sample(sampler, lon[j:e], lat[j:e])
        if progress:
            progress(e * side_d, side_d * side_d)

    ref = float(elev[half_d, half_d]) if reference_altitude is None else reference_altitude
    if side_d == side and dres == resolution:
        z = (elev - ref).astype(np.float32)
    else:
        coords = (np.arange(side) - half) * resolution / dres + half_d
        u, v = np.meshgrid(coords, coords)
        z = (_bicubic(elev, u, v) - ref).astype(np.float32)
    return LocalDem(
        side=side, resolution=resolution, data_resolution=dres, radius=radius,
        origin_lat=lat0, origin_lon=lon0, reference_altitude=ref,
        requested_reference=float('nan') if reference_altitude is None else reference_altitude,
        source=int(source), z=z)


def add_imagery(dem: LocalDem, imagery, zoom: int) -> None:
    """
    Colour every node from the imagery; `zoom` is recorded in the file.

    `imagery` has `sample_array(lon, lat) -> (..., 3)` (fast) or `sample(lon, lat) -> (r, g, b)`.
    """
    proj = EnuProjection(dem.origin_lat, dem.origin_lon)
    idx = (np.arange(dem.side) - (dem.side - 1) // 2) * dem.resolution
    east, north = np.meshgrid(idx, idx)
    lat, lon, _ = proj.to_geodetic(east, north, 0.0)
    if hasattr(imagery, 'sample_array'):
        rgb = np.clip(np.rint(imagery.sample_array(lon, lat)), 0, 255).astype(np.uint8)
    else:
        rgb = np.empty((dem.side, dem.side, 3), dtype=np.uint8)
        for j in range(dem.side):
            for i in range(dem.side):
                rgb[j, i] = imagery.sample(float(lon[j, i]), float(lat[j, i]))
    dem.rgb = rgb
    dem.imagery_zoom = max(int(zoom), 1)   # 0 would mean "no colours" to the reader


def write_dem_file(dem: LocalDem, path) -> None:
    """Write `dem` in the format of BigMap's DemGrid::save (atomically)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    zoom = dem.imagery_zoom if dem.rgb is not None else 0
    header = (dem.resolution, dem.radius, dem.origin_lat, dem.origin_lon,
              dem.reference_altitude, dem.requested_reference, dem.data_resolution)
    with open(tmp, 'wb') as f:
        f.write(MAGIC)
        f.write(struct.pack('<ii7d', dem.side, dem.source, *header))
        f.write(np.ascontiguousarray(dem.z, dtype='<f4').tobytes())
        f.write(struct.pack('<i', zoom))
        if zoom != 0:
            f.write(np.ascontiguousarray(dem.rgb, dtype=np.uint8).tobytes())
        if dem.cell_valid is not None:
            cells = np.asarray(dem.cell_valid)
            if cells.shape != (dem.side - 1, dem.side - 1):
                raise ValueError(f'cell_valid is {cells.shape}, the grid has '
                                 f'{dem.side - 1} x {dem.side - 1} cells')
            f.write(CELLS_TAG)
            f.write((cells != 0).astype(np.uint8).tobytes())
    tmp.replace(path)


def read_dem_file(path) -> LocalDem:
    """Read a file written by this module or by DemGrid::save."""
    data = Path(path).read_bytes()
    if data[:8] != MAGIC:
        raise ValueError(f'{path} is not a DEM file (bad magic)')
    if len(data) < 72:
        raise ValueError(f'{path} is truncated')
    side, source, *header = struct.unpack_from('<ii7d', data, 8)
    # what DemGrid::load (the C++ reader) accepts: side 1 is no grid, the resolution divides
    if side < 3 or side % 2 == 0 or side > _MAX_SIDE:
        raise ValueError(f'bad side {side}')
    if len(data) < 72 + 4 * side * side + 4:
        raise ValueError(f'{path} is truncated')
    resolution, radius, lat0, lon0, reference, requested, data_res = header
    if not (resolution > 0.0 and math.isfinite(resolution)) or not all(
            math.isfinite(v) for v in (radius, lat0, lon0, reference)):
        raise ValueError(f'{path} has a bad header (resolution {resolution}, radius {radius}, '
                         f'origin {lat0}, {lon0}, reference {reference})')
    offset = 8 + 8 + 56
    n = side * side
    z = np.frombuffer(data, dtype='<f4', count=n, offset=offset).reshape(side, side).copy()
    offset += 4 * n
    (zoom,) = struct.unpack_from('<i', data, offset)
    offset += 4
    rgb = None
    if zoom != 0:
        rgb = np.frombuffer(data, dtype=np.uint8, count=3 * n, offset=offset).reshape(
            side, side, 3).copy()
        offset += 3 * n
    cells = None
    if len(data) > offset:   # the optional cells block (DemGrid::load refuses it broken too)
        nc = (side - 1) * (side - 1)
        if data[offset:offset + 8] != CELLS_TAG or len(data) < offset + 8 + nc:
            raise ValueError(f'{path} has a broken cells block')
        cells = (np.frombuffer(data, dtype=np.uint8, count=nc, offset=offset + 8)
                 .reshape(side - 1, side - 1) != 0).astype(np.uint8)
    return LocalDem(
        side=side, resolution=resolution, data_resolution=data_res, radius=radius,
        origin_lat=lat0, origin_lon=lon0, reference_altitude=reference,
        requested_reference=requested, source=source, z=z, imagery_zoom=zoom, rgb=rgb,
        cell_valid=cells)


class SyntheticDem:
    """Offline deterministic terrain (hills and roughness) around an origin, for demos."""

    pixel_size_lat = 0.0

    def __init__(self, lat0: float, lon0: float):
        """Place the terrain at the origin (the same one the DEM is centred on)."""
        self.lat0, self.lon0 = lat0, lon0

    @staticmethod
    def height(x, y):
        """Height (m) at local metres (x east, y north)."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        hill = 20.0 * np.exp(-((x - 80) ** 2 + (y + 60) ** 2) / (2 * 40.0 * 40.0))
        swell = 12.0 * np.sin(x / 60.0) * np.cos(y / 45.0)
        rough = 1.5 * np.sin(x / 7.0 + y / 5.0)
        return 650.0 + hill + swell + rough

    def _local(self, lon, lat):
        m_lat = 111132.0
        m_lon = 111320.0 * math.cos(math.radians(self.lat0))
        return (np.asarray(lon) - self.lon0) * m_lon, (np.asarray(lat) - self.lat0) * m_lat

    def sample_array(self, lon, lat) -> np.ndarray:
        """Heights at lon, lat arrays."""
        return self.height(*self._local(lon, lat))

    def sample(self, lon: float, lat: float) -> float:
        """Height at one point."""
        return float(self.sample_array(np.array([lon]), np.array([lat]))[0])


class SyntheticImagery:
    """Offline imagery: a red / blue chessboard of 25 m squares with white axes."""

    def __init__(self, lat0: float, lon0: float):
        """Place the chessboard at the origin."""
        self._dem = SyntheticDem(lat0, lon0)

    def sample_array(self, lon, lat) -> np.ndarray:
        """Colours (..., 3) at lon, lat arrays."""
        x, y = self._dem._local(lon, lat)
        even = ((np.floor(x / 25.0) + np.floor(y / 25.0)) % 2) == 0
        rgb = np.stack([np.where(even, 200, 60), np.full(x.shape, 60),
                        np.where(even, 60, 200)], axis=-1).astype(np.float64)
        axes = (np.abs(x) < 2.0) | (np.abs(y) < 2.0)
        rgb[axes] = 255.0
        return rgb

    def sample(self, lon: float, lat: float):
        """Colour at one point."""
        r, g, b = self.sample_array(np.array([lon]), np.array([lat]))[0]
        return int(r), int(g), int(b)
