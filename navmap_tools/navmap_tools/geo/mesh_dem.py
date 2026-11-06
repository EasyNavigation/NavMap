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
A triangle mesh (STL) as a terrain model: the survey of a site you measured yourself.

The top surface of the mesh is rasterised to a regular height grid (vertical walls
collapse to their upper edge, overhangs keep the highest surface; cells the mesh does
not cover are filled from their neighbours, so a sample is defined everywhere). The
mesh is in the local frame of a GPS origin (x east, y north); the "elevation" of a
lon, lat is `world_elevation` plus the mesh height at its east / north offset.
`MeshDem.sample` has the interface of `geo.dem.DemGrid.sample`.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from .dem_file import EnuProjection

_METERS_PER_DEGREE = 111320.0


def load_stl_triangles(path) -> np.ndarray:
    """Triangles (N, 9) float32 of a binary or ASCII STL: x0 y0 z0 x1 y1 z1 x2 y2 z2."""
    raw = Path(path).read_bytes()
    if len(raw) >= 84:
        n = int(np.frombuffer(raw, dtype='<u4', count=1, offset=80)[0])
        if len(raw) == 84 + 50 * n:
            rec = np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('attr', '<u2')])
            return np.frombuffer(raw, dtype=rec, count=n, offset=84)['v'].astype(np.float32)
    words = raw.decode('ascii', errors='replace').split()
    values = []
    for k, word in enumerate(words):
        if word == 'vertex' and k + 3 < len(words):
            values.extend(float(w) for w in words[k + 1:k + 4])
    if len(values) < 9 or len(values) % 9 != 0:
        raise ValueError(f'not a valid STL: {path}')
    return np.array(values, dtype=np.float32).reshape(-1, 9)


@dataclass
class MeshHeightfield:
    """The top surface of a mesh as a height grid (row j, column i) at `cell` m."""

    x_min: float
    y_min: float
    cell: float
    z: np.ndarray
    coverage: float   # fraction of the bounding box covered by triangles (the rest was filled)

    @property
    def x_max(self) -> float:
        """East edge (m) of the grid."""
        return self.x_min + (self.z.shape[1] - 1) * self.cell

    @property
    def y_max(self) -> float:
        """North edge (m) of the grid."""
        return self.y_min + (self.z.shape[0] - 1) * self.cell

    @classmethod
    def from_triangles(cls, tris: np.ndarray, cell: float = 0.5) -> 'MeshHeightfield':
        """Rasterise (N, 9) triangles."""
        if len(tris) == 0:
            raise ValueError('empty mesh')
        if cell <= 0.0:
            raise ValueError('cell must be positive')
        tris = np.asarray(tris, dtype=np.float64)
        xs, ys = tris[:, 0::3], tris[:, 1::3]
        x_min, y_min = float(xs.min()), float(ys.min())
        nx = int(math.floor((float(xs.max()) - x_min) / cell)) + 1
        ny = int(math.floor((float(ys.max()) - y_min) / cell)) + 1
        z = np.full((ny, nx), np.nan, dtype=np.float32)

        for t in tris:
            x0, y0, z0, x1, y1, z1, x2, y2, z2 = t
            det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(det) < 1e-9:
                continue   # vertical wall: no area seen from above
            i0 = max(0, int(math.ceil((min(x0, x1, x2) - x_min) / cell - 1e-9)))
            i1 = min(nx - 1, int(math.floor((max(x0, x1, x2) - x_min) / cell + 1e-9)))
            j0 = max(0, int(math.ceil((min(y0, y1, y2) - y_min) / cell - 1e-9)))
            j1 = min(ny - 1, int(math.floor((max(y0, y1, y2) - y_min) / cell + 1e-9)))
            if i0 > i1 or j0 > j1:
                continue
            px = x_min + np.arange(i0, i1 + 1) * cell
            py = y_min + np.arange(j0, j1 + 1) * cell
            gx, gy = np.meshgrid(px, py)
            l0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / det
            l1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / det
            l2 = 1.0 - l0 - l1
            inside = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9)
            if not inside.any():
                continue
            height = (l0 * z0 + l1 * z1 + l2 * z2).astype(np.float32)
            dst = z[j0:j1 + 1, i0:i1 + 1]
            better = inside & (np.isnan(dst) | (height > dst))   # the highest wins
            dst[better] = height[better]

        covered = ~np.isnan(z)
        coverage = float(covered.mean())
        zmin = float(np.nanmin(z)) if covered.any() else 0.0
        z = _fill_from_neighbours(z)
        z[np.isnan(z)] = zmin
        return cls(x_min=x_min, y_min=y_min, cell=cell, z=z, coverage=coverage)

    @classmethod
    def load_stl(cls, path, cell: float = 0.5) -> 'MeshHeightfield':
        """Rasterise a binary or ASCII STL."""
        return cls.from_triangles(load_stl_triangles(path), cell)

    def sample_array(self, x, y) -> np.ndarray:
        """Bilinear heights at east / north arrays; clamped to the mesh's bounding box."""
        ny, nx = self.z.shape
        u = np.clip((np.asarray(x, dtype=np.float64) - self.x_min) / self.cell, 0.0, nx - 1.0)
        v = np.clip((np.asarray(y, dtype=np.float64) - self.y_min) / self.cell, 0.0, ny - 1.0)
        i = np.minimum(u.astype(int), max(nx - 2, 0))
        j = np.minimum(v.astype(int), max(ny - 2, 0))
        i1, j1 = np.minimum(i + 1, nx - 1), np.minimum(j + 1, ny - 1)
        tu, tv = u - i, v - j
        z = self.z.astype(np.float64)
        return ((1 - tv) * ((1 - tu) * z[j, i] + tu * z[j, i1]) +
                tv * ((1 - tu) * z[j1, i] + tu * z[j1, i1]))


def _fill_from_neighbours(z: np.ndarray) -> np.ndarray:
    """Fill NaN cells with the mean of their known 3x3 neighbours, one ring per pass."""
    z = z.copy()
    ny, nx = z.shape
    for _ in range(nx + ny):
        missing = np.isnan(z)
        if not missing.any():
            break
        padded = np.pad(z, 1, constant_values=np.nan)
        total = np.zeros_like(z, dtype=np.float64)
        count = np.zeros_like(z, dtype=np.int32)
        for dj in range(3):
            for di in range(3):
                window = padded[dj:dj + ny, di:di + nx]
                known = ~np.isnan(window)
                total += np.where(known, window, 0.0)
                count += known
        fill = missing & (count > 0)
        if not fill.any():
            break
        z[fill] = (total[fill] / count[fill]).astype(np.float32)
    return z


class MeshDem:
    """A mesh in the local frame of a GPS origin, as a terrain model over lon, lat."""

    def __init__(self, field: MeshHeightfield, lat0: float, lon0: float, world_elevation: float):
        """Place `field` at the origin; heights are `world_elevation` + the mesh height."""
        self.field = field
        self.world_elevation = world_elevation
        self._proj = EnuProjection(lat0, lon0)

    @property
    def pixel_size_lat(self) -> float:
        """Spacing of the data in degrees of latitude (what the CLI compares with)."""
        return self.field.cell / _METERS_PER_DEGREE

    def sample_array(self, lon, lat) -> np.ndarray:
        """Heights (m above sea level) at lon, lat arrays."""
        east, north, _ = self._proj.to_enu(np.asarray(lat), np.asarray(lon), 0.0)
        return self.world_elevation + self.field.sample_array(east, north)

    def sample(self, lon: float, lat: float) -> float:
        """Height at one point (the interface of `geo.dem.DemGrid.sample`)."""
        return float(self.sample_array(np.array([lon]), np.array([lat]))[0])


def hole_cells(triangles: np.ndarray, side: int, resolution: float) -> np.ndarray:
    """
    Mark the cells of a DEM grid that the xy projection of any triangle touches.

    Returns (side - 1, side - 1) bool, [j, i].

    The grid is the one of `dem_file.LocalDem` (nodes at (i - (side - 1) / 2) * resolution, cell
    (i, j) between nodes i..i+1 and j..j+1, the mesh in the same local frame). A cell is touched
    if the triangle and the cell overlap or touch (separating axes: x, y and the normals of the
    triangle's edges), the rule of a collision map: a cell is blocked if any part of an obstacle is
    in it. A vertical wall projects to a segment and still marks the cells it crosses.
    """
    nc = side - 1
    out = np.zeros((nc, nc), dtype=bool)
    if len(triangles) == 0:
        return out
    half = (side - 1) // 2
    tri = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)[:, :, :2] / resolution + half
    lo = np.floor(tri.min(axis=1)).astype(np.int64)
    hi = np.floor(tri.max(axis=1)).astype(np.int64)
    keep = (hi[:, 0] >= 0) & (hi[:, 1] >= 0) & (lo[:, 0] < nc) & (lo[:, 1] < nc)
    for t, (i0, j0), (i1, j1) in zip(tri[keep], lo[keep], hi[keep]):
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, nc - 1), min(j1, nc - 1)
        cj, ci = np.mgrid[j0:j1 + 1, i0:i1 + 1]
        hit = np.ones(ci.shape, dtype=bool)
        # x and y axes: the bounding boxes overlap by construction; the edge normals
        for k in range(3):
            e = t[(k + 1) % 3] - t[k]
            n = np.array([-e[1], e[0]])
            if not n.any():
                continue
            proj = t @ n
            # the cell's corners projected on n: centre +- half extent
            c = (ci + 0.5) * n[0] + (cj + 0.5) * n[1]
            r = 0.5 * (abs(n[0]) + abs(n[1]))
            hit &= (c - r <= proj.max()) & (c + r >= proj.min())
        out[cj[hit], ci[hit]] = True
    return out
