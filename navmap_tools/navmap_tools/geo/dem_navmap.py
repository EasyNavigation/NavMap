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
A local height grid (dem_file.LocalDem) as a NavMap.

Two NavCels per cell with ground, on the nodes of the grid (x east, y north, z its height):
the cells without ground (cell_valid, e.g. buildings) and those steeper than max_slope_deg are
left out, as holes, so the NavMap's open edges are the borders of what the robot cannot be on.
Optionally only the part connected to a point (or the largest part) is kept: what the robot can
reach, without the roofs and the closed patios that are islands of the grid.
Vectorised: a grid of tens of millions of cells is converted in seconds.
"""

from array import array
import math
from typing import Optional, Tuple, Union

from navmap_ros_interfaces.msg import NavMap, NavMapSurface
import numpy as np

from .dem_file import LocalDem


def _floats(a) -> array:
    out = array('f')
    out.frombytes(np.ascontiguousarray(a, dtype=np.float32).tobytes())
    return out


def _uints(a) -> array:
    out = array('I')
    out.frombytes(np.ascontiguousarray(a, dtype=np.uint32).tobytes())
    return out


def ground_cells(dem: LocalDem, max_slope_deg: Optional[float] = None) -> np.ndarray:
    """(side - 1, side - 1) bool [j, i]: the cells a NavCel is made for."""
    nc = dem.side - 1
    ok = np.ones((nc, nc), dtype=bool) if dem.cell_valid is None else dem.cell_valid.astype(bool)
    if max_slope_deg is not None and max_slope_deg < 90.0:
        z = dem.z.astype(np.float64)
        r = dem.resolution
        # the two triangles of a cell: (a, b, c) and (a, c, d), a = (i, j), b = (i + 1, j),
        # c = (i + 1, j + 1), d = (i, j + 1); their gradients
        za, zb, zc, zd = z[:-1, :-1], z[:-1, 1:], z[1:, 1:], z[1:, :-1]
        g1 = np.hypot((zb - za) / r, (zc - zb) / r)
        g2 = np.hypot((zc - zd) / r, (zd - za) / r)
        ok &= np.maximum(g1, g2) <= math.tan(math.radians(max_slope_deg))
    return ok


def connected_parts(ok: np.ndarray) -> np.ndarray:
    """
    Label the connected parts of the cells of `ok` (2D bool): an int per cell, -1 where False.

    Cells that share an edge have the same label, and so do their NavCels, which share edges only
    across those (two cells that touch at a corner share a vertex).

    Union-find over the edges between neighbouring cells, every edge at once: each round hooks the
    larger root of the two ends of an edge onto the smaller one and compresses the paths, until
    both ends of every edge have the same root.
    """
    idx = np.full(ok.shape, -1, np.int64)
    cells = np.flatnonzero(ok)
    idx.flat[cells] = np.arange(len(cells))
    right = ok[:, :-1] & ok[:, 1:]
    up = ok[:-1, :] & ok[1:, :]
    a = np.concatenate([idx[:, :-1][right], idx[:-1, :][up]])
    b = np.concatenate([idx[:, 1:][right], idx[1:, :][up]])
    parent = np.arange(len(cells))
    while len(a):
        pa, pb = parent[a], parent[b]
        differ = pa != pb
        if not differ.any():
            break
        a, b, pa, pb = a[differ], b[differ], pa[differ], pb[differ]
        np.minimum.at(parent, np.maximum(pa, pb), np.minimum(pa, pb))
        while True:
            grand = parent[parent]
            if np.array_equal(grand, parent):
                break
            parent = grand
    labels = np.full(ok.shape, -1, np.int64)
    labels.flat[cells] = parent
    return labels


def keep_connected(dem: LocalDem, ok: np.ndarray,
                   keep: Union[str, Tuple[float, float]]) -> np.ndarray:
    """
    Return `ok` without the cells that are not connected to `keep`.

    `keep`: a point (x, y) in the local frame, whose nearest cell of `ok` is taken (it may fall on
    a hole), or 'largest', the part of most cells.
    """
    labels = connected_parts(ok)
    if not ok.any():
        return ok
    if isinstance(keep, str):
        if keep != 'largest':
            raise ValueError("keep must be 'largest' or (x, y), not %r" % keep)
        counts = np.bincount(labels[ok])
        return labels == int(np.argmax(counts))
    x, y = (float(v) for v in keep)
    cj, ci = np.nonzero(ok)
    half = dem.resolution / 2.0
    d2 = (dem.x(ci) + half - x) ** 2 + (dem.y(cj) + half - y) ** 2
    k = int(np.argmin(d2))
    return labels == labels[cj[k], ci[k]]


def navmap_from_dem(dem: LocalDem, max_slope_deg: Optional[float] = None,
                    frame: str = 'map',
                    keep: Optional[Union[str, Tuple[float, float]]] = None) -> NavMap:
    """
    Build the NavMap message of a LocalDem (see the module docstring).

    `keep`: None, every cell with ground; (x, y), only the part connected to that point (see
    keep_connected); 'largest', the largest part.
    """
    side = dem.side
    ok = ground_cells(dem, max_slope_deg)
    if keep is not None:
        ok = keep_connected(dem, ok, keep)
    cj, ci = np.nonzero(ok)
    a = cj * side + ci
    b = a + 1
    c = a + side + 1
    d = a + side
    # Two triangles per cell, interleaved as navmap_ros::from_occupancy_grid: (a, b, c), (a, c, d)
    v0 = np.stack([a, a], axis=1).ravel()
    v1 = np.stack([b, c], axis=1).ravel()
    v2 = np.stack([c, d], axis=1).ravel()
    used, inverse = np.unique(np.concatenate([v0, v1, v2]), return_inverse=True)
    n = len(v0)
    v0, v1, v2 = inverse[:n], inverse[n:2 * n], inverse[2 * n:]
    nj, ni = np.divmod(used, side)

    msg = NavMap()
    msg.header.frame_id = frame
    msg.positions_x = _floats(dem.x(ni))
    msg.positions_y = _floats(dem.y(nj))
    msg.positions_z = _floats(dem.z[nj, ni])
    if dem.rgb is not None:
        rgb = dem.rgb[nj, ni]
        msg.has_vertex_rgba = True
        msg.colors_r = rgb[:, 0].astype(np.uint8).tobytes()
        msg.colors_g = rgb[:, 1].astype(np.uint8).tobytes()
        msg.colors_b = rgb[:, 2].astype(np.uint8).tobytes()
        msg.colors_a = np.full(len(used), 255, np.uint8).tobytes()
    else:
        msg.has_vertex_rgba = False
    msg.navcels_v0 = _uints(v0)
    msg.navcels_v1 = _uints(v1)
    msg.navcels_v2 = _uints(v2)
    surface = NavMapSurface()
    surface.frame_id = frame
    surface.navcels = _uints(np.arange(n))
    msg.surfaces = [surface]
    return msg
