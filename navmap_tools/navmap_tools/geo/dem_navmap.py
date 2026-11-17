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
Vectorised: a grid of tens of millions of cells is converted in seconds.
"""

from array import array
import math
from typing import Optional

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


def navmap_from_dem(dem: LocalDem, max_slope_deg: Optional[float] = None,
                    frame: str = 'map') -> NavMap:
    """Build the NavMap message of a LocalDem (see the module docstring)."""
    side = dem.side
    ok = ground_cells(dem, max_slope_deg)
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
