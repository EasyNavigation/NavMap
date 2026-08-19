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
ASCII PCD (x y z) writer, plus a parallel per-point RGB sidecar.

The PCD itself matches the header shape of maps/*.pcd in
src/urjc-excavation-world (VERSION 0.7, FIELDS x y z, TYPE F F F, DATA ascii)
so the same `pointcloud_to_navmap` C++ tool that already works for that world
can also consume navmap_tools' output.

Colors are written as a *separate* same-row-order CSV rather than a packed
`rgb` PCD field: PCL's XYZRGB PCD field is a float that is actually a
bit-reinterpreted packed uint32, an easy thing to get subtly wrong from
Python without a byte-for-byte match to PCL's own (un)packing. A plain CSV of
"r,g,b" per line, read back in C++ as three ordinary uint8s, has no such
pitfall and is trivial to test on both ends.
"""

from pathlib import Path

import numpy as np


def write_pcd_xyz(path, points, width=None, height=None) -> int:
    """
    Write an ASCII XYZ PCD file from an (N, 3) array-like; returns N.

    `width`/`height` mark the cloud as *organized* (PCL's convention for a
    grid-shaped point set, row-major: row j/col i at index j*width+i) rather
    than the default unorganized WIDTH=N/HEIGHT=1. navmap_tools always
    builds `points` this way (see terrain.TerrainGrid), and passing the true
    shape here lets `pointcloud_to_navmap` use the dedicated, gap-free
    `navmap_ros::from_regular_grid` mesher instead of the generic
    neighbor-search one -- see easynav_gis_tool.md for why the generic
    mesher could leave holes on an evenly-sampled, fully navigable grid.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        pts = pts.reshape(0, 3)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f'points must be an Nx3 array-like, got shape {pts.shape}')
    if not np.isfinite(pts).all():
        raise ValueError('points must not contain NaN/Inf values')

    n = pts.shape[0]
    if (width is None) != (height is None):
        raise ValueError('width and height must be given together')
    if width is not None and width * height != n:
        raise ValueError(f'width*height ({width}*{height}) must equal the point count ({n})')
    out_width, out_height = (width, height) if width is not None else (n, 1)

    with open(Path(path), 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {out_width}\n')
        f.write(f'HEIGHT {out_height}\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {n}\n')
        f.write('DATA ascii\n')
        for x, y, z in pts:
            f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')
    return n


def write_colors_csv(path, colors) -> int:
    """
    Write an (N, 3) uint8 array-like of RGB colors as a plain "r,g,b" CSV.

    Row order must match the PCD written by `write_pcd_xyz` for the same
    point set: `pointcloud_to_navmap` pairs them up positionally.
    """
    cols = np.asarray(colors)
    if cols.size == 0:
        cols = cols.reshape(0, 3)
    if cols.ndim != 2 or cols.shape[1] != 3:
        raise ValueError(f'colors must be an Nx3 array-like, got shape {cols.shape}')
    if ((cols < 0) | (cols > 255)).any():
        raise ValueError('colors must be in [0, 255]')

    n = cols.shape[0]
    with open(Path(path), 'w') as f:
        for r, g, b in cols:
            f.write(f'{int(r)},{int(g)},{int(b)}\n')
    return n
