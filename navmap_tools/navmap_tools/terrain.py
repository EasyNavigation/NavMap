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
Combine a DEM grid and an imagery mosaic into local-ENU samples.

Both --gazebo and --navmap consume the exact same `TerrainGrid` for a given
(center, size) query -- that shared source is *why* the two outputs line up
even when generated in separate invocations (see easynav_gis_tool.md).

Texture rendering is deliberately decoupled from `TerrainGrid`'s geometry
spacing (which follows the DEM's coarser ~30 m native resolution): the mesh
is coarse, but the draped texture can still use the imagery's much finer
native resolution, exactly like a game/robotics terrain that combines a
low-poly heightfield with a high-resolution diffuse texture.
"""

from dataclasses import dataclass
import sys

import numpy as np

from .geo.dem import DemGrid
from .geo.imagery import ImageryMosaic
from .geo.projection import LocalProjection


@dataclass
class TerrainGrid:
    """A regular local-ENU grid of (x, y, z) + RGB samples."""

    xs: np.ndarray
    ys: np.ndarray
    elevation: np.ndarray
    rgb: np.ndarray
    spacing_m: float
    center_elevation_amsl: float


def build_terrain_grid(
    projection: LocalProjection,
    dem: DemGrid,
    imagery: ImageryMosaic,
    size_m: float,
    spacing_m: float,
) -> TerrainGrid:
    """Sample DEM elevation + imagery color on a `spacing_m` grid over `size_m`."""
    if size_m <= 0:
        raise ValueError(f'size_m must be positive: {size_m}')
    if spacing_m <= 0:
        raise ValueError(f'spacing_m must be positive: {spacing_m}')

    half = size_m / 2.0
    n = max(1, int(round(size_m / spacing_m)))
    coords = np.linspace(-half, half, n + 1)

    center_elevation = dem.sample(projection.center_lon, projection.center_lat)

    rows = cols = len(coords)
    xs = np.zeros((rows, cols), dtype=np.float64)
    ys = np.zeros((rows, cols), dtype=np.float64)
    elevation = np.zeros((rows, cols), dtype=np.float32)
    rgb = np.zeros((rows, cols, 3), dtype=np.uint8)

    for i, y in enumerate(coords):
        for j, x in enumerate(coords):
            lon, lat = projection.to_lonlat(x, y)
            xs[i, j] = x
            ys[i, j] = y
            elevation[i, j] = dem.sample(lon, lat) - center_elevation
            rgb[i, j] = imagery.sample(lon, lat)

    return TerrainGrid(
        xs=xs,
        ys=ys,
        elevation=elevation,
        rgb=rgb,
        spacing_m=(2 * half) / n,
        center_elevation_amsl=center_elevation,
    )


def render_texture(
    projection: LocalProjection,
    imagery: ImageryMosaic,
    size_m: float,
    pixels: int = 512,
) -> np.ndarray:
    """
    Render a `pixels` x `pixels` RGB texture over the `size_m` square.

    Independent of `TerrainGrid`'s (coarser) geometry spacing, so the mesh
    can stay low-poly while the draped texture keeps the imagery's detail.
    """
    if size_m <= 0:
        raise ValueError(f'size_m must be positive: {size_m}')
    if pixels < 1:
        raise ValueError(f'pixels must be >= 1: {pixels}')

    half = size_m / 2.0
    # Sample at texel centers; texel 0 is the north-west corner (row-major,
    # top-to-bottom) to match standard image/UV conventions.
    xs = np.linspace(-half, half, pixels, endpoint=False) + (size_m / pixels) / 2.0
    ys = np.linspace(half, -half, pixels, endpoint=False) - (size_m / pixels) / 2.0

    out = np.zeros((pixels, pixels, 3), dtype=np.uint8)
    progress_every = max(1, pixels // 20)
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            lon, lat = projection.to_lonlat(x, y)
            out[i, j] = imagery.sample(lon, lat)
        if pixels >= 1000 and (i + 1) % progress_every == 0:
            print(f'[navmap_gis_tool] texture: row {i + 1}/{pixels}', file=sys.stderr)
    return out
