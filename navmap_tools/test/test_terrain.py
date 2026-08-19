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
Unit tests for navmap_tools.terrain, using fake DEM/imagery sources.

Fakes only need a `.sample(lon, lat)` method (duck typing), so these tests
don't touch the network or the real Copernicus/Esri data sources at all.
"""

from navmap_tools.geo.projection import LocalProjection
from navmap_tools.terrain import build_terrain_grid, render_texture

import numpy as np

import pytest


class _ConstDem:

    def __init__(self, value):
        self.value = value

    def sample(self, lon, lat):
        return self.value


class _ConstImagery:

    def __init__(self, color):
        self.color = color

    def sample(self, lon, lat):
        return self.color


# ---------------------------------------------------------------------------
# build_terrain_grid
# ---------------------------------------------------------------------------

def test_build_terrain_grid_rejects_non_positive_size():
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='size_m'):
        build_terrain_grid(proj, _ConstDem(0), _ConstImagery((0, 0, 0)), 0.0, 10.0)


def test_build_terrain_grid_rejects_non_positive_spacing():
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='spacing_m'):
        build_terrain_grid(proj, _ConstDem(0), _ConstImagery((0, 0, 0)), 100.0, 0.0)


def test_build_terrain_grid_shape_matches_size_over_spacing():
    proj = LocalProjection(0.0, 0.0)
    grid = build_terrain_grid(proj, _ConstDem(0), _ConstImagery((0, 0, 0)), 100.0, 25.0)
    assert grid.elevation.shape == (5, 5)  # n = round(100/25) = 4 -> 5 samples/side
    assert grid.xs.shape == (5, 5)
    assert grid.ys.shape == (5, 5)
    assert grid.rgb.shape == (5, 5, 3)


def test_build_terrain_grid_constant_dem_zeroes_out_everywhere():
    proj = LocalProjection(40.0, -3.0)
    grid = build_terrain_grid(proj, _ConstDem(500.0), _ConstImagery((0, 0, 0)), 100.0, 25.0)
    assert grid.center_elevation_amsl == pytest.approx(500.0)
    assert np.allclose(grid.elevation, 0.0)


def test_build_terrain_grid_center_sample_is_exactly_at_origin():
    proj = LocalProjection(40.0, -3.0)
    grid = build_terrain_grid(proj, _ConstDem(123.0), _ConstImagery((1, 2, 3)), 100.0, 25.0)
    mid = grid.elevation.shape[0] // 2
    assert grid.xs[mid, mid] == pytest.approx(0.0, abs=1e-6)
    assert grid.ys[mid, mid] == pytest.approx(0.0, abs=1e-6)


def test_build_terrain_grid_rgb_matches_constant_imagery():
    proj = LocalProjection(0.0, 0.0)
    grid = build_terrain_grid(proj, _ConstDem(0.0), _ConstImagery((7, 8, 9)), 100.0, 50.0)
    assert grid.rgb.dtype == np.uint8
    assert (grid.rgb == np.array([7, 8, 9], dtype=np.uint8)).all()


def test_build_terrain_grid_spacing_m_reflects_actual_grid_step():
    proj = LocalProjection(0.0, 0.0)
    grid = build_terrain_grid(proj, _ConstDem(0.0), _ConstImagery((0, 0, 0)), 100.0, 30.0)
    n = grid.elevation.shape[0] - 1
    assert grid.spacing_m == pytest.approx(100.0 / n)


def test_build_terrain_grid_minimum_size_single_cell():
    proj = LocalProjection(0.0, 0.0)
    grid = build_terrain_grid(proj, _ConstDem(0.0), _ConstImagery((0, 0, 0)), 1.0, 100.0)
    # spacing >> size -> clamped to at least a 1-cell (2x2 vertex) grid.
    assert grid.elevation.shape == (2, 2)


# ---------------------------------------------------------------------------
# render_texture
# ---------------------------------------------------------------------------

def test_render_texture_rejects_non_positive_size():
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='size_m'):
        render_texture(proj, _ConstImagery((0, 0, 0)), 0.0, pixels=8)


def test_render_texture_rejects_non_positive_pixels():
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='pixels'):
        render_texture(proj, _ConstImagery((0, 0, 0)), 100.0, pixels=0)


def test_render_texture_shape_and_dtype():
    proj = LocalProjection(0.0, 0.0)
    tex = render_texture(proj, _ConstImagery((0, 0, 0)), 100.0, pixels=16)
    assert tex.shape == (16, 16, 3)
    assert tex.dtype == np.uint8


def test_render_texture_constant_imagery_gives_uniform_texture():
    proj = LocalProjection(40.0, -3.0)
    tex = render_texture(proj, _ConstImagery((11, 22, 33)), 200.0, pixels=8)
    assert (tex == np.array([11, 22, 33], dtype=np.uint8)).all()


def test_render_texture_single_pixel():
    proj = LocalProjection(0.0, 0.0)
    tex = render_texture(proj, _ConstImagery((1, 2, 3)), 10.0, pixels=1)
    assert tex.shape == (1, 1, 3)


def test_render_texture_reports_progress_for_large_textures(capsys):
    # A large auto-sized texture (see cli.py's _texture_pixels_for, driven by
    # --zoom now defaulting to 23) can take a couple of minutes; without this
    # a long run looks hung. Threshold is pixels >= 1000.
    proj = LocalProjection(0.0, 0.0)
    render_texture(proj, _ConstImagery((1, 2, 3)), 10.0, pixels=1000)
    err = capsys.readouterr().err
    assert 'texture: row' in err
