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
"""Unit tests for navmap_tools.geo.mesh_dem (an STL as a terrain model) -- no network."""

import struct

from navmap_tools.geo.mesh_dem import (
    hole_cells,
    load_stl_triangles,
    MeshDem,
    MeshHeightfield,
)

import numpy as np

import pytest


def _quad(x0, y0, x1, y1, z):
    return np.array([
        [x0, y0, z, x1, y0, z, x1, y1, z],
        [x0, y0, z, x1, y1, z, x0, y1, z],
    ], dtype=np.float32)


def _write_binary_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<12fH', 0, 0, 1, *t, 0))


def _write_ascii_stl(path, tris):
    with open(path, 'w') as f:
        f.write('solid t\n')
        for t in tris:
            f.write('facet normal 0 0 1\nouter loop\n')
            for k in range(3):
                f.write(f'vertex {t[3 * k]} {t[3 * k + 1]} {t[3 * k + 2]}\n')
            f.write('endloop\nendfacet\n')
        f.write('endsolid t\n')


def test_binary_and_ascii_stl_give_the_same_triangles(tmp_path):
    tris = _quad(0, 0, 4, 4, 2.0)
    _write_binary_stl(tmp_path / 'b.stl', tris)
    _write_ascii_stl(tmp_path / 'a.stl', tris)
    assert load_stl_triangles(tmp_path / 'b.stl') == pytest.approx(tris)
    assert load_stl_triangles(tmp_path / 'a.stl') == pytest.approx(tris)


def test_a_broken_stl_is_reported(tmp_path):
    (tmp_path / 'x.stl').write_text('this is not a mesh')
    with pytest.raises(ValueError):
        load_stl_triangles(tmp_path / 'x.stl')


def test_flat_quad_is_a_flat_heightfield():
    field = MeshHeightfield.from_triangles(_quad(0, 0, 4, 4, 2.0), cell=0.5)
    assert field.z.shape == (9, 9)
    assert field.coverage == pytest.approx(1.0)
    assert field.sample_array(np.array([1.3]), np.array([2.7]))[0] == pytest.approx(2.0)


def test_a_slope_is_interpolated():
    tris = np.array([[0, 0, 0, 4, 0, 4, 4, 4, 4], [0, 0, 0, 4, 4, 4, 0, 4, 0]], dtype=np.float32)
    field = MeshHeightfield.from_triangles(tris, cell=0.5)
    assert field.sample_array(np.array([2.0]), np.array([1.0]))[0] == pytest.approx(2.0, abs=1e-5)
    assert field.sample_array(np.array([3.0]), np.array([1.0]))[0] == pytest.approx(3.0, abs=1e-5)


def test_the_highest_surface_wins_and_walls_collapse():
    floor = _quad(0, 0, 4, 4, 0.0)
    roof = _quad(1, 1, 3, 3, 3.0)
    # vertical: no area seen from above
    wall = np.array([[1, 1, 0, 3, 1, 0, 3, 1, 3]], dtype=np.float32)
    field = MeshHeightfield.from_triangles(np.vstack([floor, roof, wall]), cell=0.5)
    assert field.sample_array(np.array([2.0]), np.array([2.0]))[0] == pytest.approx(3.0)
    assert field.sample_array(np.array([0.5]), np.array([0.5]))[0] == pytest.approx(0.0)


def test_cells_without_triangles_are_filled_from_neighbours():
    # two separate pieces at the same height, a gap between them
    tris = np.vstack([_quad(0, 0, 2, 4, 5.0), _quad(6, 0, 8, 4, 5.0)])
    field = MeshHeightfield.from_triangles(tris, cell=0.5)
    assert field.coverage < 1.0
    assert not np.isnan(field.z).any()
    assert field.sample_array(np.array([4.0]), np.array([2.0]))[0] == pytest.approx(5.0)


def test_sampling_outside_clamps_to_the_edge():
    field = MeshHeightfield.from_triangles(_quad(0, 0, 4, 4, 2.0), cell=0.5)
    assert field.sample_array(np.array([100.0]), np.array([-100.0]))[0] == pytest.approx(2.0)


def test_empty_mesh_and_bad_cell_are_rejected():
    with pytest.raises(ValueError):
        MeshHeightfield.from_triangles(np.zeros((0, 9), dtype=np.float32))
    with pytest.raises(ValueError):
        MeshHeightfield.from_triangles(_quad(0, 0, 1, 1, 0), cell=0.0)


def test_mesh_dem_adds_the_world_elevation_at_the_enu_offset():
    field = MeshHeightfield.from_triangles(_quad(-50, -50, 50, 50, 3.0), cell=1.0)
    dem = MeshDem(field, 40.4, -3.7, world_elevation=675.0)
    assert dem.sample(-3.7, 40.4) == pytest.approx(678.0)
    assert dem.pixel_size_lat * 111320.0 == pytest.approx(1.0)


def test_hole_cells_are_the_cells_a_footprint_touches():
    # 9 x 9 nodes of 1 m around 0 (cells -4..4): a square roof of x 0.5..2.5, y -1.5..0.5
    roof = np.array([[0.5, -1.5, 9, 2.5, -1.5, 9, 2.5, 0.5, 9],
                     [0.5, -1.5, 9, 2.5, 0.5, 9, 0.5, 0.5, 9]], dtype=np.float32)
    holes = hole_cells(roof, 9, 1.0)
    assert holes.shape == (8, 8)
    # cells (i, j) with x in [i - 4, i - 3]: x 0.5..2.5 -> i 4..6, y -1.5..0.5 -> j 2..4
    expected = np.zeros((8, 8), dtype=bool)
    expected[2:5, 4:7] = True
    assert np.array_equal(holes, expected)


def test_a_vertical_wall_marks_the_cells_it_crosses_and_far_triangles_none():
    # a wall from (-3.5, 1.2) to (2.5, 1.2), 3 m high: a segment in xy
    wall = np.array([[-3.5, 1.2, 0, 2.5, 1.2, 0, 2.5, 1.2, 3]], dtype=np.float32)
    holes = hole_cells(wall, 9, 1.0)
    assert holes.sum() == 7 and holes[5, 0:7].all()   # y 1.2 -> j 5; x -3.5..2.5 -> i 0..6
    far = np.array([[50, 50, 0, 51, 50, 0, 50, 51, 0]], dtype=np.float32)
    assert not hole_cells(far, 9, 1.0).any()
    assert not hole_cells(np.zeros((0, 9), np.float32), 9, 1.0).any()


def test_a_diagonal_triangle_marks_what_it_touches_and_nothing_beyond():
    tri = np.array([[-4, -4, 0, 4, -4, 0, -4, 4, 0]], dtype=np.float32)   # the lower-left half
    holes = hole_cells(tri, 9, 1.0)
    # the long edge is x + y = 0; cell (i, j) has its lower-left corner at (i - 4, j - 4): it is
    # touched if i + j <= 8 (i + j == 8: at that corner only, which the conservative rule counts)
    for j in range(8):
        for i in range(8):
            assert holes[j, i] == (i + j <= 8), (i, j)
