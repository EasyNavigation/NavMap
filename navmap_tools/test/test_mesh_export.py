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

"""Unit tests for navmap_tools.mesh_export -- pure geometry/IO, no network."""

import struct
import xml.etree.ElementTree as ET

from navmap_tools.mesh_export import (
    _grid_to_vertices_faces,
    _uvs_for_grid,
    _vertex_normals,
    write_dae,
    write_stl,
    write_texture_png,
)
from navmap_tools.terrain import TerrainGrid

import numpy as np

from PIL import Image

import pytest


def _grid_2x2():
    return TerrainGrid(
        xs=np.array([[-1.0, 1.0], [-1.0, 1.0]]),
        ys=np.array([[1.0, 1.0], [-1.0, -1.0]]),
        elevation=np.array([[5.0, 6.0], [7.0, 8.0]], dtype=np.float32),
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        spacing_m=2.0,
        center_elevation_amsl=0.0,
    )


def _grid_3x3():
    xs, ys = np.meshgrid(np.linspace(-1, 1, 3), np.linspace(1, -1, 3))
    return TerrainGrid(
        xs=xs, ys=ys,
        elevation=np.zeros((3, 3), dtype=np.float32),
        rgb=np.zeros((3, 3, 3), dtype=np.uint8),
        spacing_m=1.0,
        center_elevation_amsl=0.0,
    )


# ---------------------------------------------------------------------------
# _grid_to_vertices_faces
# ---------------------------------------------------------------------------

def test_grid_to_vertices_faces_2x2_counts():
    verts, faces = _grid_to_vertices_faces(_grid_2x2())
    assert verts.shape == (4, 3)
    assert faces.shape == (2, 3)


def test_grid_to_vertices_faces_3x3_counts():
    verts, faces = _grid_to_vertices_faces(_grid_3x3())
    assert verts.shape == (9, 3)
    assert faces.shape == (8, 3)  # (3-1)*(3-1)*2


def test_grid_to_vertices_faces_indices_are_in_range():
    verts, faces = _grid_to_vertices_faces(_grid_3x3())
    assert faces.min() >= 0
    assert faces.max() < len(verts)


def test_grid_to_vertices_faces_vertex_z_matches_elevation():
    grid = _grid_2x2()
    verts, _ = _grid_to_vertices_faces(grid)
    assert sorted(verts[:, 2].tolist()) == [5.0, 6.0, 7.0, 8.0]


def test_grid_to_vertices_faces_every_vertex_used_at_least_once():
    verts, faces = _grid_to_vertices_faces(_grid_3x3())
    used = set(faces.reshape(-1).tolist())
    assert used == set(range(len(verts)))


def test_grid_to_vertices_faces_winding_gives_upward_normals():
    """
    Regression: a flat, level grid must wind CCW-from-above (+Z normal).

    A real generated world showed only the scene background (grey) from
    above and a flat white underside from below in Gazebo -- exactly the
    signature of a backface-culled, downward-facing front face. See this
    function's docstring in mesh_export.py for the fix and the by-hand
    cross-product check that caught it.
    """
    xs, ys = np.meshgrid(np.linspace(0.0, 1.0, 3), np.linspace(0.0, 1.0, 3))
    grid = TerrainGrid(
        xs=xs, ys=ys, elevation=np.zeros((3, 3), dtype=np.float32),
        rgb=np.zeros((3, 3, 3), dtype=np.uint8), spacing_m=0.5, center_elevation_amsl=0.0,
    )
    verts, faces = _grid_to_vertices_faces(grid)
    for a, b, c in faces:
        p0, p1, p2 = verts[a], verts[b], verts[c]
        normal = np.cross(p1 - p0, p2 - p0)
        assert normal[2] > 0, f'triangle ({a},{b},{c}) faces down: normal={normal}'


# ---------------------------------------------------------------------------
# _uvs_for_grid
# ---------------------------------------------------------------------------

def test_uvs_for_grid_corners():
    grid = _grid_2x2()
    uvs = _uvs_for_grid(grid, size_m=2.0).reshape(2, 2, 2)
    # COLLADA's v origin is the BOTTOM of the texture; gz-common's
    # ColladaLoader flips v (1.0 - v) on load to convert to Ogre's top-left
    # origin. So north (y=+1) must be written as v=1 here, so it lands on
    # v=0 (the top / north row) after gz's own flip -- south -> v=0 here.
    assert uvs[0, 0] == pytest.approx((0.0, 1.0))  # west, north
    assert uvs[0, 1] == pytest.approx((1.0, 1.0))  # east, north
    assert uvs[1, 0] == pytest.approx((0.0, 0.0))  # west, south
    assert uvs[1, 1] == pytest.approx((1.0, 0.0))  # east, south


def test_uvs_for_grid_stay_within_unit_square():
    grid = _grid_3x3()
    uvs = _uvs_for_grid(grid, size_m=2.0)
    assert (uvs >= -1e-9).all()
    assert (uvs <= 1.0 + 1e-9).all()


# ---------------------------------------------------------------------------
# _vertex_normals
#
# Regression: gz-rendering's Ogre2 PBS pipeline needs a NORMAL vertex
# attribute to shade with at all -- without one it silently renders flat
# white and never samples the diffuse texture, even though the texture/
# material data is completely valid. Confirmed empirically in Gazebo (a
# known-good reference texture stayed invisible until a NORMAL source was
# added). See mesh_export.py's write_dae docstring.
# ---------------------------------------------------------------------------

def _flat_grid(n=3, spacing=1.0):
    coords = np.linspace(-(n - 1) / 2 * spacing, (n - 1) / 2 * spacing, n)
    xs, ys = np.meshgrid(coords, coords)
    return TerrainGrid(
        xs=xs, ys=ys, elevation=np.zeros((n, n), dtype=np.float32),
        rgb=np.zeros((n, n, 3), dtype=np.uint8), spacing_m=spacing,
        center_elevation_amsl=0.0,
    )


def test_vertex_normals_shape():
    grid = _flat_grid(n=4)
    normals = _vertex_normals(grid)
    assert normals.shape == (16, 3)


def test_vertex_normals_flat_grid_points_straight_up():
    grid = _flat_grid(n=5)
    normals = _vertex_normals(grid)
    assert np.allclose(normals, np.array([0.0, 0.0, 1.0]), atol=1e-9)


def test_vertex_normals_are_unit_length():
    grid = _flat_grid(n=5)
    rng = np.random.default_rng(0)
    grid.elevation[:] = rng.uniform(-2.0, 2.0, size=grid.elevation.shape)
    normals = _vertex_normals(grid)
    lengths = np.linalg.norm(normals, axis=-1)
    assert np.allclose(lengths, 1.0)


def test_vertex_normals_tilt_away_from_uphill_direction():
    # Elevation rises with x (east): the surface tilts, so the normal must
    # lean in -x (west) -- i.e. away from the uphill direction.
    grid = _flat_grid(n=5, spacing=1.0)
    grid.elevation = grid.xs.astype(np.float32) * 0.5
    normals = _vertex_normals(grid)
    assert (normals[:, 0] < 0).all()
    assert (normals[:, 2] > 0).all()


def test_vertex_normals_no_nan_or_inf():
    grid = _flat_grid(n=6)
    rng = np.random.default_rng(1)
    grid.elevation[:] = rng.uniform(-5.0, 5.0, size=grid.elevation.shape)
    normals = _vertex_normals(grid)
    assert np.isfinite(normals).all()


# ---------------------------------------------------------------------------
# write_stl
# ---------------------------------------------------------------------------

def test_write_stl_file_size_matches_triangle_count(tmp_path):
    grid = _grid_2x2()
    path = tmp_path / 'mesh.stl'
    n_tris = write_stl(path, grid)
    assert n_tris == 2
    expected_size = 80 + 4 + n_tris * 50
    assert path.stat().st_size == expected_size


def test_write_stl_header_and_triangle_count(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.stl'
    write_stl(path, grid)
    data = path.read_bytes()
    assert data[:80] == b'\x00' * 80
    (count,) = struct.unpack('<I', data[80:84])
    assert count == 8


def test_write_stl_normals_are_unit_length_or_zero(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.stl'
    write_stl(path, grid)
    data = path.read_bytes()
    offset = 84
    for _ in range(8):
        nx, ny, nz = struct.unpack('<fff', data[offset:offset + 12])
        length = (nx ** 2 + ny ** 2 + nz ** 2) ** 0.5
        assert length == pytest.approx(1.0, abs=1e-4) or length == pytest.approx(0.0, abs=1e-6)
        offset += 50


# ---------------------------------------------------------------------------
# write_texture_png
# ---------------------------------------------------------------------------

def test_write_texture_png_roundtrip(tmp_path):
    path = tmp_path / 'tex.png'
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    rgb[0, 0] = (255, 0, 0)
    write_texture_png(path, rgb)
    with Image.open(path) as im:
        assert im.size == (6, 4)  # PIL size is (W, H)
        assert im.mode == 'RGB'
        assert im.getpixel((0, 0)) == (255, 0, 0)


@pytest.mark.parametrize(
    'bad_shape,bad_dtype',
    [((4, 4), np.uint8), ((4, 4, 4), np.uint8), ((4, 4, 3), np.float32)],
)
def test_write_texture_png_rejects_bad_input(tmp_path, bad_shape, bad_dtype):
    path = tmp_path / 'tex.png'
    with pytest.raises(ValueError):
        write_texture_png(path, np.zeros(bad_shape, dtype=bad_dtype))


# ---------------------------------------------------------------------------
# write_dae
# ---------------------------------------------------------------------------

def test_write_dae_is_valid_xml(tmp_path):
    grid = _grid_2x2()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ET.parse(path)  # raises on malformed XML


def test_write_dae_returns_triangle_count(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.dae'
    n = write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    assert n == 8


def test_write_dae_embeds_texture_filename_verbatim(tmp_path):
    grid = _grid_2x2()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='../textures/ground_42.png')
    assert '../textures/ground_42.png' in path.read_text()


def test_write_dae_declares_z_up(tmp_path):
    grid = _grid_2x2()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    assert '<up_axis>Z_UP</up_axis>' in path.read_text()


def test_write_dae_position_uv_and_normal_counts_match_vertex_count(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    accessors = root.findall('.//c:accessor', ns)
    counts = {int(a.get('count')) for a in accessors}
    # 3x3 grid -> 9 vertices; positions, uvs and normals accessors all agree.
    assert counts == {9}


def test_write_dae_vertices_element_declares_position_and_normal(tmp_path):
    # Regression: without a NORMAL input here, gz-rendering's Ogre2 PBS
    # pipeline renders flat/unlit white and never samples the diffuse
    # texture -- see the note above _vertex_normals in mesh_export.py.
    grid = _grid_2x2()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    vertices_elem = root.find('.//c:vertices', ns)
    semantics = {i.get('semantic') for i in vertices_elem.findall('c:input', ns)}
    assert semantics == {'POSITION', 'NORMAL'}
    assert root.find('.//c:source[@id="terrain_normals"]', ns) is not None


# ---------------------------------------------------------------------------
# <p> index stream shape (regression: see write_dae's docstring for why this
# matters -- gz-common's ColladaLoader sizes the per-corner stride as the
# *count of declared (semantic, offset) inputs*, not max(offset) + 1, so two
# inputs sharing one offset value silently desyncs the whole index stream
# into an invisible, garbage mesh instead of failing loudly)
# ---------------------------------------------------------------------------

def test_write_dae_vertex_and_texcoord_inputs_use_distinct_offsets(tmp_path):
    grid = _grid_2x2()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    triangles = root.find('.//c:triangles', ns)
    inputs = triangles.findall('c:input', ns)
    offsets_by_semantic = {i.get('semantic'): i.get('offset') for i in inputs}
    assert offsets_by_semantic == {'VERTEX': '0', 'TEXCOORD': '1'}


def test_write_dae_p_stream_has_two_values_per_corner(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.dae'
    n_tris = write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    p_text = root.find('.//c:triangles/c:p', ns).text
    values = p_text.split()
    # 3 corners/triangle * 2 inputs (VERTEX offset 0, TEXCOORD offset 1).
    assert len(values) == n_tris * 3 * 2


def test_write_dae_p_stream_pairs_are_equal_vertex_and_texcoord_index(tmp_path):
    grid = _grid_3x3()
    path = tmp_path / 'mesh.dae'
    write_dae(path, grid, size_m=2.0, texture_filename='tex.png')
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    values = [int(v) for v in root.find('.//c:triangles/c:p', ns).text.split()]
    pairs = list(zip(values[0::2], values[1::2]))
    assert all(vertex_idx == texcoord_idx for vertex_idx, texcoord_idx in pairs)
