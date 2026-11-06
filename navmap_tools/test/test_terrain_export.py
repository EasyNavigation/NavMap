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
"""Unit tests for navmap_tools.geo.terrain_export (a local DEM as a Gazebo terrain)."""

import struct

from navmap_tools.geo.dem_file import build_local_dem, DemSource, LocalDem
from navmap_tools.geo.terrain_export import write_gazebo_terrain, write_world_from_template

import numpy as np

import pytest


class _Ramp:
    """Height rises 1 m per 10 m to the east."""

    def sample_array(self, lon, lat):
        return 100.0 + (lon + 3.8) * 111320.0 * 0.76 / 10.0


def _dem(radius=8.0, resolution=1.0, rgb=False):
    dem = build_local_dem(_Ramp(), 40.4, -3.8, radius, resolution, DemSource.IGN)
    if rgb:
        dem.rgb = np.zeros((dem.side, dem.side, 3), dtype=np.uint8)
        dem.rgb[-1, 0] = (255, 0, 0)   # north-west node
        dem.imagery_zoom = 18
    return dem


def _stl(path):
    raw = path.read_bytes()
    (n,) = struct.unpack_from('<I', raw, 80)
    rec = np.frombuffer(
        raw, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]), count=n, offset=84)
    assert len(raw) == 84 + 50 * n
    return rec


def test_stl_has_two_triangles_per_cell_facing_up(tmp_path):
    write_gazebo_terrain(_dem(), tmp_path)
    rec = _stl(tmp_path / 'terrain.stl')
    assert len(rec) == 2 * 16 * 16   # 17 x 17 nodes
    assert (rec['n'][:, 2] > 0).all()   # counter-clockwise seen from +z
    assert np.allclose(np.linalg.norm(rec['n'], axis=1), 1.0, atol=1e-5)
    xs = rec['v'][:, 0::3]
    assert xs.min() == pytest.approx(-8.0) and xs.max() == pytest.approx(8.0)


def test_terrain_resolution_makes_the_collision_mesh_coarser(tmp_path):
    write_gazebo_terrain(_dem(radius=8.0), tmp_path, terrain_resolution=4.0)
    assert len(_stl(tmp_path / 'terrain.stl')) == 2 * 4 * 4   # every 4th node: 5 x 5
    # not a divisor of side - 1 = 16: the largest one below (4 -> 4, 6 -> 4, 16 stays)
    write_gazebo_terrain(_dem(radius=8.0), tmp_path, terrain_resolution=6.0)
    assert len(_stl(tmp_path / 'terrain.stl')) == 2 * 4 * 4


def test_obj_has_vertices_uvs_normals_and_faces_that_match(tmp_path):
    write_gazebo_terrain(_dem(), tmp_path)
    lines = (tmp_path / 'terrain.obj').read_text().splitlines()
    kinds = [ln.split()[0] for ln in lines]
    assert kinds[:2] == ['mtllib', 'usemtl']
    assert kinds.count('v') == kinds.count('vt') == kinds.count('vn') == 17 * 17
    assert kinds.count('f') == 2 * 16 * 16
    first_face = [ln for ln in lines if ln.startswith('f ')][0].split()[1:]
    assert first_face == ['1/1/1', '2/2/2', '19/19/19']   # (v00, v10, v11), 1-based
    vn = np.array([[float(x) for x in ln.split()[1:]] for ln in lines if ln.startswith('vn ')])
    assert (vn[:, 2] > 0).all()
    assert vn[:, 0].max() < 0   # the ramp rises to the east: the normal leans west


def test_texture_is_written_with_the_north_row_first(tmp_path):
    from PIL import Image
    write_gazebo_terrain(_dem(rgb=True), tmp_path)
    assert 'map_Kd terrain.png' in (tmp_path / 'terrain.mtl').read_text()
    img = np.asarray(Image.open(tmp_path / 'terrain.png'))
    assert img.shape == (17, 17, 3)
    assert tuple(img[0, 0]) == (255, 0, 0)   # the north-west node is the first texel


def test_without_imagery_there_is_no_texture(tmp_path):
    write_gazebo_terrain(_dem(), tmp_path)
    assert not (tmp_path / 'terrain.png').exists()
    assert 'map_Kd' not in (tmp_path / 'terrain.mtl').read_text()


def test_world_template_is_filled(tmp_path):
    dem = _dem()
    template = tmp_path / 't.world.in'
    template.write_text(
        '<lat>@LATITUDE@</lat><lon>@LONGITUDE@</lon><el>@ELEVATION@</el>'
        '<uri>file://@TERRAIN_DIR@/terrain.stl</uri>')
    out = tmp_path / 'out' / 'world.sdf'
    write_world_from_template(dem, template, out, tmp_path / 'terr')
    text = out.read_text()
    assert '<lat>40.4</lat>' in text and '<lon>-3.8</lon>' in text
    assert f'<el>{dem.reference_altitude:.10g}</el>' in text
    assert f'file://{(tmp_path / "terr").resolve()}/terrain.stl' in text


def test_world_template_without_terrain_dir_is_refused(tmp_path):
    template = tmp_path / 't.in'
    template.write_text('<lat>@LATITUDE@</lat>')
    with pytest.raises(ValueError, match='TERRAIN_DIR'):
        write_world_from_template(_dem(), template, tmp_path / 'w.sdf', tmp_path)


def test_local_dem_height_and_cache_match():
    dem = _dem()
    assert isinstance(dem, LocalDem)
    assert dem.height_at(0.0, 0.0) == pytest.approx(0.0, abs=1e-6)   # the origin is z = 0
    assert dem.height_at(10.0, 0.0) == pytest.approx(dem.height_at(8.0, 0.0))   # clamped
    assert dem.height_at(4.0, 0.0) > 0.3
    assert dem.matches(40.4, -3.8, 8.0, 1.0, DemSource.IGN)
    assert not dem.matches(40.4, -3.8, 9.0, 1.0, DemSource.IGN)         # another radius
    assert not dem.matches(40.4, -3.8, 8.0, 1.0, DemSource.MESH)        # another source
    assert not dem.matches(40.4, -3.8, 8.0, 1.0, DemSource.IGN, requested_reference=600.0)
    assert not dem.matches(40.4, -3.8, 8.0, 1.0, DemSource.IGN, data_resolution=5.0)
