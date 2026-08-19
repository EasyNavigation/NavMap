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

"""Unit tests for navmap_tools.pcd_writer."""

from navmap_tools.pcd_writer import write_colors_csv, write_pcd_xyz

import numpy as np

import pytest


# ---------------------------------------------------------------------------
# write_pcd_xyz
# ---------------------------------------------------------------------------

def test_write_pcd_xyz_header_fields(tmp_path):
    path = tmp_path / 'cloud.pcd'
    n = write_pcd_xyz(path, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    assert n == 2
    lines = path.read_text().splitlines()
    assert lines[0] == '# .PCD v0.7 - Point Cloud Data file format'
    assert 'VERSION 0.7' in lines
    assert 'FIELDS x y z' in lines
    assert 'SIZE 4 4 4' in lines
    assert 'TYPE F F F' in lines
    assert 'COUNT 1 1 1' in lines
    assert 'WIDTH 2' in lines
    assert 'HEIGHT 1' in lines
    assert 'POINTS 2' in lines
    assert 'DATA ascii' in lines


def test_write_pcd_xyz_data_rows_round_trip(tmp_path):
    path = tmp_path / 'cloud.pcd'
    pts = [(1.5, -2.25, 3.0), (0.0, 0.0, 0.0)]
    write_pcd_xyz(path, pts)
    data_lines = path.read_text().splitlines()[-2:]
    parsed = [tuple(float(v) for v in line.split()) for line in data_lines]
    for (x, y, z), (px, py, pz) in zip(pts, parsed):
        assert px == pytest.approx(x)
        assert py == pytest.approx(y)
        assert pz == pytest.approx(z)


def test_write_pcd_xyz_empty_cloud(tmp_path):
    path = tmp_path / 'cloud.pcd'
    n = write_pcd_xyz(path, [])
    assert n == 0
    assert 'WIDTH 0' in path.read_text()
    assert 'POINTS 0' in path.read_text()


def test_write_pcd_xyz_rejects_wrong_shape(tmp_path):
    path = tmp_path / 'cloud.pcd'
    with pytest.raises(ValueError, match='Nx3'):
        write_pcd_xyz(path, [(1.0, 2.0)])
    with pytest.raises(ValueError, match='Nx3'):
        write_pcd_xyz(path, np.zeros((2, 4)))


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_write_pcd_xyz_rejects_non_finite_values(tmp_path, bad):
    path = tmp_path / 'cloud.pcd'
    with pytest.raises(ValueError, match='NaN/Inf'):
        write_pcd_xyz(path, [(bad, 0.0, 0.0)])


def test_write_pcd_xyz_accepts_numpy_array(tmp_path):
    path = tmp_path / 'cloud.pcd'
    n = write_pcd_xyz(path, np.array([[1.0, 2.0, 3.0]]))
    assert n == 1


# ---------------------------------------------------------------------------
# write_pcd_xyz -- organized (width/height) mode
#
# navmap_tools always writes its own .pcd this way so
# navmap_ros::from_regular_grid (a dedicated, gap-free mesher) can be used
# instead of the generic neighbor-search one; see easynav_gis_tool.md.
# ---------------------------------------------------------------------------

def test_write_pcd_xyz_organized_header_fields(tmp_path):
    path = tmp_path / 'cloud.pcd'
    pts = [(float(i), float(j), 0.0) for j in range(3) for i in range(4)]
    n = write_pcd_xyz(path, pts, width=4, height=3)
    assert n == 12
    lines = path.read_text().splitlines()
    assert 'WIDTH 4' in lines
    assert 'HEIGHT 3' in lines
    assert 'POINTS 12' in lines


def test_write_pcd_xyz_organized_rejects_mismatched_count(tmp_path):
    path = tmp_path / 'cloud.pcd'
    pts = [(0.0, 0.0, 0.0)] * 12
    with pytest.raises(ValueError, match='width\\*height'):
        write_pcd_xyz(path, pts, width=4, height=4)


def test_write_pcd_xyz_requires_both_width_and_height(tmp_path):
    path = tmp_path / 'cloud.pcd'
    pts = [(0.0, 0.0, 0.0)] * 4
    with pytest.raises(ValueError, match='together'):
        write_pcd_xyz(path, pts, width=4)
    with pytest.raises(ValueError, match='together'):
        write_pcd_xyz(path, pts, height=4)


def test_write_pcd_xyz_default_is_unorganized(tmp_path):
    path = tmp_path / 'cloud.pcd'
    write_pcd_xyz(path, [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)])
    lines = path.read_text().splitlines()
    assert 'WIDTH 2' in lines
    assert 'HEIGHT 1' in lines


# ---------------------------------------------------------------------------
# write_colors_csv
# ---------------------------------------------------------------------------

def test_write_colors_csv_rows(tmp_path):
    path = tmp_path / 'colors.csv'
    n = write_colors_csv(path, [(255, 0, 0), (0, 255, 0)])
    assert n == 2
    assert path.read_text().splitlines() == ['255,0,0', '0,255,0']


def test_write_colors_csv_empty(tmp_path):
    path = tmp_path / 'colors.csv'
    n = write_colors_csv(path, [])
    assert n == 0
    assert path.read_text() == ''


def test_write_colors_csv_rejects_wrong_shape(tmp_path):
    path = tmp_path / 'colors.csv'
    with pytest.raises(ValueError, match='Nx3'):
        write_colors_csv(path, [(1, 2)])


@pytest.mark.parametrize('bad', [-1, 256, 1000])
def test_write_colors_csv_rejects_out_of_range(tmp_path, bad):
    path = tmp_path / 'colors.csv'
    with pytest.raises(ValueError, match='0, 255'):
        write_colors_csv(path, [(bad, 0, 0)])


def test_write_colors_csv_boundary_values_ok(tmp_path):
    path = tmp_path / 'colors.csv'
    write_colors_csv(path, [(0, 255, 128)])
    assert path.read_text().splitlines() == ['0,255,128']
