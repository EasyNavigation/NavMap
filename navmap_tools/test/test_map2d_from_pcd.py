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

"""Unit tests for navmap_tools.map2d_from_pcd and the PCD reader."""

from navmap_tools.map2d_from_pcd import (
    FREE, grid_from_cloud, main, OCCUPIED, UNKNOWN, write_map_server,
)
from navmap_tools.pcd_writer import read_pcd_xyz, write_pcd_xyz
import numpy as np
import pytest

RES = 0.1


def room(size=3.0, z=0.5, floor=True):
    """Return a closed square room of walls from 0 to size (m), plus floor points inside."""
    t = np.arange(0.0, size + 1e-9, RES / 2)
    walls = [[x, 0.0, z] for x in t] + [[x, size, z] for x in t] + \
        [[0.0, y, z] for y in t] + [[size, y, z] for y in t]
    pts = np.array(walls)
    if floor:
        f = np.arange(RES, size, RES)
        pts = np.vstack([pts, [[x, y, 0.0] for x in f for y in f]])
    return pts


def cell(grid, origin, x, y):
    return grid[int((y - origin[1]) / RES), int((x - origin[0]) / RES)]


def test_room_is_free_inside_occupied_on_walls_unknown_outside():
    grid, origin = grid_from_cloud(room(), (1.5, 1.5), RES, 0.1, 1.2)
    assert origin == (-1, -1)
    assert cell(grid, origin, 1.5, 1.5) == FREE
    assert cell(grid, origin, 0.55, 2.45) == FREE
    assert cell(grid, origin, 0.02, 1.5) == OCCUPIED
    assert cell(grid, origin, 3.02, 1.5) == OCCUPIED
    assert cell(grid, origin, -0.5, 1.5) == UNKNOWN     # outside the walls
    assert cell(grid, origin, 1.5, 3.5) == UNKNOWN


def test_points_outside_the_band_are_not_obstacles():
    # Floor (z = 0) and ceiling (z = 2) points do not block; only the band does
    ceiling = np.array([[x, y, 2.0] for x in np.arange(0.5, 2.5, 0.1)
                        for y in np.arange(0.5, 2.5, 0.1)])
    grid, origin = grid_from_cloud(np.vstack([room(), ceiling]), (1.5, 1.5), RES, 0.1, 1.2)
    assert cell(grid, origin, 1.0, 1.0) == FREE


def test_an_obstacle_inside_the_room_is_occupied_and_not_flooded_through():
    box = np.array([[x, y, 0.4] for x in np.arange(1.2, 1.81, 0.05)
                    for y in np.arange(1.2, 1.81, 0.05)])
    grid, origin = grid_from_cloud(np.vstack([room(), box]), (0.5, 0.5), RES, 0.1, 1.2)
    assert cell(grid, origin, 1.5, 1.5) == OCCUPIED
    assert cell(grid, origin, 2.5, 2.5) == FREE    # reached around the box


def test_seed_on_a_wall_or_outside_raises():
    with pytest.raises(ValueError, match='seed'):
        grid_from_cloud(room(), (0.0, 1.5), RES, 0.1, 1.2)
    with pytest.raises(ValueError, match='seed'):
        grid_from_cloud(room(), (50.0, 50.0), RES, 0.1, 1.2)


def test_no_points_in_the_band_raises():
    with pytest.raises(ValueError, match='no points'):
        grid_from_cloud(room(z=2.0, floor=False), (1.5, 1.5), RES, 0.1, 1.2)


def test_write_map_server_files(tmp_path):
    grid = np.array([[FREE, OCCUPIED], [UNKNOWN, FREE]], np.uint8)
    write_map_server(str(tmp_path / 'map'), grid, 0.2, (-1, -2))
    pgm = (tmp_path / 'map.pgm').read_bytes()
    assert pgm.startswith(b'P5\n2 2\n255\n')
    # Rows top-down: the last grid row (highest y) comes first
    assert pgm[-4:] == bytes([UNKNOWN, FREE, FREE, OCCUPIED])
    yaml = (tmp_path / 'map.yaml').read_text()
    assert 'image: map.pgm' in yaml and 'resolution: 0.2' in yaml and 'origin: [-1, -2, 0]' in yaml


def test_read_pcd_round_trip(tmp_path):
    pts = np.array([[0.0, 1.5, -2.25], [3.125, -4.0, 0.5]])
    write_pcd_xyz(tmp_path / 'c.pcd', pts)
    np.testing.assert_allclose(read_pcd_xyz(tmp_path / 'c.pcd'), pts)


def test_read_pcd_of_an_empty_cloud(tmp_path):
    write_pcd_xyz(tmp_path / 'e.pcd', np.zeros((0, 3)))
    assert read_pcd_xyz(tmp_path / 'e.pcd').shape == (0, 3)


def test_read_pcd_rejects_binary_and_non_pcd(tmp_path):
    (tmp_path / 'b.pcd').write_text('VERSION 0.7\nFIELDS x y z\nDATA binary\n')
    with pytest.raises(ValueError, match='ASCII'):
        read_pcd_xyz(tmp_path / 'b.pcd')
    (tmp_path / 'n.pcd').write_text('hello\n')
    with pytest.raises(ValueError, match='not a PCD'):
        read_pcd_xyz(tmp_path / 'n.pcd')


def test_main_end_to_end(tmp_path, capsys):
    write_pcd_xyz(tmp_path / 'room.pcd', room())
    main([str(tmp_path / 'room.pcd'), str(tmp_path / 'room'), '--seed', '1.5', '1.5'])
    assert (tmp_path / 'room.pgm').exists() and (tmp_path / 'room.yaml').exists()
    assert 'free' in capsys.readouterr().out
