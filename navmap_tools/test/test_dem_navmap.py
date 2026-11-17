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

"""Unit tests for navmap_tools.geo.dem_navmap: a local height grid as a NavMap."""

import math

from navmap_tools.geo.dem_file import LocalDem
from navmap_tools.geo.dem_navmap import navmap_from_dem
from navmap_tools.resample import read_navmap, write_navmap

import numpy as np


def _dem(side=5, res=1.0, z=None, cell_valid=None, rgb=None):
    return LocalDem(
        side=side, resolution=res, data_resolution=res, radius=(side - 1) / 2 * res,
        origin_lat=40.0, origin_lon=-3.0, reference_altitude=0.0,
        requested_reference=float('nan'), source=0,
        z=np.zeros((side, side), np.float32) if z is None else z, rgb=rgb,
        cell_valid=cell_valid)


def test_two_navcels_per_cell_on_the_nodes():
    msg = navmap_from_dem(_dem())
    assert len(msg.navcels_v0) == 2 * 16
    assert len(msg.positions_x) == 25
    assert min(msg.positions_x) == -2.0 and max(msg.positions_y) == 2.0
    assert len(msg.surfaces) == 1 and len(msg.surfaces[0].navcels) == 32


def test_cells_without_ground_are_holes_and_unused_vertices_go():
    cv = np.ones((4, 4), np.uint8)
    cv[1:3, 1:3] = 0                     # the four central cells: a building
    msg = navmap_from_dem(_dem(cell_valid=cv))
    assert len(msg.navcels_v0) == 2 * 12
    assert len(msg.positions_x) == 24    # only the centre node is left without a NavCel
    cv[:] = 0
    cv[0, 0] = 1
    msg = navmap_from_dem(_dem(cell_valid=cv))
    assert len(msg.navcels_v0) == 2 and len(msg.positions_x) == 4


def test_steep_cells_are_left_out():
    z = np.zeros((5, 5), np.float32)
    z[:, 4] = 5.0                        # the last column of cells rises 5 m in 1 m
    msg = navmap_from_dem(_dem(z=z), max_slope_deg=30.0)
    assert len(msg.navcels_v0) == 2 * 12
    msg = navmap_from_dem(_dem(z=z), max_slope_deg=89.0)
    assert len(msg.navcels_v0) == 2 * 16
    assert math.isclose(max(msg.positions_z), 5.0)


def test_colours_and_a_round_trip_through_a_file(tmp_path):
    rgb = np.zeros((5, 5, 3), np.uint8)
    rgb[..., 0] = 200
    msg = navmap_from_dem(_dem(rgb=rgb))
    assert msg.has_vertex_rgba and bytes(msg.colors_r)[0] == 200
    path = str(tmp_path / 'm.navmap')
    write_navmap(msg, path)
    back = read_navmap(path)
    assert len(back.navcels_v0) == 32 and len(back.positions_x) == 25
