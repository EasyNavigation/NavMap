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
from navmap_tools.geo.dem_navmap import connected_parts, keep_connected, navmap_from_dem
from navmap_tools.resample import read_navmap, write_navmap

import numpy as np
import pytest


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


def test_connected_parts_join_cells_that_share_an_edge_not_a_corner():
    ok = np.array([[1, 1, 0, 0],
                   [0, 1, 0, 1],
                   [0, 0, 1, 1],
                   [1, 0, 0, 0]], bool)
    labels = connected_parts(ok)
    assert (labels[~ok] == -1).all()
    assert labels[0, 0] == labels[0, 1] == labels[1, 1]
    assert labels[1, 3] == labels[2, 3] == labels[2, 2]
    # (1, 1) and (2, 2) touch only at a corner: their NavCels share a vertex, no edge
    assert labels[1, 1] != labels[2, 2]
    assert len({labels[0, 0], labels[2, 2], labels[3, 0]}) == 3


def _flood_fill_parts(ok):
    """Return the parts of ok by a plain flood fill, as sets of cells: the reference."""
    seen = np.zeros(ok.shape, bool)
    parts = []
    for start in zip(*np.nonzero(ok)):
        if seen[start]:
            continue
        seen[start] = True
        stack, part = [start], set()
        while stack:
            j, i = stack.pop()
            part.add((j, i))
            for n in ((j + 1, i), (j - 1, i), (j, i + 1), (j, i - 1)):
                if (0 <= n[0] < ok.shape[0] and 0 <= n[1] < ok.shape[1] and ok[n]
                        and not seen[n]):
                    seen[n] = True
                    stack.append(n)
        parts.append(frozenset(part))
    return set(parts)


def _parts(labels):
    out = {}
    for j, i in zip(*np.nonzero(labels >= 0)):
        out.setdefault(labels[j, i], set()).add((j, i))
    return {frozenset(v) for v in out.values()}


def test_connected_parts_of_a_spiral_and_of_random_grids():
    # concentric rings, each joined to the next by one cell: a corridor 20 rings long, the worst
    # case of a label that has to travel far
    n = 41
    ok = np.zeros((n, n), bool)
    lo, hi = 0, n - 1
    while lo < hi:
        ok[lo, lo:hi + 1] = ok[lo:hi + 1, hi] = ok[hi, lo:hi + 1] = True
        ok[lo + 2:hi + 1, lo] = True     # the ring is open between (lo, lo) and (lo + 2, lo)
        if lo + 2 < hi:
            ok[lo + 2, lo + 1] = True    # the cell that joins it to the next ring
        lo += 2
        hi -= 2
    assert len(_flood_fill_parts(ok)) == 1
    assert len(np.unique(connected_parts(ok)[ok])) == 1
    rng = np.random.default_rng(1)
    for density in (0.45, 0.6, 0.75):    # around the percolation threshold: parts of every size
        ok = rng.random((120, 150)) < density
        assert _parts(connected_parts(ok)) == _flood_fill_parts(ok)


def _roofs_dem():
    """
    Make 9 x 9 cells of ground, with a building and a closed patio.

    The walls of the building (its cells 0 and 4) are too steep, so its flat roof (cells 1..3,
    3 x 3) is an island; the patio is 1 cell inside a ring of holes.
    """
    z = np.zeros((10, 10), np.float32)
    z[1:5, 1:5] = 10.0                   # the nodes of the roof
    cv = np.ones((9, 9), np.uint8)
    cv[5:8, 5:8] = 0
    cv[6, 6] = 1                         # the patio
    return _dem(side=10, z=z, cell_valid=cv)


def test_navmap_keeps_only_the_part_connected_to_a_point():
    dem = _roofs_dem()
    every = navmap_from_dem(dem, max_slope_deg=30.0)
    at_origin = navmap_from_dem(dem, max_slope_deg=30.0, keep=(4.0, -4.0))
    # the roof and the patio are left out
    assert len(every.navcels_v0) - len(at_origin.navcels_v0) == 2 * (9 + 1)
    assert max(at_origin.positions_z) == 0.0
    on_the_roof = navmap_from_dem(dem, max_slope_deg=30.0, keep=(-2.0, -2.0))
    assert len(on_the_roof.navcels_v0) == 2 * 9
    assert min(on_the_roof.positions_z) == 10.0


def test_a_point_on_a_hole_takes_the_nearest_cell_and_largest_takes_the_most_cells():
    dem = _roofs_dem()
    ok = np.ones((9, 9), bool)
    ok[5:8, 5:8] = False
    ok[6, 6] = True
    patio = keep_connected(dem, ok, (2.5, 2.5))   # the centre of the patio's cell (6, 6)
    assert patio.sum() == 1 and patio[6, 6]
    near = keep_connected(dem, ok, (1.4, 1.4))   # on the hole at cell (5, 5): nearest is ground
    assert near.sum() == 81 - 9
    assert keep_connected(dem, ok, 'largest').sum() == 81 - 9
    with pytest.raises(ValueError):
        keep_connected(dem, ok, 'biggest')
    assert not keep_connected(dem, np.zeros((9, 9), bool), 'largest').any()
