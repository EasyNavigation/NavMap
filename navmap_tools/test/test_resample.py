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

"""Unit tests for navmap_tools.resample (grid-based NavMap resampling)."""

import sys

from navmap_ros_interfaces.msg import NavMapLayer, NavMapSurface
from navmap_tools.resample import (
    build_navmap, FREE, grid_from_navmap, LETHAL, main, read_navmap, read_yaml_grid,
    resample_occupancy, UNKNOWN, write_navmap,
)
import numpy as np
import pytest


def occ_layers(grid):
    return {'occupancy': (NavMapLayer.U8, np.array(grid, np.uint8))}


def run_main(monkeypatch, *args):
    monkeypatch.setattr(sys, 'argv', ['navmap_resample', *map(str, args)])
    main()


# ------------------------------ Block rules ------------------------------

def test_any_lethal_cell_makes_the_block_lethal():
    g = np.full((2, 2), FREE, np.uint8)
    g[1, 1] = LETHAL
    assert resample_occupancy(g, 2).tolist() == [[LETHAL]]


def test_lethal_wins_even_among_unknown_cells():
    g = np.full((2, 2), UNKNOWN, np.uint8)
    g[0, 0] = LETHAL
    assert resample_occupancy(g, 2).tolist() == [[LETHAL]]


def test_all_free_is_free():
    assert resample_occupancy(np.zeros((4, 4), np.uint8), 2).tolist() == [[0, 0], [0, 0]]


def test_half_known_free_is_free_less_is_unknown():
    half = np.array([[FREE, FREE], [UNKNOWN, UNKNOWN]], np.uint8)
    assert resample_occupancy(half, 2).tolist() == [[FREE]]
    quarter = np.array([[FREE, UNKNOWN], [UNKNOWN, UNKNOWN]], np.uint8)
    assert resample_occupancy(quarter, 2).tolist() == [[UNKNOWN]]


def test_costs_keep_the_highest_known_one():
    g = np.array([[10, 200], [UNKNOWN, 0]], np.uint8)
    assert resample_occupancy(g, 2).tolist() == [[200]]


def test_the_border_is_padded_with_unknown():
    # 3 x 3 cells by 2: the last row and column of blocks are mostly outside the map.
    g = np.zeros((3, 3), np.uint8)
    out = resample_occupancy(g, 2)
    assert out.shape == (2, 2)
    assert out[0, 0] == FREE
    assert out[0, 1] == FREE        # 2 of 4 cells inside, free: half known
    assert out[1, 1] == UNKNOWN     # 1 of 4 inside
    g[2, 2] = LETHAL
    assert resample_occupancy(g, 2)[1, 1] == LETHAL


def test_factor_one_keeps_the_grid():
    g = np.array([[0, 254], [255, 100]], np.uint8)
    assert resample_occupancy(g, 1).tolist() == g.tolist()


# ------------------------- NavMap structure / IO --------------------------

def test_the_navmap_has_the_grid_structure(tmp_path):
    msg = build_navmap(occ_layers([[0, 254, 255]]), (1.0, 2.0, 0.5), 0.2, 'map')
    assert len(msg.positions_x) == (3 + 1) * (1 + 1)
    assert len(msg.navcels_v0) == 2 * 3
    assert len(msg.surfaces) == 1 and len(msg.surfaces[0].navcels) == 6
    assert list(msg.layers[0].data_u8) == [0, 0, 254, 254, 255, 255]
    assert min(msg.positions_x) == pytest.approx(1.0)
    assert max(msg.positions_x) == pytest.approx(1.6)
    assert set(msg.positions_z) == {0.5}


def test_writing_and_reading_back_gives_the_same_grid(tmp_path):
    g = np.random.default_rng(0).choice([0, 50, 254, 255], size=(7, 5)).astype(np.uint8)
    path = tmp_path / 'm.navmap'
    write_navmap(build_navmap(occ_layers(g), (-3.0, 4.0, 0.0), 0.05, 'map'), str(path))
    layers, origin, res, frame = grid_from_navmap(read_navmap(str(path)))
    assert layers['occupancy'][1].tolist() == g.tolist()
    assert origin[:2] == pytest.approx((-3.0, 4.0))
    assert res == pytest.approx(0.05)
    assert frame == 'map'


def test_the_file_header_is_the_navmap_one(tmp_path):
    path = tmp_path / 'm.navmap'
    write_navmap(build_navmap(occ_layers([[0]]), (0, 0, 0), 1.0, 'map'), str(path))
    assert path.read_bytes()[:8] == b'NAVMAP\x00\x00'


def test_a_non_flat_navmap_is_rejected():
    msg = build_navmap(occ_layers([[0, 0]]), (0, 0, 0), 1.0, 'map')
    msg.positions_z[0] = 1.0
    with pytest.raises(SystemExit):
        grid_from_navmap(msg)


def test_a_navmap_with_two_surfaces_is_rejected():
    msg = build_navmap(occ_layers([[0, 0]]), (0, 0, 0), 1.0, 'map')
    msg.surfaces.append(NavMapSurface())
    with pytest.raises(SystemExit):
        grid_from_navmap(msg)


def test_other_layers_are_resampled_too(tmp_path, monkeypatch):
    layers = occ_layers(np.zeros((2, 2)))
    layers['height'] = (NavMapLayer.F32, np.array([[1.0, 2.0], [3.0, np.nan]]))
    layers['marks'] = (NavMapLayer.U8, np.array([[0, 7], [3, 0]], np.uint8))
    src, dst = tmp_path / 'in.navmap', tmp_path / 'out.navmap'
    write_navmap(build_navmap(layers, (0, 0, 0), 0.1, 'map'), str(src))
    run_main(monkeypatch, src, dst, 0.2)
    out, _, res, _ = grid_from_navmap(read_navmap(str(dst)))
    assert res == pytest.approx(0.2)
    assert out['height'][1][0, 0] == pytest.approx(2.0)   # mean of finite values
    assert out['marks'][1][0, 0] == 7                      # max


# ------------------------------- Command ----------------------------------

def write_yaml_map(tmp_path, pixels):
    """map_server trinary map; pixels with row 0 at the top, as in the image."""
    h, w = len(pixels), len(pixels[0])
    (tmp_path / 'm.pgm').write_bytes(
        f'P5\n{w} {h}\n255\n'.encode() + np.array(pixels, np.uint8).tobytes())
    (tmp_path / 'm.yaml').write_text(
        'image: m.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -2.0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return tmp_path / 'm.yaml'


def test_a_yaml_map_is_read_as_map_server_does(tmp_path):
    # top row: occupied, unknown; bottom row: free, free
    layers, origin, res, _ = read_yaml_grid(str(write_yaml_map(tmp_path, [[0, 205], [254, 254]])))
    occ = layers['occupancy']
    assert occ[0].tolist() == [FREE, FREE]          # row 0 is the bottom (origin)
    assert occ[1].tolist() == [LETHAL, UNKNOWN]
    assert origin[:2] == (-1.0, -2.0) and res == 0.05


def test_a_yaml_map_is_resampled_to_a_navmap(tmp_path, monkeypatch):
    pixels = [[254] * 4 for _ in range(4)]
    pixels[0][0] = 0  # one occupied cell (top-left)
    out = tmp_path / 'out.navmap'
    run_main(monkeypatch, write_yaml_map(tmp_path, pixels), out, 0.1)
    layers, origin, res, _ = grid_from_navmap(read_navmap(str(out)))
    assert res == pytest.approx(0.1)
    assert origin[:2] == pytest.approx((-1.0, -2.0))
    assert layers['occupancy'][1].tolist() == [[FREE, FREE], [LETHAL, FREE]]


@pytest.mark.parametrize('resolution', [0.07, 0.03, 0.125])
def test_a_resolution_not_a_multiple_is_rejected(tmp_path, monkeypatch, resolution):
    src = write_yaml_map(tmp_path, [[254, 254], [254, 254]])
    with pytest.raises(SystemExit):
        run_main(monkeypatch, src, tmp_path / 'out.navmap', resolution)
