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
"""
Unit tests for navmap_tools.geo.ign -- no network involved.

The WCS download is monkeypatched (`get_to_file` writes a canned ArcGrid body), so the
request URL, the caching and the sampling are checked without touching the IGN service.
"""

from navmap_tools.geo import ign as ign_mod
from navmap_tools.geo.cache import DiskCache
from navmap_tools.geo.ign import IgnBlock, IgnDem, load_ign_dem, parse_asc
from navmap_tools.geo.projection import BBox

import numpy as np

import pytest


def _asc(values, xll=440000.0, yll=4470000.0, cell=5.0, nodata=-9999, center=False):
    rows = '\n'.join(' '.join(str(v) for v in row) for row in values)
    corner = 'xllcenter' if center else 'xllcorner'
    corner_y = 'yllcenter' if center else 'yllcorner'
    return (
        f'ncols {len(values[0])}\nnrows {len(values)}\n{corner} {xll}\n{corner_y} {yll}\n'
        f'cellsize {cell}\nNODATA_value {nodata}\n{rows}\n')


def test_parse_asc_reads_header_and_values_north_first():
    block = parse_asc(_asc([[1, 2, 3], [4, 5, 6]]))
    assert block.values.shape == (2, 3)
    assert block.values[0, 0] == 1 and block.values[1, 2] == 6   # row 0 is the northern one
    assert (block.x_min, block.y_min, block.cell) == (440000.0, 4470000.0, 5.0)


def test_parse_asc_with_centre_corner_shifts_half_a_cell():
    block = parse_asc(_asc([[1, 2], [3, 4]], center=True))
    assert block.x_min == 440000.0 - 2.5
    assert block.y_min == 4470000.0 - 2.5


def test_parse_asc_marks_nodata_as_nan():
    block = parse_asc(_asc([[1, -9999], [3, 4]]))
    assert np.isnan(block.values[0, 1]) and block.values[0, 0] == 1


def test_parse_asc_skips_a_multipart_preamble():
    body = '--boundary\nContent-Type: application/asc\n\n' + _asc([[7]]) + '--boundary--\n'
    assert parse_asc(body).values[0, 0] == 7


@pytest.mark.parametrize('body', ['nothing here', 'ncols 2\nnrows 2\ncellsize 5\n1 2 3'])
def test_parse_asc_rejects_bad_bodies(body):
    with pytest.raises(ValueError):
        parse_asc(body)


def _block():
    # 3 x 3 cells of 5 m, UTM (0, 0)-(15, 15) in the block's own frame; north row first
    return IgnBlock(
        x_min=0.0, y_min=0.0, cell=5.0,
        values=np.array([[3, 3, 3], [2, 2, 2], [1, 1, 1]], dtype=np.float32))


def test_block_sample_is_bilinear_between_cell_centres():
    value, ok = _block().sample_array(np.array([7.5]), np.array([7.5]))   # centre of the middle
    assert ok[0] and value[0] == pytest.approx(2.0)
    value, ok = _block().sample_array(np.array([7.5]), np.array([10.0]))  # north of the middle
    assert ok[0] and value[0] == pytest.approx(2.5)


def test_block_sample_outside_is_not_answered():
    _, ok = _block().sample_array(np.array([-1.0, 7.5]), np.array([7.5, 99.0]))
    assert not ok.any()


def test_block_sample_next_to_missing_data_is_not_answered():
    block = _block()
    block.values[1, 1] = np.nan
    _, ok = block.sample_array(np.array([7.5, 2.5]), np.array([7.5, 2.5]))
    assert not ok[0] and ok[1]   # the centre needs the missing cell, the corner does not


def test_dem_uses_the_first_block_that_answers():
    far = IgnBlock(x_min=1e6, y_min=0.0, cell=5.0, values=np.zeros((2, 2), dtype=np.float32))
    dem = IgnDem(blocks=[far, _block()])
    x, y = 7.5, 7.5
    lon, lat = ign_mod._TO_UTM30.transform(x, y, direction='INVERSE')
    assert dem.sample(lon, lat) == pytest.approx(2.0, abs=1e-3)


def test_dem_without_data_says_where():
    dem = IgnDem(blocks=[_block()])
    with pytest.raises(RuntimeError, match='only covers Spain'):
        dem.sample(0.0, 0.0)


def test_pixel_size_is_the_cell_in_degrees():
    assert IgnDem(cell_m=5.0).pixel_size_lat * 111320.0 == pytest.approx(5.0)


def test_load_ign_dem_requests_blocks_once_and_caches_them(monkeypatch, tmp_path):
    urls = []

    def fake_get(url, dest, timeout, allow_404=False):
        urls.append(url)
        dest.write_text(_asc([[10, 10], [10, 10]]))
        return True

    monkeypatch.setattr(ign_mod, 'get_to_file', fake_get)
    cache = DiskCache(tmp_path)
    bbox = BBox(west=-3.7010, south=40.4000, east=-3.7000, north=40.4010)   # ~100 m
    dem = load_ign_dem(bbox, cache)
    assert len(urls) == len(dem.blocks) == 1
    url = urls[0]
    assert url.startswith('https://servicios.idee.es/wcs-inspire/mdt?service=WCS&version=2.0.1')
    assert 'coverageId=Elevacion25830_5' in url
    assert 'subset=x%28' in url and 'subset=y%28' in url
    assert 'format=application%2Fasc' in url
    load_ign_dem(bbox, cache)
    assert len(urls) == 1   # the second run came from the cache


def test_load_ign_dem_splits_a_large_area_in_blocks(monkeypatch, tmp_path):
    urls = []

    def fake_get(url, dest, timeout, allow_404=False):
        urls.append(url)
        dest.write_text(_asc([[0, 0], [0, 0]]))
        return True

    monkeypatch.setattr(ign_mod, 'get_to_file', fake_get)
    bbox = BBox(west=-3.72, south=40.39, east=-3.68, north=40.42)   # ~3.4 km x 3.3 km
    load_ign_dem(bbox, DiskCache(tmp_path), max_block_m=2000.0)
    assert len(urls) == 4
