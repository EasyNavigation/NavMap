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
Unit tests for navmap_tools.geo.dem -- no network involved.

Network-touching functions (`fetch_tile_path`, `load_dem_grid`'s HTTP path)
are exercised via monkeypatching so no real download happens here; the
actual download is covered by the end-to-end smoke test instead.
"""

from navmap_tools.geo import dem as dem_mod
from navmap_tools.geo.dem import DemGrid, load_dem_grid, tile_key, tile_url
from navmap_tools.geo.projection import BBox

import numpy as np

from PIL import Image, TiffImagePlugin, TiffTags, UnidentifiedImageError

import pytest


# ---------------------------------------------------------------------------
# tile_key / tile_url
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    'lat,lon,expected',
    [
        (40, -4, 'Copernicus_DSM_COG_10_N40_00_W004_00_DEM'),
        (0, 0, 'Copernicus_DSM_COG_10_N00_00_E000_00_DEM'),
        (-1, -1, 'Copernicus_DSM_COG_10_S01_00_W001_00_DEM'),
        (89, 179, 'Copernicus_DSM_COG_10_N89_00_E179_00_DEM'),
        (-90, -180, 'Copernicus_DSM_COG_10_S90_00_W180_00_DEM'),
    ],
)
def test_tile_key_matches_copernicus_naming(lat, lon, expected):
    assert tile_key(lat, lon) == expected


def test_tile_url_embeds_the_key_twice():
    url = tile_url(40, -4)
    key = tile_key(40, -4)
    assert url == f'https://copernicus-dem-30m.s3.amazonaws.com/{key}/{key}.tif'


# ---------------------------------------------------------------------------
# DemGrid.sample
# ---------------------------------------------------------------------------

def _grid_2x2():
    # west=0, north=1, pixel size 1 degree in both axes:
    # row0 (north, lat~1): [10, 20]
    # row1 (south, lat~0): [30, 40]
    return DemGrid(
        elevation=np.array([[10.0, 20.0], [30.0, 40.0]], dtype=np.float32),
        west=0.0, north=1.0, pixel_size_lon=1.0, pixel_size_lat=1.0,
    )


def test_sample_at_grid_corners():
    grid = _grid_2x2()
    assert grid.sample(0.0, 1.0) == pytest.approx(10.0)
    assert grid.sample(1.0, 1.0) == pytest.approx(20.0)
    assert grid.sample(0.0, 0.0) == pytest.approx(30.0)
    assert grid.sample(1.0, 0.0) == pytest.approx(40.0)


def test_sample_bilinear_center():
    grid = _grid_2x2()
    assert grid.sample(0.5, 0.5) == pytest.approx((10 + 20 + 30 + 40) / 4)


def test_sample_clamps_outside_west_south():
    grid = _grid_2x2()
    assert grid.sample(-10.0, -10.0) == pytest.approx(30.0)


def test_sample_clamps_outside_east_north():
    grid = _grid_2x2()
    assert grid.sample(10.0, 10.0) == pytest.approx(20.0)


def test_sample_single_pixel_grid_is_constant():
    grid = DemGrid(
        elevation=np.array([[42.0]], dtype=np.float32),
        west=0.0, north=1.0, pixel_size_lon=1.0, pixel_size_lat=1.0,
    )
    assert grid.sample(0.0, 0.0) == pytest.approx(42.0)
    assert grid.sample(100.0, -100.0) == pytest.approx(42.0)


# ---------------------------------------------------------------------------
# load_dem_grid (monkeypatched, no network)
# ---------------------------------------------------------------------------

def test_load_dem_grid_raises_when_no_coverage(monkeypatch):
    def always_missing(lat_floor, lon_floor, cache, force=False):
        raise FileNotFoundError('no tile')

    monkeypatch.setattr(dem_mod, 'fetch_tile_path', always_missing)
    with pytest.raises(RuntimeError, match='no DEM coverage'):
        load_dem_grid(BBox(west=0.0, south=0.0, east=0.5, north=0.5), cache=None)


def test_load_dem_grid_raises_on_mismatched_tile_shapes(monkeypatch):
    def fake_fetch(lat_floor, lon_floor, cache, force=False):
        return f'{lat_floor}_{lon_floor}'

    def fake_read(path):
        # Two tiles with different shapes -> should be rejected, not mosaicked.
        if path == '0_0':
            return np.zeros((10, 10), dtype=np.float32), 0.0, 1.0, 0.1, 0.1
        return np.zeros((20, 20), dtype=np.float32), 1.0, 1.0, 0.05, 0.05

    monkeypatch.setattr(dem_mod, 'fetch_tile_path', fake_fetch)
    monkeypatch.setattr(dem_mod, '_read_tile_array', fake_read)
    with pytest.raises(RuntimeError, match='different grid resolution'):
        load_dem_grid(BBox(west=0.0, south=0.0, east=1.5, north=0.5), cache=None)


def test_load_dem_grid_mosaics_single_tile(monkeypatch):
    def fake_fetch(lat_floor, lon_floor, cache, force=False):
        return 'only'

    def fake_read(path):
        return np.arange(4, dtype=np.float32).reshape(2, 2), 0.0, 1.0, 0.5, 0.5

    monkeypatch.setattr(dem_mod, 'fetch_tile_path', fake_fetch)
    monkeypatch.setattr(dem_mod, '_read_tile_array', fake_read)
    grid = load_dem_grid(BBox(west=0.1, south=0.1, east=0.2, north=0.2), cache=None)
    assert grid.elevation.shape == (2, 2)
    assert grid.west == pytest.approx(0.0)
    assert grid.north == pytest.approx(1.0)


def test_load_dem_grid_skips_ocean_tiles_but_keeps_land(monkeypatch):
    def fake_fetch(lat_floor, lon_floor, cache, force=False):
        if lon_floor == 0:
            raise FileNotFoundError('ocean')
        return 'land'

    def fake_read(path):
        return np.ones((2, 2), dtype=np.float32), 1.0, 1.0, 1.0, 1.0

    monkeypatch.setattr(dem_mod, 'fetch_tile_path', fake_fetch)
    monkeypatch.setattr(dem_mod, '_read_tile_array', fake_read)
    grid = load_dem_grid(BBox(west=0.1, south=0.1, east=1.9, north=0.9), cache=None)
    # Two 1x1-degree tile columns (lon 0 ocean, lon 1 land) mosaicked together.
    assert grid.elevation.shape == (2, 4)


# ---------------------------------------------------------------------------
# _read_tile_array (real GeoTIFF files written with Pillow)
# ---------------------------------------------------------------------------

_PREDICTOR_TAG = 317
_FLOATING_POINT_PREDICTOR = 3


def _write_geotiff(path, arr, scale, tiepoint, **save_args):
    ifd = TiffImagePlugin.ImageFileDirectory_v2()
    if scale is not None:
        ifd[dem_mod._MODEL_PIXEL_SCALE_TAG] = scale
        ifd.tagtype[dem_mod._MODEL_PIXEL_SCALE_TAG] = TiffTags.DOUBLE
    if tiepoint is not None:
        ifd[dem_mod._MODEL_TIEPOINT_TAG] = tiepoint
        ifd.tagtype[dem_mod._MODEL_TIEPOINT_TAG] = TiffTags.DOUBLE
    if 'predictor' in save_args:
        ifd[_PREDICTOR_TAG] = save_args.pop('predictor')
    Image.fromarray(arr).save(path, tiffinfo=ifd, **save_args)
    return path


def _elevations(rows=48, cols=64):
    # Covers negative (below sea level) and fractional values
    return (np.arange(rows * cols, dtype=np.float32).reshape(rows, cols) * 0.25 - 100.0)


def test_read_tile_array_returns_values_and_georeference(tmp_path):
    arr = _elevations()
    path = _write_geotiff(
        tmp_path / 'dem.tif', arr, (0.5, 0.25, 0.0), (0.0, 0.0, 0.0, -4.0, 41.0, 0.0))

    out, ox, oy, sx, sy = dem_mod._read_tile_array(path)

    assert out.dtype == np.float32
    assert out.shape == (48, 64)
    np.testing.assert_array_equal(out, arr)
    assert (ox, oy) == (-4.0, 41.0)
    assert (sx, sy) == (0.5, 0.25)


def test_read_tile_array_decodes_copernicus_encoding(tmp_path):
    # Copernicus GLO-30 tiles: float32, DEFLATE, floating-point predictor
    arr = _elevations()
    path = _write_geotiff(
        tmp_path / 'dem.tif', arr, (1 / 3600, 1 / 3600, 0.0),
        (0.0, 0.0, 0.0, -4.0, 41.0, 0.0),
        compression='tiff_adobe_deflate', predictor=_FLOATING_POINT_PREDICTOR)
    with Image.open(path) as written:
        assert written.tag_v2.get(_PREDICTOR_TAG) == _FLOATING_POINT_PREDICTOR

    out, ox, oy, sx, sy = dem_mod._read_tile_array(path)

    np.testing.assert_array_equal(out, arr)
    assert (ox, oy) == (-4.0, 41.0)
    assert sx == pytest.approx(1 / 3600)
    assert sy == pytest.approx(1 / 3600)


def test_read_tile_array_converts_integer_elevations_to_float(tmp_path):
    arr = np.array([[-5, 1200], [300, 4000]], dtype=np.int16)
    path = _write_geotiff(
        tmp_path / 'dem.tif', arr, (1.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.0, 1.0, 0.0))

    out, _, _, _, _ = dem_mod._read_tile_array(path)

    assert out.dtype == np.float32
    np.testing.assert_array_equal(out, arr.astype(np.float32))


@pytest.mark.parametrize(
    'scale,tiepoint',
    [
        (None, (0.0, 0.0, 0.0, -4.0, 41.0, 0.0)),
        ((0.5, 0.5, 0.0), None),
        (None, None),
    ],
)
def test_read_tile_array_rejects_tiff_without_georeference(tmp_path, scale, tiepoint):
    path = _write_geotiff(tmp_path / 'plain.tif', _elevations(4, 4), scale, tiepoint)
    with pytest.raises(ValueError, match='not a GeoTIFF'):
        dem_mod._read_tile_array(path)


def test_read_tile_array_rejects_non_tiff_file(tmp_path):
    path = tmp_path / 'error.tif'
    path.write_text('<Error><Code>AccessDenied</Code></Error>')
    with pytest.raises(UnidentifiedImageError):
        dem_mod._read_tile_array(path)


def test_load_dem_grid_reads_real_tiles_from_disk(tmp_path, monkeypatch):
    # One 2x2 tile covering lon [0, 1], lat [0, 1], no mocked reader
    arr = np.array([[10.0, 20.0], [30.0, 40.0]], dtype=np.float32)
    path = _write_geotiff(
        tmp_path / 'tile.tif', arr, (0.5, 0.5, 0.0), (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        compression='tiff_adobe_deflate', predictor=_FLOATING_POINT_PREDICTOR)

    def fake_fetch(lat_floor, lon_floor, cache, force=False):
        return path

    monkeypatch.setattr(dem_mod, 'fetch_tile_path', fake_fetch)
    grid = load_dem_grid(BBox(west=0.1, south=0.1, east=0.4, north=0.4), cache=None)

    np.testing.assert_array_equal(grid.elevation, arr)
    assert grid.west == pytest.approx(0.0)
    assert grid.north == pytest.approx(1.0)
    assert grid.pixel_size_lon == pytest.approx(0.5)
    assert grid.pixel_size_lat == pytest.approx(0.5)
