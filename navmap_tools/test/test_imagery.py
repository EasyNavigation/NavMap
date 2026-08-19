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

"""
Unit tests for navmap_tools.geo.imagery -- no network involved.

`load_imagery_mosaic`'s tile fetching is monkeypatched so no real HTTP
request happens here; the actual download is covered by the end-to-end
smoke test instead.
"""

import hashlib

from navmap_tools.geo import imagery as imagery_mod
from navmap_tools.geo.imagery import (
    find_available_zoom,
    ImageryMosaic,
    load_imagery_mosaic,
    lonlat_to_tile,
    native_resolution_m_per_px,
)
from navmap_tools.geo.projection import BBox

import numpy as np

import pytest


# ---------------------------------------------------------------------------
# lonlat_to_tile
# ---------------------------------------------------------------------------

def test_origin_is_top_left_tile_at_zoom_0():
    x, y = lonlat_to_tile(-180.0, 85.05112878, 0)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_equator_prime_meridian_is_map_center_at_any_zoom():
    for zoom in (0, 5, 19):
        x, y = lonlat_to_tile(0.0, 0.0, zoom)
        n = 2.0 ** zoom
        assert x == pytest.approx(n / 2.0)
        assert y == pytest.approx(n / 2.0)


def test_tile_x_increases_eastward():
    x_west, _ = lonlat_to_tile(-10.0, 0.0, 10)
    x_east, _ = lonlat_to_tile(10.0, 0.0, 10)
    assert x_east > x_west


# ---------------------------------------------------------------------------
# native_resolution_m_per_px
# ---------------------------------------------------------------------------

def test_native_resolution_at_zoom0_equator_matches_known_constant():
    # Standard Web Mercator ground resolution constant.
    assert native_resolution_m_per_px(0, 0.0) == pytest.approx(156543.03392804097)


def test_native_resolution_halves_per_zoom_level():
    r10 = native_resolution_m_per_px(10, 0.0)
    r11 = native_resolution_m_per_px(11, 0.0)
    assert r11 == pytest.approx(r10 / 2.0)


def test_native_resolution_finer_at_higher_latitude():
    # Web Mercator: a fixed zoom covers less real distance per pixel as
    # you move away from the equator (cos(lat) shrinks).
    equator = native_resolution_m_per_px(15, 0.0)
    high_lat = native_resolution_m_per_px(15, 60.0)
    assert high_lat < equator


def test_native_resolution_at_poles_is_zero():
    assert native_resolution_m_per_px(15, 90.0) == pytest.approx(0.0, abs=1e-6)


def test_native_resolution_symmetric_in_latitude_sign():
    north = native_resolution_m_per_px(12, 40.0)
    south = native_resolution_m_per_px(12, -40.0)
    assert north == pytest.approx(south)


def test_tile_y_increases_southward():
    _, y_north = lonlat_to_tile(0.0, 10.0, 10)
    _, y_south = lonlat_to_tile(0.0, -10.0, 10)
    assert y_south > y_north


def test_latitude_is_clamped_to_web_mercator_limit():
    # Must not raise (math domain error) at/above the poles.
    lonlat_to_tile(0.0, 90.0, 5)
    lonlat_to_tile(0.0, -90.0, 5)


# ---------------------------------------------------------------------------
# ImageryMosaic.sample
# ---------------------------------------------------------------------------

def _solid_mosaic(color, zoom=10, size=4):
    image = np.zeros((size, size, 3), dtype=np.uint8)
    image[:, :] = color
    return ImageryMosaic(image=image, zoom=zoom, tile_x0=0, tile_y0=0)


def test_sample_solid_color_returns_that_color():
    mosaic = _solid_mosaic((10, 20, 30))
    lon, lat = 5.0, 5.0  # anywhere within the tile range
    r, g, b = mosaic.sample(lon, lat)
    assert (r, g, b) == (10, 20, 30)


def test_sample_clamps_outside_mosaic_bounds():
    mosaic = _solid_mosaic((1, 2, 3))
    # Far outside the tile grid entirely.
    r, g, b = mosaic.sample(-179.0, 89.0)
    assert (r, g, b) == (1, 2, 3)


def test_sample_returns_ints_in_range():
    image = np.random.default_rng(0).integers(0, 256, size=(8, 8, 3), dtype=np.uint8)
    mosaic = ImageryMosaic(image=image, zoom=12, tile_x0=100, tile_y0=200)
    x, y = lonlat_to_tile(3.0, 45.0, 12)
    r, g, b = mosaic.sample(3.0, 45.0)
    for v in (r, g, b):
        assert isinstance(v, int)
        assert 0 <= v <= 255
    assert x is not None and y is not None  # sanity: coordinates are finite


# ---------------------------------------------------------------------------
# load_imagery_mosaic
# ---------------------------------------------------------------------------

def test_load_imagery_mosaic_rejects_bad_zoom():
    with pytest.raises(ValueError, match='zoom'):
        load_imagery_mosaic(BBox(-1, -1, 1, 1), cache=None, zoom=-1)
    with pytest.raises(ValueError, match='zoom'):
        load_imagery_mosaic(BBox(-1, -1, 1, 1), cache=None, zoom=24)


def test_load_imagery_mosaic_rejects_too_many_tiles():
    # A huge bbox at a high zoom needs far more than _MAX_TILES tiles.
    with pytest.raises(ValueError, match='tiles'):
        load_imagery_mosaic(BBox(-170, -80, 170, 80), cache=None, zoom=18)


def test_load_imagery_mosaic_stitches_tiles_in_correct_positions(monkeypatch, tmp_path):
    calls = []

    def fake_fetch(z, x, y, cache, force=False):
        calls.append((z, x, y))
        color = (x % 256, y % 256, 0)
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        img[:, :] = color
        path = tmp_path / f'{z}_{x}_{y}.png'
        from PIL import Image
        Image.fromarray(img, mode='RGB').save(path)
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)

    # A small bbox that spans exactly 2x2 tiles at a moderate zoom.
    # auto_fallback=False: isolated from find_available_zoom's own probe
    # fetch, tested separately below.
    bbox = BBox(west=-0.01, south=-0.01, east=0.01, north=0.01)
    mosaic = load_imagery_mosaic(bbox, cache=None, zoom=15, auto_fallback=False)

    assert mosaic.image.shape == (512, 512, 3)
    assert len(calls) == 4
    # Top-left tile pixel matches the (tile_x0, tile_y0) tile's synthetic color.
    top_left_color = tuple(mosaic.image[0, 0])
    assert top_left_color == (mosaic.tile_x0 % 256, mosaic.tile_y0 % 256, 0)


# ---------------------------------------------------------------------------
# _is_placeholder_tile / find_available_zoom / load_imagery_mosaic auto_fallback
#
# Esri serves a byte-identical "Map data not yet available" placeholder tile
# (not a 404) for coordinates it has no imagery for at a given zoom. A real
# generated world showed that placeholder's text repeated across every mesh
# cell after --zoom's default was raised to 23 for an area with no real
# zoom-23 coverage. See easynav_gis_tool.md for the full diagnosis.
# ---------------------------------------------------------------------------

def test_is_placeholder_tile_detects_known_hash(tmp_path, monkeypatch):
    content = b'pretend this is the exact known placeholder bytes'
    digest = hashlib.md5(content).hexdigest()
    monkeypatch.setattr(imagery_mod, '_PLACEHOLDER_MD5_HASHES', frozenset({digest}))
    path = tmp_path / 'tile.jpg'
    path.write_bytes(content)
    assert imagery_mod._is_placeholder_tile(path) is True


def test_is_placeholder_tile_false_for_unknown_content(tmp_path):
    path = tmp_path / 'tile.jpg'
    path.write_bytes(b'totally real imagery, trust me')
    assert imagery_mod._is_placeholder_tile(path) is False


def test_find_available_zoom_returns_max_zoom_when_already_real(monkeypatch, tmp_path):
    def fake_fetch(z, x, y, cache, force=False):
        path = tmp_path / f'{z}.jpg'
        path.write_bytes(b'real imagery')
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)
    zoom = find_available_zoom(0.0, 0.0, cache=None, max_zoom=20)
    assert zoom == 20


def test_find_available_zoom_steps_down_past_placeholders(monkeypatch, tmp_path):
    placeholder = b'placeholder bytes'
    monkeypatch.setattr(
        imagery_mod, '_PLACEHOLDER_MD5_HASHES',
        frozenset({hashlib.md5(placeholder).hexdigest()}))

    def fake_fetch(z, x, y, cache, force=False):
        path = tmp_path / f'{z}.jpg'
        path.write_bytes(placeholder if z >= 18 else f'real-{z}'.encode())
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)
    zoom = find_available_zoom(0.0, 0.0, cache=None, max_zoom=20)
    assert zoom == 17


def test_find_available_zoom_falls_back_to_min_zoom_if_all_placeholder(monkeypatch, tmp_path):
    placeholder = b'placeholder bytes'
    monkeypatch.setattr(
        imagery_mod, '_PLACEHOLDER_MD5_HASHES',
        frozenset({hashlib.md5(placeholder).hexdigest()}))

    def fake_fetch(z, x, y, cache, force=False):
        path = tmp_path / f'{z}.jpg'
        path.write_bytes(placeholder)
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)
    zoom = find_available_zoom(0.0, 0.0, cache=None, max_zoom=15, min_zoom=12)
    assert zoom == 12


def test_load_imagery_mosaic_auto_fallback_adjusts_zoom_and_reports(monkeypatch, tmp_path, capsys):
    placeholder = b'placeholder bytes'
    monkeypatch.setattr(
        imagery_mod, '_PLACEHOLDER_MD5_HASHES',
        frozenset({hashlib.md5(placeholder).hexdigest()}))

    def fake_fetch(z, x, y, cache, force=False):
        from PIL import Image
        path = tmp_path / f'{z}_{x}_{y}.jpg'
        if z == 20:
            path.write_bytes(placeholder)
        else:
            img = np.zeros((256, 256, 3), dtype=np.uint8)
            Image.fromarray(img, mode='RGB').save(path)
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)
    bbox = BBox(west=-0.01, south=-0.01, east=0.01, north=0.01)
    mosaic = load_imagery_mosaic(bbox, cache=None, zoom=20)

    assert mosaic.zoom == 19
    assert 'zoom 19 instead' in capsys.readouterr().err


def test_load_imagery_mosaic_auto_fallback_disabled_keeps_requested_zoom(
        monkeypatch, tmp_path):
    # With auto_fallback=False, find_available_zoom's probe (and therefore
    # _is_placeholder_tile) is never consulted at all, so every fetched tile
    # -- even a real placeholder -- is mosaicked as-is at the requested zoom.
    def fake_fetch(z, x, y, cache, force=False):
        from PIL import Image
        path = tmp_path / f'{z}_{x}_{y}.jpg'
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        Image.fromarray(img, mode='RGB').save(path)
        return path

    monkeypatch.setattr(imagery_mod, '_fetch_tile_path', fake_fetch)
    bbox = BBox(west=-0.01, south=-0.01, east=0.01, north=0.01)
    mosaic = load_imagery_mosaic(bbox, cache=None, zoom=20, auto_fallback=False)
    assert mosaic.zoom == 20
