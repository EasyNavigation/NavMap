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
Unit tests for navmap_tools.geo.pnoa -- no network involved.

`load_pnoa_mosaic`'s chunk fetching is monkeypatched so no real WMS request
happens here; the actual download is covered by the end-to-end smoke test
instead (see easynav_gis_tool.md).
"""

from navmap_tools.geo import pnoa as pnoa_mod
from navmap_tools.geo.pnoa import load_pnoa_mosaic, PnoaMosaic
from navmap_tools.geo.projection import BBox

import numpy as np

from PIL import Image

import pytest


# ---------------------------------------------------------------------------
# PnoaMosaic.sample
# ---------------------------------------------------------------------------

def _solid_mosaic(color, origin_x=-100.0, origin_y=100.0, pixel_size_m=1.0, size=200):
    image = np.zeros((size, size, 3), dtype=np.uint8)
    image[:, :] = color
    return PnoaMosaic(image=image, origin_x=origin_x, origin_y=origin_y, pixel_size_m=pixel_size_m)


def test_sample_solid_color_returns_that_color():
    mosaic = _solid_mosaic((10, 20, 30))
    r, g, b = mosaic.sample(0.0, 0.0)  # (0, 0) in EPSG:3857, well inside the mosaic
    assert (r, g, b) == (10, 20, 30)


def test_sample_clamps_outside_mosaic_bounds():
    mosaic = _solid_mosaic((1, 2, 3), origin_x=-1.0, origin_y=1.0, pixel_size_m=0.1, size=4)
    r, g, b = mosaic.sample(50.0, 50.0)  # far outside the tiny mosaic footprint
    assert (r, g, b) == (1, 2, 3)


def test_sample_returns_ints_in_range():
    image = np.random.default_rng(0).integers(0, 256, size=(8, 8, 3), dtype=np.uint8)
    mosaic = PnoaMosaic(image=image, origin_x=-4.0, origin_y=4.0, pixel_size_m=1.0)
    r, g, b = mosaic.sample(0.0, 0.0)
    for v in (r, g, b):
        assert isinstance(v, int)
        assert 0 <= v <= 255


def test_sample_interpolates_between_distinct_neighbors():
    # Two horizontally adjacent columns of different flat colors: a sample
    # landing between pixel centers should be a blend, not exactly either.
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    image[:, :2] = (0, 0, 0)
    image[:, 2:] = (200, 200, 200)
    mosaic = PnoaMosaic(image=image, origin_x=0.0, origin_y=4.0, pixel_size_m=1.0)
    # px = 2.0 lands exactly on the boundary column index -- use px=1.5-ish.
    r, g, b = mosaic.sample(*_lonlat_for_px(mosaic, px=1.5, py=1.5))
    assert 0 < r < 200


def _lonlat_for_px(mosaic, px, py):
    from pyproj import Transformer
    to_lonlat = Transformer.from_crs('EPSG:3857', 'EPSG:4326', always_xy=True)
    mx = mosaic.origin_x + px * mosaic.pixel_size_m
    my = mosaic.origin_y - py * mosaic.pixel_size_m
    return to_lonlat.transform(mx, my)


# ---------------------------------------------------------------------------
# load_pnoa_mosaic -- validation
# ---------------------------------------------------------------------------

def test_load_pnoa_mosaic_rejects_non_positive_resolution():
    bbox = BBox(west=-0.001, south=-0.001, east=0.001, north=0.001)
    with pytest.raises(ValueError, match='resolution_m_per_px'):
        load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=0.0)
    with pytest.raises(ValueError, match='resolution_m_per_px'):
        load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=-0.25)


def test_load_pnoa_mosaic_rejects_too_large_a_mosaic():
    # A large bbox at a fine resolution needs far more than _MAX_TOTAL_PIXELS.
    bbox = BBox(west=-1.0, south=-1.0, east=1.0, north=1.0)
    with pytest.raises(ValueError, match='px mosaic'):
        load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=0.25)


# ---------------------------------------------------------------------------
# load_pnoa_mosaic -- chunk fetching/stitching (network calls monkeypatched)
# ---------------------------------------------------------------------------

def _fake_fetch_chunk_factory(tmp_path, calls):
    def fake_fetch_chunk(x0, y0, x1, y1, width_px, height_px, cache, force=False):
        # calls.append is not atomic across threads (ThreadPoolExecutor runs
        # these concurrently), so filenames/colors must not depend on
        # len(calls) -- derive both from the (deterministic) chunk bbox
        # instead. PNG (not JPEG) so the exact pixel values survive the
        # round trip through disk.
        calls.append((x0, y0, x1, y1, width_px, height_px))
        color = (int(x0) % 256, int(y1) % 256, 0)
        img = np.zeros((height_px, width_px, 3), dtype=np.uint8)
        img[:, :] = color
        path = tmp_path / f'chunk_{x0:.3f}_{y1:.3f}.png'
        Image.fromarray(img, mode='RGB').save(path)
        return path

    return fake_fetch_chunk


def test_load_pnoa_mosaic_single_chunk_for_small_bbox(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(pnoa_mod, '_fetch_chunk', _fake_fetch_chunk_factory(tmp_path, calls))

    bbox = BBox(west=-0.001, south=-0.001, east=0.001, north=0.001)
    mosaic = load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=1.0)

    assert len(calls) == 1
    assert mosaic.pixel_size_m == 1.0
    assert mosaic.image.shape[2] == 3


def test_load_pnoa_mosaic_splits_into_multiple_chunks_when_over_wms_limit(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(pnoa_mod, '_fetch_chunk', _fake_fetch_chunk_factory(tmp_path, calls))
    # Force chunking with a small bbox by shrinking the server's own per-call cap.
    monkeypatch.setattr(pnoa_mod, '_MAX_WMS_PX', 50)

    bbox = BBox(west=-0.0009, south=-0.0009, east=0.0009, north=0.0009)
    mosaic = load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=1.0)

    assert len(calls) > 1
    total_requested_px = sum(w * h for (_, _, _, _, w, h) in calls)
    assert total_requested_px >= mosaic.image.shape[0] * mosaic.image.shape[1]


def test_load_pnoa_mosaic_top_left_pixel_matches_first_chunk(monkeypatch, tmp_path):
    from pyproj import Transformer

    monkeypatch.setattr(pnoa_mod, '_MAX_WMS_PX', 50)

    to_web_mercator = Transformer.from_crs('EPSG:4326', 'EPSG:3857', always_xy=True)
    bbox = BBox(west=-0.0009, south=-0.0009, east=0.0009, north=0.0009)
    west_edge, _ = to_web_mercator.transform(bbox.west, bbox.south)
    _, north_edge = to_web_mercator.transform(bbox.east, bbox.north)

    def fake_fetch_chunk(x0, y0, x1, y1, width_px, height_px, cache, force=False):
        # The chunk covering the mosaic's north-west corner is always
        # (row=0, col=0); paint every other chunk a different color so a mixup
        # would be visible at pixel (0, 0).
        is_top_left = abs(y1 - north_edge) < 1e-6 and abs(x0 - west_edge) < 1e-6
        color = (255, 0, 0) if is_top_left else (0, 255, 0)
        img = np.zeros((height_px, width_px, 3), dtype=np.uint8)
        img[:, :] = color
        path = tmp_path / f'{x0:.1f}_{y1:.1f}.png'
        Image.fromarray(img, mode='RGB').save(path)
        return path

    monkeypatch.setattr(pnoa_mod, '_fetch_chunk', fake_fetch_chunk)
    mosaic = load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=1.0)

    assert tuple(mosaic.image[0, 0]) == (255, 0, 0)


def test_load_pnoa_mosaic_reports_progress_for_many_chunks(monkeypatch, tmp_path, capsys):
    calls = []
    monkeypatch.setattr(pnoa_mod, '_fetch_chunk', _fake_fetch_chunk_factory(tmp_path, calls))
    monkeypatch.setattr(pnoa_mod, '_MAX_WMS_PX', 10)

    bbox = BBox(west=-0.0009, south=-0.0009, east=0.0009, north=0.0009)
    load_pnoa_mosaic(bbox, cache=None, resolution_m_per_px=1.0)

    assert len(calls) >= 20
    assert 'chunks' in capsys.readouterr().err
