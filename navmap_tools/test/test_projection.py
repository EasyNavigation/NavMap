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

"""Unit tests for navmap_tools.geo.projection -- no network involved."""

from navmap_tools.geo.projection import BBox, LocalProjection

import pytest


# ---------------------------------------------------------------------------
# Construction / validation
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('lat', [-90.0, 0.0, 40.3314, 90.0])
def test_valid_latitudes_accepted(lat):
    LocalProjection(lat, 0.0)


@pytest.mark.parametrize('lat', [-90.0001, 90.0001, 1000.0, -1000.0])
def test_invalid_latitudes_rejected(lat):
    with pytest.raises(ValueError, match='latitude'):
        LocalProjection(lat, 0.0)


@pytest.mark.parametrize('lon', [-180.0, 0.0, 179.9999, 180.0])
def test_valid_longitudes_accepted(lon):
    LocalProjection(0.0, lon)


@pytest.mark.parametrize('lon', [-180.0001, 180.0001, 1000.0])
def test_invalid_longitudes_rejected(lon):
    with pytest.raises(ValueError, match='longitude'):
        LocalProjection(0.0, lon)


# ---------------------------------------------------------------------------
# to_local / to_lonlat
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    'lat,lon', [(0.0, 0.0), (40.3314, -3.8356), (-33.45, -70.66), (89.0, 10.0)])
def test_center_maps_to_local_origin(lat, lon):
    proj = LocalProjection(lat, lon)
    x, y = proj.to_local(lon, lat)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize('lat,lon', [(0.0, 0.0), (40.3314, -3.8356), (-33.45, -70.66)])
def test_local_origin_maps_to_center(lat, lon):
    proj = LocalProjection(lat, lon)
    out_lon, out_lat = proj.to_lonlat(0.0, 0.0)
    assert out_lon == pytest.approx(lon, abs=1e-9)
    assert out_lat == pytest.approx(lat, abs=1e-9)


@pytest.mark.parametrize(
    'x,y', [(100.0, 0.0), (0.0, 100.0), (-250.0, 300.0), (0.0, 0.0), (5000.0, -5000.0)]
)
def test_local_to_lonlat_round_trips(x, y):
    proj = LocalProjection(40.3314, -3.8356)
    lon, lat = proj.to_lonlat(x, y)
    x2, y2 = proj.to_local(lon, lat)
    assert x2 == pytest.approx(x, abs=1e-3)
    assert y2 == pytest.approx(y, abs=1e-3)


def test_north_is_positive_y():
    proj = LocalProjection(0.0, 0.0)
    _, lat_north = proj.to_lonlat(0.0, 1000.0)
    _, lat_south = proj.to_lonlat(0.0, -1000.0)
    assert lat_north > lat_south


def test_east_is_positive_x():
    proj = LocalProjection(0.0, 0.0)
    lon_east, _ = proj.to_lonlat(1000.0, 0.0)
    lon_west, _ = proj.to_lonlat(-1000.0, 0.0)
    assert lon_east > lon_west


# ---------------------------------------------------------------------------
# square_bbox
# ---------------------------------------------------------------------------

def test_square_bbox_rejects_non_positive_size():
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='size_m'):
        proj.square_bbox(0.0)
    with pytest.raises(ValueError, match='size_m'):
        proj.square_bbox(-10.0)


@pytest.mark.parametrize('margin', [-0.1, 1.0, 1.5])
def test_square_bbox_rejects_invalid_margin_ratio(margin):
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='margin_ratio'):
        proj.square_bbox(100.0, margin_ratio=margin)


@pytest.mark.parametrize('samples', [0, 1, 3])
def test_square_bbox_rejects_too_few_samples(samples):
    proj = LocalProjection(0.0, 0.0)
    with pytest.raises(ValueError, match='samples'):
        proj.square_bbox(100.0, samples=samples)


@pytest.mark.parametrize(
    'lat,lon', [(0.0, 0.0), (40.3314, -3.8356), (-33.45, -70.66), (75.0, 20.0)])
def test_square_bbox_is_well_formed_and_contains_center(lat, lon):
    proj = LocalProjection(lat, lon)
    bbox = proj.square_bbox(500.0)
    assert isinstance(bbox, BBox)
    assert bbox.west < bbox.east
    assert bbox.south < bbox.north
    assert bbox.west <= lon <= bbox.east
    assert bbox.south <= lat <= bbox.north


def test_square_bbox_grows_with_size():
    proj = LocalProjection(40.0, -3.0)
    small = proj.square_bbox(100.0)
    large = proj.square_bbox(1000.0)
    assert (large.east - large.west) > (small.east - small.west)
    assert (large.north - large.south) > (small.north - small.south)


def test_square_bbox_grows_with_margin_ratio():
    proj = LocalProjection(40.0, -3.0)
    no_margin = proj.square_bbox(500.0, margin_ratio=0.0)
    with_margin = proj.square_bbox(500.0, margin_ratio=0.5)
    assert (with_margin.east - with_margin.west) > (no_margin.east - no_margin.west)
