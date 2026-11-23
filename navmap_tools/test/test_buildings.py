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
Unit tests for navmap_tools.geo.buildings -- no network involved.

The Catastro and Overpass answers are canned; the footprints become cells of a local grid.
"""

import json

from navmap_tools.geo import buildings as bmod
from navmap_tools.geo.buildings import fetch_buildings, footprint_cells, in_spain
from navmap_tools.geo.cache import DiskCache
from navmap_tools.geo.dem_file import EnuProjection

import numpy as np
from pyproj import CRS, Transformer


LAT0, LON0 = 40.41690, -3.70350     # the Puerta del Sol


def _square(cx, cy, half):
    return np.array([[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half],
                     [cx - half, cy + half]])


def test_a_footprint_blocks_the_cells_it_is_in_and_its_patio_stays_ground():
    side, res = 41, 1.0                 # cells of 1 m, centred on the origin, 40 x 40 m
    ext = _square(0.5, 0.5, 8.0)        # x, y in [-7.5, 8.5]
    patio = _square(0.5, 0.5, 3.0)      # x, y in [-2.5, 3.5]
    cells = footprint_cells([(ext, [patio])], side, res)
    half = (side - 1) // 2

    def cell(x, y):
        return cells[int(np.floor(y / res + half)), int(np.floor(x / res + half))]

    assert cell(6.0, 6.0)               # inside the building
    assert not cell(0.5, 0.5)           # in the patio
    assert cell(-7.4, 0.0)              # an edge crosses it: blocked (any part of a building)
    assert cell(8.4, 0.0)
    assert not cell(-9.0, 0.0)          # outside
    assert not cell(15.0, 15.0)
    # the patio's edges block their cells too
    assert cell(-2.4, 0.5) and cell(3.4, 0.5)


def test_footprints_beyond_the_grid_are_ignored():
    cells = footprint_cells([(_square(500.0, 500.0, 5.0), [])], 21, 1.0)
    assert not cells.any()


def test_in_spain():
    assert in_spain(LAT0, LON0)
    assert in_spain(28.12, -15.43)       # Las Palmas
    assert not in_spain(48.8562, 2.2976)


def _catastro_xml(rings_utm, floors):
    parts = []
    for k, (ring, fl) in enumerate(zip(rings_utm, floors)):
        pos = ' '.join('%.3f %.3f' % (x, y) for x, y in list(ring) + [ring[0]])
        parts.append(
            f'<wfs:member><bu-ext2d:BuildingPart gml:id="ES.SDGC.BU.{k}_part1">'
            f'<bu-ext2d:numberOfFloorsAboveGround>{fl}</bu-ext2d:numberOfFloorsAboveGround>'
            '<bu-ext2d:geometry><gml:Surface><gml:patches><gml:PolygonPatch><gml:exterior>'
            f'<gml:LinearRing><gml:posList>{pos}</gml:posList></gml:LinearRing></gml:exterior>'
            '</gml:PolygonPatch></gml:patches></gml:Surface></bu-ext2d:geometry>'
            '</bu-ext2d:BuildingPart></wfs:member>')
    return ('<?xml version="1.0"?><wfs:FeatureCollection '
            'xmlns:wfs="http://www.opengis.net/wfs/2.0" '
            'xmlns:gml="http://www.opengis.net/gml/3.2" '
            'xmlns:bu-ext2d="http://inspire.jrc.ec.europa.eu/schemas/bu-ext2d/2.0">' +
            ''.join(parts) + '</wfs:FeatureCollection>').encode()


def test_catastro_parts_above_ground_in_the_local_frame(monkeypatch, tmp_path):
    # two parts 10 m east of the origin, one with floors, one underground
    enu = EnuProjection(LAT0, LON0)
    to_utm = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(25830), always_xy=True)
    lat, lon, _ = enu.to_geodetic(np.array([8.0, 12.0, 12.0, 8.0]),
                                  np.array([-2.0, -2.0, 2.0, 2.0]))
    x, y = to_utm.transform(lon, lat)
    ring = np.c_[x, y]
    body = _catastro_xml([ring, ring], [3, 0])
    calls = []

    def fake_get(cache, url, params, what, timeout=120.0):
        calls.append(params)
        return body
    monkeypatch.setattr(bmod, '_cached_get', fake_get)
    footprints, source = fetch_buildings('auto', LAT0, LON0, 50.0, DiskCache(tmp_path))
    assert source == 'Catastro'
    assert len(calls) == 1               # a small area: one tile
    assert len(footprints) == 1          # the underground part is left out
    ext, patios = footprints[0]
    assert patios == []
    assert np.allclose(ext.min(axis=0), [8.0, -2.0], atol=0.01)
    assert np.allclose(ext.max(axis=0), [12.0, 2.0], atol=0.01)


def test_osm_buildings_where_the_catastro_has_none(monkeypatch, tmp_path):
    no_records = (b'<?xml version="1.0"?><ExceptionReport>No records</ExceptionReport>')
    monkeypatch.setattr(bmod, '_cached_get', lambda *a, **k: no_records)
    enu = EnuProjection(LAT0, LON0)
    lat, lon, _ = enu.to_geodetic(np.array([0.0, 6.0, 6.0, 0.0, 0.0]),
                                  np.array([0.0, 0.0, 6.0, 6.0, 0.0]))
    way = {'type': 'way', 'id': 1, 'tags': {'building': 'yes'},
           'geometry': [{'lat': float(a), 'lon': float(o)} for a, o in zip(lat, lon)]}
    roof = dict(way, id=2, tags={'building': 'roof'})

    class Answer:
        status_code = 200
        content = json.dumps({'elements': [way, roof]}).encode()
    monkeypatch.setattr(bmod.requests, 'post', lambda *a, **k: Answer())
    footprints, source = fetch_buildings('auto', LAT0, LON0, 50.0, DiskCache(tmp_path))
    assert source == 'OpenStreetMap'
    assert len(footprints) == 1          # building=roof is no obstacle at ground level
    assert np.allclose(footprints[0][0].max(axis=0), [6.0, 6.0], atol=0.01)


def test_where_the_parts_fail_the_whole_buildings_are_asked(monkeypatch, tmp_path):
    enu = EnuProjection(LAT0, LON0)
    to_utm = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(25830), always_xy=True)
    lat, lon, _ = enu.to_geodetic(np.array([8.0, 12.0, 12.0, 8.0]),
                                  np.array([-2.0, -2.0, 2.0, 2.0]))
    x, y = to_utm.transform(lon, lat)
    building = _catastro_xml([np.c_[x, y]], [0]).replace(b'BuildingPart', b'Building')
    asked = []

    def fake_get(cache, url, params, what, timeout=120.0):
        asked.append(params['typenames'])
        if params['typenames'] == 'BU:BuildingPart':
            raise RuntimeError('HTTP 404')
        return building
    monkeypatch.setattr(bmod, '_cached_get', fake_get)
    footprints, source = fetch_buildings('catastro', LAT0, LON0, 50.0, DiskCache(tmp_path))
    assert source == 'Catastro'
    assert 'BU:Building' in asked
    assert len(footprints) == 1          # a whole building: no floors, still a footprint
