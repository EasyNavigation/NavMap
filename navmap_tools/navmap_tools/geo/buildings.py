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
The footprints of the buildings of an area, from open data, as cells without ground of a grid.

  * Spain: the Catastro, through its INSPIRE buildings WFS (no key): every building part with
    its footprint (and patios) and its number of floors above ground. The service answers
    nothing for boxes over about 1 km^2, so the area is asked in tiles.
  * Anywhere else, and in Spain where the Catastro has nothing (the Basque Country and Navarra
    keep their own cadastre): OpenStreetMap, through the Overpass API (no key).

Every answer is kept in the disk cache: a second run of the same area asks nothing.
"""

import json
import math
import time
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import CRS, Transformer
import requests

from .cache import DiskCache
from .dem_file import EnuProjection

CATASTRO_URL = 'https://ovc.catastro.meh.es/INSPIRE/wfsBU.aspx'
OVERPASS_URLS = ('https://overpass-api.de/api/interpreter',
                 'https://overpass.private.coffee/api/interpreter',
                 'https://overpass.kumi.systems/api/interpreter')
_NS = {
    'gml': 'http://www.opengis.net/gml/3.2',
    'bu': 'http://inspire.jrc.ec.europa.eu/schemas/bu-ext2d/2.0',
}
_HEADERS = {'User-Agent': 'navmap_tools'}
# OpenStreetMap building=* that are no obstacle at ground level (a roof on pillars, a carport)
_OSM_NOT_AN_OBSTACLE = ('no', 'roof', 'carport')

# A footprint: exterior ring (N, 2) and its holes (patios), in the local ENU frame (m)
Footprint = Tuple[np.ndarray, List[np.ndarray]]


def in_spain(lat: float, lon: float) -> bool:
    """Whether the Catastro may hold (lat, lon): the peninsula, the islands, Ceuta and Melilla."""
    return ((35.0 <= lat <= 44.0 and -7.5 <= lon <= 5.0) or
            (41.7 <= lat <= 44.0 and -9.4 <= lon <= -6.0) or
            (27.0 <= lat <= 29.5 and -19.0 <= lon <= -13.0))


def _area_lonlat(enu: EnuProjection, radius: float, pad: float):
    """(west, south, east, north) degrees of the square of half side radius + pad."""
    r = radius + pad
    lat, lon, _ = enu.to_geodetic(np.array([-r, r, r, -r]), np.array([-r, -r, r, r]))
    return float(lon.min()), float(lat.min()), float(lon.max()), float(lat.max())


def _cached_get(cache: DiskCache, url: str, params: dict, what: str, timeout=120.0) -> bytes:
    key = url + '?' + json.dumps(params, sort_keys=True)

    def fetch(dest):
        last = None
        for attempt in range(4):
            if attempt:
                time.sleep(2.0 ** attempt)    # 2, 4, 8 s: the services are often busy
            try:
                r = requests.get(url, params=params, headers=_HEADERS, timeout=timeout)
                if r.status_code == 200:
                    dest.write_bytes(r.content)
                    return
                last = f'HTTP {r.status_code}'
            except requests.RequestException as e:
                last = str(e)
            print(f'[navmap_gis_tool] {what}: attempt {attempt + 1} failed ({last})')
        raise RuntimeError(f'{what}: {last}')
    return cache.get_or_fetch(key, '.xml', fetch).read_bytes()


def fetch_catastro(lat0: float, lon0: float, radius: float, cache: DiskCache,
                   tile: float = 800.0) -> List[Footprint]:
    """Footprints of the Catastro's building parts with floors above ground."""
    enu = EnuProjection(lat0, lon0)
    epsg = 4083 if lat0 < 30.0 else 25830     # REGCAN95 UTM 28N (Canary Islands) / ETRS89 UTM 30N
    to_proj = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(epsg), always_xy=True)
    to_ll = Transformer.from_crs(CRS.from_epsg(epsg), CRS.from_epsg(4326), always_xy=True)
    w, s, e, n = _area_lonlat(enu, radius, 20.0)
    xs, ys = to_proj.transform([w, e, e, w], [s, s, n, n])
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    nx, ny = max(1, math.ceil((x1 - x0) / tile)), max(1, math.ceil((y1 - y0) / tile))

    def local(ring):
        lon, lat = to_ll.transform(ring[:, 0], ring[:, 1])
        east, north, _ = enu.to_enu(np.asarray(lat), np.asarray(lon))
        return np.c_[east, north]

    out, seen = [], set()

    def get(tb, name, typename):
        return _cached_get(cache, CATASTRO_URL, {
            'service': 'wfs', 'version': '2.0.0', 'request': 'GetFeature',
            'typenames': typename, 'srsname': f'EPSG::{epsg}',
            'bbox': '%.2f,%.2f,%.2f,%.2f,urn:ogc:def:crs:EPSG::%d' % (tuple(tb) + (epsg,))},
            f'Catastro buildings, tile {name}')

    def ask(tb, name, depth=0):
        """Ask the building parts of a tile; one the service keeps failing on, in four."""
        try:
            body = get(tb, name, 'BU:BuildingPart')
        except RuntimeError:
            if depth >= 2:
                # the service fails on the parts of some places (an error page) but serves their
                # buildings: their footprints, without the floors
                print(f'[navmap_gis_tool] Catastro buildings, tile {name}: whole buildings')
                body = get(tb, name + ' (buildings)', 'BU:Building')
                parse(ET.fromstring(body), 'Building')
                return
            mx, my = (tb[0] + tb[2]) / 2, (tb[1] + tb[3]) / 2
            print(f'[navmap_gis_tool] Catastro buildings, tile {name}: asked in four')
            for k, sub in enumerate(((tb[0], tb[1], mx, my), (mx, tb[1], tb[2], my),
                                     (tb[0], my, mx, tb[3]), (mx, my, tb[2], tb[3]))):
                ask(sub, f'{name}.{k + 1}', depth + 1)
            return
        try:
            root = ET.fromstring(body)
        except ET.ParseError as ex:
            raise RuntimeError(f'Catastro buildings: not XML ({ex})')
        if root.tag.endswith('ExceptionReport'):
            if b'No records' in body:
                return        # nothing there (rural, or a territory it does not hold)
            raise RuntimeError('Catastro buildings: ' + body[:300].decode(errors='replace'))
        parse(root, 'BuildingPart')

    def parse(root, element):
        for part in root.iter('{%s}%s' % (_NS['bu'], element)):
            pid = part.get('{%s}id' % _NS['gml'], '')
            if pid in seen:
                continue
            seen.add(pid)
            floors = part.find('.//bu:numberOfFloorsAboveGround', _NS)
            if element == 'BuildingPart' and (
                    floors is None or not (floors.text or '').strip().isdigit() or
                    int(floors.text) < 1):
                continue      # underground, or a part with nothing above the ground
            for patch in part.iter('{%s}PolygonPatch' % _NS['gml']):
                ext = patch.find('gml:exterior', _NS)
                if ext is None:
                    continue
                try:
                    rings = [_pos_list(ext)] + [_pos_list(h)
                                                for h in patch.findall('gml:interior', _NS)]
                except (ValueError, AttributeError):
                    continue  # a malformed polygon
                out.append((local(rings[0]), [local(r) for r in rings[1:]]))

    for j in range(ny):
        for i in range(nx):
            ask((x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny,
                 x0 + (x1 - x0) * (i + 1) / nx, y0 + (y1 - y0) * (j + 1) / ny),
                f'{j * nx + i + 1}/{nx * ny}')
    return out


def _pos_list(el) -> np.ndarray:
    v = np.array(el.find('.//gml:posList', _NS).text.split(), dtype=np.float64).reshape(-1, 2)
    return v[:-1] if len(v) > 1 and np.allclose(v[0], v[-1]) else v


def fetch_osm(lat0: float, lon0: float, radius: float, cache: DiskCache) -> List[Footprint]:
    """Footprints of the OpenStreetMap buildings (ways and multipolygons)."""
    enu = EnuProjection(lat0, lon0)
    w, s, e, n = _area_lonlat(enu, radius, 20.0)
    query = ('[out:json][timeout:180];(way["building"](%f,%f,%f,%f);'
             'relation["building"]["type"="multipolygon"](%f,%f,%f,%f););out geom tags;'
             % ((s, w, n, e) * 2))

    def fetch(dest):
        last = None
        for k in range(6):
            try:
                r = requests.post(OVERPASS_URLS[k % len(OVERPASS_URLS)], data={'data': query},
                                  headers=_HEADERS, timeout=200)
                if r.status_code == 200:
                    json.loads(r.content)       # whole: not an error page
                    dest.write_bytes(r.content)
                    return
                last = f'HTTP {r.status_code}'
            except (requests.RequestException, ValueError) as ex:
                last = str(ex)
            print(f'[navmap_gis_tool] OpenStreetMap buildings: attempt {k + 1} failed ({last})')
        raise RuntimeError(f'OpenStreetMap buildings: {last}')

    data = json.loads(cache.get_or_fetch('overpass:' + query, '.json', fetch).read_bytes())

    def local(points):
        lon = np.array([p['lon'] for p in points])
        lat = np.array([p['lat'] for p in points])
        east, north, _ = enu.to_enu(lat, lon)
        ring = np.c_[east, north]
        return ring[:-1] if len(ring) > 1 and np.allclose(ring[0], ring[-1]) else ring

    out = []
    for el in data.get('elements', []):
        if el.get('tags', {}).get('building', 'yes') in _OSM_NOT_AN_OBSTACLE:
            continue
        if el['type'] == 'way':
            g = el.get('geometry') or []
            if len(g) >= 4:
                out.append((local(g), []))
        else:
            outer = [local(m['geometry']) for m in el.get('members', [])
                     if m['type'] == 'way' and m.get('role') == 'outer' and m.get('geometry')]
            inner = [local(m['geometry']) for m in el.get('members', [])
                     if m['type'] == 'way' and m.get('role') == 'inner' and m.get('geometry')]
            for ring in outer:
                if len(ring) >= 3:
                    out.append((ring, [h for h in inner if len(h) >= 3 and
                                       _inside(ring, h[:1, 0], h[:1, 1])[0]]))
    return out


def fetch_buildings(source: str, lat0: float, lon0: float, radius: float,
                    cache: DiskCache) -> Tuple[List[Footprint], str]:
    """Footprints from `source` (catastro, osm or auto) and the name of what answered."""
    if source == 'catastro' or (source == 'auto' and in_spain(lat0, lon0)):
        parts = fetch_catastro(lat0, lon0, radius, cache)
        if parts or source == 'catastro':
            return parts, 'Catastro'
    return fetch_osm(lat0, lon0, radius, cache), 'OpenStreetMap'


def _inside(ring: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Whether the points (x, y) are inside the polygon `ring` (even-odd rule)."""
    inside = np.zeros(np.shape(x), dtype=bool)
    xs, ys = ring[:, 0], ring[:, 1]
    x2, y2 = np.roll(xs, -1), np.roll(ys, -1)
    for ax, ay, bx, by in zip(xs, ys, x2, y2):
        if ay == by:
            continue
        crosses = ((ay > y) != (by > y)) & (x < (bx - ax) * (y - ay) / (by - ay) + ax)
        inside ^= crosses
    return inside


def footprint_cells(footprints: List[Footprint], side: int, resolution: float,
                    holes: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Mark the cells of a local grid that a building stands on.

    Returns (side - 1, side - 1) bool, [j, i], for the grid of `dem_file.LocalDem` (nodes at
    (i - (side - 1) / 2) * resolution, cell (i, j) between nodes i..i+1 and j..j+1). A cell is
    marked if its centre is inside a footprint (not in one of its patios) or if an edge of the
    footprint or of a patio crosses it: the rule of a collision map, a cell is blocked if any
    part of a building is in it. `holes`, if given, is updated and returned.
    """
    nc = side - 1
    out = holes if holes is not None else np.zeros((nc, nc), dtype=bool)
    half = (side - 1) // 2

    def cell_of(v):
        return np.floor(v / resolution + half).astype(np.int64)

    for ext, patios in footprints:
        if len(ext) < 3:
            continue
        lo, hi = cell_of(ext.min(axis=0)), cell_of(ext.max(axis=0))
        if hi[0] < 0 or hi[1] < 0 or lo[0] >= nc or lo[1] >= nc:
            continue
        i0, j0 = max(lo[0], 0), max(lo[1], 0)
        i1, j1 = min(hi[0], nc - 1), min(hi[1], nc - 1)
        cj, ci = np.mgrid[j0:j1 + 1, i0:i1 + 1]
        cx, cy = (ci - half + 0.5) * resolution, (cj - half + 0.5) * resolution
        inside = _inside(ext, cx, cy)
        for p in patios:
            if len(p) >= 3:
                inside &= ~_inside(p, cx, cy)
        out[cj[inside], ci[inside]] = True
        # the cells the edges cross (sampled at a quarter of a cell)
        for ring in [ext] + [p for p in patios if len(p) >= 3]:
            a, b = ring, np.roll(ring, -1, axis=0)
            for (ax, ay), (bx, by) in zip(a, b):
                steps = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / (resolution / 4))))
                t = np.linspace(0.0, 1.0, steps + 1)
                pi = cell_of(ax + t * (bx - ax))
                pj = cell_of(ay + t * (by - ay))
                keep = (pi >= 0) & (pj >= 0) & (pi < nc) & (pj < nc)
                out[pj[keep], pi[keep]] = True
    return out


def describe(footprints: List[Footprint]) -> str:
    """'N buildings (M patios)'."""
    patios = sum(len(p) for _, p in footprints)
    return f'{len(footprints)} footprints ({patios} patios)'


__all__ = ['fetch_buildings', 'fetch_catastro', 'fetch_osm', 'footprint_cells', 'in_spain',
           'describe', 'Footprint']
