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
Google Maps Platform "Map Tiles API" satellite imagery -- requires an API key.

Unlike Esri World Imagery (keyless) and PNOA (keyless, CC BY 4.0), Google
Maps Platform is a paid, keyed API, added on explicit user request after
Esri looked noticeably worse/older in their area. IMPORTANT license caveat
(see easynav_gis_tool.md for the full discussion): Google Maps Platform's
terms generally restrict caching/storing its content and forbid building a
derivative "basemap" from it, which is close to what this module's disk
cache + baked-texture pipeline does. That's a compliance question for
whoever runs this tool with a Google key, not something this code can
verify or enforce -- use at your own judgement.

Reuses geo.imagery's tile-grid math (`lonlat_to_tile`, `ImageryMosaic`,
`native_resolution_m_per_px`): the Map Tiles API serves the same 256x256
Web Mercator XYZ pyramid Esri does, just gated behind a short-lived session
token instead of being open. Session creation:

    POST https://tile.googleapis.com/v1/createSession?key=API_KEY
    {"mapType": "satellite", "language": "en-US", "region": "US"}
    -> {"session": "...", "expiry": "<unix ts>", ...}

    GET https://tile.googleapis.com/v1/2dtiles/{z}/{x}/{y}?session=...&key=...

Session tokens are documented to last a couple of hours; this module
creates one lazily (only on an actual cache miss -- a fully-cached re-run
touches the Google API not at all, so it costs nothing and needs no key) and
transparently refreshes it if it has expired, guarded by a lock since tile
fetches run concurrently. This request/response shape is implemented from
Google's published Map Tiles API documentation; it has not been exercised
against a live key in this repository (unlike Esri/PNOA, both tested
end-to-end this session) -- verify it against your own key and report back
if anything doesn't match.
"""

from concurrent.futures import as_completed, ThreadPoolExecutor
import math
import os
import sys
import threading
import time
from typing import Optional

import numpy as np

from PIL import Image

import requests

from .cache import DiskCache
from .imagery import ImageryMosaic, lonlat_to_tile
from .net import get_to_file
from .projection import BBox

API_KEY_ENV_VAR = 'GOOGLE_MAPS_API_KEY'
_CREATE_SESSION_URL = 'https://tile.googleapis.com/v1/createSession'
_TILE_URL_TEMPLATE = 'https://tile.googleapis.com/v1/2dtiles/{z}/{x}/{y}'
_TILE_SIZE = 256
_TIMEOUT_S = 60
_MAX_TILES = 30000
_MAX_WORKERS = 16
# Margin subtracted from the session's own reported expiry so a fetch
# in-flight when the check runs doesn't race an expiring token.
_SESSION_EXPIRY_MARGIN_S = 30.0


def get_api_key() -> str:
    """Read the Google Maps Platform API key from the environment; raises if unset."""
    key = os.environ.get(API_KEY_ENV_VAR, '').strip()
    if not key:
        raise RuntimeError(
            '--imagery-source google requires a Google Maps Platform API key '
            f'with the "Map Tiles API" enabled; set it in the {API_KEY_ENV_VAR} '
            'environment variable'
        )
    return key


class GoogleSession:
    """A Map Tiles API session token plus its expiry, as a wall-clock epoch second."""

    def __init__(self, token: str, expiry_epoch_s: float):
        self.token = token
        self.expiry_epoch_s = expiry_epoch_s

    def is_valid(self) -> bool:
        """Return False once within `_SESSION_EXPIRY_MARGIN_S` of the reported expiry."""
        return time.time() < (self.expiry_epoch_s - _SESSION_EXPIRY_MARGIN_S)


def create_session(api_key: str) -> GoogleSession:
    """Create a new Map Tiles API session (one HTTP POST); raises on failure."""
    resp = requests.post(
        _CREATE_SESSION_URL,
        params={'key': api_key},
        json={'mapType': 'satellite', 'language': 'en-US', 'region': 'US'},
        timeout=_TIMEOUT_S,
    )
    resp.raise_for_status()
    data = resp.json()
    return GoogleSession(token=data['session'], expiry_epoch_s=float(data['expiry']))


class _SessionProvider:
    """Lazily creates a session on first use and refreshes it once it expires."""

    def __init__(self, api_key: str):
        self._api_key = api_key
        self._lock = threading.Lock()
        self._session: Optional[GoogleSession] = None

    def get(self) -> GoogleSession:
        with self._lock:
            if self._session is None or not self._session.is_valid():
                self._session = create_session(self._api_key)
            return self._session


def _fetch_tile_path(
    z: int, x: int, y: int, sessions: _SessionProvider, api_key: str,
    cache: DiskCache, force: bool = False,
):
    key = f'imagery/google/{z}/{x}/{y}'

    def fetch(dest) -> None:
        session = sessions.get()
        url = f'{_TILE_URL_TEMPLATE.format(z=z, x=x, y=y)}?session={session.token}&key={api_key}'
        get_to_file(url, dest, _TIMEOUT_S)

    # Session creation only happens inside `fetch`, i.e. only on an actual
    # cache miss -- a fully-cached re-run never calls the Google API at all.
    return cache.get_or_fetch(key, '.jpg', fetch, force=force)


def load_google_mosaic(
    bbox: BBox, cache: DiskCache, api_key: Optional[str] = None,
    zoom: int = 20, force: bool = False,
) -> ImageryMosaic:
    """Fetch/mosaic (caching each tile) Google satellite imagery covering `bbox` at `zoom`."""
    if api_key is None:
        api_key = get_api_key()
    if not 0 <= zoom <= 23:
        raise ValueError(f'zoom out of range [0, 23]: {zoom}')

    x0f, y0f = lonlat_to_tile(bbox.west, bbox.north, zoom)
    x1f, y1f = lonlat_to_tile(bbox.east, bbox.south, zoom)
    tx0, ty0 = int(math.floor(x0f)), int(math.floor(y0f))
    tx1, ty1 = int(math.floor(x1f)), int(math.floor(y1f))
    n_tiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
    if n_tiles > _MAX_TILES:
        raise ValueError(
            f'imagery request would need {n_tiles} tiles at zoom {zoom}; '
            'reduce --size or --zoom'
        )

    sessions = _SessionProvider(api_key)
    cols = (tx1 - tx0 + 1) * _TILE_SIZE
    rows = (ty1 - ty0 + 1) * _TILE_SIZE
    mosaic = np.zeros((rows, cols, 3), dtype=np.uint8)

    def _fetch_and_place(tx: int, ty: int) -> None:
        path = _fetch_tile_path(zoom, tx, ty, sessions, api_key, cache, force=force)
        with Image.open(path) as im:
            tile_rgb = np.asarray(im.convert('RGB'))
        row_off = (ty - ty0) * _TILE_SIZE
        col_off = (tx - tx0) * _TILE_SIZE
        # Disjoint slice per (tx, ty): safe to write concurrently, no lock needed.
        mosaic[row_off:row_off + _TILE_SIZE, col_off:col_off + _TILE_SIZE] = tile_rgb

    tile_coords = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]
    with ThreadPoolExecutor(max_workers=_MAX_WORKERS) as pool:
        futures = [pool.submit(_fetch_and_place, tx, ty) for tx, ty in tile_coords]
        done = 0
        for future in as_completed(futures):
            future.result()  # re-raises any exception from the worker
            done += 1
            if n_tiles >= 500 and done % max(1, n_tiles // 20) == 0:
                print(f'[navmap_gis_tool] imagery: {done}/{n_tiles} tiles', file=sys.stderr)

    return ImageryMosaic(image=mosaic, zoom=zoom, tile_x0=tx0, tile_y0=ty0)
