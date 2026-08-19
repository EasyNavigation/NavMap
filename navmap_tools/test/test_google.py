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
Unit tests for navmap_tools.geo.google -- no real network involved.

`requests.post`/tile fetching are monkeypatched throughout; the actual
Map Tiles API contract (session creation, tile URL shape) has not been
exercised against a live key in this repository -- see the module
docstring in geo/google.py.
"""

import time

from navmap_tools.geo import google as google_mod
from navmap_tools.geo.cache import DiskCache
from navmap_tools.geo.google import (
    API_KEY_ENV_VAR,
    create_session,
    get_api_key,
    GoogleSession,
    load_google_mosaic,
)
from navmap_tools.geo.projection import BBox

import numpy as np

from PIL import Image

import pytest


# ---------------------------------------------------------------------------
# get_api_key
# ---------------------------------------------------------------------------

def test_get_api_key_raises_when_unset(monkeypatch):
    monkeypatch.delenv(API_KEY_ENV_VAR, raising=False)
    with pytest.raises(RuntimeError, match=API_KEY_ENV_VAR):
        get_api_key()


def test_get_api_key_raises_when_blank(monkeypatch):
    monkeypatch.setenv(API_KEY_ENV_VAR, '   ')
    with pytest.raises(RuntimeError, match=API_KEY_ENV_VAR):
        get_api_key()


def test_get_api_key_returns_stripped_value(monkeypatch):
    monkeypatch.setenv(API_KEY_ENV_VAR, '  my-key  ')
    assert get_api_key() == 'my-key'


# ---------------------------------------------------------------------------
# GoogleSession.is_valid
# ---------------------------------------------------------------------------

def test_session_valid_when_expiry_far_in_future():
    session = GoogleSession(token='t', expiry_epoch_s=time.time() + 3600)
    assert session.is_valid() is True


def test_session_invalid_when_expiry_in_the_past():
    session = GoogleSession(token='t', expiry_epoch_s=time.time() - 10)
    assert session.is_valid() is False


def test_session_invalid_within_expiry_margin():
    # _SESSION_EXPIRY_MARGIN_S is 30s; 5s from now is inside that margin.
    session = GoogleSession(token='t', expiry_epoch_s=time.time() + 5)
    assert session.is_valid() is False


# ---------------------------------------------------------------------------
# create_session
# ---------------------------------------------------------------------------

class _FakeResponse:

    def __init__(self, json_data, status_code=200):
        self._json = json_data
        self.status_code = status_code

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError(f'{self.status_code} error')

    def json(self):
        return self._json


def test_create_session_parses_token_and_expiry(monkeypatch):
    calls = []

    def fake_post(url, params=None, json=None, timeout=None):
        calls.append((url, params, json, timeout))
        return _FakeResponse({'session': 'abc123', 'expiry': '9999999999'})

    monkeypatch.setattr(google_mod.requests, 'post', fake_post)
    session = create_session('the-key')

    assert session.token == 'abc123'
    assert session.expiry_epoch_s == 9999999999.0
    (url, params, json_body, timeout) = calls[0]
    assert url == google_mod._CREATE_SESSION_URL
    assert params == {'key': 'the-key'}
    assert json_body['mapType'] == 'satellite'
    assert timeout == google_mod._TIMEOUT_S


def test_create_session_propagates_http_errors(monkeypatch):
    def fake_post(url, params=None, json=None, timeout=None):
        return _FakeResponse({}, status_code=403)

    monkeypatch.setattr(google_mod.requests, 'post', fake_post)
    with pytest.raises(RuntimeError, match='403'):
        create_session('bad-key')


# ---------------------------------------------------------------------------
# _SessionProvider
# ---------------------------------------------------------------------------

def test_session_provider_creates_lazily(monkeypatch):
    created = []

    def fake_create_session(api_key):
        created.append(api_key)
        return GoogleSession(token='t', expiry_epoch_s=time.time() + 3600)

    monkeypatch.setattr(google_mod, 'create_session', fake_create_session)
    provider = google_mod._SessionProvider('key')
    assert created == []  # not created until .get() is called
    provider.get()
    assert created == ['key']


def test_session_provider_reuses_valid_session(monkeypatch):
    created = []

    def fake_create_session(api_key):
        created.append(api_key)
        return GoogleSession(token='t', expiry_epoch_s=time.time() + 3600)

    monkeypatch.setattr(google_mod, 'create_session', fake_create_session)
    provider = google_mod._SessionProvider('key')
    s1 = provider.get()
    s2 = provider.get()
    assert s1 is s2
    assert len(created) == 1


def test_session_provider_refreshes_expired_session(monkeypatch):
    tokens = iter(['first', 'second'])

    def fake_create_session(api_key):
        return GoogleSession(token=next(tokens), expiry_epoch_s=time.time() - 1)

    monkeypatch.setattr(google_mod, 'create_session', fake_create_session)
    provider = google_mod._SessionProvider('key')
    s1 = provider.get()
    s2 = provider.get()
    # Both immediately expired (expiry in the past), so every .get() refreshes.
    assert s1.token == 'first'
    assert s2.token == 'second'


# ---------------------------------------------------------------------------
# _fetch_tile_path -- cache hit must never touch the session/network
# ---------------------------------------------------------------------------

def test_fetch_tile_path_cache_hit_never_creates_a_session(tmp_path, monkeypatch):
    cache = DiskCache(tmp_path)
    session_calls = []

    class _NeverCallProvider:
        def get(self):
            session_calls.append(1)
            raise AssertionError('session should not be requested on a cache hit')

    # Pre-populate the cache entry this key would resolve to.
    key = 'imagery/google/10/5/5'
    path = cache.path_for(key, '.jpg')
    path.write_bytes(b'cached tile bytes')

    result = google_mod._fetch_tile_path(10, 5, 5, _NeverCallProvider(), 'key', cache)
    assert result == path
    assert session_calls == []


def test_fetch_tile_path_cache_miss_uses_session_token_in_url(tmp_path, monkeypatch):
    cache = DiskCache(tmp_path)
    session = GoogleSession(token='sess-tok', expiry_epoch_s=time.time() + 3600)

    class _FixedProvider:
        def get(self):
            return session

    captured_urls = []

    def fake_get_to_file(url, dest, timeout):
        captured_urls.append(url)
        dest.write_bytes(b'tile bytes')

    monkeypatch.setattr(google_mod, 'get_to_file', fake_get_to_file)
    google_mod._fetch_tile_path(10, 5, 5, _FixedProvider(), 'the-key', cache)

    assert len(captured_urls) == 1
    assert 'session=sess-tok' in captured_urls[0]
    assert 'key=the-key' in captured_urls[0]


# ---------------------------------------------------------------------------
# load_google_mosaic
# ---------------------------------------------------------------------------

def test_load_google_mosaic_requires_api_key_when_not_passed(monkeypatch):
    monkeypatch.delenv(API_KEY_ENV_VAR, raising=False)
    with pytest.raises(RuntimeError, match=API_KEY_ENV_VAR):
        load_google_mosaic(BBox(-1, -1, 1, 1), cache=None, zoom=10)


def test_load_google_mosaic_rejects_bad_zoom():
    with pytest.raises(ValueError, match='zoom'):
        load_google_mosaic(BBox(-1, -1, 1, 1), cache=None, api_key='k', zoom=-1)
    with pytest.raises(ValueError, match='zoom'):
        load_google_mosaic(BBox(-1, -1, 1, 1), cache=None, api_key='k', zoom=24)


def test_load_google_mosaic_rejects_too_many_tiles():
    with pytest.raises(ValueError, match='tiles'):
        load_google_mosaic(BBox(-170, -80, 170, 80), cache=None, api_key='k', zoom=18)


def test_load_google_mosaic_stitches_tiles_in_correct_positions(monkeypatch, tmp_path):
    calls = []

    def fake_fetch(z, x, y, sessions, api_key, cache, force=False):
        calls.append((z, x, y))
        color = (x % 256, y % 256, 0)
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        img[:, :] = color
        path = tmp_path / f'{z}_{x}_{y}.png'
        Image.fromarray(img, mode='RGB').save(path)
        return path

    monkeypatch.setattr(google_mod, '_fetch_tile_path', fake_fetch)

    bbox = BBox(west=-0.01, south=-0.01, east=0.01, north=0.01)
    mosaic = load_google_mosaic(bbox, cache=None, api_key='k', zoom=15)

    assert mosaic.image.shape == (512, 512, 3)
    assert len(calls) == 4
    top_left_color = tuple(mosaic.image[0, 0])
    assert top_left_color == (mosaic.tile_x0 % 256, mosaic.tile_y0 % 256, 0)


def test_load_google_mosaic_full_cache_hit_creates_no_session(tmp_path, monkeypatch):
    # First call (cache miss) should create exactly one session; a second,
    # fully-cached call must not touch the Google API at all -- the whole
    # point of caching a paid API's tiles.
    session_creations = []

    def fake_create_session(api_key):
        session_creations.append(api_key)
        return GoogleSession(token='t', expiry_epoch_s=time.time() + 3600)

    def fake_get_to_file(url, dest, timeout):
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        Image.fromarray(img, mode='RGB').save(dest, format='JPEG')

    monkeypatch.setattr(google_mod, 'create_session', fake_create_session)
    monkeypatch.setattr(google_mod, 'get_to_file', fake_get_to_file)

    cache = DiskCache(tmp_path)
    bbox = BBox(west=-0.001, south=-0.001, east=0.001, north=0.001)

    load_google_mosaic(bbox, cache, api_key='k', zoom=15)
    assert len(session_creations) == 1

    load_google_mosaic(bbox, cache, api_key='k', zoom=15)
    assert len(session_creations) == 1  # unchanged: fully served from cache
