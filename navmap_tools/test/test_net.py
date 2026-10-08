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
Unit tests for navmap_tools.geo.net -- no real network involved.

`requests.get` is monkeypatched with small fakes; `time.sleep` is
monkeypatched to a no-op so the retry-backoff tests stay fast.
"""

from navmap_tools.geo import net as net_mod
from navmap_tools.geo.net import get_to_file

import pytest

import requests


class _FakeResponse:

    def __init__(self, status_code=200, chunks=(b'hello',)):
        self.status_code = status_code
        self._chunks = chunks

    def raise_for_status(self):
        if self.status_code >= 400:
            raise requests.exceptions.HTTPError(f'{self.status_code} error')

    def iter_content(self, chunk_size):
        yield from self._chunks

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        return False


@pytest.fixture(autouse=True)
def _no_real_sleep(monkeypatch):
    monkeypatch.setattr(net_mod.time, 'sleep', lambda _seconds: None)


def test_get_to_file_success_writes_content(tmp_path, monkeypatch):
    dest = tmp_path / 'out.bin'
    monkeypatch.setattr(
        net_mod.requests, 'get',
        lambda url, timeout, stream: _FakeResponse(chunks=(b'ab', b'cd')))
    ok = get_to_file('http://example.invalid/x', dest, timeout=5)
    assert ok is True
    assert dest.read_bytes() == b'abcd'


def test_get_to_file_raises_on_http_error_status(tmp_path, monkeypatch):
    monkeypatch.setattr(
        net_mod.requests, 'get', lambda url, timeout, stream: _FakeResponse(status_code=500))
    with pytest.raises(requests.exceptions.HTTPError):
        get_to_file('http://example.invalid/x', tmp_path / 'out.bin', timeout=5)


def test_get_to_file_404_without_allow_404_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(
        net_mod.requests, 'get', lambda url, timeout, stream: _FakeResponse(status_code=404))
    with pytest.raises(requests.exceptions.HTTPError):
        get_to_file('http://example.invalid/x', tmp_path / 'out.bin', timeout=5)


def test_get_to_file_404_with_allow_404_returns_false_and_no_file(tmp_path, monkeypatch):
    monkeypatch.setattr(
        net_mod.requests, 'get', lambda url, timeout, stream: _FakeResponse(status_code=404))
    dest = tmp_path / 'out.bin'
    ok = get_to_file('http://example.invalid/x', dest, timeout=5, allow_404=True)
    assert ok is False
    assert not dest.exists()


def test_get_to_file_retries_transient_failure_then_succeeds(tmp_path, monkeypatch):
    calls = {'n': 0}

    def fake_get(url, timeout, stream):
        calls['n'] += 1
        if calls['n'] < 3:
            raise requests.exceptions.ConnectionError('reset')
        return _FakeResponse(chunks=(b'ok',))

    monkeypatch.setattr(net_mod.requests, 'get', fake_get)
    dest = tmp_path / 'out.bin'
    ok = get_to_file('http://example.invalid/x', dest, timeout=5)
    assert ok is True
    assert dest.read_bytes() == b'ok'
    assert calls['n'] == 3


def test_get_to_file_raises_last_exception_after_exhausting_retries(tmp_path, monkeypatch):
    def always_fails(url, timeout, stream):
        raise requests.exceptions.Timeout('too slow')

    monkeypatch.setattr(net_mod.requests, 'get', always_fails)
    with pytest.raises(requests.exceptions.Timeout, match='too slow'):
        get_to_file('http://example.invalid/x', tmp_path / 'out.bin', timeout=5)


def test_get_to_file_does_not_leave_a_partial_file_on_total_failure(tmp_path, monkeypatch):
    def always_fails(url, timeout, stream):
        raise requests.exceptions.ConnectionError('nope')

    monkeypatch.setattr(net_mod.requests, 'get', always_fails)
    dest = tmp_path / 'out.bin'
    with pytest.raises(requests.exceptions.ConnectionError):
        get_to_file('http://example.invalid/x', dest, timeout=5)
    assert not dest.exists()


def test_get_to_file_number_of_attempts_matches_backoff_table_plus_one(monkeypatch, tmp_path):
    calls = {'n': 0}

    def always_fails(url, timeout, stream):
        calls['n'] += 1
        raise requests.exceptions.ConnectionError('nope')

    monkeypatch.setattr(net_mod.requests, 'get', always_fails)
    with pytest.raises(requests.exceptions.ConnectionError):
        get_to_file('http://example.invalid/x', tmp_path / 'out.bin', timeout=5)
    assert calls['n'] == 1 + len(net_mod._RETRY_BACKOFF_S)
