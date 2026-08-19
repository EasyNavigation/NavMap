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

"""Unit tests for navmap_tools.geo.cache -- no network involved."""

from pathlib import Path

from navmap_tools.geo.cache import default_cache_dir, DiskCache

import pytest


def test_default_cache_dir_ends_with_navmap_tools_gis():
    path = default_cache_dir()
    assert path.parts[-2:] == ('navmap_tools', 'gis')


def test_diskcache_creates_root_directory(tmp_path):
    root = tmp_path / 'a' / 'b' / 'cache'
    assert not root.exists()
    DiskCache(root)
    assert root.is_dir()


def test_path_for_is_deterministic(tmp_path):
    cache = DiskCache(tmp_path)
    p1 = cache.path_for('key', '.tif')
    p2 = cache.path_for('key', '.tif')
    assert p1 == p2


def test_path_for_differs_for_different_keys(tmp_path):
    cache = DiskCache(tmp_path)
    assert cache.path_for('key1', '.tif') != cache.path_for('key2', '.tif')


def test_path_for_adds_leading_dot_to_suffix(tmp_path):
    cache = DiskCache(tmp_path)
    assert cache.path_for('key', 'tif') == cache.path_for('key', '.tif')


def test_path_for_lives_under_root(tmp_path):
    cache = DiskCache(tmp_path)
    p = cache.path_for('key', '.tif')
    assert p.parent == tmp_path


def test_get_or_fetch_calls_fetch_on_first_call(tmp_path):
    cache = DiskCache(tmp_path)
    calls = []

    def fetch(dest):
        calls.append(dest)
        dest.write_text('data')

    path = cache.get_or_fetch('key', '.txt', fetch)
    assert path.read_text() == 'data'
    assert len(calls) == 1


def test_get_or_fetch_reuses_cache_on_second_call(tmp_path):
    cache = DiskCache(tmp_path)
    calls = []

    def fetch(dest):
        calls.append(dest)
        dest.write_text('data')

    cache.get_or_fetch('key', '.txt', fetch)
    cache.get_or_fetch('key', '.txt', fetch)
    assert len(calls) == 1


def test_get_or_fetch_force_refetches(tmp_path):
    cache = DiskCache(tmp_path)
    calls = []

    def fetch(dest):
        calls.append(dest)
        dest.write_text('data')

    cache.get_or_fetch('key', '.txt', fetch)
    cache.get_or_fetch('key', '.txt', fetch, force=True)
    assert len(calls) == 2


def test_get_or_fetch_propagates_fetch_exception_and_cleans_up(tmp_path):
    cache = DiskCache(tmp_path)

    def fetch(dest):
        dest.write_text('partial')
        raise RuntimeError('boom')

    with pytest.raises(RuntimeError, match='boom'):
        cache.get_or_fetch('key', '.txt', fetch)

    dest = cache.path_for('key', '.txt')
    assert not dest.exists()
    assert not (tmp_path / (dest.name + '.part')).exists()
    assert list(tmp_path.iterdir()) == []


def test_get_or_fetch_raises_if_fetch_does_not_create_dest(tmp_path):
    cache = DiskCache(tmp_path)

    def fetch(dest):
        pass  # deliberately does not create dest

    with pytest.raises(RuntimeError, match='did not create'):
        cache.get_or_fetch('key', '.txt', fetch)
    assert list(tmp_path.iterdir()) == []


def test_get_or_fetch_returns_a_path_instance(tmp_path):
    cache = DiskCache(tmp_path)
    path = cache.get_or_fetch('key', '.txt', lambda dest: dest.write_text('x'))
    assert isinstance(path, Path)
