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
Content-addressed disk cache for downloaded GIS data (DEM/imagery tiles).

Callers provide a cache key and a callable that knows how to produce the file
the first time; subsequent calls with the same key reuse the file on disk
instead of re-downloading it, satisfying the "cache en disco para no
descargarnos todo otra vez" requirement from easynav_gis_tool.md.
"""

import hashlib
from pathlib import Path
from typing import Callable

import platformdirs


def default_cache_dir() -> Path:
    """Return the default on-disk cache root for navmap_tools GIS downloads."""
    return Path(platformdirs.user_cache_dir('navmap_tools')) / 'gis'


class DiskCache:
    """A directory-backed cache keyed by an arbitrary string."""

    def __init__(self, root: Path):
        """Create (if needed) and wrap the cache directory at `root`."""
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def path_for(self, key: str, suffix: str) -> Path:
        """Return the deterministic on-disk path for a given cache key."""
        digest = hashlib.sha1(key.encode('utf-8')).hexdigest()
        safe_suffix = suffix if suffix.startswith('.') else f'.{suffix}'
        return self.root / f'{digest}{safe_suffix}'

    def get_or_fetch(
        self,
        key: str,
        suffix: str,
        fetch: Callable[[Path], None],
        force: bool = False,
    ) -> Path:
        """
        Return a cached file for `key`, downloading it via `fetch` if needed.

        `fetch(dest)` must create `dest` (e.g. by streaming an HTTP response to
        it). If it raises, no cache entry is left behind: a partial file is
        removed so a later retry doesn't see a corrupt cache hit.
        """
        dest = self.path_for(key, suffix)
        if dest.exists() and not force:
            return dest
        tmp = dest.with_name(dest.name + '.part')
        try:
            fetch(tmp)
            if not tmp.exists():
                raise RuntimeError(f'fetch() for key {key!r} did not create {tmp}')
            tmp.replace(dest)
        except BaseException:
            tmp.unlink(missing_ok=True)
            raise
        return dest
