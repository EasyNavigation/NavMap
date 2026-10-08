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
Small HTTP GET-to-file helper with retries, shared by geo/dem.py and geo/imagery.py.

DEM/imagery fetches are many sequential requests against third-party tile
servers (dozens to thousands per run); a single transient connection hiccup
on any one of them should not abort an otherwise-successful run, especially
since the disk cache means a bare re-run would otherwise have to re-fetch
nothing except that one tile anyway.
"""

import time

import requests

_RETRY_BACKOFF_S = (0.5, 1.5, 3.0)


def get_to_file(url: str, dest, timeout: float, allow_404: bool = False) -> bool:
    """
    GET `url` and stream it to `dest`, retrying transient failures.

    Retries on `requests.exceptions.RequestException` (connection errors,
    timeouts, etc.) with a short backoff; does not retry on HTTP error status
    codes (`raise_for_status()`) other than a 404 when `allow_404` is set.

    Returns `True` if the download succeeded (`dest` was created). Returns
    `False` without creating `dest` if `allow_404` is set and the server
    returned 404. Raises the last `RequestException` if every attempt fails.
    """
    last_exc = None
    for backoff in (0.0,) + _RETRY_BACKOFF_S:
        if backoff:
            time.sleep(backoff)
        try:
            with requests.get(url, timeout=timeout, stream=True) as resp:
                if allow_404 and resp.status_code == 404:
                    return False
                resp.raise_for_status()
                with open(dest, 'wb') as f:
                    for chunk in resp.iter_content(chunk_size=1 << 20):
                        f.write(chunk)
            return True
        except requests.exceptions.RequestException as e:
            last_exc = e
    raise last_exc
