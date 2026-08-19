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
Local ENU/AEQD projection centered at a GPS point, and bbox math for --size.

An Azimuthal Equidistant (AEQD) projection centered on the query point is used
because it preserves true distances (in meters) from the center along any
direction -- exactly the property needed to turn a "--size meters square
centered on --center" request into an accurate WGS84 bounding box, and to
convert every downstream local-frame vertex (x, y) back to a real (lon, lat)
to sample DEM/imagery at.
"""

from dataclasses import dataclass
from typing import Tuple

from pyproj import Transformer


@dataclass(frozen=True)
class BBox:
    """A WGS84 bounding box (degrees)."""

    west: float
    south: float
    east: float
    north: float


class LocalProjection:
    """Maps between WGS84 (lon, lat) and a local ENU plane centered at a point."""

    def __init__(self, center_lat: float, center_lon: float):
        """Build the AEQD projection centered at (center_lat, center_lon)."""
        if not -90.0 <= center_lat <= 90.0:
            raise ValueError(f'latitude out of range [-90, 90]: {center_lat}')
        if not -180.0 <= center_lon <= 180.0:
            raise ValueError(f'longitude out of range [-180, 180]: {center_lon}')
        self.center_lat = center_lat
        self.center_lon = center_lon
        proj4 = (
            f'+proj=aeqd +lat_0={center_lat} +lon_0={center_lon} '
            '+datum=WGS84 +units=m +no_defs'
        )
        self._to_local = Transformer.from_crs('EPSG:4326', proj4, always_xy=True)
        self._to_lonlat = Transformer.from_crs(proj4, 'EPSG:4326', always_xy=True)

    def to_local(self, lon: float, lat: float) -> Tuple[float, float]:
        """Project (lon, lat) to local ENU meters (x=east, y=north) from center."""
        x, y = self._to_local.transform(lon, lat)
        return x, y

    def to_lonlat(self, x: float, y: float) -> Tuple[float, float]:
        """Unproject local ENU meters (x=east, y=north) back to (lon, lat)."""
        lon, lat = self._to_lonlat.transform(x, y)
        return lon, lat

    def square_bbox(
        self, size_m: float, margin_ratio: float = 0.05, samples: int = 64
    ) -> BBox:
        """
        WGS84 bbox covering a `size_m` x `size_m` square centered at (0, 0).

        Sampled around the full perimeter (not just the 4 corners) and padded
        by `margin_ratio` of the half-size, so downstream DEM/imagery fetches
        comfortably cover every point of the square even though AEQD parallels
        are not straight lines in (lon, lat) space.
        """
        if size_m <= 0:
            raise ValueError(f'size_m must be positive: {size_m}')
        if not 0.0 <= margin_ratio < 1.0:
            raise ValueError(f'margin_ratio must be in [0, 1): {margin_ratio}')
        if samples < 4:
            raise ValueError(f'samples must be >= 4: {samples}')

        half = size_m / 2.0
        extent = half * (1.0 + margin_ratio)

        lons = []
        lats = []
        for x, y in self._perimeter_points(extent, samples):
            lon, lat = self.to_lonlat(x, y)
            lons.append(lon)
            lats.append(lat)
        return BBox(west=min(lons), south=min(lats), east=max(lons), north=max(lats))

    @staticmethod
    def _perimeter_points(half_extent: float, samples: int):
        n_per_side = max(1, samples // 4)
        step = 2.0 * half_extent / n_per_side
        pts = []
        for i in range(n_per_side + 1):
            c = -half_extent + step * i
            pts.append((c, -half_extent))
            pts.append((c, half_extent))
            pts.append((-half_extent, c))
            pts.append((half_extent, c))
        return pts
