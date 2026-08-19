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
navmap_gis_tool: build a Gazebo world and/or a NavMap from real GIS data.

Orchestrates the shared pipeline described in easynav_gis_tool.md: project a
GPS center + size into a WGS84 bbox, fetch/cache DEM + imagery over it,
sample both onto one local-ENU TerrainGrid, then hand that same grid to the
--gazebo mesh exporter and/or the --navmap point-cloud + pointcloud_to_navmap
path -- which is why the two outputs line up even run separately.
"""

import argparse
import subprocess
import sys

import numpy as np

from .geo.cache import default_cache_dir, DiskCache
from .geo.dem import load_dem_grid
from .geo.google import (
    API_KEY_ENV_VAR as GOOGLE_API_KEY_ENV_VAR, get_api_key as get_google_api_key,
    load_google_mosaic,
)
from .geo.imagery import DEFAULT_ZOOM, load_imagery_mosaic, native_resolution_m_per_px
from .geo.pnoa import DEFAULT_RESOLUTION_M as PNOA_DEFAULT_RESOLUTION_M, load_pnoa_mosaic
from .geo.projection import LocalProjection
from .mesh_export import write_dae, write_stl, write_texture_png
from .pcd_writer import write_colors_csv, write_pcd_xyz
from .scaffold import scaffold_world_package
from .terrain import build_terrain_grid, render_texture

_METERS_PER_DEGREE = 111320.0
# Bounds for the auto-sized texture (see _texture_pixels_for below): below
# this, tiny areas don't need huge textures; above this, PNG size/render
# cost grow faster than the extra detail is worth for a robotics sim asset.
_MIN_TEXTURE_PIXELS = 256
_MAX_TEXTURE_PIXELS = 4096
# The DEM/imagery sampling loop (terrain.build_terrain_grid) is pure Python,
# one pyproj + bilinear-sample call per grid point, not vectorized. Measured
# ~7.6 us/point (linear), so this many points is ~30 s of pure compute --
# generous, but still bounded so a huge --size/--resolution combination fails
# fast with a clear message instead of running for a very long time and
# producing an impractically large mesh. See easynav_gis_tool.md.
_MAX_GRID_POINTS = 4_000_000


def _texture_pixels_for(size_m: float, resolution_m_per_px: float) -> int:
    """
    Auto-size the baked texture so it actually uses the fetched imagery detail.

    Without this, a fixed low pixel count would throw away resolution that
    was just downloaded at real cost (more tiles/chunks, more time).
    Source-agnostic: takes the imagery's own ground resolution directly
    rather than a zoom level, since not every source has one (PNOA is a WMS
    service requested at an explicit meters/pixel, not a tile zoom).
    """
    ideal = round(size_m / resolution_m_per_px)
    return min(_MAX_TEXTURE_PIXELS, max(_MIN_TEXTURE_PIXELS, ideal))


def _default_package_name(lat: float, lon: float) -> str:
    def fmt(v: float) -> str:
        s = f'{abs(v):.4f}'.replace('.', 'p')
        return ('m' if v < 0 else '') + s

    return f'gis_{fmt(lat)}_{fmt(lon)}_world'


def _parse_center(value: str):
    parts = value.split(',')
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f'--center must be "LAT,LON", got {value!r}')
    try:
        lat, lon = float(parts[0]), float(parts[1])
    except ValueError as e:
        raise argparse.ArgumentTypeError(f'--center must be "LAT,LON": {e}') from e
    return lat, lon


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the argparse parser for navmap_gis_tool."""
    parser = argparse.ArgumentParser(
        prog='navmap_gis_tool',
        description=(
            'Generate a Gazebo world and/or a NavMap from real elevation + '
            'imagery data, given a GPS center and a size in meters.'
        ),
    )
    parser.add_argument(
        '--center', required=True, type=_parse_center, metavar='LAT,LON',
        help='GPS center of the area, e.g. 40.3314,-3.8356')
    parser.add_argument(
        '--size', required=True, type=float, metavar='METERS',
        help='side length (meters) of the square area centered on --center')
    parser.add_argument('--gazebo', action='store_true', help='emit a Gazebo world + models')
    parser.add_argument('--navmap', action='store_true', help='emit a .navmap file')
    parser.add_argument(
        '--resolution', type=float, default=1.0, metavar='METERS',
        help=(
            'grid sampling resolution in meters, shared by --gazebo geometry '
            'and --navmap (default: %(default)s)'
        ))
    parser.add_argument(
        '--max-slope-deg', type=float, default=30.0, metavar='DEGREES',
        help='NavMap maximum navigable slope (default: %(default)s)')
    parser.add_argument(
        '--package', default=None,
        help=(
            'output ROS package name, e.g. urjc_excavation_world -- the whole '
            'colcon package (worlds/, models/, maps/, launch/) is generated '
            'under this name (default: derived from --center)'
        ))
    parser.add_argument(
        '--output-dir', default=None,
        help='directory to create the package in (default: ./<package>)')
    parser.add_argument(
        '--cache-dir', default=None,
        help='GIS download cache directory (default: platformdirs cache dir)')
    parser.add_argument(
        '--dem-source', choices=['copernicus30'], default='copernicus30',
        help='elevation data source (default: %(default)s)')
    parser.add_argument(
        '--imagery-source', choices=['esri', 'pnoa', 'google'], default='esri',
        help=(
            'imagery data source: esri (worldwide, keyless), pnoa (Spain '
            "only -- IGN's national aerial survey, keyless, CC BY 4.0, up to "
            '25 cm/px native and often sharper than esri in rural Spain), or '
            'google (worldwide, paid -- requires a Google Maps Platform API '
            f'key with "Map Tiles API" enabled, in the {GOOGLE_API_KEY_ENV_VAR} '
            'environment variable; see easynav_gis_tool.md for a '
            'caching/licensing caveat specific to this source) '
            '(default: %(default)s)'
        ))
    parser.add_argument(
        '--zoom', type=int, default=DEFAULT_ZOOM,
        help=(
            'esri/google only: imagery tile zoom level, 0-23; higher = more '
            'detail but more tiles/download time. For esri, automatically '
            'stepped down if the requested zoom has no real coverage (see '
            'find_available_zoom) (default: %(default)s)'
        ))
    parser.add_argument(
        '--texture-pixels', type=int, default=0, metavar='N',
        help=(
            'baked texture size in pixels per side; 0 (default) auto-sizes '
            'it from --zoom/--size so the texture actually uses the fetched '
            f'imagery detail, clamped to [{_MIN_TEXTURE_PIXELS}, {_MAX_TEXTURE_PIXELS}]'
        ))
    parser.add_argument(
        '--force-refresh', action='store_true', help='bypass the GIS download cache')
    return parser


def run(argv=None) -> int:
    """Parse arguments and run the full pipeline; returns a process exit code."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if not args.gazebo and not args.navmap:
        parser.error('at least one of --gazebo/--navmap is required')
    if args.size <= 0:
        parser.error('--size must be positive')
    if args.resolution <= 0:
        parser.error('--resolution must be positive')
    if not 0 <= args.zoom <= 23:
        parser.error('--zoom must be in [0, 23]')
    if args.texture_pixels < 0:
        parser.error('--texture-pixels must be >= 0 (0 = auto)')
    if args.imagery_source == 'google':
        try:
            get_google_api_key()
        except RuntimeError as e:
            parser.error(str(e))

    n_per_side = round(args.size / args.resolution) + 1
    if n_per_side * n_per_side > _MAX_GRID_POINTS:
        parser.error(
            f'--size {args.size} / --resolution {args.resolution} would sample '
            f'{n_per_side * n_per_side} grid points (a {n_per_side}x{n_per_side} grid), '
            f'over the {_MAX_GRID_POINTS} limit; increase --resolution or reduce --size '
            '(the DEM/imagery sampling loop is pure Python and does not scale to huge grids)'
        )

    lat, lon = args.center
    package_name = args.package or _default_package_name(lat, lon)
    output_dir = args.output_dir or f'./{package_name}'
    cache_dir = args.cache_dir or default_cache_dir()
    cache = DiskCache(cache_dir)

    print(f'[navmap_gis_tool] center=({lat}, {lon}) size={args.size} m package={package_name!r}')
    print(f'[navmap_gis_tool] cache: {cache.root}')

    projection = LocalProjection(lat, lon)
    bbox = projection.square_bbox(args.size)

    print(f'[navmap_gis_tool] fetching DEM ({args.dem_source}) over {bbox} ...')
    dem = load_dem_grid(bbox, cache, force=args.force_refresh)

    if args.imagery_source == 'esri':
        print(f'[navmap_gis_tool] fetching imagery (esri, zoom={args.zoom}) ...')
        imagery = load_imagery_mosaic(bbox, cache, zoom=args.zoom, force=args.force_refresh)
        resolution_m_per_px = native_resolution_m_per_px(imagery.zoom, lat)
        imagery_desc = (
            f'Esri World Imagery, zoom {imagery.zoom} '
            f'(~{resolution_m_per_px:.2f} m/px at this latitude)'
        )
        imagery_attribution = (
            'Imagery (c) Esri, Maxar, Earthstar Geographics, and the GIS User Community.'
        )
    elif args.imagery_source == 'pnoa':
        print(
            f'[navmap_gis_tool] fetching imagery (pnoa, '
            f'{PNOA_DEFAULT_RESOLUTION_M} m/px) ...')
        imagery = load_pnoa_mosaic(
            bbox, cache, resolution_m_per_px=PNOA_DEFAULT_RESOLUTION_M, force=args.force_refresh)
        resolution_m_per_px = imagery.pixel_size_m
        imagery_desc = f'PNOA (IGN Spain), {resolution_m_per_px:.2f} m/px'
        imagery_attribution = (
            'Contains PNOA orthoimagery (c) Instituto Geografico Nacional de Espana, '
            'CC BY 4.0.'
        )
    else:
        print(f'[navmap_gis_tool] fetching imagery (google, zoom={args.zoom}) ...')
        imagery = load_google_mosaic(bbox, cache, zoom=args.zoom, force=args.force_refresh)
        resolution_m_per_px = native_resolution_m_per_px(imagery.zoom, lat)
        imagery_desc = (
            f'Google Maps Platform satellite, zoom {imagery.zoom} '
            f'(~{resolution_m_per_px:.2f} m/px at this latitude)'
        )
        imagery_attribution = 'Imagery (c) Google.'

    dem_native_m = dem.pixel_size_lat * _METERS_PER_DEGREE
    if args.resolution < dem_native_m / 2:
        print(
            f'[navmap_gis_tool] warning: --resolution {args.resolution} m is much finer '
            f"than the DEM's native ~{dem_native_m:.1f} m resolution; elevation detail "
            'below that will be smoothly interpolated, not real', file=sys.stderr)
    grid = build_terrain_grid(projection, dem, imagery, args.size, args.resolution)

    paths = scaffold_world_package(
        output_dir, package_name, lat, lon, args.size, grid.center_elevation_amsl,
        imagery_desc, imagery_attribution)
    print(f'[navmap_gis_tool] package scaffolded at {paths.root}')

    if args.navmap:
        grid_rows, grid_cols = grid.xs.shape
        points = np.stack([grid.xs, grid.ys, grid.elevation], axis=-1).reshape(-1, 3)
        colors = grid.rgb.reshape(-1, 3)
        n = write_pcd_xyz(paths.pcd_path, points, width=grid_cols, height=grid_rows)
        write_colors_csv(paths.colors_csv_path, colors)
        print(f'[navmap_gis_tool] wrote {paths.pcd_path} ({n} points)')

        exe = _find_tool_executable('pointcloud_to_navmap')
        cmd = [
            exe,
            '--input', str(paths.pcd_path),
            '--output', str(paths.navmap_path),
            '--colors', str(paths.colors_csv_path),
            '--resolution', str(args.resolution),
            '--max-slope-deg', str(args.max_slope_deg),
        ]
        print(f'[navmap_gis_tool] running: {" ".join(cmd)}')
        result = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        if result.returncode != 0:
            print('[navmap_gis_tool] pointcloud_to_navmap failed', file=sys.stderr)
            return result.returncode
        print(f'[navmap_gis_tool] wrote {paths.navmap_path}')

    if args.gazebo:
        texture_pixels = args.texture_pixels or _texture_pixels_for(
            args.size, resolution_m_per_px)
        print(f'[navmap_gis_tool] texture: {texture_pixels}x{texture_pixels} px')
        texture = render_texture(projection, imagery, args.size, pixels=texture_pixels)
        write_texture_png(paths.texture_path, texture)
        write_dae(paths.dae_path, grid, args.size, paths.texture_path.name)
        write_stl(paths.stl_path, grid)
        print(f'[navmap_gis_tool] wrote {paths.dae_path}, {paths.stl_path}')
        print(f'[navmap_gis_tool] wrote {paths.world_path}')

    print(f'[navmap_gis_tool] done: {paths.root}')
    return 0


def _find_tool_executable(name: str) -> str:
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    try:
        prefix = get_package_prefix('navmap_tools')
    except PackageNotFoundError as e:
        raise RuntimeError(
            'navmap_tools is not colcon-built/sourced (needed to locate the '
            f'compiled {name!r} tool)'
        ) from e
    return f'{prefix}/lib/navmap_tools/{name}'


def main(argv=None) -> None:
    """Console-script entry point."""
    sys.exit(run(argv))
