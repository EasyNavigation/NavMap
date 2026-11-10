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
navmap_gis_tool: build a Gazebo world and/or a NavMap from real GIS data.

Orchestrates the shared pipeline described in easynav_gis_tool.md: project a
GPS center + size into a WGS84 bbox, fetch/cache DEM + imagery over it,
sample both onto one local-ENU TerrainGrid, then hand that same grid to the
--gazebo mesh exporter and/or the --navmap point-cloud + pointcloud_to_navmap
path -- which is why the two outputs line up even run separately.
"""

import argparse
import functools
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from .geo.cache import default_cache_dir, DiskCache
from .geo.dem import load_dem_grid
from .geo.dem_file import (
    add_imagery, build_local_dem, DemSource, read_dem_file, SyntheticDem, SyntheticImagery,
    write_dem_file,
)
from .geo.google import (
    API_KEY_ENV_VAR as GOOGLE_API_KEY_ENV_VAR, get_api_key as get_google_api_key,
    load_google_mosaic,
)
from .geo.ign import load_ign_dem
from .geo.imagery import DEFAULT_ZOOM, load_imagery_mosaic, native_resolution_m_per_px
from .geo.mesh_dem import hole_cells, load_stl_triangles, MeshDem, MeshHeightfield
from .geo.pnoa import DEFAULT_RESOLUTION_M as PNOA_DEFAULT_RESOLUTION_M, load_pnoa_mosaic
from .geo.projection import LocalProjection
from .geo.terrain_export import write_gazebo_terrain, write_world_from_template
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
        '--dem-out', default=None, metavar='FILE',
        help=(
            'write the elevation as the local height-grid file bigmap_maps_manager reads '
            "(BigMap's DemGrid: terrain.dem_file). Can be the only output; the grid is "
            '--resolution metres, --size wide, centred on --center'
        ))
    parser.add_argument(
        '--dem-data-resolution', type=float, default=0.0, metavar='METERS',
        help=(
            '--dem-out: spacing at which the source is sampled, bicubically interpolated to '
            '--resolution when coarser (0: what the source supports: 5 ign, 30 copernicus30)'
        ))
    parser.add_argument(
        '--reference-altitude', type=float, default=None, metavar='METERS',
        help='--dem-out: altitude above sea level that is z = 0 (default: the height at --center)')
    parser.add_argument(
        '--dem-imagery', action='store_true',
        help='--dem-out: also store the imagery colour of every node')
    parser.add_argument(
        '--terrain-out', default=None, metavar='DIR',
        help=(
            '--dem-out: also write the grid as a Gazebo terrain in DIR (terrain.stl for the '
            'collision, terrain.obj/.mtl/.png for the visual), in the map frame: x east, y north, '
            'z from the reference altitude'
        ))
    parser.add_argument(
        '--terrain-resolution', type=float, default=4.0, metavar='METERS',
        help='--terrain-out: spacing of the collision mesh, coarser than the grid (default: 4)')
    parser.add_argument(
        '--world-template', default=None, metavar='FILE',
        help=(
            '--terrain-out: a world with @LATITUDE@, @LONGITUDE@, @ELEVATION@ (m above sea level '
            'of z = 0) and @TERRAIN_DIR@ placeholders, filled in and written to --world-out'
        ))
    parser.add_argument(
        '--world-out', default=None, metavar='FILE', help='--world-template: where the world goes')
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
        '--dem-source', choices=['copernicus30', 'ign', 'mesh', 'synthetic'],
        default='copernicus30',
        help=(
            'elevation data source: copernicus30 (worldwide, 30 m, keyless), ign (Spain only, '
            "5 m, keyless: IGN's national terrain model), mesh (an STL of your own survey, "
            'see --mesh-file) or synthetic (an offline test terrain) (default: %(default)s)'
        ))
    parser.add_argument(
        '--mesh-file', default=None, metavar='STL',
        help=(
            '--dem-source mesh: STL in the local frame of --center (x east, y north, metres); '
            'its top surface is the terrain'
        ))
    parser.add_argument(
        '--dem-holes-mesh', default=None, metavar='STL',
        help=(
            '--dem-out: STL in the local frame of --center (buildings, walls); the cells its '
            'triangles touch (seen from above) have no ground in the DEM: holes, as the walls of '
            'a NavMap. Use it with a --mesh-file of the ground alone'
        ))
    parser.add_argument(
        '--mesh-world-elevation', type=float, default=None, metavar='METERS',
        help='--dem-source mesh: altitude above sea level of the mesh z = 0 (default: 0)')
    parser.add_argument(
        '--mesh-cell', type=float, default=0.5, metavar='METERS',
        help='--dem-source mesh: cell of the height grid the mesh is rasterised to (0.5)')
    parser.add_argument(
        '--imagery-source', choices=['esri', 'pnoa', 'google', 'xyz', 'synthetic'],
        default='esri',
        help=(
            'imagery data source: esri (worldwide, keyless), xyz (tiles of any server, see '
            '--imagery-url), synthetic (an offline chessboard), pnoa (Spain '
            "only -- IGN's national aerial survey, keyless, CC BY 4.0, up to "
            '25 cm/px native and often sharper than esri in rural Spain), or '
            'google (worldwide, paid -- requires a Google Maps Platform API '
            f'key with "Map Tiles API" enabled, in the {GOOGLE_API_KEY_ENV_VAR} '
            'environment variable; see easynav_gis_tool.md for a '
            'caching/licensing caveat specific to this source) '
            '(default: %(default)s)'
        ))
    parser.add_argument(
        '--imagery-url', default=None, metavar='TEMPLATE',
        help=(
            '--imagery-source xyz: tile URL with {z}, {x} and {y}, e.g. '
            "'https://tile.example.org/{z}/{x}/{y}.jpg' (Web Mercator tiles, as OSM and Esri)"
        ))
    parser.add_argument(
        '--imagery-resolution', type=float, default=PNOA_DEFAULT_RESOLUTION_M, metavar='M_PER_PX',
        help='pnoa only: metres per pixel requested (default: %(default)s, its native best)')
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
    _mesh_heightfield.cache_clear()   # once per run: the file may have changed since the last

    if not args.gazebo and not args.navmap and not args.dem_out:
        parser.error('at least one of --gazebo/--navmap/--dem-out is required')
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
    if args.imagery_source == 'xyz' and not args.imagery_url:
        parser.error('--imagery-source xyz needs --imagery-url')
    if args.dem_source == 'mesh' and not args.mesh_file:
        parser.error('--dem-source mesh needs --mesh-file')
    if args.dem_holes_mesh and not args.dem_out:
        parser.error('--dem-holes-mesh needs --dem-out')
    if args.dem_source == 'mesh' and args.mesh_cell <= 0:
        parser.error('--mesh-cell must be positive')
    if args.dem_data_resolution < 0:
        parser.error('--dem-data-resolution must be >= 0')
    if (args.terrain_out or args.world_template) and not args.dem_out:
        parser.error('--terrain-out and --world-template need --dem-out')
    if args.world_template and not (args.terrain_out and args.world_out):
        parser.error('--world-template needs --terrain-out and --world-out')
    if args.terrain_resolution <= 0:
        parser.error('--terrain-resolution must be positive')
    if args.imagery_resolution <= 0:
        parser.error('--imagery-resolution must be positive')

    # the world / NavMap sampling loop is pure Python; --dem-out alone is vectorised
    needs_grid = args.gazebo or args.navmap
    if args.dem_out:
        need = _dem_memory_bytes(args)
        available = _available_memory_bytes()
        if available is not None and need > available:
            parser.error(
                f'--dem-out at --size {args.size} / --resolution {args.resolution} needs about '
                f'{need / 1e9:.1f} GB to build and {available / 1e9:.1f} GB are available; '
                'increase --resolution or reduce --size')
    n_per_side = round(args.size / args.resolution) + 1
    if needs_grid and n_per_side * n_per_side > _MAX_GRID_POINTS:
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

    if not needs_grid:
        return _run_dem_only(args, bbox, cache)

    print(f'[navmap_gis_tool] fetching DEM ({args.dem_source}) over {bbox} ...')
    dem = _load_dem(args, bbox, cache)

    imagery, resolution_m_per_px, imagery_desc, imagery_attribution = _load_imagery(
        args, bbox, cache, lat)

    dem_native_m = dem.pixel_size_lat * _METERS_PER_DEGREE
    if args.resolution < dem_native_m / 2:
        print(
            f'[navmap_gis_tool] warning: --resolution {args.resolution} m is much finer '
            f"than the DEM's native ~{dem_native_m:.1f} m resolution; elevation detail "
            'below that will be smoothly interpolated, not real', file=sys.stderr)

    if args.dem_out:
        _write_dem(args, dem, imagery, lat, lon)
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


# Peak memory of build_local_dem per node of its two grids (the data and the output): measured
# about 44 bytes, with a margin
_DEM_BYTES_PER_NODE = 64


def _dem_memory_bytes(args) -> int:
    """Return about how much memory building the --dem-out grid takes (bytes)."""
    radius = args.size / 2.0
    side = 2 * math.ceil(radius / args.resolution) + 1
    data_res = args.dem_data_resolution or _DEM_NATIVE_M[args.dem_source] or args.resolution
    side_d = 2 * math.ceil(radius / max(data_res, args.resolution)) + 1
    return _DEM_BYTES_PER_NODE * (side * side + side_d * side_d)


def _available_memory_bytes():
    """Return the memory available to a new process (bytes), or None where it is not known."""
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return None


@functools.lru_cache(maxsize=2)
def _mesh_heightfield(path, cell):
    """Return the MeshHeightfield of an STL, parsed once per run (it is slow: a Python loop)."""
    return MeshHeightfield.load_stl(path, cell)


def _load_dem(args, bbox, cache):
    """Load the terrain model of --dem-source: something with sample(lon, lat) -> metres."""
    if args.dem_source == 'ign':
        return load_ign_dem(bbox, cache, force=args.force_refresh)
    if args.dem_source == 'mesh':
        field = _mesh_heightfield(args.mesh_file, args.mesh_cell)
        print(
            f'[navmap_gis_tool] mesh {args.mesh_file}: {field.coverage:.0%} of its footprint '
            'is covered by triangles (the rest is filled from the neighbours)')
        return MeshDem(field, args.center[0], args.center[1], args.mesh_world_elevation or 0.0)
    if args.dem_source == 'synthetic':
        return SyntheticDem(args.center[0], args.center[1])
    return load_dem_grid(bbox, cache, force=args.force_refresh)


def _load_imagery(args, bbox, cache, lat):
    """Load the --imagery-source mosaic; also its metres per pixel, a description, a credit."""
    if args.imagery_source in ('esri', 'xyz'):
        url = args.imagery_url if args.imagery_source == 'xyz' else None
        print(f'[navmap_gis_tool] fetching imagery ({args.imagery_source}, zoom={args.zoom}) ...')
        imagery = load_imagery_mosaic(
            bbox, cache, zoom=args.zoom, force=args.force_refresh, url_template=url)
        resolution_m_per_px = native_resolution_m_per_px(imagery.zoom, lat)
        if url is None:
            desc = (
                f'Esri World Imagery, zoom {imagery.zoom} '
                f'(~{resolution_m_per_px:.2f} m/px at this latitude)'
            )
            attribution = (
                'Imagery (c) Esri, Maxar, Earthstar Geographics, and the GIS User Community.'
            )
        else:
            desc = (
                f'XYZ tiles {url}, zoom {imagery.zoom} '
                f'(~{resolution_m_per_px:.2f} m/px at this latitude)'
            )
            attribution = 'Imagery: see the terms of the tile server that was used.'
        return imagery, resolution_m_per_px, desc, attribution
    if args.imagery_source == 'synthetic':
        return (
            SyntheticImagery(args.center[0], args.center[1]), 1.0,
            'synthetic chessboard', 'Offline test imagery.')
    if args.imagery_source == 'pnoa':
        print(
            f'[navmap_gis_tool] fetching imagery (pnoa, '
            f'{args.imagery_resolution} m/px) ...')
        imagery = load_pnoa_mosaic(
            bbox, cache, resolution_m_per_px=args.imagery_resolution, force=args.force_refresh)
        resolution_m_per_px = imagery.pixel_size_m
        return (
            imagery, resolution_m_per_px,
            f'PNOA (IGN Spain), {resolution_m_per_px:.2f} m/px',
            'Contains PNOA orthoimagery (c) Instituto Geografico Nacional de Espana, '
            'CC BY 4.0.')
    print(f'[navmap_gis_tool] fetching imagery (google, zoom={args.zoom}) ...')
    imagery = load_google_mosaic(bbox, cache, zoom=args.zoom, force=args.force_refresh)
    resolution_m_per_px = native_resolution_m_per_px(imagery.zoom, lat)
    return (
        imagery, resolution_m_per_px,
        f'Google Maps Platform satellite, zoom {imagery.zoom} '
        f'(~{resolution_m_per_px:.2f} m/px at this latitude)',
        'Imagery (c) Google.')


_DEM_SOURCE_IDS = {
    'copernicus30': DemSource.COPERNICUS, 'ign': DemSource.IGN, 'mesh': DemSource.MESH,
    'synthetic': DemSource.SYNTHETIC,
}
_DEM_NATIVE_M = {'copernicus30': 30.0, 'ign': 5.0, 'mesh': 0.0, 'synthetic': 0.0}


def _source_id(args):
    """Return the id of --dem-source that the DEM file records."""
    return _DEM_SOURCE_IDS[args.dem_source]


def _dem_geometry(args):
    """Return the radius, requested reference altitude and data resolution of --dem-out."""
    radius = args.size / 2.0
    reference = args.reference_altitude
    mesh = None
    if args.dem_source == 'mesh':
        mesh = _mesh_heightfield(args.mesh_file, args.mesh_cell)
        # the DEM cannot show more than the mesh: cap the area to its footprint
        reach = math.ceil(max(abs(mesh.x_min), abs(mesh.x_max), abs(mesh.y_min), abs(mesh.y_max)))
        if reach < radius:
            print(
                f'[navmap_gis_tool] mesh {args.mesh_file} covers up to {reach:.0f} m from the '
                f'origin: radius {radius:.0f} -> {reach:.0f}')
            radius = float(reach)
        if reference is None and args.mesh_world_elevation is not None:
            reference = args.mesh_world_elevation   # z = 0 is the world's elevation
    data_res = args.dem_data_resolution or _DEM_NATIVE_M[args.dem_source]
    return radius, reference, data_res, mesh


def _run_dem_only(args, bbox, cache) -> int:
    """--dem-out without a world or a NavMap: reuse the file if it is the same, else build it."""
    lat, lon = args.center
    radius, reference, data_res, mesh = _dem_geometry(args)
    grid = None
    if not args.force_refresh and Path(args.dem_out).is_file():
        try:
            old = read_dem_file(args.dem_out)
        except (OSError, ValueError):
            old = None
        if old is not None and _is_the_same(old, args, lat, lon, radius, reference, data_res):
            print(f'[navmap_gis_tool] using the DEM {args.dem_out} (same configuration)')
            grid = old
    if grid is None:
        print(f'[navmap_gis_tool] fetching DEM ({args.dem_source}) over {bbox} ...')
        dem = (MeshDem(mesh, lat, lon, args.mesh_world_elevation or 0.0) if mesh is not None
               else _load_dem(args, bbox, cache))
        imagery = None
        if args.dem_imagery:
            try:
                imagery = _load_imagery(args, bbox, cache, lat)[0]
            except Exception as e:   # the terrain is still usable without colours
                print(f'[navmap_gis_tool] imagery unavailable ({e}): no colours', file=sys.stderr)
        grid = build_local_dem(
            dem, lat, lon, radius=radius, resolution=args.resolution,
            source=_source_id(args), reference_altitude=reference, data_resolution=data_res)
        if imagery is not None:
            add_imagery(grid, imagery, args.zoom)
        _add_holes(grid, args)
        _write_dem_and_inputs(grid, args)
        print(
            f'[navmap_gis_tool] wrote {args.dem_out} ({grid.side}x{grid.side} nodes of '
            f'{grid.resolution} m, sampled every {grid.data_resolution} m, '
            f'reference altitude {grid.reference_altitude:.1f} m)')
    if args.terrain_out:
        write_gazebo_terrain(grid, args.terrain_out, args.terrain_resolution)
        print(f'[navmap_gis_tool] wrote the Gazebo terrain in {args.terrain_out}')
    if args.world_template:
        write_world_from_template(grid, args.world_template, args.world_out, args.terrain_out)
        print(f'[navmap_gis_tool] wrote the world {args.world_out}')
    return 0


def _file_id(path):
    """Return the name and the SHA-256 of a file (None for no file)."""
    if not path:
        return None
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return {'name': Path(path).name, 'sha256': digest.hexdigest()}


def _dem_inputs(args) -> dict:
    """Return what makes a DEM besides its geometry: the input files and options, the imagery."""
    dem = {'dem_source': args.dem_source, 'holes_mesh': _file_id(args.dem_holes_mesh)}
    if args.dem_source == 'mesh':
        dem.update(mesh_file=_file_id(args.mesh_file), mesh_cell=args.mesh_cell,
                   mesh_world_elevation=args.mesh_world_elevation)
    imagery = None
    if args.dem_imagery:
        imagery = {'source': args.imagery_source, 'zoom': args.zoom}
        if args.imagery_source == 'xyz':
            imagery['url'] = args.imagery_url
        if args.imagery_source == 'pnoa':
            imagery['resolution'] = args.imagery_resolution
    return {'dem': dem, 'imagery': imagery}


def _inputs_path(dem_out) -> Path:
    """Return the file next to a DEM that records what made it (_dem_inputs)."""
    return Path(str(dem_out) + '.inputs.json')


def _write_dem_and_inputs(grid, args) -> None:
    """Write the DEM file, and next to it what made it (the binary format is BigMap's own)."""
    write_dem_file(grid, args.dem_out)
    _inputs_path(args.dem_out).write_text(json.dumps(_dem_inputs(args), indent=2) + '\n')


def _is_the_same(old, args, lat, lon, radius, reference, data_res) -> bool:
    """Whether the DEM file on disk is what these arguments would make."""
    if not old.matches(lat, lon, radius, args.resolution, _source_id(args), reference, data_res):
        return False
    if bool(args.dem_holes_mesh) != (old.cell_valid is not None):
        return False
    try:   # made from the same files and options (a DEM without this record is rebuilt)
        recorded = json.loads(_inputs_path(args.dem_out).read_text())
    except (OSError, ValueError):
        return False
    wanted = _dem_inputs(args)
    if recorded.get('dem') != wanted['dem']:
        return False
    return not args.dem_imagery or (
        old.rgb is not None and old.imagery_zoom == args.zoom and
        recorded.get('imagery') == wanted['imagery'])


def _add_holes(grid, args) -> None:
    """--dem-holes-mesh: the cells its triangles touch have no ground."""
    if not args.dem_holes_mesh:
        return
    holes = hole_cells(load_stl_triangles(args.dem_holes_mesh), grid.side, grid.resolution)
    grid.cell_valid = (~holes).astype(np.uint8)
    print(f'[navmap_gis_tool] holes from {args.dem_holes_mesh}: {holes.sum()} of {holes.size} '
          f'cells without ground')


def _write_dem(args, dem, imagery, lat, lon) -> None:
    """--dem-out next to a world / NavMap: the local height grid that bigmap_maps_manager reads."""
    radius, reference, data_res, _ = _dem_geometry(args)
    grid = build_local_dem(
        dem, lat, lon, radius=radius, resolution=args.resolution,
        source=_source_id(args), reference_altitude=reference, data_resolution=data_res)
    if imagery is not None and args.dem_imagery:
        add_imagery(grid, imagery, args.zoom)   # the zoom the DEM-only path compares with
    _add_holes(grid, args)
    _write_dem_and_inputs(grid, args)
    print(
        f'[navmap_gis_tool] wrote {args.dem_out} ({grid.side}x{grid.side} nodes of '
        f'{grid.resolution} m, sampled every {grid.data_resolution} m, '
        f'reference altitude {grid.reference_altitude:.1f} m)')
    if args.terrain_out:
        write_gazebo_terrain(grid, args.terrain_out, args.terrain_resolution)
    if args.world_template:
        write_world_from_template(grid, args.world_template, args.world_out, args.terrain_out)


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
