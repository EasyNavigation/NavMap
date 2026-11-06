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
Unit tests for navmap_tools.cli's pure argument-parsing logic.

Deliberately does not exercise `run()`: that hits the network (DEM/imagery)
and shells out to the compiled `pointcloud_to_navmap`, both covered by the
end-to-end smoke test instead (see easynav_gis_tool.md).
"""

import argparse

from navmap_tools.cli import (
    _default_package_name,
    _parse_center,
    _texture_pixels_for,
    build_arg_parser,
)
from navmap_tools.geo.imagery import DEFAULT_ZOOM, native_resolution_m_per_px

import pytest


# ---------------------------------------------------------------------------
# _default_package_name
# ---------------------------------------------------------------------------

def test_default_package_name_ends_with_world():
    assert _default_package_name(40.3314, -3.8356).endswith('_world')


def test_default_package_name_is_a_valid_identifier_segment():
    name = _default_package_name(40.3314, -3.8356)
    assert name.replace('_', '').isalnum()


def test_default_package_name_encodes_sign_for_negative_values():
    name = _default_package_name(-33.45, -70.66)
    assert 'm33p4500' in name
    assert 'm70p6600' in name


def test_default_package_name_has_no_sign_marker_for_positive_values():
    name = _default_package_name(40.3314, 3.8356)
    assert 'm40p3314' not in name
    assert 'm3p8356' not in name


def test_default_package_name_differs_for_different_coordinates():
    a = _default_package_name(40.0, -3.0)
    b = _default_package_name(41.0, -3.0)
    assert a != b


def test_default_package_name_stable_for_same_coordinates():
    assert _default_package_name(40.0, -3.0) == _default_package_name(40.0, -3.0)


def test_default_package_name_handles_zero():
    name = _default_package_name(0.0, 0.0)
    assert name == 'gis_0p0000_0p0000_world'


# ---------------------------------------------------------------------------
# _parse_center
# ---------------------------------------------------------------------------

def test_parse_center_valid():
    assert _parse_center('40.3314,-3.8356') == (40.3314, -3.8356)


def test_parse_center_with_spaces_around_numbers():
    assert _parse_center('0,0') == (0.0, 0.0)


@pytest.mark.parametrize('value', ['40.3314', '40.3314,-3.8356,1.0', '', ','])
def test_parse_center_rejects_wrong_field_count(value):
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_center(value)


@pytest.mark.parametrize('value', ['abc,1.0', '1.0,xyz'])
def test_parse_center_rejects_non_numeric(value):
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_center(value)


# ---------------------------------------------------------------------------
# build_arg_parser / run() validation
# ---------------------------------------------------------------------------

def test_center_and_size_are_required():
    parser = build_arg_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])


def test_minimal_valid_invocation_parses():
    parser = build_arg_parser()
    args = parser.parse_args(['--center', '40.0,-3.0', '--size', '100', '--gazebo'])
    assert args.center == (40.0, -3.0)
    assert args.size == 100.0
    assert args.gazebo is True
    assert args.navmap is False


def test_defaults():
    parser = build_arg_parser()
    args = parser.parse_args(['--center', '40.0,-3.0', '--size', '100', '--navmap'])
    assert args.resolution == 1.0
    assert args.max_slope_deg == 30.0
    assert args.package is None
    assert args.output_dir is None
    assert args.dem_source == 'copernicus30'
    assert args.imagery_source == 'esri'
    assert args.force_refresh is False
    assert args.zoom == DEFAULT_ZOOM
    assert args.texture_pixels == 0


def test_rejects_unknown_dem_source():
    parser = build_arg_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            ['--center', '40.0,-3.0', '--size', '100', '--navmap', '--dem-source', 'bogus'])


def test_rejects_unknown_imagery_source():
    parser = build_arg_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(
            ['--center', '40.0,-3.0', '--size', '100', '--navmap', '--imagery-source', 'bogus'])


def test_accepts_pnoa_imagery_source():
    parser = build_arg_parser()
    args = parser.parse_args(
        ['--center', '40.0,-3.0', '--size', '100', '--navmap', '--imagery-source', 'pnoa'])
    assert args.imagery_source == 'pnoa'


def test_accepts_google_imagery_source():
    parser = build_arg_parser()
    args = parser.parse_args(
        ['--center', '40.0,-3.0', '--size', '100', '--navmap', '--imagery-source', 'google'])
    assert args.imagery_source == 'google'


def test_google_imagery_source_without_api_key_fails_fast(monkeypatch):
    from navmap_tools.cli import run
    from navmap_tools.geo.google import API_KEY_ENV_VAR

    monkeypatch.delenv(API_KEY_ENV_VAR, raising=False)
    with pytest.raises(SystemExit):
        run([
            '--center', '40.0,-3.0', '--size', '100', '--navmap',
            '--imagery-source', 'google',
        ])


def test_google_imagery_source_with_api_key_passes_validation(monkeypatch):
    from navmap_tools import cli as cli_mod
    from navmap_tools.geo.google import API_KEY_ENV_VAR

    monkeypatch.setenv(API_KEY_ENV_VAR, 'fake-key')

    def fake_load_dem_grid(bbox, cache, force=False):
        raise _Sentinel()

    monkeypatch.setattr(cli_mod, 'load_dem_grid', fake_load_dem_grid)
    with pytest.raises(_Sentinel):
        cli_mod.run([
            '--center', '40.0,-3.0', '--size', '100', '--gazebo',
            '--imagery-source', 'google',
        ])


def test_at_least_one_output_flag_required():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100'])


def test_size_must_be_positive():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '0', '--gazebo'])
    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '-5', '--navmap'])


def test_resolution_must_be_positive():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100', '--resolution', '0', '--gazebo'])
    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100', '--resolution', '-1', '--navmap'])


def test_zoom_must_be_in_range():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100', '--gazebo', '--zoom', '-1'])
    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100', '--gazebo', '--zoom', '24'])


def test_texture_pixels_must_not_be_negative():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100', '--gazebo', '--texture-pixels', '-1'])


# ---------------------------------------------------------------------------
# _texture_pixels_for
#
# Auto-sizes the baked texture from the actual fetched imagery resolution:
# without this, a fixed pixel count wastes whatever extra detail --zoom paid
# to download (a real user asked for this after finding the texture blurrier
# than the source imagery; see easynav_gis_tool.md).
# ---------------------------------------------------------------------------

def test_texture_pixels_for_matches_native_resolution_at_equator():
    # At zoom 19, equator: ~0.29808 m/px: 300 m / 0.29808 =~ 1006.
    resolution = native_resolution_m_per_px(zoom=19, lat_deg=0.0)
    px = _texture_pixels_for(size_m=300.0, resolution_m_per_px=resolution)
    assert 900 <= px <= 1100


def test_texture_pixels_for_is_clamped_to_minimum():
    resolution = native_resolution_m_per_px(zoom=1, lat_deg=0.0)
    px = _texture_pixels_for(size_m=1.0, resolution_m_per_px=resolution)
    assert px == 256


def test_texture_pixels_for_is_clamped_to_maximum():
    resolution = native_resolution_m_per_px(zoom=23, lat_deg=0.0)
    px = _texture_pixels_for(size_m=100000.0, resolution_m_per_px=resolution)
    assert px == 4096


def test_texture_pixels_for_increases_with_finer_resolution():
    low = _texture_pixels_for(
        size_m=300.0, resolution_m_per_px=native_resolution_m_per_px(zoom=15, lat_deg=40.0))
    high = _texture_pixels_for(
        size_m=300.0, resolution_m_per_px=native_resolution_m_per_px(zoom=19, lat_deg=40.0))
    assert high > low


def test_texture_pixels_for_works_with_pnoa_style_resolution():
    # PNOA gives a plain meters/pixel value directly (no zoom level at all).
    px = _texture_pixels_for(size_m=100.0, resolution_m_per_px=0.25)
    assert px == 400


def test_huge_grid_is_rejected_before_touching_the_network():
    from navmap_tools.cli import run

    # 100000 m / 1 m resolution -> a 100001x100001 grid, far over the cap;
    # must fail fast on validation, not hang trying to fetch DEM/imagery.
    with pytest.raises(SystemExit):
        run(['--center', '40.0,-3.0', '--size', '100000', '--gazebo'])


class _Sentinel(Exception):
    """Raised by a stub to prove control reached past argument validation."""


def test_grid_within_the_cap_passes_validation(monkeypatch):
    from navmap_tools import cli as cli_mod

    # Stub out the network-touching step right after validation to prove
    # validation itself did not reject this combination (this is the exact
    # --size 300 --resolution 1.0 combination that a real user hit; see
    # easynav_gis_tool.md).
    def fake_load_dem_grid(bbox, cache, force=False):
        raise _Sentinel()

    monkeypatch.setattr(cli_mod, 'load_dem_grid', fake_load_dem_grid)
    with pytest.raises(_Sentinel):
        cli_mod.run(['--center', '40.0,-3.0', '--size', '300', '--gazebo'])


# ---------------------------------------------------------------------------
# --dem-out, --dem-source ign / mesh, --imagery-source xyz
# ---------------------------------------------------------------------------

_BASE = ['--center', '40.4,-3.7', '--size', '40']


def test_new_sources_are_accepted_and_have_defaults():
    parser = build_arg_parser()
    args = parser.parse_args(
        _BASE + ['--dem-out', 'd.bin', '--dem-source', 'ign', '--imagery-source', 'xyz',
                 '--imagery-url', 'https://t/{z}/{x}/{y}.jpg'])
    assert args.dem_out == 'd.bin' and args.dem_source == 'ign'
    assert args.imagery_source == 'xyz'
    assert args.dem_data_resolution == 0.0
    assert args.reference_altitude is None
    assert args.dem_imagery is False
    assert args.mesh_cell == 0.5 and args.mesh_world_elevation is None


def test_dem_out_alone_is_a_valid_output():
    from navmap_tools import cli as cli_mod

    class Stop(Exception):
        pass

    def fake_load(args, bbox, cache):
        raise Stop()

    cli_mod_load = cli_mod._load_dem
    cli_mod._load_dem = fake_load
    try:
        with pytest.raises(Stop):
            cli_mod.run(_BASE + ['--dem-out', 'd.bin'])
    finally:
        cli_mod._load_dem = cli_mod_load


@pytest.mark.parametrize('extra', [
    ['--imagery-source', 'xyz'],                      # no --imagery-url
    ['--dem-source', 'mesh'],                         # no --mesh-file
    ['--dem-source', 'mesh', '--mesh-file', 'm.stl', '--mesh-cell', '0'],
    ['--dem-data-resolution', '-1'],
])
def test_invalid_combinations_fail_fast(extra):
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(_BASE + ['--dem-out', 'd.bin'] + extra)


def test_dem_out_writes_the_file_from_a_mesh(tmp_path, capsys):
    import struct

    from navmap_tools.cli import run
    from navmap_tools.geo.dem_file import DemSource, read_dem_file

    stl = tmp_path / 'site.stl'
    quads = [(-30, -30, 30, 30, 3.0)]
    tris = []
    for x0, y0, x1, y1, z in quads:
        tris += [(x0, y0, z, x1, y0, z, x1, y1, z), (x0, y0, z, x1, y1, z, x0, y1, z)]
    with open(stl, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<12fH', 0, 0, 1, *t, 0))
    out = tmp_path / 'dem.bin'
    rc = run(_BASE + [
        '--dem-out', str(out), '--dem-source', 'mesh', '--mesh-file', str(stl),
        '--mesh-world-elevation', '675', '--resolution', '2', '--cache-dir', str(tmp_path / 'c')])
    assert rc == 0
    dem = read_dem_file(out)
    assert dem.source == DemSource.MESH
    assert dem.side == 21 and dem.resolution == 2.0
    assert dem.reference_altitude == 675.0   # z = 0 is the world's elevation (as the simulator)
    assert dem.z == pytest.approx(3.0, abs=1e-3)   # the flat mesh, 3 m above it
    assert 'wrote' in capsys.readouterr().out


def test_dem_holes_mesh_makes_the_cells_under_a_building_holes(tmp_path, capsys):
    import struct

    from navmap_tools.cli import run
    from navmap_tools.geo.dem_file import read_dem_file

    def write(path, quads):
        tris = []
        for x0, y0, x1, y1, z in quads:
            tris += [(x0, y0, z, x1, y0, z, x1, y1, z), (x0, y0, z, x1, y1, z, x0, y1, z)]
        with open(path, 'wb') as f:
            f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
            for t in tris:
                f.write(struct.pack('<12fH', 0, 0, 1, *t, 0))
    ground, roof = tmp_path / 'ground.stl', tmp_path / 'roof.stl'
    write(ground, [(-30, -30, 30, 30, 0.0)])
    write(roof, [(5.0, -3.0, 9.0, 1.0, 12.0)])   # a 4 x 4 m building
    out = tmp_path / 'dem.bin'
    args = _BASE + ['--dem-out', str(out), '--dem-source', 'mesh', '--mesh-file', str(ground),
                    '--resolution', '1', '--cache-dir', str(tmp_path / 'c')]
    assert run(args + ['--dem-holes-mesh', str(roof)]) == 0
    dem = read_dem_file(out)
    assert dem.z == pytest.approx(0.0, abs=1e-3)   # the ground alone: no roof heights
    half = (dem.side - 1) // 2
    # cell (i, j) spans x i - half .. i - half + 1: x 5..9 -> i half+5..half+8, y -3..1 likewise;
    # the roof's edges lie on cell borders, which the conservative rule counts: one more each way
    holes = dem.cell_valid == 0
    assert holes[half - 3:half + 1, half + 5:half + 9].all()
    assert holes.sum() <= 6 * 6
    assert not holes[half, half] and not holes[half + 10, half - 10]
    assert 'cells without ground' in capsys.readouterr().out
    # the same configuration without holes is another DEM: rebuilt, not reused
    assert run(args) == 0
    assert read_dem_file(out).cell_valid is None


def _run_synthetic(tmp_path, *extra):
    from navmap_tools.cli import run

    out = tmp_path / 'dem.bin'
    rc = run(_BASE + ['--dem-source', 'synthetic', '--resolution', '2', '--dem-out', str(out),
                      '--cache-dir', str(tmp_path / 'c')] + list(extra))
    assert rc == 0
    return out


def test_dem_out_with_terrain_and_world_from_the_synthetic_source(tmp_path):
    from navmap_tools.geo.dem_file import DemSource, read_dem_file

    template = tmp_path / 'w.in'
    template.write_text('@LATITUDE@ @LONGITUDE@ @ELEVATION@ @TERRAIN_DIR@')
    out = _run_synthetic(
        tmp_path, '--dem-imagery', '--imagery-source', 'synthetic', '--zoom', '18',
        '--terrain-out', str(tmp_path / 'terr'), '--terrain-resolution', '4',
        '--world-template', str(template), '--world-out', str(tmp_path / 'world.sdf'))
    dem = read_dem_file(out)
    assert dem.source == DemSource.SYNTHETIC and dem.side == 21
    assert dem.imagery_zoom == 18 and dem.rgb is not None
    terrain = tmp_path / 'terr'
    assert (terrain / 'terrain.stl').is_file() and (terrain / 'terrain.png').is_file()
    assert str((tmp_path / 'terr').resolve()) in (tmp_path / 'world.sdf').read_text()


def test_the_dem_is_reused_when_nothing_changed_and_rebuilt_when_it_did(tmp_path, capsys):
    out = _run_synthetic(tmp_path)
    first = out.read_bytes()
    capsys.readouterr()
    _run_synthetic(tmp_path)
    assert 'same configuration' in capsys.readouterr().out
    assert out.read_bytes() == first
    _run_synthetic(tmp_path, '--reference-altitude', '600')   # another configuration
    assert out.read_bytes() != first


def test_force_refresh_builds_the_dem_again(tmp_path, capsys):
    _run_synthetic(tmp_path)
    capsys.readouterr()
    _run_synthetic(tmp_path, '--force-refresh')
    assert 'same configuration' not in capsys.readouterr().out


def test_a_dem_that_lacks_the_imagery_is_rebuilt_with_it(tmp_path):
    from navmap_tools.geo.dem_file import read_dem_file

    out = _run_synthetic(tmp_path)
    assert read_dem_file(out).rgb is None
    _run_synthetic(tmp_path, '--dem-imagery', '--imagery-source', 'synthetic', '--zoom', '18')
    assert read_dem_file(out).rgb is not None


def test_a_mesh_smaller_than_the_area_caps_the_radius(tmp_path, capsys):
    import struct

    from navmap_tools.cli import run
    from navmap_tools.geo.dem_file import read_dem_file

    stl = tmp_path / 'small.stl'
    tris = [(-10, -10, 1, 10, -10, 1, 10, 10, 1), (-10, -10, 1, 10, 10, 1, -10, 10, 1)]
    with open(stl, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<12fH', 0, 0, 1, *t, 0))
    out = tmp_path / 'dem.bin'
    assert run(['--center', '40.4,-3.7', '--size', '200', '--resolution', '1',
                '--dem-source', 'mesh', '--mesh-file', str(stl), '--mesh-world-elevation', '675',
                '--dem-out', str(out), '--cache-dir', str(tmp_path / 'c')]) == 0
    dem = read_dem_file(out)
    assert dem.radius == 10.0 and dem.side == 21
    assert dem.requested_reference == 675.0   # z = 0 is the world's elevation
    assert 'radius 100 -> 10' in capsys.readouterr().out


def test_terrain_options_need_dem_out():
    from navmap_tools.cli import run

    with pytest.raises(SystemExit):
        run(_BASE + ['--gazebo', '--terrain-out', 'x'])
    with pytest.raises(SystemExit):
        run(_BASE + ['--dem-out', 'd.bin', '--world-template', 't.in'])   # no --terrain-out
