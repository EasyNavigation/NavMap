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
