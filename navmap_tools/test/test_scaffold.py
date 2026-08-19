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

"""Unit tests for navmap_tools.scaffold -- pure filesystem/templating, no network."""

import xml.etree.ElementTree as ET

from navmap_tools.scaffold import scaffold_world_package, short_name_for, world_paths

import pytest


# ---------------------------------------------------------------------------
# short_name_for
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    'package,expected',
    [
        ('urjc_excavation_world', 'urjc_excavation'),
        ('gis_40p0000_m3p0000_world', 'gis_40p0000_m3p0000'),
        ('no_suffix_here', 'no_suffix_here'),
        ('_world', '_world'),  # too short to strip without becoming empty
        ('world', 'world'),  # no "_world" suffix present
    ],
)
def test_short_name_for(package, expected):
    assert short_name_for(package) == expected


# ---------------------------------------------------------------------------
# world_paths
# ---------------------------------------------------------------------------

def test_world_paths_layout(tmp_path):
    paths = world_paths(tmp_path, 'foo_world')
    assert paths.package_name == 'foo_world'
    assert paths.name == 'foo'
    assert paths.root == tmp_path
    assert paths.model_dir == tmp_path / 'models' / 'foo'
    assert paths.meshes_dir == tmp_path / 'models' / 'foo' / 'meshes'
    assert paths.pcd_path == tmp_path / 'maps' / 'foo.pcd'
    assert paths.navmap_path == tmp_path / 'maps' / 'foo.navmap'
    assert paths.colors_csv_path == tmp_path / 'maps' / 'foo_colors.csv'
    assert paths.world_path == tmp_path / 'worlds' / 'foo.world'
    assert paths.dae_path == tmp_path / 'models' / 'foo' / 'meshes' / 'foo.dae'
    assert paths.stl_path == tmp_path / 'models' / 'foo' / 'meshes' / 'foo.stl'
    assert paths.texture_path == tmp_path / 'models' / 'foo' / 'meshes' / 'foo_texture.png'


# ---------------------------------------------------------------------------
# scaffold_world_package
# ---------------------------------------------------------------------------

_IMAGERY_DESC = 'Esri World Imagery, zoom 19 (~0.30 m/px at this latitude)'
_IMAGERY_ATTRIBUTION = (
    'Imagery (c) Esri, Maxar, Earthstar Geographics, and the GIS User Community.')


def test_scaffold_rejects_invalid_package_name(tmp_path):
    with pytest.raises(ValueError, match='package name'):
        scaffold_world_package(tmp_path, '', 0.0, 0.0, 100.0, 500.0, _IMAGERY_DESC)
    with pytest.raises(ValueError, match='package name'):
        scaffold_world_package(tmp_path, 'bad name!', 0.0, 0.0, 100.0, 500.0, _IMAGERY_DESC)


def test_scaffold_creates_expected_directory_tree(tmp_path):
    paths = scaffold_world_package(
        tmp_path, 'foo_world', 40.33, -3.83, 120.0, 650.0, _IMAGERY_DESC)
    assert (paths.root / 'package.xml').is_file()
    assert (paths.root / 'CMakeLists.txt').is_file()
    assert (paths.env_hooks_dir / 'foo_world.dsv.in').is_file()
    assert (paths.model_dir / 'model.config').is_file()
    assert (paths.model_dir / 'model.sdf').is_file()
    assert paths.world_path.is_file()
    assert (paths.launch_dir / 'foo.launch.py').is_file()
    assert (paths.root / 'README.md').is_file()
    assert paths.meshes_dir.is_dir()
    assert paths.maps_dir.is_dir()


def test_scaffold_package_xml_is_valid_xml_with_right_name(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    root = ET.parse(paths.root / 'package.xml').getroot()
    assert root.findtext('name') == 'foo_world'


def test_scaffold_world_sdf_contains_gps_center(tmp_path):
    paths = scaffold_world_package(
        tmp_path, 'foo_world', 40.33, -3.83, 100.0, 650.5, _IMAGERY_DESC)
    text = paths.world_path.read_text()
    assert '<latitude_deg>40.33</latitude_deg>' in text
    assert '<longitude_deg>-3.83</longitude_deg>' in text
    assert '<elevation>650.5</elevation>' in text
    assert '<uri>model://foo</uri>' in text


def test_scaffold_model_sdf_references_short_name_meshes(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    text = (paths.model_dir / 'model.sdf').read_text()
    assert 'model://foo/meshes/foo.stl' in text
    assert 'model://foo/meshes/foo.dae' in text


def test_scaffold_model_config_mentions_imagery_desc(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    text = (paths.model_dir / 'model.config').read_text()
    assert _IMAGERY_DESC in text


def test_scaffold_launch_py_is_valid_python(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    launch_path = paths.launch_dir / 'foo.launch.py'
    compile(launch_path.read_text(), str(launch_path), 'exec')


def test_scaffold_readme_mentions_package_and_center(tmp_path):
    paths = scaffold_world_package(
        tmp_path, 'foo_world', 40.33, -3.83, 100.0, 0.0, _IMAGERY_DESC)
    text = (paths.root / 'README.md').read_text()
    assert 'foo_world' in text
    assert '40.33' in text
    assert '-3.83' in text


def test_scaffold_readme_mentions_imagery_desc_and_attribution(tmp_path):
    paths = scaffold_world_package(
        tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC, _IMAGERY_ATTRIBUTION)
    text = (paths.root / 'README.md').read_text()
    assert _IMAGERY_DESC in text
    assert _IMAGERY_ATTRIBUTION in text


def test_scaffold_readme_imagery_attribution_defaults_to_empty(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    text = (paths.root / 'README.md').read_text()
    assert _IMAGERY_DESC in text


def test_scaffold_does_not_touch_maps_dir_contents(tmp_path):
    paths = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    sentinel = paths.maps_dir / 'foo.pcd'
    sentinel.write_text('pretend point cloud')

    # Re-scaffolding (e.g. a later --gazebo-only run into the same
    # --output-dir) must not clobber maps/ written by an earlier --navmap run.
    scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    assert sentinel.read_text() == 'pretend point cloud'


def test_scaffold_is_idempotent_on_scaffold_files(tmp_path):
    paths1 = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    text1 = paths1.world_path.read_text()
    paths2 = scaffold_world_package(tmp_path, 'foo_world', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    assert text1 == paths2.world_path.read_text()


def test_scaffold_package_without_world_suffix_uses_same_name_for_both(tmp_path):
    paths = scaffold_world_package(tmp_path, 'myarea', 0.0, 0.0, 100.0, 0.0, _IMAGERY_DESC)
    assert paths.package_name == 'myarea'
    assert paths.name == 'myarea'
    assert (paths.root / 'models' / 'myarea' / 'model.sdf').is_file()
