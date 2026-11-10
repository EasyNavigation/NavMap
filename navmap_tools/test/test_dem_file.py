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
Unit tests for navmap_tools.geo.dem_file -- the height-grid file bigmap_maps_manager reads.

`test/data/dem_golden.bin` was made by the C++ side (BigMap's former DemGrid::build and save, from
the analytic function below); BigMap keeps the same file and checks it from C++. Here the
Python writer must reproduce its bytes and its sampling must agree with its values.
"""

from pathlib import Path

from navmap_tools.geo.dem_file import (
    add_imagery,
    build_local_dem,
    DemSource,
    EnuProjection,
    read_dem_file,
    write_dem_file,
)

import numpy as np

import pytest

GOLDEN = Path(__file__).parent / 'data' / 'dem_golden.bin'


class _Analytic:
    """The function the golden file was made from."""

    def sample_array(self, lon, lat):
        return 650.0 + 30.0 * np.sin(lat * 20000.0) + 20.0 * np.cos(lon * 15000.0)


def _build():
    return build_local_dem(
        _Analytic(), 40.4, -3.7, radius=4.0, resolution=1.0, source=DemSource.COPERNICUS,
        data_resolution=2.0)


def test_python_build_agrees_with_the_cpp_golden_file():
    golden = read_dem_file(GOLDEN)
    mine = _build()
    assert mine.side == golden.side == 9
    assert mine.source == golden.source == DemSource.COPERNICUS
    assert mine.resolution == golden.resolution
    assert mine.data_resolution == golden.data_resolution == 2.0
    assert mine.reference_altitude == pytest.approx(golden.reference_altitude, abs=1e-4)
    assert np.isnan(golden.requested_reference) and np.isnan(mine.requested_reference)
    assert mine.z == pytest.approx(golden.z, abs=1e-3)   # bicubic + ENU, as the C++ side


def test_python_writer_reproduces_the_bytes_of_the_cpp_writer(tmp_path):
    out = tmp_path / 'again.bin'
    write_dem_file(read_dem_file(GOLDEN), out)
    assert out.read_bytes() == GOLDEN.read_bytes()


def test_origin_is_a_node_and_the_reference():
    dem = _build()
    assert dem.z[4, 4] == pytest.approx(0.0, abs=1e-6)


def test_direct_sampling_when_data_resolution_is_not_coarser():
    dem = build_local_dem(
        _Analytic(), 40.4, -3.7, radius=4.0, resolution=1.0, source=DemSource.IGN)
    assert dem.data_resolution == 1.0
    assert dem.side == 9
    # row j is north, column i is east
    proj = EnuProjection(40.4, -3.7)
    lat, lon, _ = proj.to_geodetic(np.array([2.0]), np.array([-3.0]), 0.0)
    expected = _Analytic().sample_array(lon, lat)[0] - dem.reference_altitude
    assert dem.z[4 - 3, 4 + 2] == pytest.approx(expected, abs=1e-3)


def test_radius_that_is_not_a_multiple_rounds_the_grid_up():
    dem = build_local_dem(
        _Analytic(), 40.4, -3.7, radius=4.2, resolution=1.0, source=DemSource.IGN)
    assert dem.side == 11   # ceil(4.2) = 5 nodes each way


def test_given_reference_altitude_is_kept():
    dem = build_local_dem(
        _Analytic(), 40.4, -3.7, radius=4.0, resolution=1.0, source=DemSource.IGN,
        reference_altitude=600.0)
    assert dem.reference_altitude == 600.0
    assert dem.requested_reference == 600.0
    assert dem.z[4, 4] == pytest.approx(_Analytic().sample_array(
        np.array([-3.7]), np.array([40.4]))[0] - 600.0, abs=1e-3)


def test_scalar_samplers_work_too():
    class Scalar:

        def sample(self, lon, lat):
            return 100.0

    dem = build_local_dem(Scalar(), 40.4, -3.7, 3.0, 1.0, DemSource.MESH)
    assert (dem.z == 0.0).all()


def test_bad_arguments_are_rejected():
    with pytest.raises(ValueError):
        build_local_dem(_Analytic(), 40.4, -3.7, radius=0.0, resolution=1.0, source=2)
    with pytest.raises(ValueError):
        build_local_dem(_Analytic(), 40.4, -3.7, radius=10.0, resolution=-1.0, source=2)
    with pytest.raises(ValueError):   # over what DemGrid::load accepts
        build_local_dem(_Analytic(), 40.4, -3.7, radius=30000.0, resolution=1.0, source=2)


def test_imagery_colours_round_trip(tmp_path):
    class Red:

        def sample(self, lon, lat):
            return (200, 10, 20)

    dem = build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN)
    add_imagery(dem, Red(), zoom=19)
    path = tmp_path / 'dem.bin'
    write_dem_file(dem, path)
    back = read_dem_file(path)
    assert back.imagery_zoom == 19
    assert back.rgb.shape == (7, 7, 3)
    assert tuple(back.rgb[3, 3]) == (200, 10, 20)
    assert back.z == pytest.approx(dem.z)


def test_file_without_imagery_has_no_colours(tmp_path):
    dem = build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN)
    path = tmp_path / 'dem.bin'
    write_dem_file(dem, path)
    back = read_dem_file(path)
    assert back.rgb is None and back.imagery_zoom == 0


def test_the_cells_with_ground_round_trip_with_and_without_colours(tmp_path):
    class Red:

        def sample(self, lon, lat):
            return (200, 10, 20)

    for colours in (False, True):
        dem = build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN)
        if colours:
            add_imagery(dem, Red(), zoom=19)
        cells = np.ones((6, 6), np.uint8)
        cells[2, 4] = 0   # a hole at cell (4, 2)
        dem.cell_valid = cells
        path = tmp_path / 'dem.bin'
        write_dem_file(dem, path)
        back = read_dem_file(path)
        assert np.array_equal(back.cell_valid, cells)
        assert (back.rgb is not None) == colours
        assert back.z == pytest.approx(dem.z)
    assert read_dem_file(GOLDEN).cell_valid is None   # older than the block: no holes


def test_a_broken_cells_block_and_the_wrong_number_of_cells_are_refused(tmp_path):
    dem = build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN)
    dem.cell_valid = np.ones((6, 6), np.uint8)
    path = tmp_path / 'dem.bin'
    write_dem_file(dem, path)
    data = path.read_bytes()
    path.write_bytes(data[:-5])
    with pytest.raises(ValueError):
        read_dem_file(path)
    path.write_bytes(data[:-36 - 8] + b'XELLS\0\0\0' + data[-36:])
    with pytest.raises(ValueError):
        read_dem_file(path)
    dem.cell_valid = np.ones((5, 6), np.uint8)
    with pytest.raises(ValueError):
        write_dem_file(dem, tmp_path / 'other.bin')


def test_read_rejects_other_files(tmp_path):
    path = tmp_path / 'junk.bin'
    path.write_bytes(b'not a dem file at all')
    with pytest.raises(ValueError):
        read_dem_file(path)


def test_enu_matches_geodesy_at_the_origin_and_round_trips():
    proj = EnuProjection(40.4, -3.7)
    e, n, u = proj.to_enu(np.array([40.4]), np.array([-3.7]), 0.0)
    assert (abs(e[0]), abs(n[0]), abs(u[0])) < (1e-6, 1e-6, 1e-6)
    lat, lon, alt = proj.to_geodetic(np.array([123.0]), np.array([-456.0]), 0.0)
    e, n, _ = proj.to_enu(lat, lon, alt)
    assert e[0] == pytest.approx(123.0, abs=1e-6)
    assert n[0] == pytest.approx(-456.0, abs=1e-6)
    # north is +y, east is +x
    lat_n, lon_n, _ = proj.to_geodetic(np.array([0.0]), np.array([100.0]), 0.0)
    assert lat_n[0] > 40.4 and lon_n[0] == pytest.approx(-3.7, abs=1e-6)
    lat_e, lon_e, _ = proj.to_geodetic(np.array([100.0]), np.array([0.0]), 0.0)
    assert lon_e[0] > -3.7


def test_synthetic_dem_and_imagery_are_offline_and_deterministic():
    from navmap_tools.geo.dem_file import SyntheticDem, SyntheticImagery
    dem = SyntheticDem(40.4, -3.7)
    # at the origin x = y = 0: 650 + 20 * exp(-(80^2 + 60^2) / 3200) + 0 + 0
    assert dem.sample(-3.7, 40.4) == pytest.approx(650.0 + 20.0 * np.exp(-10000.0 / 3200.0))
    imagery = SyntheticImagery(40.4, -3.7)
    assert imagery.sample(-3.7, 40.4) == (255, 255, 255)   # on the white axes
    r, g, b = imagery.sample_array(np.array([-3.6995]), np.array([40.4005]))[0]
    assert g == 60 and (r, b) in ((200, 60), (60, 200))    # a square of the chessboard


def test_a_dem_file_made_from_a_synthetic_terrain_round_trips(tmp_path):
    from navmap_tools.geo.dem_file import SyntheticDem, SyntheticImagery
    dem = build_local_dem(SyntheticDem(40.4, -3.7), 40.4, -3.7, 10.0, 1.0, DemSource.SYNTHETIC)
    add_imagery(dem, SyntheticImagery(40.4, -3.7), zoom=18)
    write_dem_file(dem, tmp_path / 'd.bin')
    back = read_dem_file(tmp_path / 'd.bin')
    assert back.source == DemSource.SYNTHETIC and back.imagery_zoom == 18
    assert back.matches(40.4, -3.7, 10.0, 1.0, DemSource.SYNTHETIC)


def test_truncated_file_is_reported(tmp_path):
    good = tmp_path / 'good.bin'
    write_dem_file(build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN), good)
    short = tmp_path / 'short.bin'
    short.write_bytes(good.read_bytes()[:100])
    with pytest.raises(ValueError):
        read_dem_file(short)


def _patched_header(tmp_path, side=None, resolution=None, lat=None, reference=None):
    """Return a good file with one header value replaced (a corrupt or hostile file)."""
    import struct
    good = tmp_path / 'good.bin'
    write_dem_file(build_local_dem(_Analytic(), 40.4, -3.7, 3.0, 1.0, DemSource.IGN), good)
    data = bytearray(good.read_bytes())
    s_, src, *hdr = struct.unpack_from('<ii7d', data, 8)
    if side is not None:
        s_ = side
    if resolution is not None:
        hdr[0] = resolution
    if lat is not None:
        hdr[2] = lat
    if reference is not None:
        hdr[4] = reference
    struct.pack_into('<ii7d', data, 8, s_, src, *hdr)
    bad = tmp_path / 'bad.bin'
    bad.write_bytes(bytes(data))
    return bad


@pytest.mark.parametrize('field', [
    {'side': 1}, {'resolution': 0.0}, {'resolution': -1.0}, {'resolution': float('nan')},
    {'resolution': float('inf')}, {'lat': float('nan')}, {'reference': float('inf')},
])
def test_the_reader_refuses_the_headers_the_cpp_reader_refuses(tmp_path, field):
    # DemGrid::load (bigmap) rejects side < 3, a non-positive or non-finite resolution and a
    # non-finite origin or reference: the Python reader must not accept what the C++ one will not
    # (height_at of a side-1 grid would read node -1)
    with pytest.raises(ValueError):
        read_dem_file(_patched_header(tmp_path, **field))
