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

"""Unit tests for navmap_tools.map_builder (the helpers; the node needs a simulation)."""

import math

from navmap_tools.map_builder import (
    build_arg_parser, new_candidates, parse_gz_pose, quat_to_mat, to_world, voxel, yaw_quat,
)
import numpy as np
import pytest

GZ_POSE = """Name: tiago
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
              [1.500000 -2.000000 0.000100]
              [0.000000 0.000000 1.570796]
"""


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 3, math.pi])
def test_yaw_quat_rotates_around_z(yaw):
    m = quat_to_mat(yaw_quat(yaw))
    np.testing.assert_allclose(m @ [1.0, 0.0, 0.0], [math.cos(yaw), math.sin(yaw), 0.0],
                               atol=1e-12)
    np.testing.assert_allclose(m @ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], atol=1e-12)


def test_quat_to_mat_is_a_rotation():
    q = np.array([0.1, -0.3, 0.2, 0.9])
    m = quat_to_mat(q / np.linalg.norm(q))
    np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(m) == pytest.approx(1.0)


def test_voxel_keeps_one_point_per_voxel_in_order():
    pts = np.array([[0.01, 0.01, 0.0], [0.02, 0.03, 0.01], [0.2, 0.0, 0.0], [0.21, 0.01, 0.0]])
    out = voxel(pts, 0.05)
    np.testing.assert_allclose(out, pts[[0, 2]])


def test_voxel_of_an_empty_cloud_is_empty():
    assert voxel(np.zeros((0, 3)), 0.05).shape == (0, 3)


def test_parse_gz_pose_reads_position_and_yaw():
    assert parse_gz_pose(GZ_POSE) == pytest.approx((1.5, -2.0, 1.570796))


@pytest.mark.parametrize('text', ['', 'Unable to find model tiago', '[1 2 3]'])
def test_parse_gz_pose_without_a_pose_is_none(text):
    assert parse_gz_pose(text) is None


def test_to_world_applies_the_sensor_mount_and_the_robot_pose():
    # Sensor 0.2 m ahead and 1 m up, looking forward; robot at (2, 1) facing +y
    points = np.array([[1.0, 0.0, 0.0], [0.0, 0.5, -1.0]])
    out = to_world(points, ((0.2, 0.0, 1.0), yaw_quat(0.0)), (2.0, 1.0, math.pi / 2), 0.1)
    np.testing.assert_allclose(out, [[2.0, 2.2, 1.0], [1.5, 1.2, 0.0]], atol=1e-12)


def test_to_world_drops_the_robot_itself():
    points = np.array([[0.3, 0.0, 0.5], [2.0, 0.0, 0.5], [0.0, -0.74, 0.1], [0.0, -0.76, 0.1]])
    out = to_world(points, ((0.0, 0.0, 0.0), yaw_quat(0.0)), (0.0, 0.0, 0.0), 0.75)
    np.testing.assert_allclose(out, [[2.0, 0.0, 0.5], [0.0, -0.76, 0.1]], atol=1e-12)


def ground_disc(cx, cy, radius=2.0, step=0.1):
    xs = np.arange(cx - radius, cx + radius + 1e-9, step)
    return np.array([[x, y, 0.0] for x in xs for y in xs - cx + cy
                     if math.hypot(x - cx, y - cy) <= radius])


def test_new_candidates_are_the_grid_neighbours_with_ground_and_no_obstacle():
    cloud = ground_disc(0.0, 0.0)
    out = new_candidates(0.0, 0.0, cloud, 1.0, 0.8, {(0, 0)}, (0.1, 1.2))
    assert sorted(out) == sorted([(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
                                  (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)])


def test_new_candidates_skip_tried_positions():
    cloud = ground_disc(0.0, 0.0)
    out = new_candidates(0.0, 0.0, cloud, 1.0, 0.8, {(0, 0), (1, 0), (0, 1)}, (0.1, 1.2))
    assert (1.0, 0.0) not in out and (0.0, 1.0) not in out and (-1.0, 0.0) in out


def test_new_candidates_keep_clear_of_obstacles_in_the_band():
    wall = np.array([[1.3, y, z] for y in np.arange(-2, 2.01, 0.1) for z in (0.5, 1.0)])
    high = np.array([[-1.3, y, 1.5] for y in np.arange(-2, 2.01, 0.1)])  # above the band
    cloud = np.vstack([ground_disc(0.0, 0.0), wall, high])
    out = new_candidates(0.0, 0.0, cloud, 1.0, 0.8, {(0, 0)}, (0.1, 1.2))
    assert all(cx < 1.0 for cx, _ in out)   # x = 1 is within 0.8 m of the wall
    assert (-1.0, 0.0) in out               # the overhead points do not block


def test_new_candidates_need_ground_seen_around():
    cloud = ground_disc(0.0, 0.0, radius=0.3)   # the candidates 1 m away saw no ground
    assert new_candidates(0.0, 0.0, cloud, 1.0, 0.8, {(0, 0)}, (0.1, 1.2)) == []


def test_arg_parser_takes_several_cloud_topics_and_defaults():
    args = build_arg_parser().parse_args(
        ['/tmp/house', '--model', 'tiago', '--cloud-topic', '/a/points', '/b/points',
         '--headings', '8'])
    assert args.cloud_topic == ['/a/points', '/b/points']
    assert args.headings == 8
    assert args.ground_truth_topic == ''      # pose from Gazebo
    assert args.world == 'default'
    assert tuple(args.obstacle_band) == (0.1, 1.2)


@pytest.mark.parametrize('argv', [['/tmp/x', '--cloud-topic', '/a'], ['/tmp/x', '--model', 'm']])
def test_arg_parser_requires_model_and_cloud_topic(argv):
    with pytest.raises(SystemExit):
        build_arg_parser().parse_args(argv)
