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

r"""
Build a 3D point cloud map (.pcd, e.g. for Bonxai) of a simulated Gazebo world.

The robot is teleported (gz set_pose) to free positions found as it goes and, at each one,
turned to --headings evenly spaced headings (more than 1 for sensors with a narrow field of
view, such as a camera). The clouds of its sensors (--cloud-topic, one or more) are put
together at the ground-truth poses. A candidate position is taken when ground was already seen
around it and no obstacle is within --clearance.

The ground truth is --ground-truth-topic (nav_msgs/Odometry) or, without it, the model's pose
in Gazebo (gz model -p).

Run it with the simulation up and nothing else moving the robot, e.g.:
  ros2 run navmap_tools navmap_map_builder /tmp/warehouse --world warehouse --model summit_xl \\
    --cloud-topic /front_laser/points --ground-truth-topic /ground_truth
"""

import argparse
import math
import re
import subprocess
import time

import numpy as np

from .pcd_writer import write_pcd_xyz

GROUND_Z = 0.05      # points within this height are ground
NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def quat_to_mat(q):
    """Rotation matrix of the quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quat(yaw):
    """Quaternion (x, y, z, w) of a rotation of yaw around z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def voxel(points, res):
    """Keep one point per res-sized voxel."""
    if not len(points):
        return points
    keys = np.floor(points / res).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def parse_gz_pose(text):
    """(x, y, yaw) from the output of `gz model -m <model> -p`, or None."""
    vectors = []
    for group in re.findall(r'\[([^\]]+)\]', text):
        try:
            values = [float(v) for v in group.split()]
        except ValueError:
            continue  # labels such as "[ XYZ (m) ]"
        if len(values) == 3:
            vectors.append(values)
    if len(vectors) < 2:
        return None
    return vectors[0][0], vectors[0][1], vectors[1][2]


def to_world(points, sensor_to_robot, robot_pose, robot_cut):
    """
    Put sensor points into the world.

    sensor_to_robot is ((x, y, z), (qx, qy, qz, qw)); robot_pose is (x, y, yaw), the floor at
    z = 0. Points closer than robot_cut (horizontally) to the robot are dropped: the robot itself.
    """
    (t, q) = sensor_to_robot
    pb = points @ quat_to_mat(q).T + list(t)
    pb = pb[np.hypot(pb[:, 0], pb[:, 1]) > robot_cut]
    x, y, yaw = robot_pose
    return pb @ quat_to_mat(yaw_quat(yaw)).T + [x, y, 0.0]


def new_candidates(x, y, cloud, grid, clearance, tried, obstacle_band):
    """
    Free positions around (x, y) worth visiting, given the cloud built so far.

    A neighbour on the grid is taken when it was not tried yet, no obstacle point (inside
    obstacle_band = (z_min, z_max)) is within clearance and some ground (|z| < GROUND_Z) was
    seen near it.
    """
    obst = cloud[(cloud[:, 2] > obstacle_band[0]) & (cloud[:, 2] < obstacle_band[1])]
    ground = cloud[np.abs(cloud[:, 2]) < GROUND_Z]
    result = []
    for dx, dy in NEIGHBOURS:
        cx = round((x + dx * grid) / grid) * grid
        cy = round((y + dy * grid) / grid) * grid
        if (round(cx / grid), round(cy / grid)) in tried:
            continue
        if len(obst) and np.min(np.hypot(obst[:, 0] - cx, obst[:, 1] - cy)) < clearance:
            continue
        if not len(ground) or np.sum(np.hypot(ground[:, 0] - cx, ground[:, 1] - cy) < 0.8) < 5:
            continue
        result.append((cx, cy))
    return result


def build_arg_parser():
    """Argument parser of navmap_map_builder."""
    parser = argparse.ArgumentParser(
        prog='navmap_map_builder', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', help='output prefix: writes <output>.pcd')
    parser.add_argument('--world', default='default', help='Gazebo world name')
    parser.add_argument('--model', required=True, help='robot model name in Gazebo')
    parser.add_argument('--cloud-topic', nargs='+', required=True,
                        help='sensor_msgs/PointCloud2 topics to put together')
    parser.add_argument('--ground-truth-topic', default='',
                        help='nav_msgs/Odometry ground truth (default: pose from Gazebo)')
    parser.add_argument('--robot-frame', default='base_footprint',
                        help='robot frame on the floor, where the ground truth applies')
    parser.add_argument('--start', nargs=2, type=float, default=(0.0, 0.0), metavar=('X', 'Y'),
                        help='where the robot is (first position)')
    parser.add_argument('--headings', type=int, default=1,
                        help='headings taken at each position (evenly spaced)')
    parser.add_argument('--grid', type=float, default=1.0, help='candidate spacing (m)')
    parser.add_argument('--clearance', type=float, default=0.8,
                        help='free radius around a candidate (m)')
    parser.add_argument('--obstacle-band', nargs=2, type=float, default=(0.1, 1.2),
                        metavar=('Z_MIN', 'Z_MAX'),
                        help='height band of the points that block the robot (m)')
    parser.add_argument('--robot-cut', type=float, default=0.75,
                        help='points closer than this to the robot are the robot (m)')
    parser.add_argument('--voxel', type=float, default=0.05, help='cloud resolution (m)')
    parser.add_argument('--settle', type=float, default=2.5,
                        help='wait after each teleport (s)')
    parser.add_argument('--spawn-z', type=float, default=0.15,
                        help='height the robot is teleported to (it drops to the floor) (m)')
    return parser


def main(argv=None):
    """Console entry point."""
    # ROS imports here: the helpers above are testable without a ROS environment
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    import tf2_ros

    class MapBuilder(Node):

        def __init__(self, args):
            super().__init__('map_builder', parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.args = args
            self.clouds = {}
            self.gt = None
            self.buf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buf, self)
            for topic in args.cloud_topic:
                self.create_subscription(
                    PointCloud2, topic, lambda m, t=topic: self.clouds.__setitem__(t, m),
                    qos_profile_sensor_data)
            if args.ground_truth_topic:
                self.create_subscription(Odometry, args.ground_truth_topic, self.gt_cb, 10)

        def gt_cb(self, msg):
            p = msg.pose.pose
            o = p.orientation
            yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            self.gt = (p.position.x, p.position.y, yaw)

        def spin_for(self, sec):
            t0 = time.time()
            while time.time() - t0 < sec:
                rclpy.spin_once(self, timeout_sec=0.05)

        def robot_pose(self):
            if self.args.ground_truth_topic:
                return self.gt
            for _ in range(3):  # gz model sometimes answers nothing under load
                out = subprocess.run(['gz', 'model', '-m', self.args.model, '-p'],
                                     capture_output=True, text=True, timeout=20).stdout
                pose = parse_gz_pose(out)
                if pose is not None:
                    return pose
                time.sleep(0.5)
            return None

        def teleport(self, x, y, yaw):
            qx, qy, qz, qw = yaw_quat(yaw)
            req = (f'name: "{self.args.model}", position: {{x: {x}, y: {y}, '
                   f'z: {self.args.spawn_z}}}, '
                   f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
            subprocess.run(['gz', 'service', '-s', f'/world/{self.args.world}/set_pose',
                            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '2000', '--req', req], capture_output=True, timeout=10)

        def capture(self):
            """Every sensor's last cloud in world coordinates, and the robot pose."""
            self.clouds.clear()
            self.spin_for(self.args.settle)
            pose = self.robot_pose()
            if pose is None or len(self.clouds) < len(self.args.cloud_topic):
                return None, pose
            parts = []
            for msg in self.clouds.values():
                pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                     skip_nans=True)
                pts = pts[np.isfinite(pts).all(axis=1)]
                tr = self.buf.lookup_transform(self.args.robot_frame, msg.header.frame_id,
                                               Time())
                t = tr.transform.translation
                q = tr.transform.rotation
                parts.append(to_world(pts, ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)), pose,
                                      self.args.robot_cut))
            return np.vstack(parts), pose

        def run(self):
            a = self.args
            t0 = time.time()
            while (len(self.clouds) < len(a.cloud_topic) or
                   (a.ground_truth_topic and self.gt is None)) and time.time() - t0 < 30:
                rclpy.spin_once(self, timeout_sec=0.1)

            cloud = np.zeros((0, 3))
            visited = 0
            start = (float(a.start[0]), float(a.start[1]))
            queue = [start]
            tried = set()
            while queue:
                x, y = queue.pop(0)
                key = (round(x / a.grid), round(y / a.grid))
                if key in tried:
                    continue
                tried.add(key)
                reached = True
                for k in range(a.headings):
                    yaw = 2.0 * math.pi * k / a.headings
                    if (x, y) != start or k > 0:
                        self.teleport(x, y, yaw)
                    pw, at = self.capture()
                    if pw is None or math.hypot(at[0] - x, at[1] - y) > 0.3:
                        print(f'skip ({x:.1f}, {y:.1f}) heading {yaw:.2f}: robot at {at}',
                              flush=True)
                        reached = reached and k > 0
                        continue
                    cloud = voxel(np.vstack([cloud, pw]), a.voxel)
                if not reached:
                    continue
                visited += 1
                print(f'position {visited:3d} at ({x:6.2f}, {y:6.2f}): {len(cloud)} points',
                      flush=True)
                queue.extend(new_candidates(x, y, cloud, a.grid, a.clearance, tried,
                                            a.obstacle_band))

            self.teleport(*start, 0.0)
            self.spin_for(2.0)
            write_pcd_xyz(a.output + '.pcd', cloud)
            print(f'done: {visited} positions, {len(cloud)} points in {a.output}.pcd',
                  flush=True)

    args, _ = build_arg_parser().parse_known_args(argv)  # ignores --ros-args
    rclpy.init()
    node = MapBuilder(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
