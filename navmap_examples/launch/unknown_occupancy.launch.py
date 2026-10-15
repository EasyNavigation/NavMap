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

"""Log-odds occupancy and unknown values: the example node and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz = os.path.join(get_package_share_directory('navmap_examples'), 'rviz',
                        'unknown_occupancy.rviz')
    return LaunchDescription([
        Node(package='navmap_examples', executable='04_unknown_and_occupancy', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz]),
    ])
