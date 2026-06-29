# Copyright 2026 RICE Lab, University of Genoa
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('horus_unity_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')

    # Unity bridge node
    unity_bridge_node = Node(
        package='horus_unity_bridge',
        executable='horus_unity_bridge_node',
        name='horus_unity_bridge',
        output='screen',
        parameters=[config_file] if os.path.exists(config_file) else [],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        unity_bridge_node
    ])
