# SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port', default_value='8080', description='TCP port for SDK communication'
    )

    unity_tcp_port_arg = DeclareLaunchArgument(
        'unity_tcp_port',
        default_value='10000',
        description='TCP port for Unity communication',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)',
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false', description='Enable verbose logging'
    )

    enable_unity_bridge_arg = DeclareLaunchArgument(
        'enable_unity_bridge',
        default_value='true',
        description='Enable Unity TCP bridge',
    )

    # Main HORUS backend node
    horus_backend_node = Node(
        package='horus_backend',
        executable='horus_backend_node',
        name='horus_backend',
        output='screen',
        parameters=[
            {
                'tcp_port': LaunchConfiguration('tcp_port'),
                'log_level': LaunchConfiguration('log_level'),
                'unity_tcp_port': LaunchConfiguration('unity_tcp_port'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # HORUS Unity Bridge for Unity communication
    unity_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('horus_unity_bridge'),
                'launch',
                'unity_bridge.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_unity_bridge')),
    )

    return LaunchDescription(
        [
            tcp_port_arg,
            unity_tcp_port_arg,
            log_level_arg,
            verbose_arg,
            enable_unity_bridge_arg,
            horus_backend_node,
            unity_bridge_launch,
        ]
    )
