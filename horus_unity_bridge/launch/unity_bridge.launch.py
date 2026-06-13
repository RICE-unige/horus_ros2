from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('horus_unity_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    enable_priority_scheduling = LaunchConfiguration('enable_priority_scheduling')
    parameters = [config_file] if os.path.exists(config_file) else []
    parameters.append({'enable_priority_scheduling': enable_priority_scheduling})
    
    unity_bridge_node = Node(
        package='horus_unity_bridge',
        executable='horus_unity_bridge_node',
        name='horus_unity_bridge',
        output='screen',
        parameters=parameters,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_priority_scheduling',
            default_value='false',
            description='Enable priority scheduling for outbound Unity messages.',
        ),
        unity_bridge_node
    ])
