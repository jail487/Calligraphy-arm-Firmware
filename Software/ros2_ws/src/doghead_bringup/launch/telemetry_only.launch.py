import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('doghead_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'safety_limits.yaml')

    return LaunchDescription([
        # Driver node in telemetry-only / shadow mode
        Node(
            package='doghead_driver',
            executable='driver_node',
            name='doghead_driver_node',
            output='screen',
            parameters=[config_file, {'shadow_mode': True}]
        ),
        # Telemetry validator
        Node(
            package='doghead_hil',
            executable='telemetry_validator',
            name='telemetry_validator_node',
            output='screen',
            parameters=[config_file]
        ),
    ])
