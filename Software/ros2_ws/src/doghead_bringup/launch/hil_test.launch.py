import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('doghead_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'safety_limits.yaml')

    return LaunchDescription([
        # Driver node in shadow mode
        Node(
            package='doghead_driver',
            executable='driver_node',
            name='doghead_driver_node',
            output='screen',
            parameters=[config_file, {'shadow_mode': True}]
        ),
        # Motion adapter node
        Node(
            package='doghead_motion',
            executable='motion_node',
            name='doghead_motion_node',
            output='screen',
            parameters=[config_file]
        ),
        # Telemetry validator node
        Node(
            package='doghead_hil',
            executable='telemetry_validator',
            name='telemetry_validator_node',
            output='screen',
            parameters=[config_file]
        ),
        # HIL scenario runner (actuators explicitly gated OFF by default)
        Node(
            package='doghead_hil',
            executable='scenario_runner',
            name='hil_scenario_runner_node',
            output='screen',
            parameters=[config_file, {'actuator_enabled': False, 'scenario_name': 'telemetry_only'}]
        ),
    ])
