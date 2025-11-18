from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'motion_tracking_bot'

    # Path to the YAML file
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'motion_tracking_bot.yaml'
    )

    ir_sensor_node = Node(
        package=pkg_name,
        executable='ir_sensor_node',
        name='ir_sensor_node',
        parameters=[config],
        output='screen'
    )

    speaker_node = Node(
        package=pkg_name,
        executable='speaker_node',
        name='speaker_node',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([
        ir_sensor_node,
        speaker_node,
    ])
