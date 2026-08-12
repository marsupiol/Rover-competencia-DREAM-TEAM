import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('mini_plus_localization'),
        'config',
        'ekf.yaml'
    )

    sdk_url_arg = DeclareLaunchArgument(
        'sdk_url',
        default_value='http://localhost:8000',
        description='URL del servidor SDK del Earth Rover Mini+'
    )

    bridge_node = Node(
        package='earth_rovers_sdk',
        executable='earth_rover_bridge',
        name='earth_rover_bridge',
        output='screen',
        parameters=[{'sdk_url': LaunchConfiguration('sdk_url')}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_dir]
    )

    return LaunchDescription([
        sdk_url_arg,
        bridge_node,
        ekf_node
    ])