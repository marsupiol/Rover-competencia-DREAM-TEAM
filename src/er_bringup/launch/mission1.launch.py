from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import sys


def conda_python_env():
    conda_prefix = os.environ.get('CONDA_PREFIX')
    if not conda_prefix:
        return {}

    conda_site_packages = os.path.join(
        conda_prefix,
        'lib',
        f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages',
    )
    if not os.path.isdir(conda_site_packages):
        return {}

    pythonpath = os.environ.get('PYTHONPATH', '')
    paths = [conda_site_packages]
    if pythonpath:
        paths.append(pythonpath)
    return {'PYTHONPATH': os.pathsep.join(paths)}


def generate_launch_description():
    # Directories for config files (could be empty for now)
    er_hardware_dir = get_package_share_directory('er_hardware')
    er_navigation_dir = get_package_share_directory('er_navigation')
    er_mission_dir = get_package_share_directory('er_mission')
    node_env = conda_python_env()

    return LaunchDescription([
        # Deshabilitado: se usa el bridge_node.py del EKF (paquete earth_rovers_sdk, otro workspace),
        # que ya publica earth_rover/gps y earth_rover/heading con los mismos tipos/tópicos.
        # Levantar ese stack por separado con: ros2 launch mini_plus_localization ekf.launch.py
        # antes de correr este mission1.launch.py.
        # Node(
        #     package='er_hardware',
        #     executable='sdk_bridge_node',
        #     name='sdk_bridge_node',
        #     output='screen',
        #     parameters=[os.path.join(er_hardware_dir, 'config', 'bridge_params.yaml')],
        #     additional_env=node_env,
        # ),
        Node(
            package='er_navigation',
            executable='gps_waypoint_controller',
            name='gps_waypoint_controller',
            output='screen',
            parameters=[os.path.join(er_navigation_dir, 'config', 'navigation_params.yaml')],
            additional_env=node_env,
        ),
        Node(
            package='er_mission',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[os.path.join(er_mission_dir, 'config', 'mission_params.yaml')],
            additional_env=node_env,
        ),
    ])
