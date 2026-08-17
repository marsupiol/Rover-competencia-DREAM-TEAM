from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import sys
from rclpy.parameter import Parameter # Añadir a tus imports arriba

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
    er_navigation_dir = get_package_share_directory('er_navigation')
    er_mission_dir = get_package_share_directory('er_mission')
    mpl_dir = get_package_share_directory('mini_plus_localization')
    node_env = conda_python_env()

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mpl_dir, 'launch', 'ekf.launch.py')
        )
    )

    return LaunchDescription([
        # Bridge oficial (SDK <-> ROS2): GPS, IMU, wheel odom, camara, bateria
        Node(
            package='earth_rovers_sdk',
            executable='earth_rover_bridge',
            name='earth_rover_bridge',
            output='screen',
            parameters=[{
                'sdk_url': 'http://127.0.0.1:8000',
                # Forzamos floats estrictos (ej. 1.0 en vez de 1) para evitar fallos de parseo
                'gps_position_covariance': [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 4.0,
                ],
                'odom_twist_covariance': [
                    2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 2.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.5,
                ],
            }],
            additional_env=node_env,
        ),

        # EKF local + global + navsat_transform + heading bridge
        ekf_launch,

        # Navegacion y mision: sin cambios, siguen escuchando earth_rover/gps
        # y earth_rover/heading -- ahora alimentados por el EKF.
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