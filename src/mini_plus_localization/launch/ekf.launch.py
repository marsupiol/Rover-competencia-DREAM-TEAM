from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    mpl_dir = get_package_share_directory('mini_plus_localization')
    ekf_yaml = os.path.join(mpl_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # TF estatico base_link -> earth_rover_gps (asume antena en el centro
        # del robot; ajusta --x/--y/--z si la antena esta desplazada).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'earth_rover_gps',
            ],
        ),

        # EKF local (odom frame): funde /wheel_odom (vx,vy) + /imu/data (yaw, vyaw)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_yaml],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),

        # EKF global (map frame): funde wheel_odom + imu + /odometry/gps
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_yaml],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),

        # navsat_transform: proyecta /gps/fix a /odometry/gps y devuelve el
        # GPS ya fusionado directo en earth_rover/gps (sin tocar er_navigation)

        # navsat_transform: Proyecta /gps/fix a /odometry/gps y genera /gps/filtered
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_yaml],   # sin {'use_sim_time': True}
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('gps/filtered', 'gps/filtered'),
            ],
        ),

        # Traduce el yaw fusionado (map, ENU) a heading en grados (compass),
        # publicado en earth_rover/heading -- mismo topic que ya consumía
        # gps_waypoint_controller.
        Node(
            package='mini_plus_localization',
            executable='ekf_heading_bridge.py',
            name='ekf_heading_bridge',
            output='screen',
        ),
    ])