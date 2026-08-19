from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    er_navigation_dir = get_package_share_directory('er_navigation')
    er_mission_dir = get_package_share_directory('er_mission')
    mpl_dir = get_package_share_directory('mini_plus_localization')
    
    # 1. Gestión de Entornos (Conda + ROCm para AMD)
    node_env = conda_python_env()
    
    # Preparamos un entorno específico para la IA que fuerza el uso de la RX 570
    ai_env = {'HSA_OVERRIDE_GFX_VERSION': '8.0.3'}
    ai_env.update(node_env)

    # 2. Inclusión del EKF
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mpl_dir, 'launch', 'ekf.launch.py')
        )
    )

    # ==========================================================
    # DEFINICIÓN DE NODOS
    # ==========================================================
    
    bridge_node = Node(
        package='earth_rovers_sdk',
        executable='earth_rover_bridge',
        name='earth_rover_bridge',
        output='screen',
        parameters=[{
            'sdk_url': 'http://127.0.0.1:8000',
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
    )

    # NODO NUEVO: Cerebro Visual (SAM-TP)
    perception_node = Node(
        package='er_perception',
        executable='sam_tp_perception_node',
        name='sam_tp_perception_node',
        output='screen',
        additional_env=ai_env, # Inyectamos variables para la GPU AMD
    )

    navigation_node = Node(
        package='er_navigation',
        executable='gps_waypoint_controller',
        name='gps_waypoint_controller',
        output='screen',
        parameters=[os.path.join(er_navigation_dir, 'config', 'navigation_params.yaml')],
        additional_env=node_env,
    )

    mission_node = Node(
        package='er_mission',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[os.path.join(er_mission_dir, 'config', 'mission_params.yaml')],
        remappings=[('earth_rover/gps', 'gps/filtered')],
        additional_env=node_env,
    )

    # ==========================================================
    # GRAFO DE EJECUCIÓN ORQUESTADO (Fases)
    # ==========================================================
    return LaunchDescription([
        LogInfo(msg="[FASE 1] Inicializando Hardware SDK, Filtros EKF y Cerebro Visual (PyTorch)..."),
        
        # Arrancan de inmediato:
        bridge_node,
        ekf_launch,
        perception_node,

        # Arrancan con retraso de 5 segundos para evitar la saturación de CPU de PyTorch:
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[FASE 2] Sistema estabilizado. Iniciando Control Motriz y Misión Híbrida..."),
                navigation_node,
                mission_node
            ]
        )
    ])