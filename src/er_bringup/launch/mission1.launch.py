from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    er_planning_dir = get_package_share_directory('er_planning')
    er_mission_dir = get_package_share_directory('er_mission')
    mpl_dir = get_package_share_directory('mini_plus_localization')
    
    # 1. Gestión de Entornos (Conda)
    node_env = conda_python_env()

    # 2. Argumentos de Lanzamiento
    enable_global_planning = LaunchConfiguration('enable_global_planning')
    declare_enable_global_planning = DeclareLaunchArgument(
        'enable_global_planning',
        default_value='true',
        description='Lanza persistent_map_node y global_planner_node junto a la misión',
    )

    enable_road_routing = LaunchConfiguration('enable_road_routing')
    declare_enable_road_routing = DeclareLaunchArgument(
        'enable_road_routing',
        default_value='false',
        description='Lanza road_router_node (ruteo vial OSRM nivel 1) y usa routing_waypoint en planificadores',
    )

    seed_map_path = LaunchConfiguration('seed_map_path')
    declare_seed_map_path = DeclareLaunchArgument(
        'seed_map_path',
        default_value='',
        description='Ruta absoluta al mapa semilla .npy (vacio = sin precarga)',
    )

    # 3. Inclusión del EKF
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

    # NODO: Cerebro Visual y Planificador BEV (SAM-TP + GeNIE)
    planner_node = Node(
        package='er_planning',
        executable='bev_planner_node',
        name='bev_planner_node',
        output='screen',
        parameters=[os.path.join(er_planning_dir, 'config', 'planner_params.yaml')],
        condition=UnlessCondition(enable_road_routing),
        additional_env=node_env,
    )

    # NODO: Mapa Persistente Global (Acumulador con Confianza y Decaimiento)
    persistent_map_node = Node(
        package='er_planning',
        executable='persistent_map_node',
        name='persistent_map_node',
        output='screen',
        parameters=[
            os.path.join(er_planning_dir, 'config', 'persistent_map_params.yaml'),
            {'seed_map_path': seed_map_path},
        ],
        condition=IfCondition(enable_global_planning),
        additional_env=node_env,
    )

    # NODO: Planificación Global Incremental D* Lite
    global_planner_node = Node(
        package='er_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[os.path.join(er_planning_dir, 'config', 'global_planner_params.yaml')],
        condition=IfCondition(PythonExpression([
            "'", enable_global_planning, "' == 'true' and '", enable_road_routing, "' != 'true'"
        ])),
        additional_env=node_env,
    )

    # NODO: Ruteo Vial por Grafo OSM/OSRM (Nivel 1 — trayectos largos)
    road_router_node = Node(
        package='er_planning',
        executable='road_router_node',
        name='road_router_node',
        output='screen',
        parameters=[os.path.join(er_planning_dir, 'config', 'road_router_params.yaml')],
        condition=IfCondition(enable_road_routing),
        additional_env=node_env,
    )

    # Cuando road routing está activo, los planificadores usan routing_waypoint
    planner_with_routing = Node(
        package='er_planning',
        executable='bev_planner_node',
        name='bev_planner_node',
        output='screen',
        parameters=[
            os.path.join(er_planning_dir, 'config', 'planner_params.yaml'),
            {'target_topic': 'earth_rover/routing_waypoint'},
        ],
        condition=IfCondition(enable_road_routing),
        additional_env=node_env,
    )

    global_planner_with_routing = Node(
        package='er_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            os.path.join(er_planning_dir, 'config', 'global_planner_params.yaml'),
            {'target_topic': 'earth_rover/routing_waypoint'},
        ],
        condition=IfCondition(PythonExpression([
            "'", enable_global_planning, "' == 'true' and '", enable_road_routing, "' == 'true'"
        ])),
        additional_env=node_env,
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
        declare_enable_global_planning,
        declare_enable_road_routing,
        declare_seed_map_path,
        LogInfo(msg="[FASE 1] Inicializando Hardware SDK, Filtros EKF y Planificador BEV (PyTorch/GeNIE)..."),
        
        # Arrancan de inmediato:
        bridge_node,
        ekf_launch,
        planner_node,
        planner_with_routing,
        persistent_map_node,
        global_planner_node,
        global_planner_with_routing,
        road_router_node,

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