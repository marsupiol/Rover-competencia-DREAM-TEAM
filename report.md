# Reporte de Estado e Implementación

## Estado del Proyecto

Revisamos detalladamente el estado del repositorio y la arquitectura del Rover autónomo (**Earth Rover Mini+**). La arquitectura y la lógica de los nodos ROS 2 de percepción, mapa persistente (Bayesiano), navegación (Burst & Wait + Pure Pursuit + Fallback) y planificación global (**D* Lite**) están **completamente implementadas y listas en el código fuente**.

### Resumen de los Paquetes Implementados:

1. **`er_planning`**:
   - `bev_planner_node`: Inferencia visual SAM-TP, proyección Bird's-Eye-View (BEV), banco de trayectorias polinomiales (GeNIE Path Bank) y guiado por sub-meta global.
   - `persistent_map_node`: Acumulación Bayesiana de mapas de ocupación globales ($400\text{m} \times 400\text{m}$ @ $20\text{cm/px}$) en frame `map` con decaimiento temporal.
   - `global_planner_node`: Algoritmo **D* Lite** incremental para replanificación global continua hacia la meta geodésica.
   - `road_router_node`: Ruteador sobre la grilla del mapa global.

2. **`er_navigation`**:
   - `gps_waypoint_controller`: Controlador motriz híbrido reactivo con mitigación de jitter 4G (Burst & Wait), seguimiento de trayectorias BEV locales y fallback geodésico de seguridad.

3. **`er_mission`**:
   - `mission_manager_node`: Orquestador de misiones y checkpoints con confirmación HTTP asíncrona hacia el SDK Server.

4. **`earth_rovers_sdk` & `src/sdk_server`**:
   - Bridge de comunicación bidireccional HTTP/WebSocket/MJPEG con el rover físico y sesión WebRTC vía Agora.

5. **`tools/osm_seed` & `tools/mission_prep`**:
   - Generación automática de mapa semilla desde OpenStreetMap (`generate_seed_map.py`), resolución dinámica de datum geodésico (`resolve_datum.py`) y orquestador unificado de misión (`run_mission.sh`).

---

## Cambios y Ajustes Realizados para Ejecución Nativa (Conda + ROS 2 Sistema)

1. **Correcciones en `requirements-ros.txt`**:
   - Agregados `catkin_pkg`, `empy`, `lark` y `uvicorn`. `catkin_pkg` es indispensable para que `colcon build` / `ament_cmake` procese correctamente `package.xml` al compilar el paquete C++ `mini_plus_localization` cuando la compilación se ejecuta con un entorno de Python activo.

2. **Correcciones en `requirements-ai.txt`**:
   - Agregadas dependencias geoespaciales (`osmnx`, `networkx`, `shapely`, `pyproj`) requeridas por el módulo de precarga de mapas semilla de OpenStreetMap.

3. **Corrección de Submódulos Git (`sana-earth-rover-policy`)**:
   - Identificada e instruida la inicialización del submódulo Git (`git submodule update --init --recursive`) para poder instalar en modo editable los paquetes de IA `genie` y `traversability`.

4. **Script de Instalación Unificado ([install_environment.sh](file:///home/martin/Rover-competencia-DREAM-TEAM/install_environment.sh))**:
   - Generado el script que separa claramente los paquetes del sistema (vía `sudo apt-get`) de las librerías de Python e IA (vía `pip` dentro de Conda).

---

## Siguientes Pasos Recomendados para Ejecutar

Si estás en tu entorno de Conda (`conda activate rover`), asegurate de correr los siguientes 2 comandos antes de compilar:

```bash
# 1. Instalar catkin_pkg en tu Conda para que colcon build compile mini_plus_localization:
pip install catkin_pkg empy lark

# 2. Compilar el workspace:
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Para probar la misión completa con el orquestador:
```bash
./tools/mission_prep/run_mission.sh --mission-slug <slug-de-la-mision>
```
