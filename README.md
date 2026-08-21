# Earth Rover Mini+ — Stack Autónomo ROS 2 Jazzy

Arquitectura de navegación autónoma, percepción visual profunda (SAM-TP), planificación en vista de pájaro (BEV GeNIE), mapeo global persistente Bayesiano, planificación incremental (D* Lite) y control reactivo para el rover **FrodoBots Earth Rover Mini+** sobre **ROS 2 Jazzy**.

---

## 1. Diagrama de Arquitectura del Sistema

```text
                               +-------------------------------------------------------------+
                               |                 FrodoBots Cloud / Rover Físico              |
                               +-------------------------------------------------------------+
                                     ▲ (WebRTC / Agora RTM)                  │ (MJPEG / WS)
                                     │                                       ▼
                       +-----------------------------------------------------------------------------+
                       |                       src/sdk_server (FastAPI / Hypercorn)                 |
                       |   Endpoints: POST /control | GET /feed | WS /ws/data | POST /checkpoint-reached |
                       +-----------------------------------------------------------------------------+
                                     ▲                                       │
                                     │ HTTP (10 Hz)                          │ MJPEG Stream & WebSockets
                                     │                                       ▼
+----------------------------------------------------------------------------------------------------+
| PAQUETE: earth_rovers_sdk                                                                          |
|                                                                                                    |
|  [earth_rover_bridge]                                                                              |
|    - Publica cámara frontal: earth_rover/front/image_raw (sensor_msgs/Image)                       |
|    - Publica telemetría GNSS cruda: /gps/fix (sensor_msgs/NavSatFix)                              |
|    - Publica IMU MPU-6050: /imu/data (sensor_msgs/Imu)                                             |
|    - Publica odometría cinemática: /wheel_odom (nav_msgs/Odometry)                                 |
|    - Publica rumbo magnético: earth_rover/heading (std_msgs/Float32)                               |
|    - Suscribe control motriz: cmd_vel (geometry_msgs/Twist)                                        |
+----------------------------------------------------------------------------------------------------+
       │                          │                     │                            ▲
       │ /gps/fix, /imu/data      │ image_raw           │ heading                    │ cmd_vel
       ▼                          │                     ▼                            │
+------------------------------+  │  +---------------------------------------------------------------+
| PAQUETE:                     |  │  | PAQUETE: er_planning                                          |
| mini_plus_localization       |  │  |                                                               |
|                              |  │  |  [bev_planner_node]                                           |
|  [navsat_transform_node]     |  │  |    - Inferencia de transitabilidad: SAM-TP (Hiera-tiny)        |
|    -> gps/filtered           |  │  |    - Proyección matemática a Bird's-Eye-View (BEV)            |
|    -> /odometry/gps          |  │  |    - Banco de trayectorias polinomiales (GeNIE Path Bank)     |
|    -> Srv: /fromLL           |  │  |    - Clustering adaptativo K-Means orientado a la meta        |
|                              |  │  |    - Publica: earth_rover/planned_path (nav_msgs/Path)        |
|  [ekf_filter_node_odom]      |  │  |    - Publica: earth_rover/planner_valid (std_msgs/Bool)       |
|    -> odometry/local (odom)  |  │  |    - Publica: earth_rover/planner_visualization (Image)       |
|                              |  │  |    - Publica: earth_rover/local_bev_grid (OccupancyGrid)      |
|  [ekf_filter_node_map]       |  │  +---------------------------------------------------------------+
|    -> odometry/global (map)  |  │          │                   │                     │ local_bev_grid
|                              |  │          │ planned_path      │ planner_valid       │ (base_link)
|  [ekf_heading_bridge]        |  │          ▼                   ▼                     ▼
|    -> earth_rover/heading    |  │  +---------------------------------------------------------------+
+------------------------------+  │  | PAQUETE: er_navigation                                        |
       │                          │  |                                                               |
       │ gps/filtered             │  |  [gps_waypoint_controller]                                    |
       │                          │  |    - Guard de frescura GNSS: gps_max_stale_s                  |
       │                          │  |    - Path Follower: lookahead sobre earth_rover/planned_path  |
       │                          │  |    - Fallback: cálculo de rumbo geodésico (Haversine/Bearing) |
       │                          │  |    - Recovery Mode: giro sobre el eje si el planner no halla  |
       │                          │  |      caminos viables libres de obstáculos                     |
       │                          │  |    - Máquina de estados: ALIGN (Burst & Wait) / DRIVE (Curva) |
       │                          │  |    - Publica: cmd_vel (geometry_msgs/Twist)                   |
       │                          │  |    - Publica: earth_rover/waypoint_status ("REACHED")         |
       │                          │  +---------------------------------------------------------------+
       │                          │                ▲                            │ waypoint_status
       │                          │                │ target_waypoint            ▼
       │                          │  +---------------------------------------------------------------+
       │                          │  | PAQUETE: er_mission                                           |
       │                          │  |                                                               |
       │                          │  |  [mission_manager_node]                                       |
       │                          │  |    - Orquestación de checkpoints de misión con el SDK         |
       │                          │  |    - Protocolo asíncrono multihilo (sin bloquear DDS)         |
       │                          │  |    - Publica: earth_rover/target_waypoint (NavSatFix)         |
       │                          │  |    - Publica: earth_rover/navigation_pause (Bool)             |
       │                          │  +---------------------------------------------------------------+
       │                          │
       ▼                          │
[Herramienta Standalone de Debug] │
  PAQUETE: er_perception          │
  [traversability_node] ◄─────────+
    (Percepción reactiva 2D por corredores - fuera del pipeline en vivo)
```

---

## 2. Resumen de Paquetes ROS 2

| Paquete | Descripción | Nodos Principales |
| :--- | :--- | :--- |
| **`earth_rovers_sdk`** | Bridge de comunicación bidireccional entre ROS 2 y la API HTTP/WebSocket/MJPEG del SDK. | `earth_rover_bridge` |
| **`mini_plus_localization`** | Fusión sensorial mediante EKF dual (`robot_localization`), proyección geodésica `/fromLL` y bridge de rumbo REP-105. | `ekf_filter_node_odom`, `ekf_filter_node_map`, `navsat_transform_node`, `ekf_heading_bridge`, `static_transform_publisher` |
| **`er_planning`** | Percepción de transitabilidad (SAM-TP), proyección métrica BEV, planificación local (GeNIE), mapa persistente global Bayesiano y planificación global D* Lite. | `bev_planner_node`, `persistent_map_node`, `global_planner_node` |
| **`er_navigation`** | Control motriz reactivo, seguimiento de trayectorias BEV con lookahead dinámico y fallback a GPS. | `gps_waypoint_controller` |
| **`er_mission`** | Gestor de misiones, ciclo de vida de checkpoints geodésicos y confirmación HTTP con el SDK. | `mission_manager_node` |
| **`er_bringup`** | Orquestación general del sistema y launch files consolidados de producción. | Launch: `mission1.launch.py` |
| **`er_perception`** | Herramientas standalone de análisis visual reactivo 2D (corredores) para depuración manual. | `traversability_node` |
| **`src/sdk_server`** | Servidor FastAPI/Playwright que gestiona la sesión WebRTC/Agora con el rover físico. | Aplicación Python ASGI (Hypercorn) |

---

## 3. Especificación de Nodos, Tópicos y Parámetros

### 3.1. `earth_rover_bridge` (`earth_rovers_sdk`)
Conecta con el servidor SDK local vía HTTP REST, MJPEG y WebSockets para publicar telemetría y enviar comandos de movimiento.
- **Publica:**
  - `earth_rover/front/image_raw` (`sensor_msgs/msg/Image`, BEST_EFFORT): Frame BGR8 de la cámara frontal ($1024 \times 576$).
  - `/gps/fix` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Coordenadas geodésicas crudas con covarianza escalada por HDOP.
  - `/imu/data` (`sensor_msgs/msg/Imu`, RELIABLE): Aceleraciones y velocidades angulares MPU-6050 a 50 Hz interpolados.
  - `/wheel_odom` (`nav_msgs/msg/Odometry`, RELIABLE): Odometría calculada por cinemática de ruedas.
  - `earth_rover/heading` (`std_msgs/msg/Float32`, BEST_EFFORT): Rumbo magnético ($0^\circ=\text{Norte}$, sentido horario).
  - `earth_rover/battery` (`sensor_msgs/msg/BatteryState`, BEST_EFFORT): Estado de carga de batería.
- **Consume:**
  - `cmd_vel` (`geometry_msgs/msg/Twist`): Comandos de velocidad lineal y angular hacia el rover.
- **Parámetros Clave:**
  - `sdk_url` (`str`, default: `"http://localhost:8000"`): URL base del servidor SDK.
  - `feed_fps` (`int`, default: `15`): Tasa de captura de video MJPEG.
  - `gps_position_covariance` (`list[float]`): Matriz base $3 \times 3$ de covarianza de posición GPS.
  - `odom_twist_covariance` (`list[float]`): Matriz de covarianza para velocidades de odometría.

---

### 3.2. Filtros EKF y Localización (`mini_plus_localization`)
- **`ekf_filter_node_odom`**: Estima la odometría local continua en el marco `odom` sin saltos discontinuos.
  - *Consume:* `/wheel_odom` ($v_x, v_y$) y `/imu/data` ($yaw$).
  - *Publica:* `odometry/local` y transform `odom -> base_link`.
- **`navsat_transform_node`**: Proyecta coordenadas GNSS al plano cartesiano local y expone el servicio geodésico `/fromLL`.
  - *Consume:* `/gps/fix`, `/imu/data`, `odometry/global`.
  - *Publica:* `/odometry/gps` y `gps/filtered` (`sensor_msgs/msg/NavSatFix`).
  - *Servicio:* `/fromLL` (`robot_localization/srv/FromLL`): Convierte lat/lon a coordenadas métricas $(X, Y)$ en el marco `map`.
- **`ekf_filter_node_map`**: Fusión global en el marco `map` con anclaje geográfico.
  - *Consume:* `/wheel_odom`, `/imu/data` y `/odometry/gps`.
  - *Publica:* `odometry/global` y transform `map -> odom`.
- **`ekf_heading_bridge`**: Convierte el cuaternión fusionado de `odometry/global` a rumbo de brújula ($0^\circ=\text{Norte}$, horario).
  - *Consume:* `odometry/global` (`nav_msgs/msg/Odometry`).
  - *Publica:* `earth_rover/heading` (`std_msgs/msg/Float32`).

---

### 3.3. `bev_planner_node` (`er_planning`)
Cerebro de percepción y planificación de trayectorias locales libres de obstáculos.
- **Consume:**
  - `earth_rover/front/image_raw` (`sensor_msgs/msg/Image`, BEST_EFFORT): Imagen frontal.
  - `gps/filtered` (`sensor_msgs/msg/NavSatFix`, BEST_EFFORT): Posición GPS filtrada.
  - `earth_rover/heading` (`std_msgs/msg/Float32`, BEST_EFFORT): Rumbo del rover.
  - `earth_rover/target_waypoint` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Meta geodésica actual.
- **Publica:**
  - `earth_rover/planned_path` (`nav_msgs/msg/Path`, RELIABLE): Camino óptimo en marco `base_link` ($+X$ adelante, $+Y$ izquierda).
  - `earth_rover/planner_valid` (`std_msgs/msg/Bool`, BEST_EFFORT): `True` si se encontró un camino válido libre de colisiones.
  - `earth_rover/planner_visualization` (`sensor_msgs/msg/Image`, BEST_EFFORT): Debug visual con overlay de costos y trayectorias.
  - `earth_rover/local_bev_grid` (`nav_msgs/msg/OccupancyGrid`, BEST_EFFORT): Grilla BEV local cruda en marco `base_link`.
- **Parámetros Clave:**
  - `local_bev_grid_topic` (`str`, default: `"earth_rover/local_bev_grid"`): Tópico para publicar la grilla BEV local.
  - `resolution_m_per_px` (`float`, default: `0.03`): Resolución métrica de la grilla BEV ($3\text{ cm/px}$).
  - `forward_range_m` (`float`, default: `4.0`): Horizonte longitudinal local ($4\text{ m}$).
  - `side_range_m` (`float`, default: `2.0`): Semiancho lateral local ($4\text{ m}$ de ancho total).
  - `grid_size` (`int`, default: `240`): Dimensión de la grilla interna del planificador ($240 \times 240$).
  - `threshold_cost` (`float`, default: `0.50`): Umbral de costo a partir del cual se considera obstáculo.
  - `use_clustering` (`bool`, default: `true`): Habilita K-Means adaptativo para seleccionar la rama hacia el waypoint.

---

### 3.4. `gps_waypoint_controller` (`er_navigation`)
Controlador motriz híbrido reactivo con mitigación de jitter 4G (Burst & Wait), seguimiento de trayectorias BEV locales y fallback geodésico.
- **Consume:**
  - `gps/filtered` (`sensor_msgs/msg/NavSatFix`, BEST_EFFORT): Posición actual filtrada.
  - `earth_rover/heading` (`std_msgs/msg/Float32`, BEST_EFFORT): Rumbo actual del rover.
  - `earth_rover/target_waypoint` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Coordenadas del waypoint objetivo.
  - `earth_rover/planned_path` (`nav_msgs/msg/Path`, BEST_EFFORT): Trayectoria generada por `bev_planner_node`.
  - `earth_rover/planner_valid` (`std_msgs/msg/Bool`, BEST_EFFORT): Validez del camino planificado.
  - `earth_rover/navigation_pause` (`std_msgs/msg/Bool`, RELIABLE): Pausa comandada por el gestor de misión.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Estado de misión.
- **Publica:**
  - `cmd_vel` (`geometry_msgs/msg/Twist`, RELIABLE): Velocidad lineal y angular comandada.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Notificación `"REACHED"` al alcanzar la meta.
- **Parámetros Clave:**
  - `gps_max_stale_s` (`float`, default: `2.0`): Tiempo máximo tolerado sin recibir GPS antes de frenar por seguridad.
  - `path_following_enabled` (`bool`, default: `true`): Activa el seguimiento de `earth_rover/planned_path`.
  - `path_max_stale_s` (`float`, default: `1.0`): Tiempo de expiración del path BEV antes de caer en fallback a GPS puro.
  - `lookahead_distance_m` (`float`, default: `1.0`): Distancia de anticipación euclídea sobre el path.
  - `recovery_turn_speed` (`float`, default: `0.3`): Velocidad angular de escaneo cuando el planner reporta bloqueo total.
  - `goal_tolerance_m` (`float`, default: `13.0`): Radio geodésico de llegada al checkpoint.
  - `align_threshold_deg` (`float`, default: `18.0`): Umbral de error angular para pasar de `ALIGN` a `DRIVE`.
  - `turn_burst_s` (`float`, default: `0.25`) y `pause_after_turn_s` (`float`, default: `0.8`): Parámetros Burst & Wait anti-latencia.

---

### 3.5. `mission_manager_node` (`er_mission`)
Orquestador de misión de alto nivel encargado de interactuar con los endpoints de la competencia.
- **Consume:**
  - `earth_rover/gps` (`gps/filtered` vía remapping, `sensor_msgs/msg/NavSatFix`, BEST_EFFORT): Posición GPS.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Señal `"REACHED"` enviada por el controlador.
- **Publica:**
  - `earth_rover/target_waypoint` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Checkpoint activo publicado.
  - `earth_rover/navigation_pause` (`std_msgs/msg/Bool`, RELIABLE): Freno/reanudación de navegación.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Señal `"MISSION_FINISHED"`.
- **Parámetros Clave:**
  - `sdk_url` (`str`, default: `"http://localhost:8000"`): URL del servidor SDK.
  - `checkpoint_max_distance_m` (`float`, default: `14.5`): Distancia máxima para validar el disparo de `POST /checkpoint-reached`.
  - `pre_post_stop_s` (`float`, default: `2.5`): Tiempo de detención total antes de enviar el HTTP POST.

---

### 3.6. `traversability_node` (`er_perception`) *(Standalone / Debug)*
Nodo histórico de percepción reactiva 2D por corredores basada en SAM-TP.
- **Estado:** Mantenido como herramienta standalone para pruebas manuales y diagnósticos; no forma parte del launch automático de producción.
- **Publica:** `earth_rover/traversability_blocked` (`std_msgs/msg/Bool`), `earth_rover/traversability_angular_bias` (`std_msgs/msg/Float32`), `earth_rover/traversability_overlay` (`sensor_msgs/msg/Image`).

---

## 4. Mapa Persistente y Planificador Global (D* Lite)

### 4.1. Flujo de Datos Arquitectónico

```text
+------------------------------------+
|       Cámara Frontal (RGB)         |
+------------------------------------+
                  │ image_raw
                  ▼
+----------------------------------------------------------------------------------------------------+
| bev_planner_node (er_planning)                                                                     |
|   - Percepción de transitabilidad SAM-TP (Hiera-tiny)                                              |
|   - Proyección a Bird's-Eye-View (BEV) local                                                       |
|   - Publica grilla local: earth_rover/local_bev_grid (nav_msgs/OccupancyGrid, frame 'base_link')   |
+----------------------------------------------------------------------------------------------------+
                  │ local_bev_grid (base_link, 0.03 m/px)
                  ▼
+----------------------------------------------------------------------------------------------------+
| persistent_map_node (er_planning)                                                                  |
|   - Acumulación Bayesiana de evidencia en marco global 'map' (TF: map -> base_link)               |
|   - Ganancias de acierto/fallo: hit_gain=+15.0, miss_gain=-10.0, clamp [-100.0, +100.0]            |
|   - Decaimiento exponencial periódico (decay_factor=0.95 @ 1.0s) para olvidar obstáculos dinámicos |
|   - Publica: earth_rover/persistent_map (nav_msgs/OccupancyGrid, frame 'map', 0.20 m/px)           |
+----------------------------------------------------------------------------------------------------+
                  │ persistent_map (map frame)
                  ▼
+----------------------------------------------------------------------------------------------------+
| global_planner_node (er_planning)                                                                  |
|   - Algoritmo D* Lite (Koenig & Likhachev, 2002/2005)                                              |
|   - Proyección de meta geodésica vía servicio /fromLL de robot_localization                        |
|   - Reparación incremental de vértices ante cambios de costo y movimiento del rover (km)           |
|   - Publica: earth_rover/global_path (nav_msgs/Path, frame 'map')                                  |
|   - Publica: earth_rover/global_planner_valid (std_msgs/Bool)                                      |
+----------------------------------------------------------------------------------------------------+
                  │
                  ▼
   [Solo para inspección / depuración en RViz — NO consumido por gps_waypoint_controller todavía]
```

> [!IMPORTANT]
> **Nota de Integración y Alcance:**
> Este mapa global persistente y su planificador D* Lite todavía **NO** están integrados con `gps_waypoint_controller`. El rover sigue navegando en producción exactamente igual que antes de este cambio (utilizando `bev_planner_node` local y el fallback a rumbo geodésico). La integración del camino global como fuente de guiado es un paso futuro separado.

---

### 4.2. Especificación de los Nuevos Nodos

#### A) `persistent_map_node` (`er_planning`)
- **Consume:**
  - `earth_rover/local_bev_grid` (`nav_msgs/msg/OccupancyGrid`, BEST_EFFORT): Grilla local BEV generada por percepción.
  - Transform TF `map -> base_link`: Proporcionada por `mini_plus_localization` (`robot_localization`).
- **Publica:**
  - `earth_rover/persistent_map` (`nav_msgs/msg/OccupancyGrid`, RELIABLE, 1.0 Hz): Mapa de ocupación global persistente en marco `map` ($400\text{ m} \times 400\text{ m}$ @ $0.20\text{ m/px}$).
- **Parámetros Clave (`persistent_map_params.yaml`):**

| Parámetro | Tipo | Default | Descripción |
| :--- | :--- | :--- | :--- |
| `local_grid_topic` | `str` | `"earth_rover/local_bev_grid"` | Tópico de la grilla local de entrada. |
| `map_topic` | `str` | `"earth_rover/persistent_map"` | Tópico de publicación del mapa global. |
| `map_frame` | `str` | `"map"` | Marco de coordenadas global (REP-105). |
| `map_width_m` / `map_height_m` | `float` | `400.0` | Dimensiones métricas del mapa global ($400\text{ m} \times 400\text{ m}$). |
| `map_resolution_m_per_px` | `float` | `0.20` | Resolución de la grilla persistente ($20\text{ cm/px}$). |
| `map_origin_x_m` / `map_origin_y_m` | `float` | `-200.0` | Esquina inferior izquierda del mapa centrada en el origen `map`. |
| `hit_gain` | `float` | `15.0` | Incremento de confianza por celda observada como obstáculo. |
| `miss_gain` | `float` | `10.0` | Decremento de confianza por celda observada como transitable. |
| `confidence_max` | `float` | `100.0` | Saturación máxima y mínima de confianza ($[-100, +100]$). |
| `occupied_threshold` | `float` | `55.0` | Umbral de confianza para marcar celda como ocupada (100). |
| `free_threshold` | `float` | `-20.0` | Umbral de confianza para marcar celda como libre (0). |
| `decay_period_s` | `float` | `1.0` | Período del temporizador de decaimiento temporal. |
| `decay_factor` | `float` | `0.95` | Factor multiplicativo de decaimiento ($\text{conf} \times 0.95$). |
| `map_publish_period_s` | `float` | `1.0` | Período de publicación del mapa global. |
| `tf_lookup_timeout_s` | `float` | `0.2` | Timeout para la resolución de transforms TF. |

---

#### B) `global_planner_node` (`er_planning`)
- **Consume:**
  - `earth_rover/persistent_map` (`nav_msgs/msg/OccupancyGrid`, RELIABLE): Mapa global persistente.
  - `earth_rover/target_waypoint` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Coordenadas geodésicas de la meta.
  - Transform TF `map -> base_link`: Pose actual del rover en el mapa.
  - Servicio `/fromLL` (`robot_localization/srv/FromLL`): Conversión geodésica oficial a $(X, Y)$ en `map`.
- **Publica:**
  - `earth_rover/global_path` (`nav_msgs/msg/Path`, RELIABLE): Trayectoria global óptima planificada en marco `map`.
  - `earth_rover/global_planner_valid` (`std_msgs/msg/Bool`, RELIABLE): `True` si se encontró un camino finito hasta la meta.
- **Parámetros Clave (`global_planner_params.yaml`):**

| Parámetro | Tipo | Default | Descripción |
| :--- | :--- | :--- | :--- |
| `map_topic` | `str` | `"earth_rover/persistent_map"` | Tópico del mapa de entrada. |
| `target_topic` | `str` | `"earth_rover/target_waypoint"` | Tópico de la meta geodésica activa. |
| `global_path_topic` | `str` | `"earth_rover/global_path"` | Tópico de salida del camino global (frame `map`). |
| `global_planner_valid_topic` | `str` | `"earth_rover/global_planner_valid"` | Indicador de validez de camino global. |
| `from_ll_service` | `str` | `"/fromLL"` | Nombre del servicio geodésico de `robot_localization`. |
| `replan_min_period_s` | `float` | `2.0` | Período mínimo entre re-planificaciones de D* Lite. |
| `occupied_cost_threshold` | `float` | `55.0` | Costo a partir del cual una celda es intransitable ($\infty$). |
| `unknown_cell_cost` | `float` | `20.0` | Costo asignado a celdas desconocidas (favorece terreno visto sin bloquear exploración). |
| `connectivity` | `int` | `8` | Conectividad de la grilla ($8$ vecinos con costo diagonal $\sqrt{2} \cdot \text{res}$). |
| `tf_lookup_timeout_s` | `float` | `0.2` | Timeout para lookup TF `map -> base_link`. |

---

### 4.3. Ejecución Aislada de Prueba (`global_map_test.launch.py`)

Para probar el pipeline completo de percepción BEV, mapa persistente y D* Lite de forma aislada sin activar el control motriz:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

# Lanzar los tres nodos de planificación y mapeo juntos
ros2 launch er_planning global_map_test.launch.py
```

Para inspeccionar las salidas en tiempo real:
```bash
# Inspeccionar el camino global en RViz o por terminal
ros2 topic echo /earth_rover/global_path

# Inspeccionar el estado de validez del planificador global
ros2 topic echo /earth_rover/global_planner_valid

# Visualizar en RViz2 los tópicos:
# - /earth_rover/local_bev_grid (OccupancyGrid, frame: base_link)
# - /earth_rover/persistent_map (OccupancyGrid, frame: map)
# - /earth_rover/global_path (Path, frame: map)
```

---

## 5. Ejecución del Stack Completo

### 5.1. Variables de Entorno y Credenciales
Configurar las credenciales en `src/sdk_server/.env` (tomando como base `.env.sample`):
```bash
SDK_API_TOKEN="tu_token_frodobots"
BOT_SLUG="slug_del_rover"
MISSION_SLUG="mission-1"
```

### 5.2. Lanzamiento del Stack Integrado (Producción)
```bash
# 1. En una terminal: Iniciar el servidor SDK
cd /root/ros2_ws/src/sdk_server
hypercorn main:app --bind 0.0.0.0:8000

# 2. En otra terminal: Lanzar la misión completa en ROS 2
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch er_bringup mission1.launch.py
```

### 5.3. Ejecución con Docker / Docker Compose
El workspace incluye soporte para despliegues contenerizados:
- **`Dockerfile`**: Imagen base sobre `osrf/ros:jazzy-ros-base` con dependencias compiladas.
- **`docker-compose.yml`**: Orquesta el contenedor montando el `.env` local (`ROVER_MODE: "full"`).
```bash
docker compose up --build
```

---

## 6. Guía de Depuración y Ejecución Modular

### 6.1. Ejecutar sólo `bev_planner_node` (con cámara real o simulada)
```bash
source install/setup.bash
ros2 run er_planning bev_planner_node --ros-args --params-file src/er_planning/config/planner_params.yaml
```
Para visualizar el mapa BEV y las trayectorias evaluadas en tiempo real:
```bash
ros2 run rqt_image_view rqt_image_view /earth_rover/planner_visualization
```

### 6.2. Ejecutar `traversability_node` (Herramienta Standalone)
```bash
source install/setup.bash
ros2 run er_perception traversability_node --ros-args --params-file src/er_perception/config/perception_params.yaml
```

### 6.3. Inspección de Telemetría y Diagnóstico de Rumbo
```bash
# Monitoreo del estado del controlador híbrido
ros2 topic echo /earth_rover/waypoint_status

# Verificación de trayectorias locales planificadas
ros2 topic echo /earth_rover/planned_path

# Verificación de validez del planner local
ros2 topic echo /earth_rover/planner_valid
```

---

## 7. Setup del Modelo de Inteligencia Artificial (SAM-TP & GeNIE)

El pipeline de percepción y planificación utiliza el modelo **SAM-TP** (Segment Anything Model 2.1 Hiera-tiny con prompt de transitabilidad aprendido) y la librería **GeNIE Path Planner**.

### 7.1. Instalación Canónica de Dependencias
En el entorno Python donde se ejecutan los nodos:
```bash
# 1. PyTorch y TorchVision (CPU o CUDA según hardware)
pip install torch torchvision

# 2. Paquete vendoreado GeNIE (sam2 + genie_path_planner)
pip install --no-build-isolation -e src/third_party/sana-earth-rover-policy/genie

# 3. Paquete de transitabilidad y soporte Hugging Face
pip install -e 'src/third_party/sana-earth-rover-policy/traversability[hf]'

# 4. Dependencias complementarias
pip install hydra-core "scikit-learn<1.5" "scipy<1.15" huggingface-hub
```

### 7.2. Modos de Ejecución: Local vs. Servidor GPU
1. **Local (CPU / GPU integrada):** El modelo descarga automáticamente los pesos `checkpoint_finetuned_v2.pt` desde Hugging Face (`sanatem/samtp-mini-traversability`) y los almacena en `~/.cache/rover_traversability/`.
2. **Servidor Acelerado (GPU Dedicada):** Para procesamiento en tiempo real con latencias de inferencia $<50\text{ ms}$, se recomienda ejecutar el contenedor sobre hardware con NVIDIA Container Toolkit (`--gpus all` o `device: cuda`).

---

## 8. Estado Actual y Limitaciones Conocidas

1. **Rendimiento de Inferencia en CPU:** En procesadores x86 estándar sin GPU dedicada, el ciclo completo de inferencia SAM-TP toma entre $4.0\text{ s}$ y $5.5\text{ s}$ por frame (medido en tests reales). Gracias a la arquitectura desacoplada en hilos independientes y al fallback de frescura (`path_max_stale_s`), el bucle de control no se congela, pero la navegación en tiempo real a alta velocidad requiere aceleración GPU.
2. **Dimensiones de la Huella (`footprint_px`):** El parámetro `footprint_px` en `planner_params.yaml` está configurado por defecto en 10 píxeles ($\approx 30\text{ cm}$). Se encuentra pendiente la confirmación milimétrica en banco de pruebas del chasis real del Earth Rover Mini+.
3. **Mapeo Persistente y Planificador Global D\* Lite:** Implementados en `er_planning` (`persistent_map_node` y `global_planner_node`) con launch standalone `global_map_test.launch.py`. Se encuentran activos para inspección y validación visual; su integración con `gps_waypoint_controller` y la fusión con el planificador local BEV está planificada para la siguiente etapa de desarrollo.
4. **Reemplazo en Vivo de Traversability:** `traversability_node` (percepción 2D basada en franjas de imagen) ha sido desacoplado del pipeline en vivo a favor de `bev_planner_node` (que provee proyección métrica BEV y trayectorias continuas).
5. **Jitter de Red 4G/LTE:** La latencia variable en la transmisión de comandos y telemetría del rover es mitigada mediante la máquina de estados *Burst & Wait* y los filtros EKF duales.
6. **Límite de Escala del Mapa Persistente (grilla densa):** `persistent_map_node` usa hoy una grilla densa de tamaño fijo (400m x 400m @ 0.20m/px = 4 millones de celdas, ~1.6GB en RAM como float32). El decaimiento (`_decay_timer_cb`) y el diff de costos (`_on_map`) procesan la grilla COMPLETA en cada ciclo, sin importar cuánto del mapa esté realmente cerca del rover en ese momento. Esto es adecuado para el área de una competencia (cientos de metros), pero NO escala a trayectos largos (por ejemplo, 80km entre dos puntos): a esa distancia, incluso un corredor angosto de 200m de ancho ya requiere del orden de 400 millones de celdas (~1.6GB adicionales), y el costo de procesamiento por ciclo crece con el tamaño TOTAL del mapa acumulado durante todo el viaje, no con la distancia restante al checkpoint. D* Lite en sí mismo no es el cuello de botella (sus estructuras `g`/`rhs` son diccionarios dispersos que escalan con el camino buscado, no con el mapa completo) — el límite está en la infraestructura de `persistent_map_node` alrededor de él. Ver sección 9 (Roadmap) para el diseño propuesto que resuelve esto.

---

## 9. Próximos Módulos y Roadmap

- [ ] **Integración de `earth_rover/global_path` en el Controlador de Navegación:** Conectar la trayectoria global generada por D* Lite con el bucle de control de `gps_waypoint_controller` para guiar la búsqueda de trayectorias locales hacia sub-metas intermedias en lugar de únicamente la meta geodésica directa.
- [ ] **Arquitectura de Tres Niveles Jerárquicos para Trayectos Largos (ej. 80km entre dos ciudades/universidades):** reemplazar la grilla densa de tamaño fijo de `persistent_map_node` por una ventana local rodante ("rolling window") de tamaño constante (cientos de metros) que se re-centra alrededor de la posición actual del rover, descartando lo que queda muy atrás — acota memoria y cómputo a un tamaño constante sin importar la distancia total del viaje (mismo patrón que el "rolling_window" del costmap global de Nav2). Para la decisión de "por qué calles ir" a lo largo de todo el trayecto, se necesita además un nivel superior de ruteo vial basado en un grafo de calles (tipo OSRM/OpenStreetMap) que entregue una secuencia de waypoints/tramos intermedios — una grilla de celdas no es la representación adecuada para decisiones a escala de kilómetros. Con esto, el sistema queda en tres niveles: (1) ruteo vial por grafo (kilómetros), (2) mapa persistente + D* Lite en ventana rodante (cientos de metros, entre waypoints intermedios) — lo que existe hoy —, y (3) planificador local reactivo BEV (4m, ya implementado en `bev_planner_node`). Fuera de alcance de la competencia actual; queda documentado para una fase posterior.
- [ ] **Validación End-to-End con Servidor GPU Remoto:** Despliegue del nodo de inferencia en estación base remota y transmisión de trayectorias planificadas comprimidas vía DDS/ZeroMQ.
- [ ] **Evasión Reactiva Lateral Fina (Wall Following):** Integración de control de contorno lateral en pasillos estrechos cuando ambos lados presentan obstáculos cercanos.
- [ ] **Calibración Dinámica de Tracción y Deslizamiento:** Ajuste empírico de las matrices de covarianza de odometría según el tipo de terreno (arena, asfalto, césped).
