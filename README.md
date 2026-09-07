# Earth Rover Mini+ — Stack Autónomo ROS 2 Jazzy

Arquitectura de navegación autónoma, percepción visual profunda (SAM-TP), planificación en vista de pájaro (BEV GeNIE), mapeo global persistente Bayesiano, planificación incremental (D* Lite) y control reactivo para el rover **FrodoBots Earth Rover Mini+** sobre **ROS 2 Jazzy**.

---

## 1. Diagrama de Arquitectura del Sistema

> Para una explicación detallada del funcionamiento interno de cada nodo, ver [ARCHITECTURE.md](ARCHITECTURE.md).

> [!WARNING]
> **ESTADO DE LA CALIBRACIÓN DE CÁMARA (LIMITACIÓN PRINCIPAL DEL SISTEMA):**  
> Los parámetros intrínsecos de la cámara ($f_x, f_y, c_x, c_y$) y extrínsecos ($X = +0.110\text{ m}, h = 0.140\text{ m}, \text{pitch} = -8.0^\circ$, procedentes de las estimaciones a ojo del repositorio base: *"~14 cm above ground, ~11 cm forward, pitched ~8 degrees down"*) son actualmente **nominales y aproximados**, no calibrados en banco óptico sobre el chasis físico. La proyección métrica BEV y la cobertura lateral tienen un error no cuantificado hasta que se ejecute la calibración física con tablero ChArUco en el rover real.

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
|    - Publica rumbo magnético: earth_rover/heading_raw (std_msgs/Float32)                           |
|    - Publica diagnóstico de compuerta de inclinación: earth_rover/tilt_gate_diag (std_msgs/String) |
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
|                              |  │  |    - Gobernador dinámico de velocidad P95                     |
|  [ekf_filter_node_odom]      |  │  |    - Publica: earth_rover/planned_path (nav_msgs/Path)        |
|    -> odometry/local (odom)  |  │  |    - Publica: earth_rover/planner_valid (std_msgs/Bool)       |
|                              |  │  |    - Publica: earth_rover/safe_velocity_limit (Float32)      |
|  [ekf_filter_node_map]       |  │  |    - Publica: earth_rover/planner_visualization (Image)       |
|    -> odometry/global (map)  |  │  |    - Publica: earth_rover/local_bev_grid (OccupancyGrid)      |
|                              |  │  +---------------------------------------------------------------+
|  [ekf_heading_bridge]        |  │          │             │                   │ safe_velocity_limit
|    -> earth_rover/heading    |  │          │ planned_path│ planner_valid     ▼
+------------------------------+  │          ▼             ▼     +-----------------------------------+
       │                          │  +---------------------------| PAQUETE: er_navigation            |
       │ gps/filtered             │  |                           |                                   |
       │                          │  |  [gps_waypoint_controller]|                                   |
       │                          │  |    - Gobernador Fail-Safe: require_velocity_governor          |
       │                          │  |    - Guard de frescura GNSS & Heading (heading_max_stale_s)   |
       │                          │  |    - Path Follower: lookahead sobre earth_rover/planned_path  |
       │                          │  |    - Fallback: rumbo geodésico (geodesic_fallback_speed)      |
       │                          │  |    - Recovery Mode: giro si el planner no halla caminos       |
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
  - `/imu/data` (`sensor_msgs/msg/Imu`, RELIABLE): Aceleraciones y velocidades angulares MPU-6050 (1 muestra promediada por paquete de telemetría a ~0.5 Hz con timestamp actual de ROS).
  - `/wheel_odom` (`nav_msgs/msg/Odometry`, RELIABLE): Odometría calculada por cinemática de ruedas.
  - `earth_rover/heading` (`std_msgs/msg/Float32`, BEST_EFFORT): Rumbo de brújula ($0^\circ=\text{Norte}$, sentido horario).
  - `earth_rover/heading_uncertainty` (`std_msgs/msg/Float32`, BEST_EFFORT): Incertidumbre de orientación estimada por el bridge inercial ($^\circ$).
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
- **`ekf_heading_bridge`**: Convierte el cuaternión fusionado de `odometry/global` a rumbo de brújula ($0^\circ=\text{Norte}$, horario) y propaga continuamente el rumbo mediante integración giroscópica de `/imu/data` (`gyro_z`) entre actualizaciones discretas del compás (~0.5 Hz), monitoreando la incertidumbre analítica.
  - *Consume:* `odometry/global` (`nav_msgs/msg/Odometry`), `/imu/data` (`sensor_msgs/msg/Imu`).
  - *Publica:* `earth_rover/heading` (`std_msgs/msg/Float32`, rumbo continuo), `earth_rover/heading_compass` (`std_msgs/msg/Float32`, brújula absoluta), `earth_rover/heading_uncertainty` (`std_msgs/msg/Float32`, incertidumbre en grados).

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
  - `local_bev_grid_topic` (`str`, default: `"earth_rover/local_bev_grid"`): Tópico de la grilla BEV local métrica ($134 \times 134$ celdas a $0.03\text{ m/px}$, $4.00\text{ m} \times 4.00\text{ m}$).
  - `resolution_m_per_px` (`float`, default: `0.03`): Resolución métrica de la grilla BEV ($3\text{ cm/px}$).
  - `forward_range_m` (`float`, default: `4.0`): Horizonte longitudinal local ($4\text{ m}$).
  - `side_range_m` (`float`, default: `2.0`): Semiancho lateral local ($2.0\text{ m}$ hacia cada lado, $4\text{ m}$ ancho total).
  - `grid_size` (`int`, default: `240`): Dimensión discreta de la grilla interna del muestreador GeNIE ($240 \times 240$).
  - `footprint_px` (`int`, default: `0`): Radio de huella en píxeles (si es `0`, `compute_footprint_px()` deriva dinámicamente $20\text{ px}$ para la grilla del planner $240 \times 240$ desde las dimensiones físicas del chasis: $D_{\text{circ}} = 0.314\text{ m}$, reescalado $240/134$ y margen $1.05$).
  - `threshold_cost` (`float`, default: `0.50`): Umbral de costo a partir del cual se considera obstáculo.
  - `use_clustering` (`bool`, default: `true`): Habilita K-Means adaptativo para seleccionar la rama hacia el waypoint.

---

### 3.4. `gps_waypoint_controller` (`er_navigation`)
Controlador motriz híbrido reactivo con mitigación de jitter 4G (Burst & Wait proporcional), seguimiento de trayectorias BEV locales acotadas y fallback geodésico.
- **Consume:**
  - `gps/filtered` (`sensor_msgs/msg/NavSatFix`, BEST_EFFORT): Posición actual filtrada.
  - `earth_rover/heading` (`std_msgs/msg/Float32`, BEST_EFFORT): Rumbo actual del rover (inercial propagado).
  - `earth_rover/heading_uncertainty` (`std_msgs/msg/Float32`, BEST_EFFORT): Incertidumbre analítica de rumbo en grados.
  - `earth_rover/target_waypoint` (`sensor_msgs/msg/NavSatFix`, RELIABLE): Coordenadas del waypoint objetivo.
  - `earth_rover/planned_path` (`nav_msgs/msg/Path`, BEST_EFFORT): Trayectoria generada por `bev_planner_node`.
  - `earth_rover/planner_valid` (`std_msgs/msg/Bool`, BEST_EFFORT): Validez del camino planificado.
  - `earth_rover/navigation_pause` (`std_msgs/msg/Bool`, RELIABLE): Pausa comandada por el gestor de misión.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Estado de misión.
- **Publica:**
  - `cmd_vel` (`geometry_msgs/msg/Twist`, RELIABLE): Fracciones normalizadas de acelerador lineal (`linear.x` $\in [-1.0, 1.0]$) y angular (`angular.z` $\in [-1.0, 1.0]$) para el bridge del SDK.
  - `earth_rover/waypoint_status` (`std_msgs/msg/String`, RELIABLE): Notificación `"REACHED"` al alcanzar la meta.
  - `earth_rover/control_debug` (`std_msgs/msg/String`, BEST_EFFORT): Telemetría en tiempo real y ciclo de trabajo (`drive_pct`, `turn_pct`, `pause_pct`, `recovery_pct`).
- **Parámetros Clave:**
  - `forward_throttle` (`float`, default: `0.40`): Fracción normalizada de acelerador lineal en avance (`DRIVE`).
  - `turn_throttle` (`float`, default: `0.70`): Fracción normalizada de acelerador angular en alineación (`ALIGN`).
  - `drive_correction_gain` (`float`, default: `0.01`): Ganancia proporcional para corrección angular en `DRIVE`.
  - `max_drive_angular` (`float`, default: `0.45`): Límite de acelerador angular en `DRIVE` ($0.01 \times 45.0^\circ$, sin saturación prematura).
  - `recovery_turn_throttle` (`float`, default: `0.30`): Fracción de acelerador angular en `RECOVERY` (bidireccional, gira hacia el rumbo geodésico).
  - `recovery_max_duration_s` (`float`, default: `20.0`): Duración máxima del giro en `RECOVERY` antes de forzar reintento a `ALIGN`.
  - `geodesic_fallback_throttle` (`float`, default: `0.20`): Acelerador conservador en navegación geodésica pura.
  - `max_linear_speed_mps` (`float`, default: `1.111`): Velocidad física máxima de referencia (m/s) para conversión de $v_{\text{safe}}$.
  - `gps_max_stale_s` (`float`, default: `2.0`): Tiempo máximo tolerado sin recibir GPS antes de frenar por seguridad.
  - `heading_max_stale_s` (`float`, default: `3.5`): Tiempo máximo tolerado sin recibir rumbo antes de frenar por seguridad.
  - `path_following_enabled` (`bool`, default: `true`): Activa el seguimiento de `earth_rover/planned_path`.
  - `path_max_stale_s` (`float`, default: `8.0`): Tiempo de expiración del path BEV antes de caer en fallback a GPS puro.
  - `lookahead_distance_m` (`float`, default: `1.0`): Distancia de anticipación euclídea sobre el path.
  - `goal_tolerance_m` (`float`, default: `13.0`): Radio geodésico de llegada al checkpoint.
  - `align_threshold_deg` (`float`, default: `18.0`) y `coarse_align_threshold_deg` (`float`, default: `25.0`): Umbrales de alineación angular fino y grueso para conmutar a `DRIVE`.
  - `drive_abort_threshold_deg` (`float`, default: `65.0`) y `drive_abort_dwell_s` (`float`, default: `1.5`): Histéresis asimétrica para abortar `DRIVE` hacia `ALIGN` ante desvío geodésico persistente.
  - `max_bev_deviation_deg` (`float`, default: `45.0`): Desviación angular máxima que BEV puede comandar respecto al rumbo geodésico en `DRIVE`.
  - `turn_burst_min_s` (`float`, default: `0.15`), `turn_burst_max_s` (`float`, default: `1.20`), `yaw_rate_deg_s` (`float`, default: `17.0`), `turn_burst_damping` (`float`, default: `0.6`): Ráfaga de giro proporcional al error angular ($t_{\text{burst}} = (|e| / 17.0) \cdot 0.6$).
  - `pause_after_turn_s` (`float`, default: `0.5`): Pausa mínima tras pulso de giro para estabilización física.
  - `heading_trust_threshold_deg` (`float`, default: `10.0`): Umbral de incertidumbre propagada que autoriza nueva ráfaga sin esperar al compás.

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
+----------------------------------------------------------------------------------------------------+
|                                       Cámara Frontal (RGB)                                         |
+----------------------------------------------------------------------------------------------------+
                                                   │ image_raw
                                                   ▼
+----------------------------------------------------------------------------------------------------+
| bev_planner_node (er_planning)                                                                     |
|   - Inferencia de transitabilidad SAM-TP y proyección a Bird's-Eye-View (BEV) local                |
|   - Planificador reactivo de trayectorias (GeNIE) guiado por sub-meta global o rumbo geodésico     |
|   - Publica grilla local: earth_rover/local_bev_grid (nav_msgs/OccupancyGrid, frame 'base_link')   |
|   - Publica camino local: earth_rover/planned_path (nav_msgs/Path, frame 'base_link')              |
+----------------------------------------------------------------------------------------------------+
         │                                                            │
         │ local_bev_grid (base_link, 0.03 m/px)                      │ planned_path (base_link)
         ▼                                                            ▼
+-----------------------------------------------------------------+  +-------------------------------+
| persistent_map_node (er_planning)                               |  | gps_waypoint_controller       |
|   - Acumulación Bayesiana de evidencia en marco global 'map'    |  | (er_navigation)               |
|   - Ganancias: hit_gain=+15.0, miss_gain=-10.0, [-100, +100]    |  |   - Seguimiento reactivo con  |
|   - Decaimiento exponencial: decay_factor=0.7738 @ 5.0s         |  |     lookahead dinámico (1.0m) |
|   - Publica: earth_rover/persistent_map (OccupancyGrid, frame   |  |   - Fallback a rumbo geodésico|
|     'map', 0.20 m/px)                                           |  |     si planned_path expira    |
+-----------------------------------------------------------------+  |   - Publica: cmd_vel          |
         │                                                            +-------------------------------+
         │ persistent_map (map frame)
         ▼
+-----------------------------------------------------------------+
| global_planner_node (er_planning)                               |
|   - Algoritmo D* Lite incremental (Koenig & Likhachev)          |
|   - Proyección de meta geodésica vía servicio /fromLL           |
|   - Reparación incremental de vértices ante cambios en el mapa  |
|   - Publica: earth_rover/global_path (nav_msgs/Path, 'map')     |
|   - Publica: earth_rover/global_planner_valid (std_msgs/Bool)   |
+-----------------------------------------------------------------+
         │
         │ earth_rover/global_path + global_planner_valid
         └────────────────────────────────────────────────────────────► (retroalimenta como sub-meta
                                                                        al bev_planner_node de arriba —
                                                                        mismo nodo, no uno nuevo)
```

> [!IMPORTANT]
> **Integración en Producción y Flujo de Guiado Asesor:**
> `persistent_map_node` y `global_planner_node` están integrados en el launch de producción [`mission1.launch.py`](src/er_bringup/launch/mission1.launch.py) detrás del flag de lanzamiento `enable_global_planning` (habilitado por default: `true`).
> El camino global D* Lite no comanda directamente el chasis: alimenta una **sub-meta asesora** dentro de `bev_planner_node` (`_compute_global_subgoal_base_link()`), proyectada al marco `base_link` a una distancia métrica de anticipación (`global_lookahead_distance_m: 3.5m`). `bev_planner_node` sigue siendo el único nodo que evalúa colisiones visuales inmediatas y decide el movimiento local (`planned_path`), el cual es ejecutado por `gps_waypoint_controller` (el controlador no sufrió modificaciones). Si el planificador global se desactiva (`enable_global_planning:=false`) o el camino global pierde frescura, `bev_planner_node` realiza un fallback transparente e inmediato al rumbo geodésico directo (*Fail-Open*).

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

# Lanzamiento estándar con planificador global activado (default):
ros2 launch er_bringup mission1.launch.py

# Lanzamiento con planificador global desactivado (modo fallback reactivo):
ros2 launch er_bringup mission1.launch.py enable_global_planning:=false
```

### 5.3. Ejecución con Docker / Docker Compose
El workspace incluye soporte para despliegues contenerizados:
- **`Dockerfile`**: Imagen base sobre `osrf/ros:jazzy-ros-base` con dependencias compiladas.
- **`docker-compose.yml`**: Orquesta el contenedor montando el `.env` local (`ROVER_MODE: "full"`).
```bash
docker compose up --build
# O con soporte GPU NVIDIA:
docker compose -f docker-compose.gpu.yml up --build -d
```

### 5.4. Flujo de Desarrollo y Pruebas Rápidas

Para iterar ágilmente sobre el código sin rebuilds innecesarios de Docker:

1. **Levantar el contenedor en segundo plano (una sola vez):**
   ```bash
   docker compose -f docker-compose.gpu.yml up -d
   ```
   El contenedor `mini_plus_rover` quedará corriendo indefinidamente con el código fuente del host montado en tiempo real (`./src:/root/ros2_ws/src`) y las credenciales `.env` enlazadas.

2. **Entrar al contenedor para correr o probar nodos:**
   ```bash
   docker exec -it mini_plus_rover bash
   ```
   Desde esta terminal interactiva, ejecutar directamente los comandos de ROS 2:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /root/ros2_ws/install/setup.bash
   ros2 run er_planning bev_planner_node --ros-args --params-file src/er_planning/config/planner_params.yaml
   # O lanzar launch files directamente:
   ros2 launch er_planning bev_planner_only.launch.py
   ```
   > [!IMPORTANT]
   > **Nunca ejecutar `docker run` nuevo por cada prueba:** `docker run` crea contenedores anónimos adicionales que pueden quedar colgados en segundo plano reteniendo memoria VRAM de la GPU. Usar siempre `docker exec -it mini_plus_rover bash`.

3. **Cuándo Rebuildear Docker vs. Cuándo NO:**
   * **NO hace falta `docker compose build`:** Cualquier cambio o edición de código Python en `src/` se refleja inmediatamente en el contenedor gracias al bind mount de `./src`.
   * **Compilación liviana interna (`colcon build` < 1s):** Si agregás un nuevo `entry_point` en `setup.py`, un paquete nuevo, o para sincronizar scripts de consola, ejecutá dentro del contenedor (`docker exec`):
     ```bash
     colcon build --symlink-install --packages-select <nombre_paquete>
     source install/setup.bash
     ```
   * **SÍ hace falta `docker compose build`:** Únicamente si se modifican `requirements-ai.txt`, `src/sdk_server/requirements.txt`, paquetes de sistema `apt` o el propio `Dockerfile`.

4. **Memoria Operativa para Agentes de IA:**
   * Las directrices de entorno, decisiones de arquitectura y métricas de referencia están consolidadas en `MEMORY.md` y `GEMINI.md` / `AGENTS.md` en la raíz del repo (ignoradas por Git).
   * Antigravity CLI / Gemini cargan automáticamente estas reglas al iniciar cualquier sesión de trabajo.

### 5.5. Cómo Correr una Misión Completa (Punto de Entrada Único)

Una vez levantado el contenedor (`docker compose -f docker-compose.gpu.yml up -d`), se ejecuta la misión completa con una única instrucción de launch consolidada:

```bash
docker exec -it mini_plus_rover bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch er_bringup mission_manager.launch.py mission_slug:=mission-1 bot_slug:=luke-notch-quirk enable_global_planning:=false
```

Este comando levanta de forma coordinada:
1. **Bridge SDK (`earth_rover_bridge`):** Enlace bidireccional WebSocket y HTTP con el servidor SDK del rover.
2. **Fusión Sensorial EKF (`ekf.launch.py`):** Filtros odometría local, global y transformación geodésica `/fromLL`.
3. **Percepción y Planificación Local BEV (`bev_planner_node`):** Inferencia SAM-TP en GPU y trayectorias GeNIE optimizadas.
4. **Controlador Motriz (`gps_waypoint_controller`):** Seguimiento de trayectorias locales y máquina de estados anti-latencia *Burst & Wait*.
5. **Gestor de Misión con Estado (`mission_manager_node`):** Disparo automático de `POST /start-mission`, descarga de checkpoints, orquestación de paradas de seguridad y envío de `POST /checkpoint-reached`.
6. *(Opcional)* **Mapa Persistente y D\* Lite (`enable_global_planning:=true`):** Habilita acumulación Bayesiana y planificación global de respaldo.

### 5.6. Precarga de Mapa Semilla (OpenStreetMap Prior) y Orquestador de Misión

Para evitar que `persistent_map_node` y D* Lite arranquen a ciegas en zonas inexploradas, el sistema permite precargar un prior geográfico de baja/moderada confianza a partir de datos reales de OpenStreetMap (veredas y calles).

#### 1. Verificación previa de cobertura OSM
Antes de una competencia, verifique en [OpenStreetMap](https://www.openstreetmap.org) la zona geográfica de la misión:
- **Veredas y sendas peatonales** (tags `highway=footway`, `path`, `pedestrian`, `sidewalk`, `steps`): se clasificarán como prioritarias/transitables (`-80.0`).
- **Calles vehiculares** (tags `highway=residential`, `service`, `tertiary`, `secondary`, `primary`, `unclassified`): se clasificarán como vías a evitar pero transitables en caso necesario (`+60.0`), sin penalizar veredas paralelas.
- Si una zona no cuenta con veredas mapeadas en OSM, el generador asignará confianza neutra (`0.0`) y el rover navegará descubriendo el terreno con la cámara frontal y SAM-TP.

#### 2. Requisito de Datum Dinámico
La conversión de coordenadas requiere que el frame `map` esté anclado a un datum determinista resuelto desde el Checkpoint #1 de la misión SDK (mediante `tools/mission_prep/resolve_datum.py`), con rumbo fijo `yaw = 0.0` (alineado al Norte verdadero según REP-105 ENU).

#### 3. Flujo Integrado con el Wrapper Único (`run_mission.sh`)
El script orquestador realiza todos los pasos preparatorios de forma automatizada:
```bash
# Ejecución completa (resolución de datum + generación OSM con caché + launch de misión):
./tools/mission_prep/run_mission.sh --mission-slug <slug_de_la_mision>

# Ejecución salteando la generación del mapa semilla OSM:
./tools/mission_prep/run_mission.sh --mission-slug <slug_de_la_mision> --skip-seed-map

# Ejecución forzando la re-descarga de datos OSM (ignorando caché local):
./tools/mission_prep/run_mission.sh --mission-slug <slug_de_la_mision> --force-download
```

#### 4. Ejecución Modular de Pasos (Debug / Desarrollo)
```bash
# Paso 1: Resolver datum dinámico desde el servidor SDK
python3 tools/mission_prep/resolve_datum.py --sdk-url http://localhost:8000 --output src/mini_plus_localization/config/datum_resolved.yaml

# Paso 2: Levantar navsat_transform_node temporalmente
ros2 run robot_localization navsat_transform_node \
  --ros-args \
  --params-file src/mini_plus_localization/config/ekf.yaml \
  --params-file src/mini_plus_localization/config/datum_resolved.yaml \
  -r imu:=/imu/data -r gps/fix:=/gps/fix -r odometry/filtered:=/odometry/global &

# Paso 3: Generar mapa semilla .npy (usa servicio /fromLL y guarda en tools/osm_seed/cache/)
python3 tools/osm_seed/generate_seed_map.py \
  --datum-file src/mini_plus_localization/config/datum_resolved.yaml \
  --config src/er_planning/config/persistent_map_params.yaml \
  --output tools/osm_seed/seed_map.npy

# Paso 4: Detener navsat_transform_node temporal (pkill o kill <PID>)

# Paso 5: Lanzar la misión cargando el mapa semilla generado
ros2 launch er_bringup mission1.launch.py seed_map_path:=$(pwd)/tools/osm_seed/seed_map.npy
```

#### 5. Confianza Parcial / Moderada y Fail-Open
El mapa semilla se escala intencionalmente con `seed_confidence_scale: 0.3`. Una celda de vereda (`-80.0`) arranca en `-24.0` (por debajo de `free_threshold = -20.0` para iniciar con costo óptimo 1.0 en D* Lite), pero tan sólo 2 observaciones reales de obstáculo desde la cámara (`+15.0` cada una: `-24 + 15 + 15 = +6 > 0`) revierten la evidencia si la vereda está bloqueada por obras o vallas. Si el servidor OSM no está disponible y no hay caché, el wrapper continúa sin mapa semilla (`seed_map_path=""`, grilla en cero) sin abortar la misión.

#### 6. Atribución de Datos de OpenStreetMap
Los datos geográficos utilizados para el mapa semilla provienen de OpenStreetMap y están licenciados bajo la [Open Database License (ODbL)](https://opendatacommons.org/licenses/odbl/). Si la visualización del mapa o sus derivados se presentan públicamente, debe incluirse la atribución: *"© OpenStreetMap contributors"*.

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

### 6.4. Grabación de Datos para Identificación de Sistema (System ID)
```bash
ros2 bag record \
  earth_rover/control_debug earth_rover/bridge_debug \
  earth_rover/heading gps/filtered /imu/data cmd_vel \
  earth_rover/waypoint_status \
  -o system_id_run_$(date +%Y%m%d_%H%M%S)
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

---

## 8. Estado Actual y Limitaciones Conocidas

1. **Calibración Óptica Pendiente:** Los parámetros intrínsecos ($f_x, f_y, c_x, c_y$) y extrínsecos ($X=+0.110\text{ m}, h=0.140\text{ m}, \text{pitch}=-8.0^\circ$) son estimaciones nominales a ojo procedentes del docstring del repositorio base (*"~14 cm above ground, ~11 cm forward of base origin, pitched ~8 degrees down"*). La proyección BEV tiene error no cuantificado hasta que se ejecute la calibración física con tablero ChArUco en el rover real.
2. **Dependencia de Sensor de Rumbo Único:** No existe una segunda fuente de orientación absoluta independiente del compás magnético. La odometría cinemática `/wheel_odom` proyecta velocidades integrando el mismo rumbo de brújula que alimenta `/imu/data` ($V_{\text{yaw}}$ en `odom0` está deshabilitado); ante perturbaciones magnéticas, ambas fuentes se desvían de forma correlacionada y el EKF no puede detectar la inconsistencia.
3. **Supuesto de Desaceleración $a_{\text{brake}} = 1.5\text{ m/s}^2$ No Verificado:** El valor $1.5\text{ m/s}^2$ es un supuesto teórico del que dependen la ecuación de frenado del gobernador y el horizonte de seguridad. Requiere protocolo de medición empírico en hormigón seco y baja adherencia (mojado/gravilla).
4. **Mapeo Acelerador $\leftrightarrow$ Velocidad Lineal Sin Medir:** No se ha caracterizado empíricamente la curva real de velocidad física (m/s) en función de la fracción de acelerador comandada (`forward_throttle`), el estado de carga de la batería y la fricción del suelo.
5. **Sesgos de IMU Sin Calibrar en Disco:** Los archivos `gyro_bias.json` y `accel_bias.json` no existen en producción. El nodo `earth_rover_bridge` opera con un fallback limpio a $0.0$, requiriendo captura estática de sesgos inerciales en reposo.
6. **Telemetría de RPMs Sin Explotar:** El payload WebSocket del SDK reporta lecturas de `rpms` de tracción motriz, pero actualmente no se consumen debido a la falta de mapeo verificado de índices de rueda, convención de signo y cuantificación del deslizamiento cinemático lateral (*slip*).
7. **Canal de Confianza No Cableado:** La formulación de costo combinada con canal de confianza del modelo de segmentación fue validada fuera de línea, pero actualmente no está cableada en el pipeline en tiempo real de `bev_planner_node`.
8. **Tabla de Rayos Fisheye No Implementada:** El ray-casting homográfico BEV (`bev_proj`) toma ~44.5 ms en GPU; la sustitución por una tabla de consulta precomputada (LUT) para corregir distorsión fisheye de gran angular ($126^\circ$) está pendiente de la calibración de cámara.
9. **Límite Angular `max_drive_angular = 0.45` Sin Validación de Derrape:** Configurado analíticamente como $\text{gain} \times \text{max\_deviation} = 0.01 \times 45.0^\circ = 0.45$ para eliminar la saturación previa a 15°; no ha sido medido dinámicamente con avance concurrente para evaluar el derrape lateral característico del rodado skid-steer.
10. **Baja Exposición Operativa del Modo DRIVE en Campo:** Históricamente el régimen de avance sostenido `DRIVE` se ejecutó durante menos del 2% del tiempo de misión en pruebas reales debido a los bugs de alineación previos; gran parte del lazo de guiado dinámico se valida en banco y simulación.
11. **Gobernador Dinámico de Velocidad (Fail-Safe):** Resuelve la ecuación cuadrática de frenado según la latencia percentil 95 ($t_{\text{plan,P95}}$). Con `require_velocity_governor: true`, inmoviliza el rover ($v=0.0\text{ m/s}$) si no se recibe límite o si expira (>3.0s con path following), y cancela también el giro angular.
12. **Filtro Complementario Roll/Pitch y Diagnóstico Inercial:** Reemplaza al antiguo watchdog por una compuerta estadística dual (media $|\|\mathbf{a}\| - 1.0| < 0.08\text{ g}$, dispersión $\sigma_m < 0.06\text{ g}$) con propagación por integración giroscópica ($P_k = P_{k-1} + Q \cdot \Delta t$) y telemetría en `earth_rover/tilt_gate_diag`.
13. **Rendimiento CPU vs GPU:** En CPU (Ryzen 3 3200G), SAM-TP toma $\approx 4.5\text{ s}$, disparando el corte del gobernador a $v=0.0\text{ m/s}$ (*Stop & Wait*). La operación fluida a 4.2–5.3 Hz requiere GPU (RTX 5060).
14. **Modos de Despliegue `ROVER_MODE`:** `ROVER_MODE: "manual"` (por defecto en `docker-compose.gpu.yml`) para depuración y tests; `ROVER_MODE: "full"` (por defecto en `docker-compose.yml`) lanza `mission1.launch.py`.

---

## 9. Estado de Integración y Validación

### 9.0. Changelog Reciente (Fixes Críticos y Mejoras de Infraestructura)

- **Gobernador Dinámico de Velocidad P95 y Fail-Safe (`bev_planner_node` & `gps_waypoint_controller`):**
  - Adaptación continua de velocidad segura según latencia P95 del ciclo, eliminación del fail-open inseguro y parámetro `require_velocity_governor` para evitar deadlocks en modos aislados. Cancelación de giro angular ante parada de seguridad en DRIVE.
- **Filtro Complementario Roll/Pitch con Compuerta Estadística (`bridge_node.py`):**
  - Sustitución del watchdog discontinuo por integración continua de giróscopo con compuerta de aceleración y tópico de diagnóstico `earth_rover/tilt_gate_diag`.
- **Guarda de Expiración de Rumbo (`gps_waypoint_controller`):**
  - Implementación de `heading_max_stale_s: 3.5` para abortar la navegación ante congelamiento de datos de brújula.
- **Protección Atómica de Respuestas HTTP y Warm Start (`mission_manager_node`):**
  - Protección de variables de respuesta con `threading.Lock()` y restablecimiento inmediato de navegación hacia checkpoints en reanudaciones.
- **Carga Automática de Calibración Inercial (`bridge_node.py`):**
  - Lectura automática de `gyro_bias.json` y `accel_bias.json` con fallback a parámetros base.

---

## 11. Simulación y Banco de Pruebas Gazebo (FrodoBots Mini+ V6.2)

Se dispone de un modelo cinemático URDF y un mundo sintético SDF en el paquete [`mini_plus_localization`](src/mini_plus_localization/) para validación offline de control, odometría, inclinaciones y navegación sin requerir hardware físico ni GPU con CUDA:

* **Modelo URDF del Rover:** [`src/mini_plus_localization/urdf/mini_plus.urdf`](src/mini_plus_localization/urdf/mini_plus.urdf)
  * Dimensiones oficiales de chasis: $250\text{ mm} \times 190\text{ mm} \times 195\text{ mm}$, masa $1.4\text{ kg}$, despeje $45\text{ mm}$.
  * Cinemática Skid-Steer con 4 ruedas de $95\text{ mm}$ de diámetro ($r = 0.0475\text{ m}$), vía de $160\text{ mm}$.
  * Plugins de simulación para ROS 2 Jazzy: `diff_drive` (skid-steer 4WD), `imu_plugin` (MPU-6050 a 50 Hz), `camera_controller` (cámara frontal $1024 \times 576$, HFOV $126^\circ$), y `gps_controller` (GNSS WGS84).
* **Mundo Sintético de Prueba:** [`src/mini_plus_localization/worlds/test_track.world`](src/mini_plus_localization/worlds/test_track.world)
  * Plano horizontal base de $20\text{ m} \times 20\text{ m}$ anclado geodésicamente en Ciudad de México ($19.432608^\circ, -99.133209^\circ$).
  * Corredor tipo vereda de $1.5\text{ m}$ de ancho con bordillos laterales de $15\text{ cm}$ de altura.
  * Rampa de $10^\circ$ de pendiente ($3.0\text{ m}$ de longitud) para validación del filtro complementario y seguimiento inercial.
  * Rampa de $18^\circ$ de pendiente ($2.5\text{ m}$ de longitud), correspondiente a la pendiente máxima nominal del Mini+ V6.2.
  * Obstáculos verticales para pruebas de evasión reactiva.

### 11.1. Alcance y Límites de la Simulación

> [!WARNING]
> **Advertencia sobre Parámetros Geométricos sin Respaldo Empírico:**
> 1. **Pitch de Cámara ($-8^\circ$):** Proviene del valor nominal adoptado tentativamente en el repositorio base. Gazebo **NO PUEDE** validar la calibración de cámara ni la proyección matemática a BEV, porque su verdad de terreno sintética coincide por construcción con el valor geométrico asignado al sensor. La calibración óptica debe realizarse contra el rover físico.
> 2. **Batalla entre ejes ($150\text{ mm}$):** Es una estimación geométrica (la ficha técnica oficial solo especifica la vía de $160\text{ mm}$ y el largo total de $250\text{ mm}$). Debe medirse físicamente en el chasis real.

---

## 12. Suite de Tests Offline y Verificación Continua

Para validar matemáticamente algoritmos, transformaciones geométricas, gobernadores y fusiones sensoriales sin requerir hardware físico ni GPU:

```bash
# Ejecutar suite completa de tests unitarios y de integración offline (24 tests)
python3 -m pytest src/er_planning/test -v
```

### Cobertura de la Suite (24 tests):
1. **Filtro Complementario Roll/Pitch (5 tests):** Convergencia en reposo, seguimiento en rampas dinámicas, integración inercial con compuerta cerrada y continuidad sin saltos en transiciones.
2. **Gobernador Dinámico de Velocidad (4 tests):** Validación estricta de la tabla de frenado cuadrático, corte por piso de velocidad ($0.15\text{ m/s}$), reacción a picos P95 y latencias extremas.
3. **Controlador Motriz y Conversión a Acelerador (3 tests):** Verificación de estados del gobernador en acelerador normalizado, función `speed_to_throttle` (cero, sub-máximo, saturación a fondo, negativos) y timeout de rumbo desactualizado (`heading_max_stale_s`).
4. **Guarda de Retención GPS en Mission Manager (1 test):** Validación ante micro-cortes transitorios de GNSS ($< 10\text{ s}$ retiene coordenadas con warning, $> 10\text{ s}$ rechaza de forma estricta).
5. **Geometría BEV e Isotropía (4 tests):** Isotropía a $0.03\text{ m/px}$, huella de colisión (`footprint_px`), compensación sintética de inclinación y canal de confianza.
6. **Fusión Semántica Dual y Mapa Persistente (4 tests):** Contrato Bayesiano de 5 casos, decaimiento temporal sobre veredas OSM, equivalencia sin capa semántica y neutralidad de celdas.
7. **Progresión Temporal de Evidencia (3 tests):** Aciertos consecutivos, aciertos intercalados con decaimiento temporal y recuperación tras fallos.

---

## 13. Procedimiento de Transición y Despliegue en Máquina GPU (RTX 5060)

Para migrar y ejecutar el entorno en la estación de trabajo con GPU dedicada:

### 13.1. Pasos de Inicialización
```bash
# 1. Actualizar repositorio
git pull origin main

# 2. Levantar contenedor con aceleración NVIDIA
docker compose -f docker-compose.gpu.yml build
docker compose -f docker-compose.gpu.yml up -d

# 3. Acceder al contenedor interactivo
docker exec -it mini_plus_rover bash
```

### 13.2. Verificaciones Obligatorias al Iniciar
```python
# Verificar disponibilidad de CUDA y reconocimiento de la RTX 5060
python3 -c "import torch; assert torch.cuda.is_available(); print('GPU OK:', torch.cuda.get_device_name(0))"
```

### 13.3. Parámetros y Credenciales Locales
* **Credenciales SDK (`src/sdk_server/.env`):** Configurar `SDK_API_TOKEN`, `BOT_SLUG` y `MISSION_SLUG` específicos del bot asignado para la prueba.
* **Confirmar `ROVER_MODE`:** Confirmar que `docker-compose.gpu.yml` mantenga `ROVER_MODE: "manual"` para permitir la ejecución manual y profiling de nodos.
