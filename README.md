# Earth Rover Mini+ — Stack Autónomo ROS 2 Jazzy

Arquitectura de navegación autónoma, percepción visual profunda (SAM-TP), planificación en vista de pájaro (BEV GeNIE) y control reactivo para el rover **FrodoBots Earth Rover Mini+** sobre **ROS 2 Jazzy**.

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
|                              |  │  |    - Clustering adaptativo K-Means orientado a la meta        |
|  [ekf_filter_node_odom]      |  │  |    - Publica: earth_rover/planned_path (nav_msgs/Path)        |
|    -> odometry/local (odom)  |  │  |    - Publica: earth_rover/planner_valid (std_msgs/Bool)       |
|                              |  │  |    - Publica: earth_rover/planner_visualization (Image)       |
|  [ekf_filter_node_map]       |  │  +---------------------------------------------------------------+
|    -> odometry/global (map)  |  │                │                            │
|                              |  │                │ planned_path               │ planner_valid
|  [ekf_heading_bridge]        |  │                ▼                            ▼
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
       │                          │                │
       ▼                          │                │
[Herramienta Standalone de Debug] │                │
  PAQUETE: er_perception          │                │
  [traversability_node] ◄─────────+                │
    (Percepción reactiva 2D por corredores - fuera del pipeline en vivo)
```

---

## 2. Resumen de Paquetes ROS 2

| Paquete | Descripción | Nodos Principales |
| :--- | :--- | :--- |
| **`earth_rovers_sdk`** | Bridge de comunicación bidireccional entre ROS 2 y la API HTTP/WebSocket/MJPEG del SDK. | `earth_rover_bridge` |
| **`mini_plus_localization`** | Fusión sensorial mediante EKF dual (`robot_localization`) y bridge de rumbo REP-105. | `ekf_filter_node_odom`, `ekf_filter_node_map`, `navsat_transform_node`, `ekf_heading_bridge`, `static_transform_publisher` |
| **`er_planning`** | Percepción neuronal de transitabilidad (SAM-TP), proyección geométrica BEV y planificación de caminos (GeNIE). | `bev_planner_node` |
| **`er_navigation`** | Control motriz reactivo, seguimiento de trayectorias BEV con lookahead dinámico y fallback a GPS. | `gps_waypoint_controller` |
| **`er_mission`** | Gestor de misiones, ciclo de vida de checkpoints geodésicos y confirmación HTTP con el SDK. | `mission_manager_node` |
| **`er_bringup`** | Orquestación general del sistema y launch files consolidados. | Launch: `mission1.launch.py` |
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
- **`navsat_transform_node`**: Proyecta coordenadas GNSS al plano cartesiano local.
  - *Consume:* `/gps/fix`, `/imu/data`, `odometry/global`.
  - *Publica:* `/odometry/gps` y `gps/filtered` (`sensor_msgs/msg/NavSatFix`).
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
- **Parámetros Clave:**
  - `resolution_m_per_px` (`float`, default: `0.03`): Resolución métrica de la grilla BEV ($3\text{ cm/px}$).
  - `forward_range_m` (`float`, default: `4.0`): Horizonte longitudinal local ($4\text{ m}$).
  - `side_range_m` (`float`, default: `2.0`): Semiancho lateral local ($4\text{ m}$ de ancho total).
  - `grid_size` (`int`, default: `240`): Dimensión de la grilla del planificador ($240 \times 240$).
  - `threshold_cost` (`float`, default: `0.50`): Umbral de costo a partir del cual se considera obstáculo.
  - `use_clustering` (`bool`, default: `true`): Habilita K-Means adaptativo para seleccionar la rama hacia el waypoint.

---

### 3.4. `gps_waypoint_controller` (`er_navigation`)
Controlador motriz híbrido reactivo con mitigación de jitter 4G (Burst & Wait), seguimiento de trayectorias BEV y fallback geodésico.
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

## 4. Ejecución del Stack Completo

### 4.1. Variables de Entorno y Credenciales
Configurar las credenciales en `src/sdk_server/.env` (tomando como base `.env.sample`):
```bash
SDK_API_TOKEN="tu_token_frodobots"
BOT_SLUG="slug_del_rover"
MISSION_SLUG="mission-1"
```

### 4.2. Lanzamiento del Stack Integrado (Producción)
```bash
# 1. En una terminal: Iniciar el servidor SDK
cd /root/ros2_ws/src/sdk_server
hypercorn main:app --bind 0.0.0.0:8000

# 2. En otra terminal: Lanzar la misión completa en ROS 2
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch er_bringup mission1.launch.py
```

### 4.3. Ejecución con Docker / Docker Compose
El workspace incluye soporte para despliegues contenerizados:
- **`Dockerfile`**: Imagen base sobre `osrf/ros:jazzy-ros-base` con dependencias compiladas.
- **`docker-compose.yml`**: Orquesta el contenedor montando el `.env` local (`ROVER_MODE: "full"`).
```bash
docker compose up --build
```

---

## 5. Guía de Depuración y Ejecución Modular

### 5.1. Ejecutar sólo `bev_planner_node` (con cámara real o simulada)
```bash
source install/setup.bash
ros2 run er_planning bev_planner_node --ros-args --params-file src/er_planning/config/planner_params.yaml
```
Para visualizar el mapa BEV y las trayectorias evaluadas en tiempo real:
```bash
ros2 run rqt_image_view rqt_image_view /earth_rover/planner_visualization
```

### 5.2. Ejecutar `traversability_node` (Herramienta Standalone)
```bash
source install/setup.bash
ros2 run er_perception traversability_node --ros-args --params-file src/er_perception/config/perception_params.yaml
```

### 5.3. Inspección de Telemetría y Diagnóstico de Rumbo
```bash
# Monitoreo del estado del controlador híbrido
ros2 topic echo /earth_rover/waypoint_status

# Verificación de trayectorias planificadas
ros2 topic echo /earth_rover/planned_path

# Verificación de validez del planner
ros2 topic echo /earth_rover/planner_valid
```

---

## 6. Setup del Modelo de Inteligencia Artificial (SAM-TP & GeNIE)

El pipeline de percepción y planificación utiliza el modelo **SAM-TP** (Segment Anything Model 2.1 Hiera-tiny con prompt de transitabilidad aprendido) y la librería **GeNIE Path Planner**.

### 6.1. Instalación Canónica de Dependencias
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

### 6.2. Modos de Ejecución: Local vs. Servidor GPU
1. **Local (CPU / GPU integrada):** El modelo descarga automáticamente los pesos `checkpoint_finetuned_v2.pt` desde Hugging Face (`sanatem/samtp-mini-traversability`) y los almacena en `~/.cache/rover_traversability/`.
2. **Servidor Acelerado (GPU Dedicada):** Para procesamiento en tiempo real con latencias de inferencia $<50\text{ ms}$, se recomienda ejecutar el contenedor sobre hardware con NVIDIA Container Toolkit (`--gpus all` o `device: cuda`).

---

## 7. Estado Actual y Limitaciones Conocidas

1. **Rendimiento de Inferencia en CPU:** En procesadores x86 estándar sin GPU dedicada, el ciclo completo de inferencia SAM-TP toma entre $4.0\text{ s}$ y $5.5\text{ s}$ por frame (medido en tests reales). Gracias a la arquitectura desacoplada en hilos independientes y al fallback de frescura (`path_max_stale_s`), el bucle de control no se congela, pero la navegación en tiempo real a alta velocidad requiere aceleración GPU.
2. **Dimensiones de la Huella (`footprint_px`):** El parámetro `footprint_px` en `planner_params.yaml` está configurado por defecto en 10 píxeles ($\approx 30\text{ cm}$). Se encuentra pendiente la confirmación milimétrica en banco de pruebas del chasis real del Earth Rover Mini+.
3. **Reemplazo en Vivo de Traversability:** `traversability_node` (percepción 2D basada en franjas de imagen) ha sido desacoplado del pipeline en vivo a favor de `bev_planner_node` (que provee proyección métrica BEV y trayectorias continuas).
4. **Jitter de Red 4G/LTE:** La latencia variable en la transmisión de comandos y telemetría del rover es mitigada mediante la máquina de estados *Burst & Wait* y los filtros EKF duales.

---

## 8. Próximos Módulos y Roadmap

- [ ] **Validación End-to-End con Servidor GPU Remoto:** Despliegue del nodo de inferencia en estación base remota y transmisión de trayectorias planificadas comprimidas vía DDS/ZeroMQ.
- [ ] **Evasión Reactiva Lateral Fina (Wall Following):** Integración de control de contorno lateral en pasillos estrechos cuando ambos lados presentan obstáculos cercanos.
- [ ] **Calibración Dinámica de Tracción y Deslizamiento:** Ajuste empírico de las matrices de covarianza de odometría según el tipo de terreno (arena, asfalto, césped).

