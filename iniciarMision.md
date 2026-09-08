  Los comandos exactos para ejecutar el stack en dos terminales del host (asumiendo que el contenedor persistente mini_plus_rover ya está corriendo):

  │ Note
  │ Si el contenedor no estuviera levantado previamente, asegurate de iniciarlo con:
  │ docker compose -f docker-compose.gpu.yml up -d
  ──────
  ### Terminal 1 — Servidor SDK (FrodoBots / WebRTC Agora / Telemetría)

  Ejecutá desde la raíz de tu workspace en el host:

    docker exec -it mini_plus_rover bash -c "python3 /root/ros2_ws/run_sdk.py"

  (O si preferís entrar primero a la terminal interactiva del contenedor:)

    docker exec -it mini_plus_rover bash
    python3 run_sdk.py

  │ Verificación: Esperá a ver en los logs de esta terminal:
  │
  │   INFO:     Application startup complete.
  │   INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
  ──────
  ### Terminal 2 — Stack Autónomo de Misión (ROS 2)

  Una vez que el SDK esté respondiendo en la Terminal 1, abrí la segunda terminal en el host y ejecutá:

    ./run_mission1.sh

  (O de forma explícita mediante docker exec:)

    docker exec -it mini_plus_rover bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch er_bringup mission1.launch.
  py"
  ──────
  ### Parámetros Opcionales Útiles para la Terminal 2:

  • Modo reactivo puro (sin mapa persistente Bayesiano ni D* Lite):
    ./run_mission1.sh enable_global_planning:=false

  • Detención limpia: Para detener el sistema, presioná Ctrl + C primero en la Terminal 2 (para frenar el movimiento y guardar logs de ROS) y luego
  en la Terminal 1 (para cerrar la sesión WebRTC del SDK).
