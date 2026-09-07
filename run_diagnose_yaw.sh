#!/usr/bin/env bash
# ==============================================================================
# Helper para ejecutar el diagnóstico de signo de yaw dentro del contenedor
# ==============================================================================
set -e

CONTAINER_NAME="mini_plus_rover"

# Si no se pasan argumentos, corre por defecto la ráfaga de 3s a w=0.70
ARGS="${*:-"--burst --w 0.70 --duration 3.0"}"

docker exec -it "${CONTAINER_NAME}" bash -c "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run er_navigation diagnose_yaw_sign ${ARGS}"
