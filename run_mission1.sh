#!/usr/bin/env bash
# ==============================================================================
# Lanzador de la Misión 1 en ROS 2 dentro del contenedor mini_plus_rover
# ==============================================================================
set -e

CONTAINER_NAME="mini_plus_rover"

echo "=== Lanzando Misión 1 (er_bringup mission1.launch.py) ==="
docker exec -it "${CONTAINER_NAME}" bash -c "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch er_bringup mission1.launch.py $*"
