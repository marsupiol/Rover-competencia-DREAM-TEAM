#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
source "$ROOT_DIR/.venv/bin/activate"
cd "$ROOT_DIR"

# Inicia el SDK en segundo plano
export PYTHONPATH="$ROOT_DIR/src/sdk_server:$PYTHONPATH"
python3 "$ROOT_DIR/run_sdk.py" &
SDK_PID=$!

echo "SDK iniciado con PID $SDK_PID"

# Espera unos segundos para que el servidor HTTP arranque
sleep 5

# Fuente de ROS2 y del workspace para el bridge y ekf
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"

# Inicia el bridge ROS
ros2 run earth_rovers_sdk earth_rover_bridge --ros-args -p sdk_url:=http://localhost:8000 &
BRIDGE_PID=$!

echo "Bridge iniciado con PID $BRIDGE_PID"

# Inicia el EKF
ros2 run robot_localization ekf_node --ros-args --params-file "$ROOT_DIR/src/mini_plus_localization/config/ekf.yaml" &
EKF_PID=$!

echo "EKF iniciado con PID $EKF_PID"

echo "---"
echo "SDK PID: $SDK_PID"
echo "Bridge PID: $BRIDGE_PID"
echo "EKF PID: $EKF_PID"
echo ""
echo "Para detener todo: kill $SDK_PID $BRIDGE_PID $EKF_PID"
wait $SDK_PID $BRIDGE_PID $EKF_PID
