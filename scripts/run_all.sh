#!/usr/bin/env bash
# Opción B — Con EKF (mini_plus_localization)
# SDK -> ekf.launch.py (bridge + navsat_transform + doble EKF) -> mission1.launch.py
#
# Nota: hoy gps_waypoint_controller sigue consumiendo earth_rover/gps y
# earth_rover/heading crudos (los mismos que publica el bridge), no la salida
# fusionada del EKF (/odometry/filtered/*). Esta opción corre el EKF en
# paralelo para tener la localización fusionada disponible, pero no cambia
# todavía el comportamiento de navegación respecto a la Opción A.
set -e

ROOT_DIR="/root/ros2_ws"
PID_DIR="$ROOT_DIR/.mission1_pids"
mkdir -p "$PID_DIR"

cleanup() {
    echo ""
    echo "Deteniendo procesos..."
    [ -n "$EKF_PID" ] && kill "$EKF_PID" 2>/dev/null || true
    [ -n "$SDK_PID" ] && kill "$SDK_PID" 2>/dev/null || true
    rm -f "$PID_DIR"/*.pid
    echo "Listo."
}
trap cleanup EXIT INT TERM

echo "=== 1. Verificando MISSION_SLUG en .env ==="
if ! grep -qE "^MISSION_SLUG=" "$ROOT_DIR/src/sdk_server/.env"; then
    echo "ADVERTENCIA: no encontré una línea MISSION_SLUG=... activa (sin '#') en"
    echo "  $ROOT_DIR/src/sdk_server/.env"
    echo "El SDK puede fallar en /start-mission si no está seteada. Editá el .env y reintentá."
    echo "Continuando de todos modos en 5s (Ctrl+C para cancelar)..."
    sleep 5
fi

echo "=== 2. Iniciando SDK ==="
source "$ROOT_DIR/.venv/bin/activate"
cd "$ROOT_DIR"
export PYTHONPATH="$ROOT_DIR/src/sdk_server:$PYTHONPATH"
python3 "$ROOT_DIR/run_sdk.py" &
SDK_PID=$!
echo "$SDK_PID" > "$PID_DIR/sdk.pid"
echo "SDK iniciado con PID $SDK_PID"

echo "Esperando a que el SDK responda en :8000..."
for i in $(seq 1 15); do
    if curl -sf http://localhost:8000/data > /dev/null 2>&1; then
        echo "SDK OK."
        break
    fi
    sleep 1
    if [ "$i" -eq 15 ]; then
        echo "ADVERTENCIA: el SDK no respondió en 15s. Sigo igual, pero revisá los logs si algo falla."
    fi
done

echo "=== 3. Iniciando stack EKF (bridge + navsat_transform + doble EKF) ==="
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"

ros2 launch mini_plus_localization ekf.launch.py sdk_url:=http://localhost:8000 &
EKF_PID=$!
echo "$EKF_PID" > "$PID_DIR/ekf.pid"
echo "Stack EKF iniciado con PID $EKF_PID"

sleep 3

echo "=== 4. Lanzando misión 1 (gps_waypoint_controller + mission_manager_node) ==="
echo "---"
echo "SDK PID: $SDK_PID"
echo "EKF PID: $EKF_PID"
echo ""
echo "Ctrl+C detiene todo (SDK + EKF/bridge + nodos de misión)."
echo "---"

ros2 launch er_bringup mission1.launch.py