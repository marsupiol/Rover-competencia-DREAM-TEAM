#!/bin/bash
set -e

WS=/root/ros2_ws

# --- Entorno ROS ---
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

# --- .env del SDK ---
# Si no montaste/copiaste un .env real, generamos uno a partir del sample para
# que el contenedor al menos arranque (hay que editar credenciales reales).
if [ ! -f "$WS/src/sdk_server/.env" ] && [ -f "$WS/src/sdk_server/.env.sample" ]; then
  echo "[entrypoint] No hay .env en src/sdk_server; copiando .env.sample como base."
  echo "[entrypoint] Montá tu propio .env (docker run -v ./.env:/root/ros2_ws/src/sdk_server/.env) para credenciales reales."
  cp "$WS/src/sdk_server/.env.sample" "$WS/src/sdk_server/.env"
fi

# ROVER_MODE controla qué se lanza:
#   full   (default) -> SDK + bridge + EKF (mini_plus_localization) + navegación/misión (er_bringup mission1)
#   ekf    -> SDK + sólo bridge + EKF (equivalente a la opción B de run_all.sh, sin mission1)
#   bridge -> SDK + sólo bridge directo, sin EKF (equivalente a run_mission1_bridge.sh, opción A)
#   manual -> no arranca nada automáticamente (ni SDK ni nodos ROS), contenedor vivo para control manual
ROVER_MODE="${ROVER_MODE:-full}"
SDK_URL="${SDK_URL:-http://localhost:8000}"
PIDS=()

# --- SDK server (Hypercorn) ---
# En modo manual no se arranca automáticamente.
if [ "$ROVER_MODE" != "manual" ]; then
  cd "$WS/src/sdk_server"
  echo "[entrypoint] Iniciando SDK server en 0.0.0.0:8000"
  nohup hypercorn main:app --bind 0.0.0.0:8000 > "$WS/sdk.log" 2>&1 &
  PIDS+=("$!")

  echo "[entrypoint] Esperando a que el SDK responda en :8000..."
  for i in $(seq 1 15); do
    if curl -sf http://localhost:8000/data > /dev/null 2>&1; then
      echo "[entrypoint] SDK OK."
      break
    fi
    sleep 1
    if [ "$i" -eq 15 ]; then
      echo "[entrypoint] ADVERTENCIA: el SDK no respondió en 15s, sigo igual."
    fi
  done

  cd "$WS"
  source /opt/ros/jazzy/setup.bash
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
fi

case "$ROVER_MODE" in
  full)
    echo "[entrypoint] ROVER_MODE=full -> bridge + EKF + navegación + misión"
    # mission1.launch.py ya incluye ekf.launch.py (IncludeLaunchDescription),
    # así que NO lo lanzamos por separado acá: hacerlo duplicaba cada nodo del
    # EKF (base_link_to_gps, ekf_filter_node_odom, ekf_filter_node_map,
    # navsat_transform, ekf_heading_bridge) con el mismo nombre.
    ros2 launch er_bringup mission1.launch.py &
    PIDS+=("$!")
    ;;
  ekf)
    echo "[entrypoint] ROVER_MODE=ekf -> bridge + EKF (sin nodos de misión)"
    ros2 launch mini_plus_localization ekf.launch.py sdk_url:="$SDK_URL" &
    PIDS+=("$!")
    ;;
  bridge)
    echo "[entrypoint] ROVER_MODE=bridge -> sólo bridge (sin EKF)"
    ros2 run earth_rovers_sdk earth_rover_bridge --ros-args -p sdk_url:="$SDK_URL" &
    PIDS+=("$!")
    ;;
  manual)
    echo "[entrypoint] ROVER_MODE=manual -> no se arranca nada automáticamente. Usá 'docker exec -it mini_plus_rover bash' para control total."
    ;;
  *)
    echo "[entrypoint] ROVER_MODE desconocido: $ROVER_MODE (usar full|ekf|bridge|manual)"
    exit 1
    ;;
esac

# Si cualquiera de los procesos (SDK, bridge/EKF, misión) muere, el contenedor
# termina en vez de quedar "vivo" a medias. Con --restart unless-stopped en
# docker/compose eso da un reinicio limpio de todo el stack.
cleanup() {
  trap - EXIT INT TERM
  echo "[entrypoint] Señal recibida, deteniendo procesos..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  [ -n "$WAIT_PID" ] && kill "$WAIT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "${#PIDS[@]}" -gt 0 ]; then
  wait -n "${PIDS[@]}"
  EXIT_CODE=$?
  echo "[entrypoint] Un proceso del stack terminó (exit $EXIT_CODE). Cerrando contenedor."
  exit "$EXIT_CODE"
else
  # En modo manual no hay procesos automáticos de fondo.
  # Mantenemos el contenedor vivo esperando señales (SIGTERM/SIGINT) para salir limpiamente.
  sleep infinity &
  WAIT_PID=$!
  wait "$WAIT_PID" || true
fi