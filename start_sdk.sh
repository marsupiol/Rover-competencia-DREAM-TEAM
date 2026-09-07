#!/bin/bash
set -e
cd /root/ros2_ws
pkill -f '/root/ros2_ws/run_sdk.py' >/dev/null 2>&1 || true
pkill -f 'uvicorn' >/dev/null 2>&1 || true
pkill -f 'hypercorn' >/dev/null 2>&1 || true
sleep 1
PYTHON="python3"
if [ -f "/root/ros2_ws/.venv/bin/python3" ]; then
    PYTHON="/root/ros2_ws/.venv/bin/python3"
fi
nohup $PYTHON /root/ros2_ws/run_sdk.py > /root/ros2_ws/sdk.log 2>&1 &
echo $! > /root/ros2_ws/sdk.pid
