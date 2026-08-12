#!/bin/bash
set -e
cd /root/ros2_ws
pkill -f '/root/ros2_ws/run_sdk.py' >/dev/null 2>&1 || true
pkill -f 'uvicorn' >/dev/null 2>&1 || true
sleep 1
nohup /root/ros2_ws/.venv/bin/python3 /root/ros2_ws/run_sdk.py > /root/ros2_ws/sdk.log 2>&1 &
echo  > /root/ros2_ws/sdk.pid
