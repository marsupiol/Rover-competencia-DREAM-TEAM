#!/bin/bash
set -e

# Load ROS environment
source /opt/ros/jazzy/setup.bash

# Source workspace (if built)
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi

# Start the SDK (Hypercorn) in background
cd /root/ros2_ws/src/sdk_server
if command -v hypercorn >/dev/null 2>&1; then
  echo "Starting SDK server on 0.0.0.0:8000"
  nohup hypercorn main:app --bind 0.0.0.0:8000 > /root/ros2_ws/sdk.log 2>&1 &
else
  echo "hypercorn not found; attempting to run with python -m hypercorn"
  nohup python3 -m hypercorn main:app --bind 0.0.0.0:8000 > /root/ros2_ws/sdk.log 2>&1 &
fi

# Give SDK a moment to start
sleep 2

# Launch ROS nodes (bridge + ekf) in foreground so container stays alive
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi

echo "Launching ROS nodes (bridge + ekf)"
exec ros2 launch mini_plus_localization ekf.launch.py sdk_url:=http://localhost:8000
