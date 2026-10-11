#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$ROOT/evidence"
bash "$ROOT/build.sh" 2>&1 | tee "$ROOT/evidence/build.log"
{
  date -Iseconds
  uname -a
  lscpu
  free -m
  python3 --version
  dpkg-query -W ros-humble-rclpy ros-humble-ackermann-msgs ros-humble-rosbag2
} > "$ROOT/evidence/environment.txt"
/usr/bin/time -v -o "$ROOT/evidence/mock_runtime.txt" bash "$ROOT/accept.sh"
# Real CARLA optional: bash ros2/accept.sh --carla --host <server> (requires exclusive tick ownership).
