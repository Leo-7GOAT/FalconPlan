#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
ROOT=$(cd "$(dirname "$0")" && pwd)
MODE=${1:-mock_io}
[[ "$MODE" == mock_io || "$MODE" == carla ]] || { echo "Use mock_io or carla" >&2; exit 1; }
BAG=$(find "$ROOT/evidence/$MODE" -maxdepth 1 -type d -name 'bag_*' | sort | tail -n 1)
[[ -n "$BAG" ]] || { echo "No recorded bag" >&2; exit 1; }
python3 "$ROOT/replay.py" --bag "$BAG" --output "$ROOT/evidence/$MODE/replay.json"
