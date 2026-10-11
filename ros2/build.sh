#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
ROOT=$(cd "$(dirname "$0")" && pwd)
WS=${FALCON_ROS_WS:-$HOME/falcon_acceptance_ws}
mkdir -p "$WS/src"
TARGET="$WS/src/falcon_ros"
if [[ -e "$TARGET" || -L "$TARGET" ]]; then
  [[ -L "$TARGET" && "$(readlink -f "$TARGET")" == "$(readlink -f "$ROOT/falcon_ros")" ]] || { echo "Refusing to overwrite existing package: $TARGET" >&2; exit 1; }
else
  ln -s "$ROOT/falcon_ros" "$TARGET"
fi
cd "$WS"
colcon build --packages-select falcon_ros
echo "source $WS/install/setup.bash"
