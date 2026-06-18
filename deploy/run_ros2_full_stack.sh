#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/ros2_ws"

colcon build --symlink-install
source install/setup.bash
ros2 launch mobile_ros_ros2 full_stack.launch.py \
  metrics_source:=jsonl \
  metrics_jsonl:="$ROOT/benchmarks/oai_rfsim_metrics.jsonl"
