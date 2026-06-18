#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/ros1_ws"

catkin_make
source devel/setup.bash
roslaunch mobile_ros_ros1 full_stack.launch \
  metrics_source:=jsonl \
  metrics_jsonl:="$ROOT/benchmarks/oai_rfsim_metrics.jsonl"
