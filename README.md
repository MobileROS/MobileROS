# MobileROS

MobileROS is a wireless-native robotics runtime that connects ROS or ROS 2
applications with OpenAirInterface radio telemetry and slice-control hooks. It
implements the Hub-Engines-Cells model from the paper and provides both
hardware and no-hardware validation paths.

## What is included

- Shared Python runtime in `mobile_ros/`
  - OAI metric providers: JSONL replay, UDP, shared memory
  - Elastic Stream, Adaptive Voxel, and slice-promotion policies
  - Runtime orchestration, slice clients, typed frame/task data, and task cells
  - Task adapters for visual SLAM, LiDAR perception, V2X safety,
    multi-robot map sharing, and partitioned execution
  - Paper-baseline replay and comparison helpers
- ROS 1 workspace in `ros1_ws/`
  - Catkin package for Noetic
  - Hub, Camera, SLAM, LiDAR, slice client, V2X safety, and partition nodes
- ROS 2 workspace in `ros2_ws/`
  - Ament Python package for Foxy/Humble
  - Matching Hub, Camera, SLAM, LiDAR, slice client, V2X safety, and partition nodes
- OAI integration files in `oai_integration/`
  - RF simulator run script
  - UDP metric export hook
  - Hook copy helper for OAI scheduler integration
  - USRP B210/X410 deployment notes
- Third-party setup in `third_party/`
  - ORB-SLAM3 and OpenPCDet install scripts
  - OpenAirInterface version target and integration manifest
- Reproducibility assets in `benchmarks/`
  - Paper table baselines
  - Deterministic replay runner
  - OAI RF simulator-style metric stream
- Recording dashboard in `tools/slam_network_dashboard.py`
  - Browser visualization of SLAM trajectory, map density, PRB allocation,
    throughput, latency, packet loss, and adaptation policy.

## Reproducibility boundary

`benchmarks/run_replay.py --mode replay` uses reference values transcribed from
the paper to exercise the same reporting and comparison paths. It is a
deterministic software replay, not a new hardware measurement.

Use `--mode hardware --observed <measured.json>` for actual USRP/OAI or OAI RF
simulator measurements. Hardware reports should be stored separately from replay
reports.

## Quick start without USRP

```bash
python3 -m unittest discover -s tests
python3 benchmarks/run_replay.py --mode replay
python3 tools/mobileros_cli.py --case all --frames 20 --output benchmarks/results/cli_cases_report.json
python3 tools/slam_network_dashboard.py
```

Open `http://127.0.0.1:8765` and record the browser window for the SLAM/network
visualization.

## ROS 1

```bash
cd ros1_ws
catkin_make
source devel/setup.bash
roslaunch mobile_ros_ros1 full_stack.launch metrics_jsonl:=$(pwd)/../benchmarks/oai_rfsim_metrics.jsonl
```

The full stack publishes MobileROS policy, adapted camera frames, SLAM feature
packets, adapted point clouds, slice commands, V2X safety commands, and
partition decisions.

## ROS 2

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch mobile_ros_ros2 full_stack.launch.py metrics_jsonl:=$(pwd)/../benchmarks/oai_rfsim_metrics.jsonl
```

The ROS 2 wrapper uses the same policy JSON and topic layout as ROS 1.

## Task backends

The runnable adapters are in `mobile_ros/tasks/`. Heavy robotics backends are
not vendored:

- Visual SLAM: ORB-SLAM3 from `https://github.com/UZ-SLAMLab/ORB_SLAM3.git`.
- LiDAR perception: OpenPCDet from `https://github.com/open-mmlab/OpenPCDet.git`.
- Radio/network: OpenAirInterface v2.1.0 with RF simulator or USRP B210/X410.

Install helpers:

```bash
bash third_party/install_orb_slam3.sh
bash third_party/install_openpcdet.sh
```

The adapters keep MobileROS responsible for network-aware frame rates,
resolution, keyframe flow, voxel size, partitioning, and slice priority. The
external task libraries remain the task backends.

## OpenAirInterface RF simulator

```bash
export OAI_ROOT=/opt/openairinterface5g
export MOBILEROS_ROOT=$PWD

bash oai_integration/scripts/run_oai_rfsim.sh gnb
bash oai_integration/scripts/run_oai_rfsim.sh ue
python3 oai_integration/scripts/export_jsonl_to_udp.py --input benchmarks/oai_rfsim_metrics.jsonl
```

For ROS nodes, set `metrics_source:=udp` so the Hub consumes the UDP metric
stream.

## USRP hardware path

1. Verify UHD:

```bash
uhd_find_devices
uhd_usrp_probe
```

2. Build OAI with the experiment version and n78 40 MHz TDD configuration.
3. Add the hook in `oai_integration/hooks/` or export the same JSON metric
   contract from the existing E2/KPM collector.
4. Start `slicing/gnb_slice_manager/gnb_slice_manager.py` to receive UE state
   reports and RRM commands.
5. Run ROS 1 or ROS 2 wrappers with `metrics_source:=udp` or a JSONL log
   captured from the hardware run.

## Experiment comparison

Replay:

```bash
python3 benchmarks/run_replay.py --mode replay --output benchmarks/results/replay_report.json
```

Hardware or RF simulator summary:

```bash
python3 benchmarks/run_replay.py --mode hardware --observed benchmarks/results/measured.json --tolerance 0.05
```

The expected JSON structure is:

```json
{
  "table_iii_orb_slam3_prb": {
    "low_prb_30": {
      "link_rate_mbps": [4.67, 0.15],
      "ul_latency_ms": [44.7, 1.5]
    }
  }
}
```

Only metrics present in the measured file are compared.

## Repository notes

The upstream repository contains an OAI source mirror with Windows-incompatible
`Zone.Identifier` paths. On Windows, use sparse checkout for MobileROS runtime
work and clone/apply OAI patches in a Linux environment for OAI builds.
