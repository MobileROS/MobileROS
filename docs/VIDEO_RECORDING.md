# Video recording workflow

Use the browser dashboard when USRP hardware is unavailable or when a clean
screen-recording scene is needed.

```bash
python3 tools/slam_network_dashboard.py --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765`.

The dashboard cycles through low, medium, and high PRB conditions from the paper
baseline. It shows:

- SLAM ground-truth trajectory and estimated trajectory.
- Map point density changes with PRB allocation.
- Throughput, latency, packet loss, and keyframe rate.
- Policy changes including slice promotion under low-PRB critical phases.

For a live ROS/OAI video, run either ROS workspace and record RViz together with
the dashboard:

```bash
python3 oai_integration/scripts/export_jsonl_to_udp.py --input benchmarks/oai_rfsim_metrics.jsonl
ros2 launch mobile_ros_ros2 slam_elastic_stream.launch.py metrics_source:=udp
python3 tools/slam_network_dashboard.py
```

Replace the JSONL exporter with the OAI UDP hook for hardware recordings.
