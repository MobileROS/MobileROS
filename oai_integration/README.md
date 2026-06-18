# OpenAirInterface integration

MobileROS integrates with OpenAirInterface through a narrow metric and control
contract instead of requiring robotics code to include OAI headers.

## Supported paths

1. `rfsim`: no USRP required. OAI gNB and UE run with the RF simulator and emit
   metrics through JSONL, UDP, or shared memory.
2. `usrp-b210`: OAI gNB with USRP B210, band n78, 40 MHz TDD profile.
3. `usrp-x410`: OAI gNB with USRP X410 for the V2X/RSU-style setup described
   in the paper.

The repository uses the shared `mobile_ros.oai.OaiMetricProvider` interface for
all paths. ROS 1 and ROS 2 wrappers consume the same policy JSON, so the
application adaptation logic remains identical.

## Runtime contract

Metric records are JSON objects with these fields:

```json
{
  "timestamp": 0.0,
  "ue_id": "ue-slam-01",
  "rnti": "0x4601",
  "sinr_db": 5.0,
  "rsrp_dbm": -92.0,
  "rsrq_db": -9.5,
  "prb_util": 0.30,
  "mcs": 9,
  "throughput_mbps": 4.67,
  "latency_ms": 44.7,
  "packet_loss_pct": 3.20,
  "jitter_ms": 14.0
}
```

Control commands from MobileROS to the gNB slice manager are JSON objects:

```json
{
  "action": "PROMOTE_SLICE",
  "ue_id": "ue-slam-01",
  "intent": "LOW_LATENCY",
  "criticality_score": 0.92,
  "deadline_ms": 200
}
```

## RF simulator workflow

```bash
export OAI_ROOT=/opt/openairinterface5g
export MOBILEROS_ROOT=$PWD

bash oai_integration/scripts/run_oai_rfsim.sh gnb
bash oai_integration/scripts/run_oai_rfsim.sh ue

python3 slicing/gnb_slice_manager/gnb_slice_manager.py --report-bind 0.0.0.0:60000 --rrm-bind 0.0.0.0:60001
python3 ros2_ws/src/mobile_ros_ros2/mobile_ros_ros2/mobile_ros_hub_node.py
```

For local development without OAI binaries, use:

```bash
python3 benchmarks/run_replay.py --mode replay
python3 tools/slam_network_dashboard.py
```

## Hardware workflow

1. Install UHD and verify the radio:

```bash
uhd_find_devices
uhd_usrp_probe
```

2. Build OAI with the same commit/tag used for the experiment.
3. Apply the hook files from `oai_integration/hooks/` or export the same JSON
   metric contract from an existing E2/KPM collector.
4. Start OAI gNB with the n78 TDD configuration and start the MobileROS gNB
   slice manager.
5. Run `benchmarks/run_replay.py --mode hardware --observed <measured.json>`
   to compare measured summaries against the paper baseline.

Hardware results must be stored separately from replay reports. Replay mode is
for deterministic software verification only.
