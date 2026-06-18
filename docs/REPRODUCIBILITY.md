# Reproducibility workflow

MobileROS separates three result classes:

1. Paper baseline: values transcribed from the paper tables in
   `benchmarks/paper_baselines.json`.
2. Software replay: deterministic execution against the paper baseline. This is
   useful for verifying reporting, policy selection, and visualization.
3. Measured run: OAI RF simulator or USRP/OAI logs collected from an actual run.

Software replay must not be reported as a new hardware experiment.

## No-hardware validation

```bash
python3 -m unittest discover -s tests
python3 benchmarks/run_replay.py --mode replay
python3 oai_integration/scripts/export_jsonl_to_udp.py --input benchmarks/oai_rfsim_metrics.jsonl
```

The JSONL stream emulates OAI RF simulator metrics and drives the same
MobileROS policy path used by ROS 1 and ROS 2.

## Hardware or RF simulator comparison

Produce a measured summary JSON with the same table/row/metric shape as
`benchmarks/paper_baselines.json`, then run:

```bash
python3 benchmarks/run_replay.py --mode hardware --observed benchmarks/results/measured.json --tolerance 0.05
```

Use a tolerance that reflects the run-to-run variation in the paper. The
comparison report lists every metric, expected value, observed value, and pass
status.

## SLAM task setup

Recommended open-source task stack:

- ORB-SLAM3 for distributed visual SLAM.
- Standard `sensor_msgs/Image` camera topics through `camera_cell_node`.
- OAI RF simulator for no-USRP integration tests.
- USRP B210/X410 for hardware runs.

MobileROS modifies source-side data generation and scheduling policy; the SLAM
frontend should remain unmodified where possible.
