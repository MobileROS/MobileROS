# MobileROS

MobileROS integrates radio-awareness directly into the robotics runtime, enabling application-driven dynamic network resource allocation through a Hub-Engine-Cell architecture. The system provides bidirectional control loops between robot tasks and 5G RAN, with built-in degraded mode failsafe for network disruptions.

## Core Architecture

**Hub**: Global policy coordinator managing cross-robot resource allocation, engine lifecycle, and system-wide optimization. Monitors network state and distributes policies to engines.

**Engines**: Local execution units performing specialized functions:
- **Radio Information Engine**: Collects PHY/MAC metrics, predicts bandwidth/latency, detects signal loss
- **Physical Adaptive Engine**: Adjusts MCS/PRB allocation, compression ratios based on channel conditions and Hub policies
- **Cross-Domain Engine**: Translates application semantics (task criticality) to network requests (slice promotion/demotion)

**Cells**: Robot-side policy executors with cached last-known-good policies for degraded mode operation.

## Key Capabilities

**Application-Driven Bidirectional Slicing**:
- Robot→Network: Task state triggers resource upgrades (BestEffort → URLLC semantics via RRM commands)
- Network→Robot: Congestion/degradation feedback triggers local adaptation (codec change, frame rate reduction)

**Degraded Mode Failsafe**:
- Heartbeat timeout (0.5s): switch to cached policy
- Extended partition (5.0s): enter conservative mode with safe fallback behavior
- Auto-resync on recovery without state pollution

**Dynamic Resource Adaptation**:
- Real-time MCS selection based on SNR/BLER feedback and RL models
- PRB allocation considering task criticality and spectrum efficiency
- Adaptive HARQ retransmission and compression ratios

## Quick Start

### Native Build
```bash
./install.sh --native
./build/mobileros_example
```
Default build uses `ENABLE_OAI=OFF` (hardware-agnostic). Enable OAI integration with `cmake -S . -B build -DENABLE_OAI=ON` after satisfying prerequisites in [docs/Network_Configuration_Guide.md](docs/Network_Configuration_Guide.md).

### Python Examples (No ROS Required)
```bash
# Failsafe demo: degraded → conservative → resync
python3 -m examples.failsafe_demo

# Network-aware camera with SIGNAL_LOST handling
python3 -m examples.network_aware_camera

# Force degraded mode for testing
FORCE_DEGRADED_MODE=true python3 -m examples.failsafe_demo
```

### RRM Policy and Slicing
```bash
# Start gNB slice manager with custom policy
python3 slicing/gnb_slice_manager/gnb_slice_manager.py --policy-file rrmPolicy.json

# Start Hub slicing controller (sends RRM commands)
python3 slicing/hub_slicing_controller/slicing_controller.py --gnb-addr 127.0.0.1:60001
```
Policy files (`rrmPolicy.json`, `rrmPolicy_sub.json`) define PRB ratios, sub-slice mappings, and UE-to-slice assignments. See [docs/Network_Configuration_Guide.md](docs/Network_Configuration_Guide.md) for schema details.

### Docker Deployment
For reproducible environments across software/RFSIM/hardware scenarios, see [docs/Docker_Guide.md](docs/Docker_Guide.md). Quick example:
```bash
# Software-only (no OAI dependencies)
docker compose -f docker/docker-compose.profiles.yml --profile core up --build

# With OAI and USRP B210
docker compose -f docker/docker-compose.profiles.yml --profile b210 up --build
```

## Configuration Files

**Network Topologies** (`docs/network_setup/`):
- `simulated/gnb.conf`: softmodem RF simulator (no hardware)
- `indoor_lab/gnb.conf`: USRP B210 with external 10 MHz clock

**RRM Policies** (repository root):
- `rrmPolicy.json`: default slice PRB allocation
- `rrmPolicy_sub.json`: sub-slice scheduling weights
- `rrmPolicy_sub_multiUE.json`: multi-UE slice mappings

## System Validation

Run the slicing pipeline (no hardware required):
```bash
# Terminal 1: gNB slice manager
python3 slicing/gnb_slice_manager/gnb_slice_manager.py

# Terminal 2: Hub slicing controller (emits mock triggers)
python3 slicing/hub_slicing_controller/slicing_controller.py

# Terminal 3: UE slice manager (sends criticality reports)
python3 slicing/ue_slice_manager/ue_slice_manager.py
```
Expected: RRM commands propagate from Hub → gNB scheduler, PRB weights update based on criticality scores.

## Supported Platforms
- Ubuntu 20.04 LTS
- Ubuntu 22.04 LTS

## Documentation
- [Network Configuration Guide](docs/Network_Configuration_Guide.md): OAI integration, kernel/UHD requirements, RRM policy schema
- [Docker Guide](docs/Docker_Guide.md): multi-scenario Docker workflows, OAI CN5G deployment
- [RC Verification Report](docs/RC_VERIFICATION_REPORT.md): build/test validation results

## Namespace Migration
The repository uses the unified `mobile_ros` namespace. `wireless_ros` remains as a deprecated compatibility shim and emits a warning on import.
