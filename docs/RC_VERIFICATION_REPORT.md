# Release Candidate Verification Report

Date: 2025-12-18
Repository: MobileROS

## Environment
- OS: Linux (uname -a -> Linux 6e8950690be3 6.12.13 x86_64)
- Python: 3.11.12
- CMake: 3.28.3
- Docker: unavailable in CI container (`docker --version` -> command not found)

## A. Naming audit and structure
- Commands:
  - `grep -R "WirelessROS" -n .` → no matches.
  - `grep -R --exclude-dir=.git "wireless_ros" -n .` → no matches after removing deprecated references.
- Fix: updated legacy shim message (`wireless_ros/__init__.py`) and README wording; cleaned historical engine import to `mobile_ros`.
- Structure checks: verified required files exist and are non-empty: `docker/Dockerfile`, `docker/docker-compose.yml`, `docker/entrypoint.sh`, `docs/Network_Configuration_Guide.md`, `README.md`; directories `docs/network_setup/`, `examples/`, `.github/workflows/` present.

## B. Build and install
- Native (ENABLE_OAI=OFF):
  - `cmake -S . -B build_no_oai -DENABLE_OAI=OFF` → success.
  - `cmake --build build_no_oai -j` → success, built `mobileros_core` and `mobileros_example`.
- ENABLE_OAI=ON configure:
  - `cmake -S . -B build_oai -DENABLE_OAI=ON` → expected **FATAL_ERROR**: missing `asn1c` with guidance to install per `docs/Network_Configuration_Guide.md`.
- install.sh:
  - `bash -n install.sh` → pass.
  - `./install.sh --help` → usage text shown.
  - `./install.sh --native` → succeeds; builds core (ENABLE_OAI=OFF) and reports verification binary path.

## C. Docker reproducibility
- `docker build -f docker/Dockerfile .` → failed (docker command unavailable in CI container). Marked **Environment limitation**.
- `docker compose -f docker/docker-compose.yml up --build` → failed (docker command unavailable). Marked **Environment limitation**.

## D. Runtime examples
- `python -m examples.network_aware_camera` → success; observed channel updates, SIGNAL_LOST triggers, conservative policy resync.
- `python -m examples.failsafe_demo` → success; heartbeat stop -> degraded, >5s -> conservative fallback, hub resync after heartbeats.
- `python -m examples.radioinfo_mock_demo` → success; mock metrics repeatedly trigger SIGNAL_LOST and conservative policy without ROS topics.

## E. ROS drop-in and slicing
- ROS drop-in: repository includes catkin workspace `ros_pkgs/` with `package.xml`/`plugin.xml` files and launch examples (`launch/mobile_ros.launch`). Not executed here (ROS not installed) but manifests and launch files present for drop-in replacement.
- Slicing pipeline (mock, no hardware): executed combined run with gNB slice manager, hub slicing controller, and UE slice manager using UDP sockets. Logs confirmed report reception, RRM command promotions/resets, and scheduler updates.

## F. Conclusions
- All checks under A/B/D passed after fixes.
- C failed due to missing Docker engine in CI environment; reproducibility steps documented for hardware/CI with Docker available.
- E validated via manifest inspection and mock slicing run; no hardware required for tested paths.

