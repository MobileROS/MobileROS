# MobileROS

MobileROS integrates radio-awareness directly into the robotics runtime. The repository now uses the unified `mobile_ros` namespace; `wireless_ros` remains only as a deprecated compatibility shim and will be removed in a future release...

## Supported platforms
- Ubuntu 20.04 LTS (tested)
- Ubuntu 22.04 LTS (tested)

## Quick start (Docker)
The Docker workflow is the recommended, hardware-agnostic path:

```bash
./install.sh --docker
```

This builds the image defined in `docker/Dockerfile`, installs the MobileROS core with `ENABLE_OAI=OFF`, and starts the default compose stack. Stop with `CTRL+C`.

For advanced scenarios (RFSIM without hardware, or USRP B210 with hardware passthrough), use the profiles-based compose:

```bash
# Software-only (core)
docker compose -f docker/docker-compose.profiles.yml --profile core up --build

# OAI with RF simulator (no hardware)
docker compose -f docker/docker-compose.profiles.yml --profile rfsim up --build

# OAI with USRP B210 (requires hardware)
docker compose -f docker/docker-compose.profiles.yml --profile b210 up --build
```

See [docs/Docker_Guide.md](docs/Docker_Guide.md) for detailed instructions on each scenario, including OAI Core Network (CN5G) deployment.

## Native install (developers)
Native builds are intended for advanced users. Dependencies are checked up front and will exit with actionable guidance if missing.

```bash
./install.sh --native
# Run the example binary to confirm the build
./build/mobileros_example
```

The native path builds only the MobileROS core (`ENABLE_OAI=OFF`) to avoid hardware requirements. Enable OAI later with `cmake -S . -B build -DENABLE_OAI=ON` once you satisfy the network prerequisites in the network guide..

## Example workflow
1. Clone the repo: `git clone https://github.com/MobileROS/MobileROS.git && cd MobileROS`
2. Choose Docker or native install (commands above).
3. Validate the build: `./build/mobileros_example`
4. For radio experiments, pick a config from `docs/network_setup/` and follow [docs/Network_Configuration_Guide.md](docs/Network_Configuration_Guide.md) before enabling OAI.

## Additional documentation
- [Docker Guide](docs/Docker_Guide.md): reproducible Docker workflows for core/RFSIM/B210 scenarios, OAI CN5G deployment.
- [Network Configuration Guide](docs/Network_Configuration_Guide.md): kernel/UHD expectations, OAI branch/commit, RRM policy loading, troubleshooting.
- `docs/network_setup/`: curated OAI configuration files. `simulated/gnb.conf` aligns with the paper's simulated experiments; `indoor_lab/gnb.conf` matches the lab USRP setup.
