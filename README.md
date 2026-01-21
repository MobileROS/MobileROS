# MobileROS

MobileROS integrates radio-awareness directly into the robotics runtime. The repository now uses the unified `mobile_ros` namespace; `wireless_ros` remains only as a deprecated compatibility shim and will be removed in a future release..

## Supported platforms
- Ubuntu 20.04 LTS (tested)
- Ubuntu 22.04 LTS (tested)

## Quick start (Docker)
The Docker workflow is the recommended, hardware-agnostic path:

```bash
./install.sh --docker
```

This builds the image defined in `docker/Dockerfile`, installs the MobileROS core with `ENABLE_OAI=OFF`, and starts the default compose stack. Stop with `CTRL+C`. When you need radio hardware, set `ENABLE_OAI=ON` and enable device mapping as documented in [docs/Network_Configuration_Guide.md](docs/Network_Configuration_Guide.md).

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
- [Network Configuration Guide](docs/Network_Configuration_Guide.md): kernel/UHD expectations, OAI branch/commit, troubleshooting.
- `docs/network_setup/`: curated OAI configuration files. `simulated/gnb.conf` aligns with the paper's simulated experiments; `indoor_lab/gnb.conf` matches the lab USRP setup.
