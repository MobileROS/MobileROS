# Network Configuration Guide

This guide captures the network-specific prerequisites for running MobileROS with OpenAirInterface (OAI).
It has been validated on **Ubuntu 20.04** and **Ubuntu 22.04** using the `develop` branch of OAI at commit `9dca0d5`.

## Kernel and driver requirements
- Recommended kernel: **5.15** (generic) with PREEMPT disabled.
- Required packages: `libsctp-dev`, `libfftw3-dev`, `libconfig++-dev`, `libboost-all-dev`, `libzmq3-dev`.
- USRP users should install **UHD 4.4** and verify `uhd_usrp_probe` succeeds before enabling hardware passthrough.

## OAI build profile
- Branch: `develop`
- Commit: `9dca0d5` (matches the validation for the configs in `docs/network_setup`)
- Build type: `RelWithDebInfo`
- CMake option: `-DENABLE_OAI=ON` when compiling inside this repository

## Network topology examples
- **Simulated gNB/UE loopback**: uses softmodem RF simulator, no radio hardware required. Configuration is in `docs/network_setup/simulated/gnb.conf`.
- **Indoor USRP (lab)**: assumes a single USRP B210 connected over USB 3.0 with external 10 MHz clock. Configuration is in `docs/network_setup/indoor_lab/gnb.conf`.

## Common issues
- **Missing asn1c**: install the ASN.1 compiler (`sudo apt-get install asn1c`) or set `-DASN1C_EXEC=/path/to/asn1c`.
- **UHD device not detected**: ensure the container runs with `--device /dev/bus/usb` or `--privileged` and the host udev rules are installed.
- **Kernel modules**: `sctp` must be loaded (`sudo modprobe sctp`).
- **Clocking**: for indoor lab configs, align to an external 10 MHz reference or adjust `clock_source=internal` in the config.

Refer back to this guide whenever `ENABLE_OAI=ON` is used or radio hardware is attached. For quick validation without hardware, keep `ENABLE_OAI=OFF`.
