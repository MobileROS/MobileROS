# Docker Guide for MobileROS

This guide provides reproducible Docker workflows for MobileROS across three deployment scenarios: software-only (core), RFSIM (OAI without hardware), and USRP B210 (full hardware stack).

## Prerequisites

- Docker Engine 20.10+ with Compose plugin
- For B210 scenario: USRP B210 hardware, UHD-compatible kernel, host udev rules

Install Docker Compose plugin if missing:
```bash
sudo apt-get install -y docker-compose-plugin
```

## Scenario 1: Software-Only (ENABLE_OAI=OFF)

Build and run the MobileROS core without any radio dependencies:

```bash
docker compose -f docker/docker-compose.profiles.yml --profile core up --build
```

Expected output:
```
[MobileROS] core-only build (0.1.0-core) initialized at <timestamp>
This binary is built with ENABLE_OAI=OFF. Radio hardware is mocked.
```

Verification:
```bash
docker exec -it mobileros-core-1 /bin/bash
# Inside container:
./build/mobileros_example
```

This scenario is suitable for:
- CI/CD pipelines
- Developing MobileROS control logic without radio access
- Running examples (failsafe_demo.py, network_aware_camera.py)

## Scenario 2: RFSIM (ENABLE_OAI=ON, No Hardware)

Build the OAI-enabled image and run in RF simulator mode:

```bash
docker compose -f docker/docker-compose.profiles.yml --profile rfsim up --build
```

Expected output:
```
RFSIM mode: OAI stack initialized without hardware mapping
```

This scenario validates:
- OAI integration and interface paths
- Engine<->Hub control loops without RF transmission
- Slicing manager interaction with scheduler hooks

Use environment variable `USE_RFSIM=1` to enable RF simulator binaries in the container.

## Scenario 3: USRP B210 (ENABLE_OAI=ON + Hardware)

### Host preparation

1. Install UHD 4.4 on the host:
```bash
sudo apt-get install -y libuhd-dev uhd-host
```

2. Verify B210 detection:
```bash
uhd_usrp_probe
```
Expected: device address and serial number displayed.

3. Load kernel module:
```bash
sudo modprobe sctp
```

### Run with hardware passthrough

```bash
docker compose -f docker/docker-compose.profiles.yml --profile b210 up --build
```

The compose file maps `/dev/bus/usb` and sets `privileged: true` with real-time limits:
- `ulimits.rtprio: 99`
- `ulimits.memlock: -1`
- `network_mode: host`

Expected output:
```
B210 detected; ready for gNB launch
```

Inside the container, verify UHD access:
```bash
docker exec -it mobileros-b210-1 uhd_usrp_probe
```

Launch gNB with a config from `docs/network_setup/indoor_lab/gnb.conf`:
```bash
docker exec -it mobileros-b210-1 bash
# Inside container:
/opt/oai/targets/bin/nr-softmodem.Rel15 -O /opt/mobileros/docs/network_setup/indoor_lab/gnb.conf
```

## OAI Core Network (CN5G)

A standalone 5G Core Network stack is provided in `docker/oai-cn5g/`. This stack does not depend on MobileROS but can be used to validate end-to-end connectivity.

### Start CN5G

```bash
cd docker/oai-cn5g
docker compose up -d
```

Services:
- `oai-mysql` (192.168.70.131): subscriber database
- `oai-nrf` (192.168.70.130): Network Repository Function
- `oai-amf` (192.168.70.132): Access and Mobility Management Function
- `oai-smf` (192.168.70.133): Session Management Function
- `oai-upf` (192.168.70.134): User Plane Function
- `oai-ext-dn` (192.168.70.135): External Data Network

Default subscriber (in `database/oai_db.sql`):
- IMSI: 208950000000032
- K: fec86ba6eb707ed08905757b1bb44b8f
- OPC: C42449363BBAD02B66D16BC975D77CC1

### Change subscriber credentials

Edit `docker/oai-cn5g/database/oai_db.sql` and modify the INSERT statement:
- `ueid`: IMSI (must match your UE)
- `encPermanentKey`: K (128-bit hex)
- `encOpcKey`: OPC (128-bit hex)

Restart MySQL after editing:
```bash
docker compose down
docker compose up -d
```

### Network conflict resolution

If 192.168.70.128/26 conflicts with existing networks on your host, edit `docker/oai-cn5g/docker-compose.yml`:

1. Change `networks.public_net.ipam.config.subnet` to a free range (e.g., `192.168.80.0/26`)
2. Update all `ipv4_address` fields to match the new subnet:
   - oai-nrf: 192.168.80.130
   - oai-mysql: 192.168.80.131
   - oai-amf: 192.168.80.132
   - oai-smf: 192.168.80.133
   - oai-upf: 192.168.80.134
   - oai-ext-dn: 192.168.80.135
3. Update environment variables `NRF_IPV4_ADDRESS`, `UPF_IPV4_ADDRESS`, `MYSQL_SERVER` to match

## Troubleshooting

### asn1c not found (ENABLE_OAI=ON)
The `oai` Docker target installs asn1c automatically. If building natively, install:
```bash
sudo apt-get install asn1c
```

### B210 not detected in container
Verify host-side detection first:
```bash
uhd_usrp_probe
```
If detected on host but not in container:
- Check `--device /dev/bus/usb` is correctly mapped
- Ensure container runs with `privileged: true`
- Confirm host udev rules are installed

### Permission denied for /dev/bus/usb
Add your user to the `dialout` or `usb` group:
```bash
sudo usermod -aG dialout $USER
```
Log out and log back in.

### Network performance issues with B210
Enable real-time scheduling on the host:
```bash
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```
Increase memlock limits in `/etc/security/limits.conf`:
```
* soft memlock unlimited
* hard memlock unlimited
```

### CN5G services fail to start
Check logs for each service:
```bash
docker logs oai-mysql
docker logs oai-amf
```
Common issues:
- MySQL not ready: wait for healthcheck to pass
- Port conflicts: ensure 3306, 80, 8080 are available on host

## Validation Commands

After starting any scenario, validate the stack:

### Core scenario
```bash
docker exec -it mobileros-core-1 python3 -m examples.failsafe_demo
```

### RFSIM scenario
```bash
docker exec -it mobileros-rfsim-1 python3 slicing/gnb_slice_manager/gnb_slice_manager.py --policy-file rrmPolicy.json
```

### B210 scenario
```bash
docker exec -it mobileros-b210-1 uhd_usrp_probe
```

### CN5G
```bash
docker exec -it oai-mysql mysql -uroot -plinux -e "SELECT ueid FROM oai_db.AuthenticationSubscription;"
```
Expected: `208950000000032`
