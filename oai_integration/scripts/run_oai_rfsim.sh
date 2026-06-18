#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-gnb}"
OAI_ROOT="${OAI_ROOT:-/opt/openairinterface5g}"
MOBILEROS_ROOT="${MOBILEROS_ROOT:-$(pwd)}"
GNB_CONF="${GNB_CONF:-$MOBILEROS_ROOT/docs/network_setup/simulated/gnb.conf}"

case "$ROLE" in
  gnb)
    exec "$OAI_ROOT/cmake_targets/ran_build/build/nr-softmodem" -O "$GNB_CONF" --rfsim
    ;;
  ue)
    exec "$OAI_ROOT/cmake_targets/ran_build/build/nr-uesoftmodem" --rfsim --rfsimulator.serveraddr 127.0.0.1
    ;;
  *)
    echo "usage: $0 {gnb|ue}" >&2
    exit 2
    ;;
esac
