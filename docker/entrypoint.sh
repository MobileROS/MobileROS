#!/usr/bin/env bash
set -euo pipefail

echo "[INFO] Starting MobileROS container (ENABLE_OAI=${ENABLE_OAI:-OFF})"
if [[ "${ENABLE_OAI:-OFF}" == "ON" ]]; then
  echo "[WARN] ENABLE_OAI=ON expects radio dependencies and device mapping. See docs/Network_Configuration_Guide.md."
fi

exec "$@"
