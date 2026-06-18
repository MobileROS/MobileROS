#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 /path/to/openairinterface5g" >&2
  exit 2
fi

OAI_ROOT="$(cd "$1" && pwd)"
HOOK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../hooks" && pwd)"
TARGET="$OAI_ROOT/openair2/LAYER2/NR_MAC_gNB"

if [ ! -d "$TARGET" ]; then
  echo "OAI NR MAC scheduler directory not found: $TARGET" >&2
  exit 1
fi

cp "$HOOK_DIR/mobile_ros_oai_hook.c" "$TARGET/mobile_ros_oai_hook.c"
cp "$HOOK_DIR/mobile_ros_oai_hook.h" "$TARGET/mobile_ros_oai_hook.h"

cat <<'MSG'
Hook files copied.

Integrate the hook by including mobile_ros_oai_hook.h in the NR gNB scheduler
translation unit that observes UE scheduling state, then call:

  mobile_ros_oai_publish_metric_json(json_string);

or call the typed helper in mobile_ros_oai_hook.h after computing UE metrics.
Build OAI normally after adding the new source file to the relevant CMake
target.
MSG
