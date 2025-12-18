#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOC_PATH="$ROOT_DIR/docs/Network_Configuration_Guide.md"

usage() {
  cat <<USAGE
Usage: $0 [--docker | --native]

--docker   Build the MobileROS container image and start the default stack via docker compose.
--native   Install native build dependencies and compile with ENABLE_OAI=OFF (hardware-agnostic).

Refer to $DOC_PATH for OAI-specific kernel/driver guidance.
USAGE
}

ensure_dep() {
  local dep="$1"
  local hint="$2"
  if ! command -v "$dep" >/dev/null 2>&1; then
    echo "[ERROR] Missing dependency: $dep" >&2
    echo "Install suggestion: $hint" >&2
    echo "For more details see $DOC_PATH" >&2
    exit 1
  fi
}

MODE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker)
      MODE="docker"
      ;;
    --native)
      MODE="native"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if [[ -z "$MODE" ]]; then
  usage
  exit 1
fi

if [[ "$MODE" == "docker" ]]; then
  ensure_dep docker "sudo apt-get install -y docker.io"
  if ! docker compose version >/dev/null 2>&1; then
    echo "[ERROR] Docker Compose plugin is missing." >&2
    echo "Install suggestion: sudo apt-get install -y docker-compose-plugin" >&2
    echo "For more details see $DOC_PATH" >&2
    exit 1
  fi
  echo "[INFO] Building MobileROS container image..."
  docker compose -f "$ROOT_DIR/docker/docker-compose.yml" build
  echo "[INFO] Starting MobileROS stack (CTRL+C to stop)"
  docker compose -f "$ROOT_DIR/docker/docker-compose.yml" up
  exit 0
fi

# Native path
ensure_dep cmake "sudo apt-get install -y cmake"
ensure_dep git "sudo apt-get install -y git"
ensure_dep python3 "sudo apt-get install -y python3"
ensure_dep pip3 "sudo apt-get install -y python3-pip"
ensure_dep make "sudo apt-get install -y build-essential"

BUILD_DIR="$ROOT_DIR/build"
mkdir -p "$BUILD_DIR"
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DENABLE_OAI=OFF
cmake --build "$BUILD_DIR"

if command -v python3 >/dev/null 2>&1 && [[ -f "$ROOT_DIR/requirements.txt" ]]; then
  pip3 install -r "$ROOT_DIR/requirements.txt"
fi

echo "[INFO] Native build complete. Run '$BUILD_DIR/mobileros_example' to verify." 
