#!/usr/bin/env bash
set -euo pipefail

PREFIX="${1:-$PWD/third_party/src}"
REPO="$PREFIX/ORB_SLAM3"
mkdir -p "$PREFIX"

if [ ! -d "$REPO/.git" ]; then
  git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$REPO"
fi

cd "$REPO"
git checkout 4452a3c6e5b90e5544bf6e775e2e70a3daabbfd7
chmod +x build.sh
./build.sh

echo "ORB_SLAM3_ROOT=$REPO"
