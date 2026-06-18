#!/usr/bin/env bash
set -euo pipefail

PREFIX="${1:-$PWD/third_party/src}"
REPO="$PREFIX/OpenPCDet"
mkdir -p "$PREFIX"

if [ ! -d "$REPO/.git" ]; then
  git clone https://github.com/open-mmlab/OpenPCDet.git "$REPO"
fi

cd "$REPO"
git checkout 233f849d721105e89e42d3c0d4be656dcde0fde9
python -m pip install -r requirements.txt
python setup.py develop

echo "OPENPCDET_ROOT=$REPO"
