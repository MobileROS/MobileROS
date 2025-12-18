#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

error=0

# Detect legacy wireless_ros imports outside the shim
legacy_imports=$(rg -n "\b(from|import)\s+wireless_ros\b" --glob '!wireless_ros/__init__.py' --glob '!ci-scripts/check_legacy_namespace.sh' || true)
if [[ -n "$legacy_imports" ]]; then
  echo "ERROR: Legacy wireless_ros imports found (only wireless_ros/__init__.py is allowed):"
  echo "$legacy_imports"
  error=1
fi

# Detect deprecated legacy class name
legacy_symbol_pattern="Wireless""ROS"
legacy_symbols=$(rg -n --fixed-strings "$legacy_symbol_pattern" --glob '!ci-scripts/check_legacy_namespace.sh' || true)
if [[ -n "$legacy_symbols" ]]; then
  echo "ERROR: Deprecated legacy namespace token found (${legacy_symbol_pattern}):"
  echo "$legacy_symbols"
  error=1
fi

if [[ $error -ne 0 ]]; then
  exit 1
fi

echo "Legacy namespace checks passed."
