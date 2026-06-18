#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 tools/mobileros_cli.py \
  --metrics benchmarks/oai_rfsim_metrics.jsonl \
  --case all \
  --frames 30 \
  --output benchmarks/results/cli_cases_report.json
