#!/usr/bin/env bash
# The maintained student deliverable is the standalone HTML reader.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
python3 scripts/build-teaching-reader.py --out docs/ARC_Laboratories.html
