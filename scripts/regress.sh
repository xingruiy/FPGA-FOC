#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# regress.sh - run every testbench under sim/ through scripts/simulate.sh
# and print a one-line verdict per TB. Exit code 0 iff all pass.
#
# Usage: scripts/regress.sh
# ---------------------------------------------------------------------------
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

fails=0
total=0
for f in "$ROOT"/sim/tb_*.sv; do
  tb="$(basename "$f" .sv)"
  total=$((total + 1))
  if "$SCRIPT_DIR/simulate.sh" "$tb" > /dev/null 2>&1; then
    printf 'PASS  %s\n' "$tb"
  else
    printf 'FAIL  %s\n' "$tb"
    fails=$((fails + 1))
  fi
done

echo "=== $((total - fails))/$total passed"
[ "$fails" -eq 0 ]
