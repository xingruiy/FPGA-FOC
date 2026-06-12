#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# simulate.sh - compile + run one SystemVerilog testbench with Vivado xsim.
#
# Non-project flow: xvlog (compile) -> xelab (elaborate) -> xsim (run).
# foc_pkg.sv is always compiled first; everything else under rtl/ and sim/
# is globbed. UNISIM library (and glbl, when available) is linked so TBs may
# instantiate raw Xilinx primitives such as XADC.
#
# Every testbench is self-checking and must print exactly one banner:
#     TB_PASS: <name>      on success
#     TB_FAIL: ...         on failure
# The script's exit code reflects that banner (plus any xsim Error/Fatal).
#
# Usage:
#     scripts/simulate.sh [TOP] [--gui]
#
#     TOP     top-level testbench module (default: tb_foc_pkg)
#     --gui   open the xsim GUI instead of a batch --runall
#
# Artifacts land in build/sim/<TOP>/ so the source tree stays clean.
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# ---- arguments ------------------------------------------------------------
TOP="tb_foc_pkg"
GUI=0
for arg in "$@"; do
  case "$arg" in
    --gui) GUI=1 ;;
    -h|--help)
      sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    -*) echo "simulate.sh: unknown option '$arg'" >&2; exit 2 ;;
    *)  TOP="$arg" ;;
  esac
done

# ---- toolchain ------------------------------------------------------------
for tool in xvlog xelab xsim; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "simulate.sh: '$tool' not found in PATH." >&2
    echo "  Source the Vivado settings first, e.g.:" >&2
    echo "    source /home/xingrui/amd/2025.2/Vivado/settings64.sh" >&2
    exit 1
  fi
done
VIVADO_ROOT="$(cd -- "$(dirname -- "$(command -v xvlog)")/.." && pwd)"

# ---- sources: foc_pkg.sv first, then rtl/ + sim/ --------------------------
PKG="$ROOT/rtl/foc/foc_pkg.sv"
SV_FILES=()
[ -f "$PKG" ] && SV_FILES+=("$PKG")
while IFS= read -r f; do
  [ "$f" = "$PKG" ] && continue
  SV_FILES+=("$f")
done < <(find "$ROOT/rtl" "$ROOT/sim" -name '*.sv' 2>/dev/null | sort)
if [ "${#SV_FILES[@]}" -eq 0 ]; then
  echo "simulate.sh: no .sv sources found under rtl/ or sim/" >&2
  exit 1
fi

# ---- build directory ------------------------------------------------------
BUILD="$ROOT/build/sim/$TOP"
rm -rf "$BUILD"
mkdir -p "$BUILD"
# $readmemh / SIM_MONITOR_FILE paths are relative to the xsim working dir:
# stage .mem tables and .txt analog stimulus files there
find "$ROOT/rtl" -name '*.mem' -exec cp {} "$BUILD/" \; 2>/dev/null || true
find "$ROOT/sim" -name '*.txt' -exec cp {} "$BUILD/" \; 2>/dev/null || true
cd "$BUILD"

echo "=== sources ==="
printf '  %s\n' "${SV_FILES[@]}"
echo "=== top     : $TOP"
echo "=== workdir : $BUILD"
echo

# 1) compile
echo ">>> xvlog"
xvlog --sv "${SV_FILES[@]}"

# glbl (needed for UNISIM primitives like XADC); compile when available
ELAB_TOPS=("work.$TOP")
GLBL_SRC="$VIVADO_ROOT/data/verilog/src/glbl.v"
if [ -f "$GLBL_SRC" ]; then
  xvlog "$GLBL_SRC"
  ELAB_TOPS+=("work.glbl")
fi

# 2) elaborate
echo ">>> xelab"
xelab "${ELAB_TOPS[@]}" -s "${TOP}_snap" --debug typical \
      --timescale 1ns/1ps -L unisims_ver -L unimacro_ver -L secureip

# 3) run
if [ "$GUI" -eq 1 ]; then
  echo ">>> xsim (gui)"
  exec xsim "${TOP}_snap" --gui
fi

echo ">>> xsim (batch)"
xsim "${TOP}_snap" --runall | tee run.out

# ---- verdict ---------------------------------------------------------------
if grep -q "TB_PASS" run.out && ! grep -qE "TB_FAIL|Error:|Fatal:" run.out; then
  echo "=== RESULT: PASS ($TOP)"
  exit 0
else
  echo "=== RESULT: FAIL ($TOP)" >&2
  exit 1
fi
