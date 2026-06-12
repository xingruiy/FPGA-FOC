# ============================================================================
# build.tcl - Vivado non-project flow for foc_top (Arty S7-50).
#
#   vivado -mode batch -source tcl/build.tcl
#
# Reads foc_pkg.sv first, then globs rtl/*/*.sv (no create_ip phase - the
# design instantiates only BUFG/IBUF/XADC primitives directly). Artifacts
# land in build/impl/.
# ============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT   [file normalize $SCRIPT_DIR/..]
set PART   xc7s50csga324-1
set TOP    foc_top
set OUTDIR $ROOT/build/impl

file mkdir $OUTDIR

# $readmemh (sincos_lut.mem) resolves relative to the launch directory
file copy -force $ROOT/rtl/math/sincos_lut.mem [file join [pwd] sincos_lut.mem]

# ---- sources: package first, then one module per file under rtl/ ----------
read_verilog -sv $ROOT/rtl/foc/foc_pkg.sv
foreach f [lsort [glob -nocomplain $ROOT/rtl/*/*.sv]] {
    if {[file tail $f] eq "foc_pkg.sv"} { continue }
    read_verilog -sv $f
}
read_xdc $ROOT/xdc/arty_s7.xdc

# ---- synthesis --------------------------------------------------------------
synth_design -top $TOP -part $PART
write_checkpoint   -force $OUTDIR/post_synth.dcp
report_utilization -file  $OUTDIR/util_synth.rpt

# ---- implementation ----------------------------------------------------------
opt_design
place_design
route_design
write_checkpoint -force $OUTDIR/post_route.dcp

# ---- sign-off reports ---------------------------------------------------------
report_timing_summary -file $OUTDIR/timing.rpt
report_utilization    -file $OUTDIR/util_route.rpt
report_drc            -file $OUTDIR/drc.rpt

# ---- bitstream -----------------------------------------------------------------
write_bitstream -force $OUTDIR/$TOP.bit

puts "=== DONE -> $OUTDIR/$TOP.bit ==="
