# ============================================================================
# program.tcl - program the Arty S7-50 over the Digilent USB-JTAG.
#
#   vivado -mode batch -source tcl/program.tcl
# ============================================================================

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT [file normalize $SCRIPT_DIR/..]
set BIT  $ROOT/build/impl/foc_top.bit

if {![file exists $BIT]} {
    error "bitstream not found: $BIT - run tcl/build.tcl first"
}

open_hw_manager
connect_hw_server
open_hw_target
set dev [lindex [get_hw_devices xc7s50*] 0]
current_hw_device $dev
set_property PROGRAM.FILE $BIT $dev
program_hw_devices $dev
puts "=== programmed $BIT ==="
close_hw_manager
