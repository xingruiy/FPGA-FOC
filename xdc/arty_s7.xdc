# ============================================================================
# arty_s7.xdc - Arty S7-50 (xc7s50csga324-1) constraints for foc_top
#
# Pin provenance:
#  - Analog (VAUX) and clock-capability data queried from the Vivado part
#    database (authoritative for the package).
#  - Board-level net assignments (oscillator, reset button, USB-UART, PMOD
#    connectors) follow the Digilent Arty S7-50 master XDC; RE-VERIFY them
#    against the official Digilent master XDC for your board revision
#    before first programming.
#  - PMOD JA/JB usage below is THIS PROJECT'S wiring convention for the
#    DRV8316REVM and hall harness - match your actual wiring (Phase 6.0).
# ============================================================================

# ---- 100 MHz oscillator (bank 34, 1.35 V bank on Arty S7) ------------------
set_property -dict { PACKAGE_PIN R2 IOSTANDARD SSTL135 } [get_ports clk100]
create_clock -name sys_clk -period 10.000 [get_ports clk100]

# ---- reset button (RESET, active low) --------------------------------------
set_property -dict { PACKAGE_PIN C18 IOSTANDARD LVCMOS33 } [get_ports ck_rstn]
set_false_path -from [get_ports ck_rstn]

# ---- USB-UART bridge --------------------------------------------------------
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports uart_rx_i]
set_property -dict { PACKAGE_PIN R12 IOSTANDARD LVCMOS33 } [get_ports uart_tx_o]
set_false_path -from [get_ports uart_rx_i]
set_false_path -to   [get_ports uart_tx_o]

# ---- PMOD JA: gate signals + DRVOFF + nFAULT (wiring convention) ------------
# JA1..JA4 (top row), JA7..JA10 (bottom row)
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports gate_ah]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports gate_al]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports gate_bh]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports gate_bl]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports gate_ch]
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 } [get_ports gate_cl]
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVCMOS33 } [get_ports drv_off]
set_property -dict { PACKAGE_PIN N18 IOSTANDARD LVCMOS33 } [get_ports drv_nfault]

# gate lines: pull DOWN while the FPGA is unconfigured (pre-config Hi-Z must
# not turn any FET on); DRVOFF pulls UP so the driver stays disabled
set_property PULLDOWN true [get_ports gate_ah]
set_property PULLDOWN true [get_ports gate_al]
set_property PULLDOWN true [get_ports gate_bh]
set_property PULLDOWN true [get_ports gate_bl]
set_property PULLDOWN true [get_ports gate_ch]
set_property PULLDOWN true [get_ports gate_cl]
set_property PULLUP   true [get_ports drv_off]
# nFAULT is open-drain on the DRV8316: pull up, async into a 2-FF sync
set_property PULLUP true [get_ports drv_nfault]
set_false_path -from [get_ports drv_nfault]

# ---- PMOD JB: DRV8316 SPI (top row) + halls (bottom row) --------------------
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports drv_sclk]
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports drv_mosi]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports drv_miso]
set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports drv_csn]
set_false_path -from [get_ports drv_miso]
set_false_path -to   [get_ports {drv_sclk drv_mosi drv_csn}]

set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports {hall[0]}]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports {hall[1]}]
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports {hall[2]}]
# hall sensors are open-collector: pull up; async into 2-FF sync + debounce
set_property PULLUP true [get_ports {hall[*]}]
set_false_path -from [get_ports {hall[*]}]

# ---- XADC analog inputs (dedicated ADxP/ADxN sites, from part database) -----
# phase A shunt amp -> VAUX1 (A1, outer header, on-board 0-3.3V->0-1V divider)
# phase B shunt amp -> VAUX9 (A2, outer header, same divider; VAUX9 is the
# simultaneous-sampling partner of VAUX1); both unipolar, INIT_4D=0x0000.
set_property -dict { PACKAGE_PIN B15 IOSTANDARD LVCMOS33 } [get_ports xa_p]
set_property -dict { PACKAGE_PIN A15 IOSTANDARD LVCMOS33 } [get_ports xa_n]
set_property -dict { PACKAGE_PIN E12 IOSTANDARD LVCMOS33 } [get_ports xb_p]
set_property -dict { PACKAGE_PIN D12 IOSTANDARD LVCMOS33 } [get_ports xb_n]

# ---- bitstream options -------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
