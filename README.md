# FOC current loop for Arty S7-50 + DRV8316REVM

SystemVerilog FOC **current/torque inner loop** for a 3-phase BLDC
(Moons ECU16052H24-S002, hall feedback, 24 V bus). Zero Xilinx IP — plain
SV + raw `XADC`/`BUFG` primitives.

## Hardware

| | | |
|---|---|---|
|![alt text](./.github/artys7.png)|![alt text](./.github/gatedrive.png)|![alt text](./.github/motor.png)|

Tested on:
- Arty S7-50 (Xilinx XC7A50T-1FGG676C)
- DRV8316REVM (24 V, 100 kHz PWM, 24 V, 100 kHz Hall)
- Moons ECU16052H24-S002

## Layout

```
rtl/foc/    foc_pkg, clarke, park, inv_park, pi_controller, svpwm,
            foc_core, foc_top, clk_rst_gen
rtl/hall/   hall_decode, hall_angle_est (12-entry calibrated edge table)
rtl/pwm/    pwm_gen (center-aligned, dead-time, cnt_peak ADC trigger)
rtl/spi/    drv8316_spi (config + readback-verify + fault poll)
rtl/math/   sincos_lut (+ sincos_lut.mem from scripts/gen_sincos_lut.py)
rtl/adc/    xadc_iface (raw XADC, dual S/H), current_offset_cal
rtl/uart/   uart_rx, uart_tx, cmd_telemetry (framed protocol + watchdog)
sim/        one self-checking tb_<module>.sv per module + bldc_plant
scripts/    simulate.sh (xsim), regress.sh, gen_sincos_lut.py
tcl/        build.tcl (non-project synth->bitstream), program.tcl
xdc/        arty_s7.xdc
```

## Simulate (Vivado xsim only)

```sh
source ~/amd/2025.2/Vivado/settings64.sh   # if xvlog is not in PATH
scripts/simulate.sh tb_foc_top             # one TB (--gui for waves)
scripts/regress.sh                         # all 20 TBs
```

## Build / program

```sh
vivado -mode batch -source tcl/build.tcl     # -> build/impl/foc_top.bit
vivado -mode batch -source tcl/program.tcl
```

## Host UART protocol (115200 8N1)

Host → FPGA: ASCII lines terminated by CR, LF or CR+LF; decimal integer
arguments, optional leading `-`. Keywords are case-insensitive but must be
spelled in full (there is no checksum, so partial matches are rejected
with `?`). Every accepted command echoes `OK` and kicks the 100 ms
watchdog (silence ⇒ iq_ref ramps to 0, gates off). ESC stops telemetry.

| Command            | Effect                                        |
|--------------------|-----------------------------------------------|
| `enable [0\|1]`    | enable/disable drive (bare = enable)          |
| `disable`          | alias for `enable 0`                          |
| `iq <int16>`       | iq_ref, Q1.15 raw (1.0 = 32767 = 1.25 A)      |
| `kp <uint16>`      | proportional gain, Q4.12                      |
| `ki <uint16>`      | integral gain, Q4.12 (Ts folded in)           |
| `cal`              | offset calibration (only while disabled)      |
| `ping`             | watchdog kick only                            |
| `tele`             | start telemetry streaming                     |
| `ol <0\|1>`        | open-loop mode                                |
| `vq <int16>`       | open-loop Vq, Q1.15                           |
| `speed <int16>`    | open-loop speed, angle codes/period           |
| `hall <idx> <ang>` | hall edge table write: idx 0–11, angle 0–65535|

FPGA → host every 100 ms, ASCII line (48 bytes):
```
id=XXXX iq=XXXX th=XXXX om=XXXX f=XX s=XX e=XX\r\n
```
All values are raw hex: Q1.15 two's-complement for `id`/`iq` (1.0 = 7FFF = 1.25 A),
unsigned 16-bit for `th`, signed 16-bit two's-complement for `om`, 8-bit for `f`/`s`/`e`.
Readable directly in PuTTY or any terminal at 115200 8N1.

`s` (status_flags) bits `[7:0]`: `{cfg_done, cfg_err, ocp_trip, wd_timeout, enable, cal_busy, sat_any, nfault}`.
`e` (err_flags) bits `[2:0]`: `{uart_frame_err_sticky, hall_illegal_sticky, hall_illegal_live}`
(sticky bits clear on reset; a live illegal hall code also kills the gates
in closed loop).

## Operating point (locked — see docs/plan.md)

24 V bus (no 12 V phase), f_sw 80 kHz, MAX_MOD 0.87, current full scale
±1.25 A (CSA gain 1.2 V/A), OCP trip 0.9 A, single 100 MHz clock.
Default gains Kp = 850 (Q4.12), Ki = 130, ≈1.5 kHz bandwidth.

## XDC note

Board-level pins follow the Digilent master XDC from memory —
verify clock (R2/SSTL135), reset (C18), UART (V12/R12) and the PMOD
JA/JB convention against the real wiring before first programming.
Analog pins (VAUX1 = B15/A15, VAUX9 = E12/D12) are authoritative from
the part database; confirm they are exposed on the board's analog header
(A1/A2, outer row, with the on-board 0–3.3 V → 0–1 V divider).

## Docs

+ [PWM Generation](docs/pwm.md), 
+ [Hall Decoding](docs/hall.md)
+ [FOC Control](docs/foc.md)
