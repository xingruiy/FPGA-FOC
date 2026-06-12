# FOC Motor Controller — Step-by-Step Plan
### Arty S7-50 + DRV8316REVM + Moons ECU16052H24-S002 (Hall feedback)

**Goal:** SystemVerilog FOC **current/torque inner loop** for a 3-phase BLDC.
iq commanded over UART, current sensing via XADC on the DRV8316 low-side CSAs,
angle from 3 Hall sensors. One module per file. **Zero Xilinx IP** — plain SV,
fully simulable with **Vivado xsim** (non-project `xvlog → xelab → xsim` flow).

---

## Non-Goals (out of scope for this project)

- Speed loop, position loop, trajectory control — anything outside the current loop.
- Sensorless / observer-based angle estimation. Halls only.
- Dynamic (per-period) selection of which two phases to sample. v1 samples **A and B, always**; C is reconstructed. The 6-aux-input wiring that enables dynamic pair selection is a documented upgrade path, not v1 work.
- Overmodulation / six-step / field weakening. MAX_MOD stays conservative.
- **12 V operation.** The motor is powered at **24 V from the first power-up**; safety during bring-up comes from the bench supply's hard current limit, low-modulation early tests, and the RTL OCP — not from a reduced bus voltage.
- **Verilator.** xsim is the one and only simulator; SVA assertions are used freely.
- Multiplier sharing / area optimization. v1 uses dedicated DSPs per transform.
- Xilinx IP cores (clk_wiz, CORDIC IP, xadc_wiz). Replaced by BUFG, a BRAM sin/cos LUT, and the raw `XADC` primitive.
- Full mechanical + Hall-emitting plant model. Integration sim uses an electrical RL + back-EMF plant with an idealized angle source.

---

## Datasheet Facts & Reminders

### Motor — Moons ECU16052H24-S002
| Fact | Value | Consequence |
|---|---|---|
| Pole pairs | **1** | θ_elec = θ_mech; Halls are absolute over the full mechanical rev. **But**: Hall placement error maps 1:1 into electrical angle → calibrate a **per-edge angle table** (12 entries, direction-dependent), not a single offset. |
| Inductance | 0.253 mH | Very low. At 24 V / 80 kHz / D=0.5: **~0.30 A p-p ripple ≈ 1.4× rated current.** Mitigate with bench-supply current limit, low-modulation early tests, and OCP headroom budgeting (see operating point). |
| Resistance | 3.16 Ω ±10% | τ_e = L/R ≈ 80 µs. With Ts = 12.5 µs and one period transport delay, delay ≈ τ_e/6 — include in PI tuning. |
| Torque constant | 14.85 mNm/A | Telemetry sanity scaling. |
| Rated current | **0.22 A** | Tiny vs. driver capability. Drives every CSA/ADC/OCP decision below. With ±0.15 A ripple peak on top, worst-case instantaneous current at rated operation ≈ 0.37 A — comfortably under the 0.9 A trip. |
| Stall current | 7.6 A | Upper bound sanity only. |
| Speed constant | 643 rpm/V | 24 V ≈ 15.4 krpm no-load ≈ 257 Hz electrical — ample margin for current-loop work; Hall edge rate ≈ 1.5 kHz worst case, trivial for the estimator. |
| Encoder | none listed | **Verify the 3 Halls are physically present and wired before anything else.** |

### Driver — DRV8316 / DRV8316REVM
- CSAs are **low-side**: SOx = VREF/2 ± Gain·I, valid only while the low-side FET conducts → sample at the **PWM counter peak**.
- **Internal OCP levels (~16/24 A) protect the driver, not this motor.** RTL is the motor's real overcurrent protection and **must trip inside the measured full-scale range**.
- Chosen sensing chain: CSA gain **1.2 V/A** (max) → full-scale ≈ **±1.25 A** → RTL hard trip **~0.9 A**, optional slow I²t limit near 0.3 A continuous.
- SOx is mid-biased at AVDD/2 (~1.65 V). The front-end divider must **shift common mode** into the XADC bipolar window's allowed range, not merely attenuate. Divider draw ~100 µA; RC cutoff ≈ 100× f_sw is TI's FOC guidance — recompute for 80 kHz.
- DRV8316 minimum dead-time spec sets the floor for `DEADTIME_NS`; size above it.
- **EVM trap:** the REVM has an onboard MCU driving PWM/SPI for TI's GUI. Identify and set the jumpers/headers that isolate it and hand PWM + SPI + nFAULT to the external connector. This is **step 0** of hardware bring-up.

### XADC (Spartan-7)
- Dual simultaneous-S/H mode samples **fixed pairs VAUX[i] / VAUX[i+8]** — this is why dynamic phase selection is deferred. Wire phase A and B SOx to one valid pair.
- 12-bit nominal; expect ~10.5 ENOB in a switching environment → ~2–4 mA effective resolution at ±1.25 A FS, i.e. ~1–2% of rated current. Acceptable, and the reason CSA gain is maxed.
- Instantiate the raw `XADC` primitive with INIT_xx attributes (no wizard). In simulation, use the **UNISIM XADC model** (`xelab -L unisims_ver` plus `glbl.v`; analog stimulus via the `SIM_MONITOR_FILE` text file) — no hand-written DRP stub unless the unisim model proves unworkable.

### Operating-point decisions (locked)
| Param | v1 value | Rationale |
|---|---|---|
| Vbus | **24 V — always** | No 12 V phase. Fault energy bounded by bench-supply current limit during bring-up, not by bus voltage. |
| `F_SW_HZ` | **80_000** | Ripple ≈ 0.30 A p-p at 24 V — accepted: instantaneous peak at rated current ≈ 0.37 A vs 0.9 A OCP trip. ~9.3-bit duty resolution at 100 MHz center-aligned; 1250 cycles/period is ample. |
| `MAX_MOD` | **0.87** | Guarantees A/B low-side windows survive at all times (fixed-pair sampling). |
| `I_FULLSCALE_A` | ±1.25 | From CSA gain 1.2 V/A + front-end scaling. |
| `OCP_TRIP_A` | 0.9 | Inside measured range; protects the motor; clears the 0.37 A worst-case rated-operation peak with margin. |
| Clocking | single 100 MHz, IBUF→BUFG | No MMCM/PLL, no CDC. 2-FF synchronizers on Halls and nFAULT. |

### Numeric format
- θ: unsigned 16-bit, full circle = 2¹⁶. sin/cos: Q1.15 from LUT.
- I/O currents & voltages: Q1.15. Clarke/Park/inv-Park/SVPWM internals: **Q3.13** (or full-width products, saturate only at module outputs). Q1.15 throughout would overflow at Clarke (√3) and SVPWM sums.
- PI gains: dedicated Q-format (Kp can exceed 1; Ki folds in Ts) with documented post-multiply shift. UART host uses identical scaling.

---

## Design Overview

```
Halls ──► hall_decode ──► hall_angle_est ──► θ, ω        (per-edge cal table, Np=1)
                                              │
                              θ ──► sincos_lut ──► sinθ, cosθ
                                              │
XADC (raw primitive, dual S/H, phases A+B    │
      @ cnt_peak) ──► xadc_iface ──► offset_cal ──► ia, ib (ic = −ia−ib)
                                              │
                     clarke ──► park ──► id, iq
                                              │
   iq_ref (UART), id_ref = 0 ──► pi_d / pi_q ──► vd, vq   (vector-magnitude limit)
                                              │
                     inv_park ──► svpwm ──► da, db, dc    (zero-seq inject, MAX_MOD)
                                              │
                     pwm_gen ──► 6 gates ──► DRV8316 ──► motor
                                              │
                     drv8316_spi ◄── config / fault poll ─┘

foc_core   : PWM-rate scheduler (sample → math chain → duty latch, 1-period delay).
foc_top    : I/O map. Safe-state oe = enable & nFAULT_sync & ~ocp_trip & ~wd_timeout
             is COMBINATIONAL between pwm_gen and pins; DRVOFF asserted in parallel.
cmd_telemetry : UART frames in (enable, iq_ref, gains) / out (id, iq, θ, ω, vbus, faults)
                + 100 ms host watchdog → ramp iq_ref to 0.
Vbus       : one extra VAUX divider; scales SVPWM (at 24 V) and goes out in telemetry.
```

### File layout — existing repo structure, extended only where necessary

```
rtl/foc/    foc_pkg.sv  clarke.sv  park.sv  inv_park.sv  pi_controller.sv
            svpwm.sv  foc_core.sv  foc_top.sv  clk_rst_gen.sv
rtl/hall/   hall_decode.sv  hall_angle_est.sv
rtl/pwm/    pwm_gen.sv
rtl/spi/    drv8316_spi.sv
rtl/math/   sincos_lut.sv  sincos_lut.mem          (new)
rtl/adc/    xadc_iface.sv  current_offset_cal.sv   (new)
rtl/uart/   uart_rx.sv  uart_tx.sv  cmd_telemetry.sv (new)
sim/        tb_<module>.sv per step; bldc_plant.sv; tb_foc_top.sv  (flat)
scripts/    simulate.sh   gen_sincos_lut.py        (all scripts live here)
tcl/        build.tcl (non-project: read foc_pkg.sv first, glob rtl/, no create_ip)
            program.tcl
xdc/        arty_s7.xdc                            (existing name kept)
docs/       plan.md  pwm.md
build/      all generated artifacts (sim + impl), untracked
```

One module per file, filename = module name, `foc_pkg.sv` the only non-module
RTL file. No new top-level folders; new `rtl/` subfolders only as listed.

### Simulation flow (xsim only)

`scripts/simulate.sh [TOP] [--gui]` — non-project flow, written fresh:
1. `xvlog --sv` `rtl/foc/foc_pkg.sv` first, then all globbed `rtl/**/*.sv` + `sim/*.sv`.
2. `xelab work.$TOP -s ${TOP}_snap --debug typical -L unisims_ver` (+ `glbl.v` when the TB instantiates the XADC unisim).
3. `xsim --runall` in batch (artifacts under `build/sim/`), `--gui` for waves.
Every TB is self-checking and prints a single `PASS`/`FAIL` banner that the
script greps for its exit code. SVA assertions are used throughout (xsim
supports them; this is one reason Verilator was dropped).

---

## Step-by-Step Plan — minimal verifiable modules

Each step produces a module (or pair) plus a self-checking testbench in `sim/` and a documentation file in `docs/` stating the theory/math/decision/design keys
for the module. run via `scripts/simulate.sh tb_<module>`. Do not start a step until the previous step's TB passes.

### Phase 0 — Foundation
**0.0 `scripts/simulate.sh`** — the xsim runner described above (written fresh; the previous version was intentionally deleted and is reference-only in git history).
✓ Verify: runs a trivial smoke TB to PASS; nonzero exit on FAIL and on xvlog/xelab errors.

**0.1 `rtl/foc/foc_pkg.sv`** — Q-format typedefs, all locked parameters above, DRV8316 register map constants, `sat()` / `round()` / `q_mul()` helpers.
✓ Verify: helper-function unit TB (saturation corners, rounding bias, known products).

**0.2 `rtl/math/sincos_lut.sv`** — quarter-wave 16-bit sin/cos LUT, 1 BRAM, `$readmemh` table generated by checked-in `scripts/gen_sincos_lut.py`.
✓ Verify: sweep all 2¹⁶ angles in TB; max error vs. double-precision sin/cos ≤ 1 LSB Q1.15.

### Phase 1 — Core math (pure, no I/O)
**1.1 `clarke.sv`**, **1.2 `park.sv`**, **1.3 `inv_park.sv`** (all `rtl/foc/`) — Q3.13 internals.
✓ Verify: per-module golden vectors, then a combined round-trip TB: clarke→park→inv_park→inv-clarke recovers inputs within quantization bound; no saturation flags for in-range stimulus, correct saturation for √3-scale stimulus.

**1.4 `pi_controller.sv`** — generic PI, output saturation input, anti-windup from *applied* output.
✓ Verify: step response vs. bit-accurate Python model; windup test (saturate, release, no overshoot from integrator).

**1.5 `svpwm.sv`** — min/max zero-sequence injection, duties centered on 0.5, MAX_MOD cap, scaled by measured Vbus (24 V nominal).
✓ Verify: sweep vα,vβ over the hexagon: duty range, zero-seq correctness, MAX_MOD never exceeded, **min low-side window ≥ (sample aperture + settling) at every point**.

**1.6 d/q limiter** (inside `foc_core` later, but test now as a function in `foc_pkg`) — vector-magnitude clamp to Vdc/√3 with vd priority; clamped values exported for anti-windup.
✓ Verify: directed cases on and beyond the circle; vd priority preserved.

### Phase 2 — PWM and angle
**2.1 `rtl/pwm/pwm_gen.sv`** — written fresh: center-aligned complementary PWM, dead-time ≥ DRV8316 min, double-buffered duty at period boundary, `cnt_peak` strobe, per-phase output-enable.
✓ Verify: SVA assertions: complementary outputs never both high (shoot-through); dead-time width exact; duty accuracy ±1 cycle; `cnt_peak` lands at counter peak; oe forces both-off within 1 cycle.

**2.2 `rtl/hall/hall_decode.sv`** — 2-FF sync, debounce, 6-step sector, direction, illegal-state (000/111) flag.
✓ Verify: spun-rotor stimulus both directions, glitch injection, illegal-state detection.

**2.3 `rtl/hall/hall_angle_est.sv`** — inter-edge ω, intra-sector interpolation against the **12-entry calibrated edge table** (loadable over UART, identity default), standstill hold, **interpolation guard: θ never crosses the next expected edge angle before the edge arrives**.
✓ Verify: constant speed (θ error bound), acceleration/deceleration (guard prevents overshoot), standstill, direction reversal, skewed edge table.

### Phase 3 — Sensing
**3.1 `rtl/adc/xadc_iface.sv`** — raw `XADC` primitive, bipolar dual-S/H on the fixed A/B pair, conversion triggered by `cnt_peak`, plus sequenced single-channel **Vbus** read.
✓ Verify: TB instantiates the **UNISIM XADC model** with a `SIM_MONITOR_FILE` analog stimulus (xelab `-L unisims_ver` + `glbl.v`); correct trigger alignment, channel mapping, sign handling. Fall back to a thin behavioral stub only if the unisim model is unworkable.

**3.2 `rtl/adc/current_offset_cal.sv`** — offset capture averaging ≥64 samples, subtract, reconstruct ic = −(ia+ib), saturating.
✓ Verify: injected offsets removed; reconstruction exact; averaging reduces injected noise as expected.

### Phase 4 — Driver config & host link
**4.1 `rtl/spi/drv8316_spi.sv`** — SPI master + config FSM: 6× PWM mode, CSA gain = 1.2 V/A, slew, OCP register (documented as driver-protection only); periodic fault/status poll; readback-verify every config write.
✓ Verify: TB SPI slave model checks frame timing, write/readback mismatch handling, fault-poll cadence.

**4.2 `rtl/uart/uart_rx.sv` / `uart_tx.sv`** — ✓ loopback TB, baud tolerance ±2%.

**4.3 `rtl/uart/cmd_telemetry.sv`** — framed protocol with checksum: in {enable, iq_ref, Kp, Ki, edge-table load}; out {id, iq, θ, ω, vbus, fault flags}; **100 ms watchdog → iq_ref ramps to 0** while enabled.
✓ Verify: frame parse/garbage rejection, checksum, watchdog fires and ramps, telemetry field scaling round-trips against `foc_pkg` Q-formats.

### Phase 5 — Integration
**5.1 `rtl/foc/foc_core.sv`** — PWM-rate scheduler: cnt_peak sample → cal → clarke → park → PI (id_ref=0) → limiter → inv_park → svpwm → duty buffer latched at next boundary. The **one-period transport delay is explicit and documented**; default Kp/Ki tuned for τ_e ≈ 80 µs with that delay at **24 V plant gain** (target bandwidth ≈ 1–2 kHz, phase-margin checked in the Python model).
✓ Verify: open-loop mode TB (fixed vd,vq, idealized currents) — dataflow timing, no node saturates at rated operating points.

**5.2 `sim/bldc_plant.sv`** — electrical plant: R = 3.16 Ω, L = 0.253 mH, back-EMF from 643 rpm/V, **24 V bus**, driven by per-period averaged phase voltages; programmable θ(t) source.
✓ Verify: against analytic RL step response; observed ripple consistent with the predicted ~0.30 A p-p when driven switched (sanity check on the operating point).

**5.3 `sim/tb_foc_top.sv`** — closed loop, UART-injected iq_ref steps.
✓ Verify: iq tracks ref (rise time vs. design bandwidth), id → 0, no windup on saturating step, ocp_trip fires on a forced overcurrent, watchdog ramp-down on UART silence, combinational safe-state kills gates on injected nFAULT within 1 cycle.

**5.4 `rtl/foc/clk_rst_gen.sv` + `foc_top.sv` + `xdc/arty_s7.xdc` + `tcl/build.tcl`**
Repoint the existing non-project `tcl/build.tcl` at `foc_top` (read `foc_pkg.sv` first, glob `rtl/`, **no `create_ip` phase**). XDC: 100 MHz create_clock; false_path + pull-config on Halls/nFAULT (pull-up); pull-downs on 6 gate lines (FPGA pre-config Hi-Z); VAUX pair for A/B + Vbus channel; UART; SPI; DRVOFF.
✓ Verify: clean synth/impl (no critical warnings on inference), timing met, post-route resource sanity (DSP count matches dedicated-multiplier expectation).

### Phase 6 — Hardware bring-up (each step gates the next; 24 V throughout)
**6.0** Write full hardware wiring guide and testing procedure (including this phase content) under docs/hardware.md.
**6.1** Build the analog front-end per the divider/RC values written into the README (common-mode shift + ±0.5 V scaling + Vbus divider). Measure SOx DC level at zero current on a scope before connecting to VAUX.
**6.2** **24 V rail, bench supply hard current-limited (~0.3 A).** SPI config + readback; nFAULT high. The supply current limit is the primary energy bound for all bring-up steps.
**6.3** Offset cal **with gates enabled at 50/50/50 duty** (near-zero average current but realistic switching common-mode), ≥64-sample average.
**6.4** Per-edge Hall calibration: open-loop low-current vector swept slowly through 360° in both directions; record commanded θ at each of the 12 transitions; load the edge table over UART; persist values in README.
**6.5** Open-loop V/f spin at low modulation: validate dead-time on scope, cnt_peak sampling, current reconstruction under real switching (compare ia+ib+ic ≈ 0 residual in telemetry); confirm measured ripple ≈ 0.30 A p-p prediction.
**6.6** Close the loop: small iq_ref step; confirm id → 0, iq tracks; watch nFAULT and ocp_trip counters; verify instantaneous peaks stay within OCP margin.
**6.7** Only after 6.6 is stable: tune gains at 24 V; progressively raise the bench-supply current limit toward rated operation, re-checking ripple and OCP margins at each step.