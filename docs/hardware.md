# Hardware — Datasheet Facts & Bring-Up

The numbers that drive every sizing decision in the RTL, plus the
power-on bring-up procedure. Operating-point parameters as they appear in
the code are in [`docs/config.md`](config.md); the control math is in
[`docs/foc.md`](foc.md).

Stack: **Arty S7-50** (XC7A50T-1FGG676C) + **DRV8316REVM** (24 V) +
**Moons ECU16052H24-S002** (3-phase BLDC, hall feedback).

---

## 1. Motor — Moons ECU16052H24-S002

| Fact | Value | Consequence |
|---|---|---|
| Pole pairs | **1** | θ_elec = θ_mech; halls are absolute over the full mechanical rev. **But** hall placement error maps 1:1 into electrical angle → calibrate a **per-edge angle table** (12 entries, direction-dependent), not a single offset. |
| Inductance | 0.253 mH **line-to-line** (per-phase L_s = 127 µH) | Very low. At 24 V / 80 kHz / D = 0.5 the two-phase conduction loop (2·L_s = 254 µH) gives **~0.30 A p-p ripple ≈ 1.4× rated current.** Mitigate with bench-supply current limit, low-modulation early tests, and OCP headroom budgeting. |
| Resistance | 3.16 Ω ±10% **line-to-line** (per-phase R_s = 1.58 Ω) | τ_e = L_s/R_s ≈ 80 µs. With T_s = 12.5 µs and one period transport delay, delay ≈ τ_e/6 — include in PI tuning. The dq control math uses the **per-phase** values (`docs/foc.md` §5). |
| Torque constant | 14.85 mNm/A | Telemetry sanity scaling. |
| Rated current | **0.22 A** | Tiny vs. driver capability. Drives every CSA/ADC/OCP decision. With ±0.15 A ripple peak, worst-case instantaneous current at rated operation ≈ 0.37 A — comfortably under the 0.9 A trip. |
| Stall current | 7.6 A | Upper-bound sanity only. |
| Speed constant | 643 rpm/V | 24 V ≈ 15.4 krpm no-load ≈ 257 Hz electrical — ample margin; hall edge rate ≈ 1.5 kHz worst case, trivial for the estimator. |
| Encoder | none | **Verify the 3 halls are physically present and wired before anything else.** |

---

## 2. Driver — DRV8316 / DRV8316REVM

- CSAs are **low-side**: SOx = VREF/2 ± Gain·I, valid only while the
  low-side FET conducts → sample at the **PWM counter peak**.
- **Internal OCP levels (~16/24 A) protect the driver, not this motor.**
  The RTL is the motor's real overcurrent protection and **must trip
  inside the measured full-scale range**.
- Sensing chain: CSA gain **1.2 V/A** (max) → full scale ≈ **±1.25 A** →
  RTL hard trip **~0.9 A**, optional slow I²t limit near 0.3 A continuous.
- SOx is mid-biased at AVDD/2 (~1.65 V). The front-end divider must
  **shift common mode** into the XADC bipolar window, not merely
  attenuate. Divider draw ~100 µA; RC cutoff ≈ 100× f_sw is TI's FOC
  guidance — recompute for 80 kHz.
- DRV8316 minimum dead-time spec sets the floor for `DEADTIME_NS`; size
  above it.
- **EVM trap:** the REVM has an onboard MCU driving PWM/SPI for TI's GUI.
  Identify and set the jumpers/headers that isolate it and hand PWM + SPI
  + nFAULT to the external connector. This is **step 0** of bring-up.

---

## 3. XADC (Spartan-7)

- Dual simultaneous-S/H mode samples **fixed pairs VAUX[i] / VAUX[i+8]** —
  this is why dynamic phase selection is deferred. Wire phase A and B SOx
  to one valid pair.
- 12-bit nominal; expect ~10.5 ENOB in a switching environment → ~2–4 mA
  effective resolution at ±1.25 A FS, i.e. ~1–2 % of rated current.
  Acceptable, and the reason CSA gain is maxed.
- Instantiate the raw `XADC` primitive with INIT_xx attributes (no
  wizard). In simulation use the **UNISIM XADC model** (`xelab
  -L unisims_ver` + `glbl.v`; analog stimulus via the `SIM_MONITOR_FILE`
  text file).

---

## 4. Pin / XDC notes

Board-level pins follow the Digilent master XDC — **verify against the
real wiring before first programming**:

- Clock R2 (SSTL135), reset C18, UART V12/R12, PMOD JA/JB convention.
- Analog pins VAUX1 = B15/A15, VAUX9 = E12/D12 are authoritative from the
  part database; confirm they are exposed on the board's analog header
  (A1/A2, outer row, with the on-board 0–3.3 V → 0–1 V divider).
- Pull-ups on halls/nFAULT; pull-downs on the 6 gate lines (FPGA is Hi-Z
  pre-config); DRVOFF asserted in parallel with the gate output-enable.

---

## 5. Bring-up procedure (24 V throughout; each step gates the next)

The bench supply's **hard current limit (~0.3 A)** is the primary energy
bound for every step — not a reduced bus voltage.

0. **Isolate the EVM MCU.** Set the REVM jumpers so PWM + SPI + nFAULT
   come from the external connector.
1. **Analog front-end.** Build the divider/RC (common-mode shift +
   ±0.5 V scaling + Vbus divider). Measure SOx DC level at zero current on
   a scope before connecting to VAUX.
2. **Power + SPI.** 24 V rail, hard current-limited. Run SPI config +
   readback; confirm nFAULT high.
3. **Offset cal** with gates enabled at 50/50/50 duty (near-zero average
   current, realistic switching common-mode), ≥64-sample average.
4. **Per-edge hall calibration.** Open-loop low-current vector swept
   slowly through 360° both directions; record commanded θ at each of the
   12 transitions; load the edge table over UART (`hall <idx> <ang>`).
5. **Open-loop V/f spin** at low modulation: validate dead-time on scope,
   `cnt_peak` sampling, current reconstruction (ia+ib+ic ≈ 0 residual in
   telemetry); confirm measured ripple ≈ 0.30 A p-p.
6. **Close the loop.** Small iq_ref step; confirm id → 0, iq tracks; watch
   nFAULT and ocp_trip counters; verify instantaneous peaks stay within
   OCP margin.
7. **Tune & ramp.** Only after step 6 is stable: tune gains at 24 V;
   progressively raise the bench-supply current limit toward rated
   operation, re-checking ripple and OCP margins at each step.
