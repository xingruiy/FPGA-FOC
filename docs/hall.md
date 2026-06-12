# Hall Sensing and Angle Estimation

Theory, math and implementation of `rtl/hall/hall_decode.sv` and
`rtl/hall/hall_angle_est.sv` — from three raw Hall inputs to a continuous
electrical angle θ and speed ω for the Park transforms.

---

## 1. Hall sensor basics

Three Hall-effect switches placed 120° (electrical) apart each output the
sign of the rotor flux, giving a 3-bit code `{C,B,A}` that steps through
six legal states per electrical revolution — one transition every 60°.
The codes `000` and `111` never occur on a healthy motor and indicate a
broken wire or unpowered sensor.

This motor (Moons ECU16052H24-S002) has **one pole pair**, so electrical
and mechanical angle coincide and the six Hall edges are *absolute*
position references over the full revolution — no index search or initial
alignment is needed.

Sector mapping used throughout (forward rotation walks 0→1→…→5→0):

| code `{C,B,A}` | 001 | 011 | 010 | 110 | 100 | 101 | 000/111 |
|---|---|---|---|---|---|---|---|
| sector | 0 | 1 | 2 | 3 | 4 | 5 | illegal |

This is the standard Gray-like 6-step sequence: exactly one bit changes
per legal transition, which is what makes single-bit glitch rejection and
direction detection cheap.

## 2. `hall_decode` — synchronize, debounce, decode

1. **2-FF synchronizer** per input — the Halls are asynchronous to the
   100 MHz clock (the XDC also cuts timing on them with `set_false_path`).
2. **Debounce**: a new code must be stable for `DEBOUNCE_CYC` (16) clocks
   before being accepted. 160 ns is far below the minimum edge spacing
   (≈ 650 µs at the 15.4 krpm no-load ceiling) but absorbs the 1-bit
   skew inherent to independently-synchronized inputs: when two FFs
   resolve a real edge on different clocks, the intermediate code lasts
   one clock and is rejected.
3. **Sector decode** per the table above; `illegal` is a level flag while
   the debounced code is 000/111, and an illegal code never updates
   `sector`.
4. **Direction**: on an accepted change, if the new sector is the
   successor of the old one `dir <= 1` (forward), if the predecessor
   `dir <= 0`. A multi-step jump (only possible if edges were missed)
   keeps the previous direction rather than guessing.
5. `edge_strobe` pulses for one clock per accepted legal sector change —
   the only event the estimator listens to.

## 3. Why a 12-entry calibrated edge table

With $N_p = 1$, Hall **placement error maps 1:1 into electrical angle**.
A ±5° mechanical placement tolerance would be a ±5° electrical angle
error — a 0.4 % torque loss is not the issue; the d/q cross-coupling it
creates is. A single global offset cannot fix it because each of the six
sensor edges has its *own* error.

Additionally the *physical* switching point of a Hall sensor differs
between approach directions (magnetic hysteresis), so the angle at which
the system *enters* sector s moving forward is not the angle at which it
enters s moving in reverse.

Hence: **12 calibration entries** = 6 forward-entering + 6
reverse-entering edge angles, loadable at runtime over UART (command
0x05) and defaulting to the ideal 60° grid:

| index | meaning | identity default (angle codes) |
|---|---|---|
| s (0…5) | angle of the edge *entering* sector s, forward | 0, 10923, 21845, 32768, 43691, 54613 |
| 6+s | angle of the edge *entering* sector s, reverse | 10923, 21845, 32768, 43691, 54613, 0 |

(the reverse entry for sector s is its *upper* boundary — entering from
above). Angles are `angle_t`: unsigned 16 bit, full circle = $2^{16}$
codes, so all subtractions below wrap correctly for free.

The calibration procedure (Phase 6.4) drives the motor open-loop with a
slow low-current rotating vector, records the commanded θ at each of the
12 transitions, and writes them back over UART.

## 4. `hall_angle_est` — speed and interpolation

Between edges the angle is advanced by dead reckoning at the last
measured speed.

### 4.1 Inter-edge speed

At each accepted edge in a consistent direction, the angle distance
between this edge and the previous one (`traveled`, from the table) is
divided by the elapsed time `t_cnt` (clocks):

$$inc = \frac{traveled \ll 16}{t\_cnt} \quad \text{[angle codes per clock, Q16]}$$

The division runs on a **serial restoring divider, one bit per clock,
32 clocks total** — invisible next to the ≥ 1000-clock minimum edge
spacing, and it keeps a 32-bit divider out of the timing graph. The
result is capped at `INC_MAX` (16 codes/clk ≈ 6× the motor's physical
speed ceiling) as a sanity bound.

The rate is *not* computed (and the previous rate is discarded) when the
edge is the first one, follows a direction reversal, or the traveled
distance is implausible (> 90°, i.e. missed edges) — one fresh
same-direction interval must be measured before interpolation resumes.

### 4.2 Interpolation with overshoot guard

Within a sector:

$$\theta = \theta_{edge} \pm (acc \gg 16), \qquad acc \mathrel{+}= inc \ \text{per clock}$$

with the **guard**: `acc` is clamped at `dist_q16`, the table distance
from the current edge to the *next expected* edge. Therefore θ
asymptotically approaches — but never crosses — the next edge angle
before that edge actually arrives.

The guard is what makes deceleration safe: dead reckoning at a stale
(higher) speed would otherwise run θ past the commutation boundary,
injecting an angle error that the Park transform turns directly into a
torque error in the wrong direction. The worst case inside one sector is
then bounded by the sector width itself, and the error resets to zero at
every edge.

### 4.3 Standstill, reversal, cold start

- **Standstill**: no edge for `TIMEOUT_CYC` (≈ 42 ms — about 4 Hz
  electrical, well below any useful speed) ⇒ `moving = 0`, ω = 0, θ
  freezes at its current value.
- **Reversal**: resets the rate (see 4.1); θ snaps to the new entering
  edge angle and holds until a fresh interval is measured.
- **Before the first edge**: θ outputs the *center* of the current Hall
  sector — the minimax choice, bounding the initial error to ±30°. After
  reset (no sector knowledge at all) the reset value is sector 0's
  center.

### 4.4 Speed output

$$\omega = \pm\frac{inc \cdot PERIOD\_CYC}{2^{16}}
        \quad \text{[angle codes per PWM period, signed]}$$

with `PERIOD_CYC` $= 2 \cdot PWM\_ARR = 1250$. This unit is chosen
because every consumer runs at the PWM rate. Conversion for the host:

$$\omega_{elec}\,[\text{rad/s}] = \omega_{codes} \cdot \frac{2\pi}{2^{16}} \cdot F_{sw},
\qquad \text{rpm} = \omega_{codes} \cdot \frac{60 \cdot F_{sw}}{2^{16}} \ (N_p = 1)$$

Both θ and ω outputs are **registered** — one clock of staleness (10 ns)
is irrelevant against a 12.5 µs control period, and it keeps the
interpolation adders/multiplier out of every downstream timing path.

## 5. Latency / error budget

| Source | Bound |
|---|---|
| synchronizer + debounce | 18 clk = 180 ns ≈ 0.03° at max speed |
| speed quantization | 1/t_cnt relative; < 0.1 % above 100 rpm |
| intra-sector dead reckoning | exact at constant ω; ≤ sector width during accel, guard-bounded during decel |
| placement / hysteresis | removed by the 12-entry table after calibration |

## 6. Verification

`sim/tb_hall_decode.sv`: spun-rotor stimulus in both directions, glitch
injection shorter than the debounce window, illegal-state flagging,
direction flips, multi-step jump tolerance.

`sim/tb_hall_angle_est.sv`: constant-speed θ tracking error bound,
acceleration/deceleration with the guard property checked every clock
(θ never crosses the pending edge angle), standstill timeout, direction
reversal re-lock, and a skewed (non-identity) calibration table loaded at
runtime — including that tracking resumes against the *new* table.
