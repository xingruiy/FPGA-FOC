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

## 4. `hall_angle_est` — the PLL observer

The estimator is a port of the bench-proven STM32 observer (`hall.c` in
the reference project): instead of snapping θ to each edge and dead
reckoning in between, it keeps a continuous estimate $(\hat\theta,
\hat\omega)$ and applies **soft corrections** at each edge:

$$\text{per PWM period (tick):} \quad \hat\theta \mathrel{+}= \hat\omega$$
$$\text{per accepted edge:} \quad
  \hat\theta \mathrel{+}= K_P \cdot \text{wrap}(\theta_{bnd} - \hat\theta), \qquad
  \hat\omega \mathrel{+}= K_W \cdot (\omega_{edge} - \hat\omega)$$

with $K_P = K_W = 0.3$ (Q0.16 constants `PLL_KP_Q16`/`PLL_KW_Q16`,
compile-time — the STM32 shipped 0.3/0.3 untouched; a runtime `pllk`
command is a noted extension point). $\theta_{bnd}$ is the crossed
boundary's angle from the 12-entry table. `wrap()` is free: the signed
16-bit difference of two angle codes is exactly the ±180° wrap.

Soft correction is what makes the observer robust where snapping is not:
an uncalibrated or unequal sector boundary, sensor hysteresis, or edge
jitter each pull $\hat\theta$ by only 30 % of their error, so
sector-to-sector disagreement averages out instead of being injected
into the Park transform verbatim.

### 4.1 Fixed-point state

| State | Format | Notes |
|---|---|---|
| $\hat\theta$ (`theta_q`) | Q16.16 angle codes (32-bit, wraps mod $2^{32}$) | output is the upper 16 bits |
| $\hat\omega$ (`omega_q`) | Q16.16 codes/period, signed, clamped ±`OMEGA_MAX_CODES` (512) | fractional bits matter: ≈ 1.4 codes/period at 100 rpm |

The per-period unit makes the integration dt-free: $\hat\omega$ is
*per PWM period* and the tick (wired to `pwm_gen`'s `cnt_peak` in
`foc_top`) *is* the period — the STM32's $\hat\theta \mathrel{+}=
\hat\omega \, dt$ with $dt$ folded in. The 3-clock update finishes long
before the XADC delivers `sample_valid`, so `foc_core` Parks with the
just-integrated angle, mirroring the STM32's observer-at-top-of-ISR
ordering.

### 4.2 Edge speed measurement

At each accepted same-direction adjacent edge, the *calibrated* angle
distance between this edge and the previous one (`traveled`, from the
table) is divided by the elapsed time `t_cnt` (clocks):

$$\omega_{edge} = \pm\frac{traveled \ll 16}{t\_cnt} \cdot PERIOD\_CYC
  \quad \text{[codes/period, Q16.16]}$$

The division runs on a **serial restoring divider, one bit per clock,
32 clocks total** — invisible next to the ≥ `MIN_EDGE_CYC` edge spacing.
The θ and ω corrections for a rate-carrying edge are queued together
when the divide completes and applied at the next tick.

Deviation from the STM32 (which assumes a fixed 60° per edge): using the
calibrated traveled distance makes $\omega_{edge}$ exact even with
unequal sectors.

### 4.3 Edge guards (mapping `hall.c`)

| Condition | Action |
|---|---|
| edge < `MIN_EDGE_CYC` (50 µs) after the last accepted one | ignored entirely (contact bounce; the decoder's 160 ns debounce is for synchronization glitches, this guard is for mechanical chatter) |
| non-adjacent sector jump | tracking state updates, **no** PLL handoff |
| direction reversal | θ correction only; $\hat\omega$ is **zeroed** and re-measured fresh (deviation: the STM32 blends a speed measured *across* the reversal — hysteresis-dominated, deliberately not ported) |
| first edge after reset or stale | θ correction only (no rate history) |

### 4.4 Standstill, cold start

- **Stale**: no edge for `TIMEOUT_CYC` (**100 ms**, matching the STM32)
  ⇒ `moving = 0`, ω = 0, θ freezes at the **center of the current
  sector** and the observer re-arms as at cold start. (The old
  interpolator held the last θ instead — center-freeze is the minimax
  choice and the STM32 semantics.)
- **Before the first edge**: θ outputs the *center* of the current Hall
  sector — bounding the initial error to ±30°. After reset (no sector
  knowledge at all) the reset value is sector 0's center.

### 4.5 Dynamics and units

ω unit: angle codes per PWM period (`PERIOD_CYC` $= 2 \cdot PWM\_ARR =
1250$ clocks), chosen because every consumer runs at the PWM rate.
Conversion for the host:

$$\omega_{elec}\,[\text{rad/s}] = \omega_{codes} \cdot \frac{2\pi}{2^{16}} \cdot F_{sw},
\qquad \text{rpm} = \omega_{codes} \cdot \frac{60 \cdot F_{sw}}{2^{16}} \ (N_p = 1)$$

Convergence is geometric: each edge removes 30 % of the remaining θ and
ω error, so a speed step settles in roughly 12–20 edges (2–3 electrical
revolutions). θ advances as a **per-period staircase**; `foc_core`
samples it once per period at the same tick, so consumers see no
staircase. Because the boundary error is measured at the tick *after*
the physical crossing (as on the STM32), the locked estimate carries a
small systematic offset ≈ ω · ½ period — about 110 codes (0.6°) at the
motor's speed ceiling.

## 5. Latency / error budget

| Source | Bound |
|---|---|
| synchronizer + debounce | 18 clk = 180 ns ≈ 0.03° at max speed |
| speed quantization | 1/t_cnt relative; < 0.1 % above 100 rpm |
| tick staircase | ≤ ω · 1 period; invisible to per-period consumers |
| correction timing skew | ≈ ω · ½ period systematic (≤ 0.6° at ceiling) |
| placement / hysteresis | 0.3-soft-corrected uncalibrated; removed by the 12-entry table after calibration |

## 6. Verification

`sim/tb_hall_decode.sv`: spun-rotor stimulus in both directions, glitch
injection shorter than the debounce window, illegal-state flagging,
direction flips, multi-step jump tolerance.

`sim/tb_hall_angle_est.sv` (PLL semantics — speed changes are ramped,
as a real rotor's inertia dictates): cold-start center, tick-aligned
tracking error bound after lock at several speeds including a
non-integer codes/period speed (fractional $\hat\omega$), a **no-snap
invariant** (per-tick $|\Delta\theta - \omega|$ bounded — soft
corrections only), **unequal physical sectors against the identity
table** (bounded error without calibration — the PLL's reason to exist),
tight tracking with the calibrated table both directions, bounce-pair
rejection inside `MIN_EDGE_CYC`, stale freeze at the sector center, and
direction-reversal re-lock with ω re-measured fresh.
