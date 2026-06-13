# PWM Generation

Theory, math and implementation of `rtl/pwm/pwm_gen.sv` — the 3-phase
center-aligned complementary PWM generator with dead-time insertion.

---

## 1. Why center-aligned PWM

A triangular (up/down) carrier is used instead of a sawtooth for three
reasons, all of which matter for this design:

1. **Current sampling.** With high-side-on centered on the counter
   *trough*, the low-side FET conduction window is centered on the counter
   *peak*. The DRV8316 current-sense amplifiers are **low-side** — their
   output is only valid while the low-side FET conducts — so the counter
   peak is the one moment guaranteed to be (a) inside the low-side window
   of all three phases and (b) at the **midpoint of the current ripple**,
   where the instantaneous current equals the period-average current. The
   `cnt_peak` strobe triggers the XADC for exactly this reason.
2. **Harmonics.** Center-aligned switching places phase-voltage edges
   symmetrically, pushing ripple energy to twice the switching frequency
   for the line-to-line voltages.
3. **Glitch-free update.** All three compare values change simultaneously
   at the counter trough (`update` strobe), where no edge can be in flight.

## 2. Frequency and resolution

The counter runs 0 → ARR → 0, so one PWM period is $2 \cdot ARR$ clock
cycles (the trough and peak values are each visited once):

$$F_{pwm} = \frac{F_{clk}}{2 \cdot ARR}$$

In the general case a prescaler $N$ and an $M$-bit counter give
$F_{pwm} = F_{clk} / (2(N{+}1) \cdot 2^M)$; this design needs no prescaler
because the period is set directly by ARR.

Project values (locked in `foc_pkg.sv`):

| Parameter | Value | Result |
|---|---|---|
| `F_CLK_HZ` | 100 MHz | single BUFG clock, no PLL |
| `PWM_ARR`  | 625 | $F_{pwm}$ = 100 MHz / 1250 = **80 kHz** |
| duty resolution | $\log_2 625 \approx 9.3$ bit | ~0.16 % duty steps |
| `DEADTIME_CYC` | 20 (200 ns) | above the DRV8316 minimum dead time |

80 kHz was chosen for the very low motor inductance (the two-phase
conduction loop is $2 L_s$ = 254 µH — the datasheet's line-to-line
0.253 mH — at 24 V): the worst-case current ripple at $D = 0.5$ is

$$\Delta I_{pp} \approx \frac{V_{bus}}{4 \, L \, F_{pwm}}
              = \frac{24}{4 \cdot 0.254\,\text{mH} \cdot 80\,\text{kHz}}
              \approx 0.30\ \text{A p-p},$$

accepted because the instantaneous peak at rated current
(0.22 A + 0.15 A ripple ≈ 0.37 A) still clears the 0.9 A OCP trip with
margin.

## 3. Duty → compare value

Duties arrive as Q1.15 in [0, 1). The compare value is the rounded
product

$$ccr = \left\lfloor \frac{duty \cdot ARR + 2^{14}}{2^{15}} \right\rfloor
      = \text{round}(duty \cdot ARR), \qquad ccr \in [0, ARR]$$

(`ccr_of()` in the RTL; negative or zero duty maps to 0). The three `ccr`
registers are **double-buffered**: they only load at `cnt == 0` (the
`update` strobe), so a duty written mid-period cannot produce a truncated
or doubled pulse. While the counter is stopped (`!en`) the buffers track
the inputs so the first enabled period is already correct.

## 4. Output formation and exact pulse widths

The raw (ideal, pre-dead-time) high-side command is

```
raw = en && (cnt < ccr)
```

Across one period the counter takes the values
`0,1,…,ARR-1,ARR,ARR-1,…,1`; the comparison is true for `ccr` cycles on
the way up and `ccr − 1` on the way down, so:

- ideal high-side pulse width = $2 \cdot ccr - 1$ cycles, centered on the
  trough;
- after dead time delays the rising edge, the **steady-state high-side
  pulse width is $2 \cdot ccr - 1 - DT$ cycles** (this exact formula is
  asserted in `tb_pwm_gen`);
- low side is the complement, similarly shortened by DT on each turn-on.

The duty seen by the motor is therefore $\approx ccr/ARR$ with a constant
dead-time loss of $DT/(2 \cdot ARR)$ = 1.6 % — common-mode across phases
at the operating points of interest, and absorbed by the current loop.

## 5. Dead-time insertion

Each phase has two independent down-counters `t_h`, `t_l` preloaded with
`DT`:

- when `raw` deasserts, the high gate drops **immediately** and `t_h`
  reloads; the low gate may only rise after its own timer has counted
  `DT` cycles with both gates off;
- symmetrically for the opposite transition.

Properties (all checked by assertions/TB):

- the two gates of a phase are **never simultaneously high** under any
  input sequence, including duty 0 %, 100 % and enable glitches;
- the gap between a complementary turn-off and the partner's turn-on is
  **exactly** DT cycles in steady state, and ≥ DT in all cases;
- `oe[i] = 0` (the per-phase kill input, driven combinationally by the
  safe-state logic in `foc_top`) forces both gates of that phase low
  within **one clock** and re-arms both timers, so a re-enable always
  pays a fresh dead time;
- `en = 0` stops the counter and all six gates.

DT = 200 ns sits above the DRV8316's specified minimum dead time; the
driver also inserts its own protective handshake, so the FPGA value is
the system dead time only if it is the larger of the two — verify on a
scope during bring-up (Phase 6.5).

## 6. Interface summary

| Port | Dir | Meaning |
|---|---|---|
| `duty_a/b/c` (Q1.15) | in | sampled into `ccr` at each `update` |
| `oe[2:0]` | in | per-phase async-kill, {c,b,a} |
| `pwm_{a,b,c}{h,l}` | out | six gate signals |
| `cnt`, `cnt_peak` | out | counter value; 1-clk strobe at `cnt == ARR` — XADC trigger |
| `update` | out | 1-clk strobe at `cnt == 0` — duty load / foc timing reference |

The duties produced by `svpwm` are clamped to $0.5 \pm MAX\_MOD/2$
(MAX_MOD = 0.87), which guarantees a minimum low-side conduction window of
$(1 - 0.935) \cdot T_{sw} = 812$ ns per period — the budget for XADC
acquisition plus front-end settling. See `docs/foc.md` §6.

## 7. Verification (`sim/tb_pwm_gen.sv`)

- pulse-run measurement of every steady-state width against
  $2 \cdot ccr - 1 - DT$ over a duty sweep including 0 and ~100 %;
- shoot-through assertion (`!(h && l)`) continuously armed;
- dead-gap measurement distinguishing complementary transitions (exactly
  DT) from same-gate re-rises at duty extremes (≥ DT);
- `cnt_peak`/`update` placement, double-buffer latching point, and the
  1-clock `oe` kill.
