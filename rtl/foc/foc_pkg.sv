// ============================================================================
// foc_pkg.sv
//
//  Shared package for the FOC current-loop project: numeric types, locked
//  operating-point parameters (see docs/config.md), DRV8316 register constants
//  and the fixed-point helper functions used by every math module.
//
//  Numeric conventions
//    - q15_t  : Q1.15  signed, +/-1.0 range.  External I/O (currents,
//               voltages, duties, sin/cos) use this format.
//    - q13_t  : Q3.13  signed, +/-4.0 range.  Internal Clarke/Park/SVPWM
//               arithmetic, where sqrt(3) scaling would overflow Q1.15.
//    - angle_t: unsigned 16 bit, full electrical circle = 2^16 codes.
//    - All multiplies produce full-width products; rounding is
//      round-half-up (bias then arithmetic shift); saturation only at
//      format boundaries.
// ============================================================================

package foc_pkg;

  // ------------------------------------------------------------------
  // Clocking / PWM operating point (locked)
  // ------------------------------------------------------------------
  parameter int unsigned F_CLK_HZ = 100_000_000; // single BUFG clock, no PLL
  parameter int unsigned F_SW_HZ  = 80_000;

  // Center-aligned counter: 0 .. PWM_ARR .. 0, period = 2*PWM_ARR clk cycles.
  parameter int unsigned PWM_ARR = F_CLK_HZ / (2 * F_SW_HZ); // 625 -> 80 kHz

  // Dead time in clk cycles; must stay above the DRV8316 minimum dead time.
  parameter int unsigned DEADTIME_NS  = 200;
  parameter int unsigned DEADTIME_CYC = (DEADTIME_NS * (F_CLK_HZ / 1_000_000))
                                        / 1000; // 20 cycles @ 100 MHz

  // ------------------------------------------------------------------
  // Types
  // ------------------------------------------------------------------
  typedef logic signed [15:0] q15_t;  // Q1.15
  typedef logic signed [15:0] q13_t;  // Q3.13
  typedef logic        [15:0] angle_t;

  parameter q15_t Q15_MAX = 16'sh7FFF; //  0.999969
  parameter q15_t Q15_MIN = 16'sh8000; // -1.0
  parameter q13_t Q13_MAX = 16'sh7FFF; //  3.999878
  parameter q13_t Q13_MIN = 16'sh8000; // -4.0
  parameter q13_t Q13_ONE = 16'sh2000; //  1.0 in Q3.13

  // ------------------------------------------------------------------
  // Current / voltage scaling (locked)
  // ------------------------------------------------------------------
  // CSA gain 1.2 V/A + front-end scaling -> XADC full scale = +/-1.25 A,
  // i.e. q15 current 1.0 == 1.25 A.
  parameter real I_FULLSCALE_A = 1.25;
  parameter real VBUS_NOM_V    = 24.0;  // 24 V always; no 12 V operation

  parameter q15_t MAX_MOD_Q15  = 16'sd28508; // 0.87  modulation cap
  parameter q15_t OCP_TRIP_Q15 = 16'sd23593; // 0.9 A / 1.25 A FS = 0.72

  // ------------------------------------------------------------------
  // DRV8316 SPI register map
  //   Frame (16 bit, MSB first): {RW, A5..A0, PARITY, D7..D0}
  //   RW = 1 read, 0 write; PARITY = even parity over the whole frame.
  //   Field encodings below follow datasheet rev; re-verify against the
  //   exact silicon rev during Phase 4 bring-up.
  // ------------------------------------------------------------------
  parameter logic [5:0] DRV_REG_IC_STAT = 6'h00;
  parameter logic [5:0] DRV_REG_STAT1   = 6'h01;
  parameter logic [5:0] DRV_REG_STAT2   = 6'h02;
  parameter logic [5:0] DRV_REG_CTRL1   = 6'h03; // REG_LOCK
  parameter logic [5:0] DRV_REG_CTRL2   = 6'h04; // PWM_MODE, SLEW, SDO_MODE
  parameter logic [5:0] DRV_REG_CTRL3   = 6'h05; // OVP, PWM 100% duty sel
  parameter logic [5:0] DRV_REG_CTRL4   = 6'h06; // OCP config (driver-level!)
  parameter logic [5:0] DRV_REG_CTRL5   = 6'h07; // CSA_GAIN, ASR/AAR
  parameter logic [5:0] DRV_REG_CTRL6   = 6'h08; // buck regulator
  parameter logic [5:0] DRV_REG_CTRL10  = 6'h0C; // driver delay compensation

  parameter logic [2:0] DRV_REG_UNLOCK  = 3'b011; // CTRL1 unlock code
  parameter logic [2:0] DRV_REG_LOCK    = 3'b110; // CTRL1 lock code
  parameter logic [1:0] DRV_PWM_MODE_6X = 2'b00;  // CTRL2.PWM_MODE
  parameter logic [1:0] DRV_CSA_GAIN_1V2= 2'b11;  // CTRL5.CSA_GAIN = 1.2 V/A

  // Config values written by drv8316_spi at startup. Field packing per
  // datasheet rev B; RE-VERIFY against the silicon rev during Phase 6
  // bring-up (the SPI TB checks mechanics and that these exact values
  // land in the slave registers, not the field semantics).
  parameter logic [7:0] DRV_CTRL1_CFG = {5'b0, DRV_REG_UNLOCK};
  parameter logic [7:0] DRV_CTRL2_CFG = {3'b0, 2'b01, DRV_PWM_MODE_6X, 1'b0};
                                        // SLEW=01 (50 V/us), 6x PWM
  parameter logic [7:0] DRV_CTRL4_CFG = 8'h00; // OCP defaults: driver-level
                                               // protection only (16 A class)
  parameter logic [7:0] DRV_CTRL5_CFG = {6'b0, DRV_CSA_GAIN_1V2}; // 1.2 V/A

  // ------------------------------------------------------------------
  // Fixed-point helpers
  // ------------------------------------------------------------------

  // Saturate a 32-bit value to Q1.15 / Q3.13 (same 16-bit container).
  function automatic q15_t sat16(input logic signed [31:0] x);
    if      (x > 32'sd32767)  return 16'sh7FFF;
    else if (x < -32'sd32768) return 16'sh8000;
    else                      return q15_t'(x[15:0]);
  endfunction

  // Round-half-up arithmetic shift right by N (N >= 1).
  function automatic logic signed [31:0] rnd_shr
      (input logic signed [31:0] x, input int unsigned n);
    return (x + (32'sd1 <<< (n - 1))) >>> n;
  endfunction

  // Q1.15 * Q1.15 -> Q1.15, rounded, saturating (only -1 * -1 saturates).
  function automatic q15_t q15_mul(input q15_t a, input q15_t b);
    logic signed [31:0] p;
    p = 32'(a) * 32'(b);            // Q2.30
    return sat16(rnd_shr(p, 15));
  endfunction

  // Q3.13 * Q3.13 -> Q3.13, rounded, saturating.
  function automatic q13_t q13_mul(input q13_t a, input q13_t b);
    logic signed [31:0] p;
    p = 32'(a) * 32'(b);            // Q6.26
    return sat16(rnd_shr(p, 13));
  endfunction

  // Q3.13 * Q1.15 -> Q3.13, rounded, saturating (e.g. current * sin/cos).
  function automatic q13_t q13_q15_mul(input q13_t a, input q15_t b);
    logic signed [31:0] p;
    p = 32'(a) * 32'(b);            // Q4.28
    return sat16(rnd_shr(p, 15));
  endfunction

  // Format conversions.
  function automatic q13_t q15_to_q13(input q15_t x); // exact range, rounded
    return q13_t'(rnd_shr(32'(x), 2));
  endfunction

  function automatic q15_t q13_to_q15(input q13_t x); // saturating
    return sat16(32'(x) <<< 2);
  endfunction

  // ------------------------------------------------------------------
  // d/q output vector limiter (vd priority)
  // ------------------------------------------------------------------

  // Integer square root, floor(sqrt(x)), classic non-restoring (16 steps).
  function automatic logic [15:0] isqrt32(input logic [31:0] x);
    logic [31:0] op, res, one;
    op  = x;
    res = '0;
    one = 32'h4000_0000;
    for (int i = 0; i < 16; i++) begin
      if (op >= res + one) begin
        op  = op - (res + one);
        res = (res >> 1) + one;
      end else begin
        res = res >> 1;
      end
      one = one >> 2;
    end
    return res[15:0];
  endfunction

  typedef struct packed {
    q15_t vd;
    q15_t vq;
    logic clamped;
  } dq_lim_t;

  // Clamp the (vd, vq) vector to magnitude <= vmax with vd priority:
  // vd is range-clamped first and keeps its value; vq gets what is left
  // (sqrt(vmax^2 - vd^2)). Clamped values are exported so the PI
  // anti-windup sees the *applied* output. vmax > 0 (typically Vdc/sqrt(3)
  // scaled by the measured bus voltage).
  function automatic dq_lim_t dq_limit(input q15_t vd, input q15_t vq,
                                       input q15_t vmax);
    logic signed [31:0] vd32, vq32, vqlim32;
    logic [31:0] vd2, vq2, vmax2, rem;
    logic [15:0] vqlim;
    dq_lim_t r;
    r.clamped = 1'b0;

    vd32 = 32'(vd);
    if (vd32 > 32'(vmax))       begin vd32 =  32'(vmax); r.clamped = 1'b1; end
    else if (vd32 < -32'(vmax)) begin vd32 = -32'(vmax); r.clamped = 1'b1; end

    vmax2 = 32'(vmax) * 32'(vmax);
    vd2   = unsigned'(vd32 * vd32);
    rem   = vmax2 - vd2; // >= 0 by construction

    vq32 = 32'(vq);
    vq2  = unsigned'(vq32 * vq32);
    if (vq2 > rem) begin
      vqlim = isqrt32(rem);
      vq32  = (vq32 < 0) ? -32'(vqlim) : 32'(vqlim);
      r.clamped = 1'b1;
    end

    r.vd = q15_t'(vd32);
    r.vq = q15_t'(vq32);
    return r;
  endfunction

endpackage : foc_pkg
