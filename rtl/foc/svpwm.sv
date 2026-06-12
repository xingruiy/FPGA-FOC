// ============================================================================
// svpwm.sv
//
//  Space-vector PWM by min/max zero-sequence injection.
//
//  Inputs valpha/vbeta are Q1.15, normalized to the DC link (1.0 == Vdc),
//  so the linear hexagon is |v| <= 1/sqrt(3) and modulation index
//  m = |v| * sqrt(3).
//
//      va =  valpha
//      vb = -valpha/2 + (sqrt(3)/2) * vbeta
//      vc = -valpha/2 - (sqrt(3)/2) * vbeta
//      voff = -(max + min)/2
//      d_x  = 0.5 + (v_x + voff)          clamped to 0.5 +/- MAX_MOD/2
//
//  Duties are Q1.15 in [0, 1), centered on 0.5. The MAX_MOD clamp
//  (per-phase backstop; the upstream d/q limiter should keep vectors
//  inside) guarantees min(duty) >= (1-MAX_MOD)/2, i.e. the low-side
//  conduction window never shrinks below (1-MAX_MOD)*Tsw/2 = 812 ns at
//  80 kHz - the XADC sample aperture + settling budget. `sat` flags any
//  clamping. Latency: 3 clk (phase projection, then min/max offset, then
//  clamp - each registered for 100 MHz), in_valid piped to out_valid.
// ============================================================================

module svpwm
  import foc_pkg::*;
(
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,
  input  q15_t valpha,
  input  q15_t vbeta,
  output logic out_valid,
  output q15_t da,
  output q15_t db,
  output q15_t dc,
  output logic sat
);

  localparam logic signed [31:0] HALF_Q15   = 32'sd16384; // 0.5
  localparam logic signed [31:0] SQRT3_2    = 32'sd28378; // sqrt(3)/2 Q1.15
  localparam logic signed [31:0] REL_MAX    = 32'(MAX_MOD_Q15) >>> 1; // 14254

  function automatic logic signed [18:0] clamp_rel
      (input logic signed [18:0] x);
    if      (x >  19'(REL_MAX)) return  19'(REL_MAX);
    else if (x < -19'(REL_MAX)) return -19'(REL_MAX);
    else                        return x;
  endfunction

  // stage 1: project onto the three phases (the two multiplies), register.
  // |v_phase| <= (1/2 + sqrt(3)/2) * 1.0 < 2.0, so Q3.15 (18 bit) holds it;
  // the narrow width keeps the stage-2/3 carry chains short for 100 MHz.
  logic signed [17:0] va_r, vb_r, vc_r;

  always_ff @(posedge clk) begin
    va_r <= 18'(valpha);
    vb_r <= 18'(rnd_shr(-32'(valpha) * HALF_Q15 + 32'(vbeta) * SQRT3_2, 15));
    vc_r <= 18'(rnd_shr(-32'(valpha) * HALF_Q15 - 32'(vbeta) * SQRT3_2, 15));
  end

  // stage 2: min/max zero-sequence offset, register voff + delayed phases
  logic signed [17:0] vmax, vmin;
  logic signed [17:0] voff_r, va_d, vb_d, vc_d;

  always_comb begin
    vmax = (va_r > vb_r) ? va_r : vb_r;  if (vc_r > vmax) vmax = vc_r;
    vmin = (va_r < vb_r) ? va_r : vb_r;  if (vc_r < vmin) vmin = vc_r;
  end

  always_ff @(posedge clk) begin
    voff_r <= 18'(-((19'(vmax) + 19'(vmin) + 19'sd1) >>> 1)); // -rnd_shr(.,1)
    va_d   <= va_r;
    vb_d   <= vb_r;
    vc_d   <= vc_r;
  end

  // stage 3: inject offset + MAX_MOD clamp
  logic signed [18:0] ra, rb, rc;
  logic clamp_a, clamp_b, clamp_c;

  always_comb begin
    ra = 19'(va_d) + 19'(voff_r);
    rb = 19'(vb_d) + 19'(voff_r);
    rc = 19'(vc_d) + 19'(voff_r);
    clamp_a = (ra > REL_MAX) || (ra < -REL_MAX);
    clamp_b = (rb > REL_MAX) || (rb < -REL_MAX);
    clamp_c = (rc > REL_MAX) || (rc < -REL_MAX);
  end

  always_ff @(posedge clk) begin
    da  <= q15_t'(HALF_Q15 + clamp_rel(ra));
    db  <= q15_t'(HALF_Q15 + clamp_rel(rb));
    dc  <= q15_t'(HALF_Q15 + clamp_rel(rc));
    sat <= clamp_a || clamp_b || clamp_c;
  end

  logic v1, v2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1        <= 1'b0;
      v2        <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      v1        <= in_valid;
      v2        <= v1;
      out_valid <= v2;
    end
  end

endmodule
