// ============================================================================
// tb_svpwm.sv - property sweep of the SVPWM zero-sequence injector.
//
//  Grid sweep of (valpha, vbeta) over and beyond the hexagon. Checks:
//   - duty range:    0.5 +/- MAX_MOD/2, never exceeded (=> min low-side
//                    conduction window >= 812 ns at 80 kHz)
//   - centering:     max(d)-0.5 == 0.5-min(d) (min/max injection), unclamped
//   - line-to-line:  d_a - d_b == v_a - v_b (zero-seq cancels), unclamped
//   - sat flag:      asserted iff a per-phase clamp engaged (with a small
//                    rounding deadband around the threshold)
// ============================================================================
`timescale 1ns / 1ps

module tb_svpwm;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0, in_valid = 0;
  q15_t valpha, vbeta;
  logic out_valid, sat;
  q15_t da, db, dc;

  svpwm dut (.*);
  always #5 clk = ~clk;

  int errors = 0;
  localparam int REL_MAX = 14254;          // MAX_MOD/2 in Q1.15
  localparam int D_LO = 16384 - REL_MAX - 1; // +/-1 LSB rounding slack
  localparam int D_HI = 16384 + REL_MAX + 1;

  real va_r, vb_r, vc_r, vmax_r, vmin_r, voff_r, ra_r, rb_r, rc_r, rmax_r;
  int  dmax, dmin, ll_got, ll_exp;

  task automatic run_one(input int a, input int b);
    int t_out;
    @(negedge clk);
    valpha = q15_t'(a); vbeta = q15_t'(b); in_valid = 1;
    @(negedge clk);
    in_valid = 0;
    t_out = 0;
    while (out_valid !== 1'b1 && t_out < 10) begin @(negedge clk); t_out++; end
    if (out_valid !== 1'b1) begin
      $display("  MISMATCH out_valid not set"); errors++;
    end

    // real-arithmetic reference
    va_r = real'(a);
    vb_r = -real'(a) / 2.0 + $sqrt(3.0) / 2.0 * real'(b);
    vc_r = -real'(a) / 2.0 - $sqrt(3.0) / 2.0 * real'(b);
    vmax_r = (va_r > vb_r) ? va_r : vb_r; if (vc_r > vmax_r) vmax_r = vc_r;
    vmin_r = (va_r < vb_r) ? va_r : vb_r; if (vc_r < vmin_r) vmin_r = vc_r;
    voff_r = -(vmax_r + vmin_r) / 2.0;
    ra_r = va_r + voff_r; rb_r = vb_r + voff_r; rc_r = vc_r + voff_r;
    rmax_r = (ra_r < 0 ? -ra_r : ra_r);
    if ((rb_r < 0 ? -rb_r : rb_r) > rmax_r) rmax_r = (rb_r < 0 ? -rb_r : rb_r);
    if ((rc_r < 0 ? -rc_r : rc_r) > rmax_r) rmax_r = (rc_r < 0 ? -rc_r : rc_r);

    // 1) duty range - the hard guarantee for low-side sampling
    if (int'(da) < D_LO || int'(da) > D_HI ||
        int'(db) < D_LO || int'(db) > D_HI ||
        int'(dc) < D_LO || int'(dc) > D_HI) begin
      $display("  MISMATCH duty range a=%0d b=%0d: %0d %0d %0d", a, b, da, db, dc);
      errors++;
    end

    // 2) sat flag with rounding deadband around the clamp threshold
    if (rmax_r > real'(REL_MAX) + 3.0 && sat !== 1'b1) begin
      $display("  MISMATCH sat not set a=%0d b=%0d (rmax=%f)", a, b, rmax_r);
      errors++;
    end
    if (rmax_r < real'(REL_MAX) - 3.0 && sat !== 1'b0) begin
      $display("  MISMATCH sat spurious a=%0d b=%0d (rmax=%f)", a, b, rmax_r);
      errors++;
    end

    // 3+4) only meaningful when nothing clamps
    if (rmax_r < real'(REL_MAX) - 3.0) begin
      dmax = int'(da); if (int'(db) > dmax) dmax = int'(db);
      if (int'(dc) > dmax) dmax = int'(dc);
      dmin = int'(da); if (int'(db) < dmin) dmin = int'(db);
      if (int'(dc) < dmin) dmin = int'(dc);
      if ((dmax - 16384) - (16384 - dmin) > 2 ||
          (dmax - 16384) - (16384 - dmin) < -2) begin
        $display("  MISMATCH centering a=%0d b=%0d: dmax=%0d dmin=%0d",
                 a, b, dmax, dmin);
        errors++;
      end
      ll_got = int'(da) - int'(db);
      ll_exp = int'(va_r - vb_r + (va_r > vb_r ? 0.5 : -0.5));
      if (ll_got - ll_exp > 3 || ll_got - ll_exp < -3) begin
        $display("  MISMATCH line-line a=%0d b=%0d: got=%0d exp=%0d",
                 a, b, ll_got, ll_exp);
        errors++;
      end
    end

    @(negedge clk);
  endtask

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // directed: zero vector -> all duties 0.5
    run_one(0, 0);
    if (int'(da) != 16384 || int'(db) != 16384 || int'(dc) != 16384) begin
      $display("  MISMATCH zero vector duties: %0d %0d %0d", da, db, dc);
      errors++;
    end

    // grid sweep: +/-0.9 FS in 0.05 steps (covers hexagon and beyond)
    for (int a = -29491; a <= 29491; a += 1638)
      for (int b = -29491; b <= 29491; b += 1638)
        run_one(a, b);

    // ring just inside MAX_MOD: m = 0.85 -> |v| = 0.4907, must never sat
    for (int k = 0; k < 64; k++) begin
      run_one(int'(0.4907 * 32768.0 * $cos(6.28318530718 * k / 64.0)),
              int'(0.4907 * 32768.0 * $sin(6.28318530718 * k / 64.0)));
      if (sat) begin
        $display("  MISMATCH sat inside MAX_MOD ring k=%0d", k); errors++;
      end
    end

    if (errors == 0) $display("TB_PASS: tb_svpwm");
    else             $display("TB_FAIL: tb_svpwm (%0d errors)", errors);
    $finish;
  end

endmodule
