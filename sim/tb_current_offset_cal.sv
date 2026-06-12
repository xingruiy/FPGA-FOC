// ============================================================================
// tb_current_offset_cal.sv - offset calibration + ic reconstruction tests.
//
//  - injected DC offsets are removed exactly after a 64-sample calibration
//  - alternating +/-n noise during calibration averages out exactly
//  - random noise: offset estimate lands within a statistical bound
//  - ic == -(ia + ib) exactly, with saturation at the rails
//  - out_valid suppressed while calibrating
// ============================================================================
`timescale 1ns / 1ps

module tb_current_offset_cal;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0;
  logic in_valid = 0, cal_start = 0, cal_done, cal_busy, out_valid;
  q15_t ia_raw = 0, ib_raw = 0;
  q15_t ia, ib, ic;

  current_offset_cal #(.CAL_SAMPLES(64)) dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  task automatic push(input int a, input int b);
    @(negedge clk);
    ia_raw = q15_t'(a); ib_raw = q15_t'(b); in_valid = 1;
    @(negedge clk);
    in_valid = 0;
  endtask

  function automatic int clamp16(input int x);
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return x;
  endfunction

  task automatic check_out(input int exp_a, input int exp_b);
    int exp_c;
    exp_c = clamp16(-exp_a - exp_b);
    if (int'(ia) != exp_a || int'(ib) != exp_b || int'(ic) != exp_c) begin
      $display("  MISMATCH out ia=%0d/%0d ib=%0d/%0d ic=%0d/%0d",
               ia, exp_a, ib, exp_b, ic, exp_c);
      errors++;
    end
  endtask

  task automatic run_cal_const(input int off_a, input int off_b);
    @(negedge clk); cal_start = 1;
    @(negedge clk); cal_start = 0;
    for (int i = 0; i < 64; i++) push(off_a, off_b);
    @(negedge clk);
    if (!cal_done) begin
      $display("  MISMATCH cal_done not set"); errors++;
    end
  endtask

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- constant offsets removed exactly --------------------------------
    run_cal_const(1500, -2300);
    push(1500, -2300);  #1 check_out(0, 0);
    push(1500 + 1000, -2300 + 500); #1 check_out(1000, 500);
    push(1500 - 4000, -2300 + 4000); #1 check_out(-4000, 4000);

    // ---- out_valid suppressed during calibration ---------------------------
    begin
      int seen = 0;
      @(negedge clk); cal_start = 1;
      @(negedge clk); cal_start = 0;
      for (int i = 0; i < 64; i++) begin
        push(100, 200);
        if (out_valid) seen++;
      end
      // (the strobe lags in_valid by 1: sample the cycle after each push)
      if (seen != 0) begin
        $display("  MISMATCH out_valid during calibration"); errors++;
      end
    end

    // ---- alternating noise averages out exactly ----------------------------
    @(negedge clk); cal_start = 1;
    @(negedge clk); cal_start = 0;
    for (int i = 0; i < 64; i++) begin
      push(700 + ((i % 2 == 0) ? 250 : -250),
           -900 + ((i % 2 == 0) ? -125 : 125));
    end
    @(negedge clk);
    push(700, -900); #1 check_out(0, 0);

    // ---- real operating point: samples straddle zero ----------------------
    // xadc_iface converts offset-binary to two's complement, so the
    // zero-current mid-bias lands at ~0 and noisy samples alternate sign;
    // the signed average must come out exact (no wrap artifacts).
    @(negedge clk); cal_start = 1;
    @(negedge clk); cal_start = 0;
    for (int i = 0; i < 64; i++) begin
      push(8 + ((i % 2 == 0) ? 200 : -200),
           -8 + ((i % 2 == 0) ? -200 : 200));
    end
    @(negedge clk);
    push(8, -8);            #1 check_out(0, 0);
    push(8 + 300, -8 - 300); #1 check_out(300, -300);

    // ---- random noise: estimate within bound -------------------------------
    @(negedge clk); cal_start = 1;
    @(negedge clk); cal_start = 0;
    for (int i = 0; i < 64; i++) begin
      push(1200 + $urandom_range(0, 400) - 200,
           -800 + $urandom_range(0, 400) - 200);
    end
    @(negedge clk);
    push(1200, -800); #1
    // uniform +/-200 -> sigma ~115, mean-of-64 sigma ~15; allow 5 sigma
    if (int'(ia) > 75 || int'(ia) < -75 || int'(ib) > 75 || int'(ib) < -75)
    begin
      $display("  MISMATCH noisy cal residual ia=%0d ib=%0d", ia, ib);
      errors++;
    end

    // ---- ic saturation ------------------------------------------------------
    run_cal_const(0, 0);
    push(-20000, -20000); #1
    if (int'(ic) != 32767) begin
      $display("  MISMATCH ic saturation got=%0d", ic); errors++;
    end
    push(20000, 20000); #1
    if (int'(ic) != -32768) begin
      $display("  MISMATCH ic neg saturation got=%0d", ic); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_current_offset_cal");
    else $display("TB_FAIL: tb_current_offset_cal (%0d errors)", errors);
    $finish;
  end

endmodule
