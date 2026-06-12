// ============================================================================
// tb_clarke.sv - golden-vector test of the Clarke transform.
//
//  Integer-exact expected values (the DUT rounding is mirrored bit-exactly),
//  directed corner cases, random in-range vectors (no sat flag allowed) and
//  sqrt(3)-scale stimulus that must saturate with the flag set.
// ============================================================================
`timescale 1ns / 1ps

module tb_clarke;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0, in_valid = 0;
  q15_t ia, ib;
  logic out_valid, sat;
  q15_t ialpha, ibeta;

  clarke dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  // bit-exact model of the DUT arithmetic (independent coding)
  function automatic int model_beta(input int a, input int b);
    longint s, p;
    s = a + 2 * b;
    p = s * 18919;
    p = (p + 16384) >>> 15;
    if (p > 32767)  p = 32767;
    if (p < -32768) p = -32768;
    return int'(p);
  endfunction

  function automatic bit model_sat(input int a, input int b);
    longint s, p;
    s = a + 2 * b;
    p = ((s * 18919) + 16384) >>> 15;
    return (p > 32767) || (p < -32768);
  endfunction

  task automatic run_one(input int a, input int b, input bit expect_sat);
    int exp_beta;
    int t_out;
    @(negedge clk);
    ia = q15_t'(a); ib = q15_t'(b); in_valid = 1;
    @(negedge clk);
    in_valid = 0;
    t_out = 0;
    while (out_valid !== 1'b1 && t_out < 10) begin @(negedge clk); t_out++; end
    exp_beta = model_beta(a, b);
    if (out_valid !== 1'b1) begin
      $display("  MISMATCH out_valid not set for a=%0d b=%0d", a, b); errors++;
    end
    if (ialpha !== q15_t'(a)) begin
      $display("  MISMATCH alpha a=%0d got=%0d", a, ialpha); errors++;
    end
    if (int'(ibeta) != exp_beta) begin
      $display("  MISMATCH beta a=%0d b=%0d got=%0d exp=%0d",
               a, b, ibeta, exp_beta); errors++;
    end
    if (sat !== expect_sat) begin
      $display("  MISMATCH sat a=%0d b=%0d got=%b exp=%b",
               a, b, sat, expect_sat); errors++;
    end
    @(negedge clk);
  endtask

  int a, b;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // directed: zero, balanced 3-phase at theta=0 (ia=I, ib=-I/2 -> beta=0..)
    run_one(0, 0, 1'b0);
    run_one(16384, -8192, 1'b0);     // beta = (16384-16384)/sqrt3 = 0
    run_one(0, 16384, 1'b0);         // beta = 2*0.5/sqrt3 = 0.57735
    run_one(-16384, 8192, 1'b0);
    run_one(32767, -16384, 1'b0);

    // random in-range balanced vectors: |i| <= 0.55 keeps |beta| < 1
    for (int t = 0; t < 500; t++) begin
      a = $urandom_range(0, 36044) - 18022; // +/-0.55 FS
      b = $urandom_range(0, 36044) - 18022;
      run_one(a, b, model_sat(a, b));
      if (model_sat(a, b)) begin
        $display("  MISMATCH test gen produced sat case unexpectedly"); errors++;
      end
    end

    // sqrt(3)-scale stimulus: must saturate and flag
    run_one(32767, 32767, 1'b1);
    run_one(-32768, -32768, 1'b1);
    run_one(0, 32767, 1'b1);         // beta = 2/sqrt3 = 1.1547

    if (errors == 0) $display("TB_PASS: tb_clarke");
    else             $display("TB_FAIL: tb_clarke (%0d errors)", errors);
    $finish;
  end

endmodule
