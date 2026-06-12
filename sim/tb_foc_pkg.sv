// ============================================================================
// tb_foc_pkg.sv - unit tests for the foc_pkg helper functions.
//
//  Checks saturation corners, round-half-up behavior and known products
//  for q15/q13 multiplies and the format conversions.
// ============================================================================
`timescale 1ns / 1ps

module tb_foc_pkg;
  import foc_pkg::*;

  int errors = 0;

  task automatic check(input string name,
                       input logic signed [31:0] got,
                       input logic signed [31:0] exp);
    if (got !== exp) begin
      $display("  MISMATCH %-28s got=%0d exp=%0d", name, got, exp);
      errors++;
    end
  endtask

  initial begin
    // ---- sat16 -----------------------------------------------------
    check("sat16(+40000)",  sat16(32'sd40000),  32'sd32767);
    check("sat16(-40000)",  sat16(-32'sd40000), -32'sd32768);
    check("sat16(+32767)",  sat16(32'sd32767),  32'sd32767);
    check("sat16(-32768)",  sat16(-32'sd32768), -32'sd32768);
    check("sat16(1234)",    sat16(32'sd1234),   32'sd1234);

    // ---- rnd_shr (round half up) ------------------------------------
    check("rnd_shr(3,1)",   rnd_shr(32'sd3, 1),  32'sd2);   //  1.5 ->  2
    check("rnd_shr(-3,1)",  rnd_shr(-32'sd3, 1), -32'sd1);  // -1.5 -> -1
    check("rnd_shr(5,2)",   rnd_shr(32'sd5, 2),  32'sd1);   //  1.25 -> 1
    check("rnd_shr(-5,2)",  rnd_shr(-32'sd5, 2), -32'sd1);  // -1.25 -> -1
    check("rnd_shr(6,2)",   rnd_shr(32'sd6, 2),  32'sd2);   //  1.5 ->  2

    // ---- q15_mul ----------------------------------------------------
    check("q15 0.5*0.5",    q15_mul(16'sd16384, 16'sd16384),  32'sd8192);
    check("q15 -0.5*0.5",   q15_mul(-16'sd16384, 16'sd16384), -32'sd8192);
    check("q15 min*min",    q15_mul(Q15_MIN, Q15_MIN),  32'sd32767); // sat
    check("q15 max*max",    q15_mul(Q15_MAX, Q15_MAX),  32'sd32766);
    check("q15 max*min",    q15_mul(Q15_MAX, Q15_MIN), -32'sd32767);
    check("q15 tie up",     q15_mul(16'sd1, 16'sd16384),   32'sd1);  // +0.5lsb
    check("q15 neg tie",    q15_mul(-16'sd1, 16'sd16384),  32'sd0);  // -0.5lsb

    // ---- q13_mul ----------------------------------------------------
    check("q13 1.0*1.0",    q13_mul(Q13_ONE, Q13_ONE),       32'sd8192);
    check("q13 2.0*2.0",    q13_mul(16'sd16384, 16'sd16384), 32'sd32767); // sat
    check("q13 -2*2",       q13_mul(-16'sd16384, 16'sd16384), -32'sd32768); // sat
    check("q13 1.5*2.0",    q13_mul(16'sd12288, 16'sd16384), 32'sd24576);

    // ---- q13_q15_mul --------------------------------------------------
    check("q13q15 1.0*0.5", q13_q15_mul(Q13_ONE, 16'sd16384), 32'sd4096);
    check("q13q15 3.9*0.5", q13_q15_mul(16'sd31949, 16'sd16384), 32'sd15975);
    check("q13q15 1.0*-1",  q13_q15_mul(Q13_ONE, Q15_MIN), -32'sd8192);

    // ---- conversions --------------------------------------------------
    check("q15->q13 max",   q15_to_q13(Q15_MAX),  32'sd8192);
    check("q15->q13 min",   q15_to_q13(Q15_MIN), -32'sd8192);
    check("q15->q13 0.5",   q15_to_q13(16'sd16384), 32'sd4096);
    check("q13->q15 1.0",   q13_to_q15(Q13_ONE),  32'sd32767); // sat
    check("q13->q15 -1.0",  q13_to_q15(-16'sd8192), -32'sd32768);
    check("q13->q15 0.122", q13_to_q15(16'sd1000), 32'sd4000);

    // ---- parameter sanity ---------------------------------------------
    check("PWM_ARR",        PWM_ARR,      32'sd625);
    check("DEADTIME_CYC",   DEADTIME_CYC, 32'sd20);

    if (errors == 0) $display("TB_PASS: tb_foc_pkg");
    else             $display("TB_FAIL: tb_foc_pkg (%0d errors)", errors);
    $finish;
  end

endmodule
