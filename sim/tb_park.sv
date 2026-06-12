// ============================================================================
// tb_park.sv - golden-vector test of the Park transform.
//
//  Expected values are computed integer-exact in the TB (same rounding rule,
//  independent coding). Directed angle cases plus random vectors; saturation
//  flagged exactly when the rounded result leaves Q1.15.
// ============================================================================
`timescale 1ns / 1ps

module tb_park;
  import foc_pkg::*;

  localparam real PI = 3.14159265358979323846;

  logic clk = 0, rst_n = 0, in_valid = 0;
  q15_t ialpha, ibeta, sin_t, cos_t;
  logic out_valid, sat;
  q15_t id, iq;

  park dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  function automatic longint rnd15(input longint acc);
    return (acc + 16384) >>> 15;
  endfunction

  function automatic int clamp16(input longint x);
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return int'(x);
  endfunction

  task automatic run_one(input int a, input int b, input int s, input int c);
    longint d_acc, q_acc, d_exp, q_exp;
    bit exp_sat;
    int t_out;
    d_acc = longint'(a) * c + longint'(b) * s;
    q_acc = longint'(b) * c - longint'(a) * s;
    d_exp = rnd15(d_acc);
    q_exp = rnd15(q_acc);
    exp_sat = (d_exp > 32767) || (d_exp < -32768) ||
              (q_exp > 32767) || (q_exp < -32768);

    @(negedge clk);
    ialpha = q15_t'(a); ibeta = q15_t'(b);
    sin_t  = q15_t'(s); cos_t = q15_t'(c);
    in_valid = 1;
    @(negedge clk);
    in_valid = 0;
    t_out = 0;
    while (out_valid !== 1'b1 && t_out < 10) begin @(negedge clk); t_out++; end
    if (out_valid !== 1'b1) begin
      $display("  MISMATCH out_valid not set"); errors++;
    end
    if (int'(id) != clamp16(d_exp) || int'(iq) != clamp16(q_exp)) begin
      $display("  MISMATCH a=%0d b=%0d s=%0d c=%0d: id=%0d/%0d iq=%0d/%0d",
               a, b, s, c, id, clamp16(d_exp), iq, clamp16(q_exp));
      errors++;
    end
    if (sat !== exp_sat) begin
      $display("  MISMATCH sat a=%0d b=%0d s=%0d c=%0d got=%b exp=%b",
               a, b, s, c, sat, exp_sat); errors++;
    end
    @(negedge clk);
  endtask

  int a, b, s, c;
  real th;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // directed: theta = 0 -> id = alpha, iq = beta (cos = 0.99997)
    run_one(16384, 8192, 0, 32767);
    // theta = 90 deg -> id = beta, iq = -alpha
    run_one(16384, 8192, 32767, 0);
    // theta = 45 deg, balanced
    run_one(16384, 16384, 23170, 23170);
    // zero vector
    run_one(0, 0, 23170, 23170);

    // random in-range vectors at random angles
    for (int t = 0; t < 500; t++) begin
      a  = $urandom_range(0, 45874) - 22937; // +/-0.7
      b  = $urandom_range(0, 45874) - 22937;
      th = 2.0 * PI * $urandom_range(0, 65535) / 65536.0;
      s  = int'($sin(th) * 32767.0);
      c  = int'($cos(th) * 32767.0);
      run_one(a, b, s, c);
    end

    // sqrt(2)-scale saturation: full-scale vector at 45 deg
    run_one(32767, 32767, 23170, 23170);
    run_one(-32768, -32768, 23170, 23170);

    if (errors == 0) $display("TB_PASS: tb_park");
    else             $display("TB_FAIL: tb_park (%0d errors)", errors);
    $finish;
  end

endmodule
