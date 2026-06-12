// ============================================================================
// tb_inv_park.sv - golden-vector test of the inverse Park transform.
// ============================================================================
`timescale 1ns / 1ps

module tb_inv_park;
  import foc_pkg::*;

  localparam real PI = 3.14159265358979323846;

  logic clk = 0, rst_n = 0, in_valid = 0;
  q15_t vd, vq, sin_t, cos_t;
  logic out_valid, sat;
  q15_t valpha, vbeta;

  inv_park dut (.*);
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

  task automatic run_one(input int d, input int q, input int s, input int c);
    longint a_exp, b_exp;
    bit exp_sat;
    int t_out;
    a_exp = rnd15(longint'(d) * c - longint'(q) * s);
    b_exp = rnd15(longint'(d) * s + longint'(q) * c);
    exp_sat = (a_exp > 32767) || (a_exp < -32768) ||
              (b_exp > 32767) || (b_exp < -32768);

    @(negedge clk);
    vd = q15_t'(d); vq = q15_t'(q);
    sin_t = q15_t'(s); cos_t = q15_t'(c);
    in_valid = 1;
    @(negedge clk);
    in_valid = 0;
    t_out = 0;
    while (out_valid !== 1'b1 && t_out < 10) begin @(negedge clk); t_out++; end
    if (out_valid !== 1'b1) begin
      $display("  MISMATCH out_valid not set"); errors++;
    end
    if (int'(valpha) != clamp16(a_exp) || int'(vbeta) != clamp16(b_exp)) begin
      $display("  MISMATCH d=%0d q=%0d s=%0d c=%0d: va=%0d/%0d vb=%0d/%0d",
               d, q, s, c, valpha, clamp16(a_exp), vbeta, clamp16(b_exp));
      errors++;
    end
    if (sat !== exp_sat) begin
      $display("  MISMATCH sat got=%b exp=%b", sat, exp_sat); errors++;
    end
    @(negedge clk);
  endtask

  int d, q, s, c;
  real th;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    run_one(16384, 0, 0, 32767);          // theta=0: va=vd
    run_one(0, 16384, 32767, 0);          // theta=90: va=-vq, vb=0
    run_one(16384, -8192, 23170, 23170);  // 45 deg mixed
    run_one(0, 0, 23170, 23170);

    for (int t = 0; t < 500; t++) begin
      d  = $urandom_range(0, 45874) - 22937;
      q  = $urandom_range(0, 45874) - 22937;
      th = 2.0 * PI * $urandom_range(0, 65535) / 65536.0;
      s  = int'($sin(th) * 32767.0);
      c  = int'($cos(th) * 32767.0);
      run_one(d, q, s, c);
    end

    // saturating cases
    run_one(32767, -32768, 23170, 23170);
    run_one(-32768, 32767, 23170, 23170);

    if (errors == 0) $display("TB_PASS: tb_inv_park");
    else             $display("TB_FAIL: tb_inv_park (%0d errors)", errors);
    $finish;
  end

endmodule
