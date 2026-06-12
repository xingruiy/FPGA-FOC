// ============================================================================
// tb_sincos_lut.sv - exhaustive check of the sin/cos LUT.
//
//  Streams all 2^16 angles back to back and compares both outputs against
//  double-precision sin/cos scaled to Q1.15. Required: |error| <= 1 LSB.
//  Also checks the out_valid latency pipe.
// ============================================================================
`timescale 1ns / 1ps

module tb_sincos_lut;
  import foc_pkg::*;

  localparam real PI = 3.14159265358979323846;
  localparam real TOL = 1.000001; // <= 1 LSB, epsilon for float fuzz

  logic   clk = 0;
  logic   rst_n = 0;
  logic   in_valid = 0;
  angle_t theta = '0;
  logic   out_valid;
  q15_t   sin_o, cos_o;

  sincos_lut dut (.*);

  always #5 clk = ~clk; // 100 MHz

  // input angle delayed to align with the 3-cycle DUT latency
  angle_t theta_d1, theta_d2, theta_d3;
  logic   v_d1 = 0, v_d2 = 0, v_d3 = 0;
  always_ff @(posedge clk) begin
    theta_d1 <= theta;    theta_d2 <= theta_d1; theta_d3 <= theta_d2;
    v_d1     <= in_valid; v_d2     <= v_d1;     v_d3     <= v_d2;
  end

  int  errors = 0;
  int  n_checked = 0;
  real exp_s, exp_c, err_s, err_c, max_err = 0.0;

  always_ff @(posedge clk) begin
    if (rst_n && out_valid !== v_d3) begin
      $display("  MISMATCH out_valid latency at %0t", $time);
      errors++;
    end
    if (out_valid) begin
      exp_s = $sin(2.0 * PI * theta_d3 / 65536.0) * 32768.0;
      exp_c = $cos(2.0 * PI * theta_d3 / 65536.0) * 32768.0;
      err_s = sin_o - exp_s; if (err_s < 0) err_s = -err_s;
      err_c = cos_o - exp_c; if (err_c < 0) err_c = -err_c;
      if (err_s > max_err) max_err = err_s;
      if (err_c > max_err) max_err = err_c;
      if (err_s > TOL || err_c > TOL) begin
        if (errors < 20)
          $display("  MISMATCH theta=%0d sin=%0d (exp %f) cos=%0d (exp %f)",
                   theta_d3, sin_o, exp_s, cos_o, exp_c);
        errors++;
      end
      n_checked++;
    end
  end

  initial begin
    repeat (4) @(posedge clk);
    rst_n = 1;
    @(posedge clk);

    // stream all angles back to back
    for (int a = 0; a < 65536; a++) begin
      theta    <= angle_t'(a);
      in_valid <= 1'b1;
      @(posedge clk);
    end
    in_valid <= 1'b0;
    repeat (5) @(posedge clk);

    $display("  checked %0d angles, max |error| = %f LSB", n_checked, max_err);
    if (errors == 0 && n_checked == 65536)
      $display("TB_PASS: tb_sincos_lut");
    else
      $display("TB_FAIL: tb_sincos_lut (%0d errors, %0d checked)",
               errors, n_checked);
    $finish;
  end

endmodule
