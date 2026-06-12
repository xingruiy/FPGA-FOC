// ============================================================================
// tb_transform_chain.sv - round-trip test of the full transform chain.
//
//  sincos_lut -> clarke -> park -> inv_park -> (TB inverse Clarke)
//  must recover the original (ia, ib) within a small quantization bound.
//  Also checks that no module raises its sat flag for in-range stimulus.
// ============================================================================
`timescale 1ns / 1ps

module tb_transform_chain;
  import foc_pkg::*;

  localparam real PI  = 3.14159265358979323846;
  localparam real TOL = 6.0; // LSB: 3 rounding stages + LUT amplitude error

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // --- sincos_lut ---
  logic   lut_iv = 0, lut_ov;
  angle_t theta;
  q15_t   sin_t, cos_t;
  sincos_lut u_lut (.clk, .rst_n, .in_valid(lut_iv), .theta,
                    .out_valid(lut_ov), .sin_o(sin_t), .cos_o(cos_t));

  // --- clarke ---
  logic ck_iv = 0, ck_ov, ck_sat;
  q15_t ia, ib, ialpha, ibeta;
  clarke u_ck (.clk, .rst_n, .in_valid(ck_iv), .ia, .ib,
               .out_valid(ck_ov), .ialpha, .ibeta, .sat(ck_sat));

  // --- park ---
  logic pk_iv = 0, pk_ov, pk_sat;
  q15_t id, iq;
  park u_pk (.clk, .rst_n, .in_valid(pk_iv),
             .ialpha, .ibeta, .sin_t, .cos_t,
             .out_valid(pk_ov), .id, .iq, .sat(pk_sat));

  // --- inv_park ---
  logic ip_iv = 0, ip_ov, ip_sat;
  q15_t valpha, vbeta;
  inv_park u_ip (.clk, .rst_n, .in_valid(ip_iv),
                 .vd(id), .vq(iq), .sin_t, .cos_t,
                 .out_valid(ip_ov), .valpha, .vbeta, .sat(ip_sat));

  int errors = 0;
  real a_rec, b_rec, err_a, err_b, max_err = 0.0;

  task automatic run_one(input int a_in, input int b_in, input int th);
    // 1) sin/cos for this angle
    @(negedge clk);
    theta = angle_t'(th); lut_iv = 1;
    @(negedge clk);
    lut_iv = 0;
    wait (lut_ov === 1'b1); @(negedge clk);

    // 2) clarke
    ia = q15_t'(a_in); ib = q15_t'(b_in); ck_iv = 1;
    @(negedge clk);
    ck_iv = 0;
    while (ck_ov !== 1'b1) @(negedge clk);
    if (ck_sat) begin $display("  MISMATCH clarke sat in-range"); errors++; end

    // 3) park (alpha/beta now registered)
    pk_iv = 1;
    @(negedge clk);
    pk_iv = 0;
    while (pk_ov !== 1'b1) @(negedge clk);
    if (pk_sat) begin $display("  MISMATCH park sat in-range"); errors++; end

    // 4) inv_park
    ip_iv = 1;
    @(negedge clk);
    ip_iv = 0;
    while (ip_ov !== 1'b1) @(negedge clk);
    if (ip_sat) begin $display("  MISMATCH inv_park sat in-range"); errors++; end

    // 5) TB inverse Clarke: a = alpha, b = (-alpha + sqrt(3)*beta)/2
    a_rec = real'(valpha);
    b_rec = (-real'(valpha) + $sqrt(3.0) * real'(vbeta)) / 2.0;
    err_a = a_rec - a_in; if (err_a < 0) err_a = -err_a;
    err_b = b_rec - b_in; if (err_b < 0) err_b = -err_b;
    if (err_a > max_err) max_err = err_a;
    if (err_b > max_err) max_err = err_b;
    if (err_a > TOL || err_b > TOL) begin
      $display("  MISMATCH roundtrip a=%0d b=%0d th=%0d: rec a=%f b=%f",
               a_in, b_in, th, a_rec, b_rec);
      errors++;
    end
  endtask

  int a, b, th;
  real mag, phi, al_r, be_r;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // directed corners
    run_one(16384, -8192, 0);
    run_one(16384, -8192, 16384);
    run_one(-16384, 8192, 49152);
    run_one(0, 0, 12345);

    // random in-range vectors: |i_vec| <= 0.9 keeps every node in range
    // (independent ia/ib would let the alpha-beta magnitude exceed 1.0)
    for (int t = 0; t < 300; t++) begin
      mag  = 0.9 * $urandom_range(0, 10000) / 10000.0;
      phi  = 2.0 * PI * $urandom_range(0, 65535) / 65536.0;
      al_r = mag * $cos(phi);
      be_r = mag * $sin(phi);
      a    = int'(al_r * 32768.0);
      b    = int'((-al_r + $sqrt(3.0) * be_r) / 2.0 * 32768.0);
      th   = $urandom_range(0, 65535);
      run_one(a, b, th);
    end

    $display("  max roundtrip error = %f LSB", max_err);
    if (errors == 0) $display("TB_PASS: tb_transform_chain");
    else $display("TB_FAIL: tb_transform_chain (%0d errors)", errors);
    $finish;
  end

endmodule
