// ============================================================================
// tb_dq_limit.sv - tests for foc_pkg::isqrt32 and foc_pkg::dq_limit.
//
//  isqrt32: floor-sqrt property r*r <= x < (r+1)*(r+1) on corners + random.
//  dq_limit: directed cases on / inside / beyond the circle, vd priority,
//  sign preservation, and the clamped flag for anti-windup.
// ============================================================================
`timescale 1ns / 1ps

module tb_dq_limit;
  import foc_pkg::*;

  int errors = 0;

  task automatic check_sqrt(input logic [31:0] x);
    longint r;
    r = longint'(isqrt32(x));
    if (r * r > x || (r + 1) * (r + 1) <= x) begin
      $display("  MISMATCH isqrt32(%0d) = %0d", x, r);
      errors++;
    end
  endtask

  task automatic check_lim(input int vd, input int vq, input int vmax,
                           input int exp_vd, input int exp_vq,
                           input bit exp_cl);
    dq_lim_t r;
    r = dq_limit(q15_t'(vd), q15_t'(vq), q15_t'(vmax));
    if (int'(r.vd) != exp_vd || int'(r.vq) != exp_vq
        || r.clamped !== exp_cl) begin
      $display("  MISMATCH dq_limit(%0d,%0d,%0d) = (%0d,%0d,%b) exp (%0d,%0d,%b)",
               vd, vq, vmax, r.vd, r.vq, r.clamped, exp_vd, exp_vq, exp_cl);
      errors++;
    end
  endtask

  // invariant-style random check
  task automatic check_lim_rand(input int vd, input int vq, input int vmax);
    dq_lim_t r;
    longint mag2, max2, in2;
    r    = dq_limit(q15_t'(vd), q15_t'(vq), q15_t'(vmax));
    mag2 = longint'(r.vd) * r.vd + longint'(r.vq) * r.vq;
    max2 = longint'(vmax) * vmax;
    in2  = longint'(vd) * vd + longint'(vq) * vq;
    if (mag2 > max2) begin
      $display("  MISMATCH |out|>vmax: (%0d,%0d,%0d)", vd, vq, vmax); errors++;
    end
    if (in2 <= max2 && (int'(r.vd) != vd || int'(r.vq) != vq || r.clamped)) begin
      $display("  MISMATCH in-range modified: (%0d,%0d,%0d)", vd, vq, vmax);
      errors++;
    end
    if (in2 > max2 && !r.clamped) begin
      $display("  MISMATCH clamped flag missing: (%0d,%0d,%0d)", vd, vq, vmax);
      errors++;
    end
    // vd priority: vd only ever range-clamped, never scaled
    if (int'(r.vd) != ((vd > vmax) ? vmax : (vd < -vmax) ? -vmax : vd)) begin
      $display("  MISMATCH vd priority: (%0d,%0d,%0d) -> vd=%0d",
               vd, vq, vmax, r.vd);
      errors++;
    end
    if ((vq < 0) != (r.vq < 0) && r.vq != 0) begin
      $display("  MISMATCH vq sign: (%0d,%0d,%0d) -> vq=%0d",
               vd, vq, vmax, r.vq);
      errors++;
    end
  endtask

  localparam int VMAX = 18918; // 1/sqrt(3) in Q1.15

  initial begin
    // ---- isqrt32 ----------------------------------------------------
    check_sqrt(0);  check_sqrt(1);  check_sqrt(2);  check_sqrt(3);
    check_sqrt(4);  check_sqrt(15); check_sqrt(16); check_sqrt(17);
    check_sqrt(32'h3FFF_FFFF); check_sqrt(32'h4000_0000);
    check_sqrt(32'hFFFF_FFFF);
    for (int t = 0; t < 2000; t++) check_sqrt($urandom());

    // ---- dq_limit directed -------------------------------------------
    // well inside: untouched
    check_lim(1000, 2000, VMAX, 1000, 2000, 1'b0);
    check_lim(-5000, -5000, VMAX, -5000, -5000, 1'b0);
    check_lim(0, 0, VMAX, 0, 0, 1'b0);
    // vd beyond vmax: vd clamps, vq collapses to 0
    check_lim(20000, 5000, VMAX, VMAX, 0, 1'b1);
    check_lim(-20000, -5000, VMAX, -VMAX, 0, 1'b1);
    // vq beyond, vd = 0: vq clamps to vmax
    check_lim(0, 30000, VMAX, 0, VMAX, 1'b1);
    check_lim(0, -30000, VMAX, 0, -VMAX, 1'b1);
    // vd priority at 45 deg overload: vd kept, vq = sqrt(vmax^2 - vd^2)
    // sqrt(18918^2 - 15000^2) = sqrt(132890724) = 11527 (floor)
    check_lim(15000, 15000, VMAX, 15000, 11527, 1'b1);
    check_lim(15000, -15000, VMAX, 15000, -11527, 1'b1);
    // exactly on the circle: untouched
    check_lim(VMAX, 0, VMAX, VMAX, 0, 1'b0);
    check_lim(0, -VMAX, VMAX, 0, -VMAX, 1'b0);

    // ---- dq_limit random invariants ------------------------------------
    for (int t = 0; t < 5000; t++) begin
      check_lim_rand($urandom_range(0, 65535) - 32768,
                     $urandom_range(0, 65535) - 32768,
                     $urandom_range(1000, 30000));
    end

    if (errors == 0) $display("TB_PASS: tb_dq_limit");
    else             $display("TB_FAIL: tb_dq_limit (%0d errors)", errors);
    $finish;
  end

endmodule
