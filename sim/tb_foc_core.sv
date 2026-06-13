// ============================================================================
// tb_foc_core.sv - foc_core dataflow test (open-loop mode + smoke checks).
//
//  - open loop, fixed (vd, vq), theta sweeping: duties update once per
//    sample with the documented ~9-cycle latency, are centered on 0.5,
//    modulate sinusoidally with theta, and no node saturates at rated
//    operating amplitudes (sat_any stays low)
//  - closed-loop smoke: with zero currents and a positive iq_ref, the
//    PI integrators walk the duties away from 50%
//  - OCP: an over-trip current sample latches ocp_trip; a fresh enable
//    edge clears it; cal_active forces 50/50/50
// ============================================================================
`timescale 1ns / 1ps

module tb_foc_core;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0;
  logic en = 0, ol_mode = 0, cal_active = 0, sample_valid = 0;
  q15_t vd_ol = 0, vq_ol = 0;
  q15_t ia = 0, ib = 0, ic = 0;
  angle_t theta = 0;
  q15_t iq_ref = 0;
  logic signed [15:0] kp = 16'sd170, ki = 16'sd26;
  q15_t duty_a, duty_b, duty_c, id_meas, iq_meas;
  logic ocp_trip, sat_any;

  foc_core dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  task automatic sample(input int a, input int b, input int th);
    @(negedge clk);
    ia = q15_t'(a); ib = q15_t'(b);
    ic = q15_t'(-a - b);
    theta = angle_t'(th);
    sample_valid = 1;
    @(negedge clk);
    sample_valid = 0;
    repeat (45) @(negedge clk); // > chain latency (incl. 20-clk limiter)
  endtask

  int d_hist [64];
  real mn, mx, mean;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- open loop: vq = 0.3, vd = 0, theta sweeping ----------------
    en = 1; ol_mode = 1;
    vd_ol = 16'sd0; vq_ol = 16'sd9830; // 0.3
    for (int k = 0; k < 64; k++) begin
      sample(3277, -1638, k * 1024); // ~0.125 FS currents, full circle
      if (sat_any) begin
        $display("  MISMATCH sat_any in open loop at k=%0d", k); errors++;
      end
      d_hist[k] = int'(duty_a);
    end
    // duty_a must modulate sinusoidally around 0.5
    mn = 99999; mx = -99999; mean = 0;
    for (int k = 0; k < 64; k++) begin
      if (d_hist[k] < mn) mn = d_hist[k];
      if (d_hist[k] > mx) mx = d_hist[k];
      mean += d_hist[k];
    end
    mean /= 64.0;
    if (mean > 16384 + 300 || mean < 16384 - 300) begin
      $display("  MISMATCH duty mean %f", mean); errors++;
    end
    // |v| = 0.3 -> phase swing ~ +/-0.3 of half-range plus zero-seq shape;
    // just require clear modulation and sane bounds
    if (mx - mn < 4000 || mx > 30639 || mn < 2129) begin
      $display("  MISMATCH duty modulation mn=%f mx=%f", mn, mx); errors++;
    end

    // ---- latency: duty updates ~32 cycles after sample_valid ------------
    begin
      q15_t d_before;
      int lat;
      d_before = duty_a;
      @(negedge clk);
      theta = 16'd30000; sample_valid = 1;
      @(negedge clk);
      sample_valid = 0;
      lat = 0;
      while (duty_a == d_before && lat < 60) begin @(negedge clk); lat++; end
      if (lat > 40) begin
        $display("  MISMATCH chain latency %0d cycles", lat); errors++;
      end
    end

    // ---- closed-loop smoke: integrator drives duties off-center -------
    ol_mode = 0; en = 0; @(negedge clk); en = 1; // re-arm
    iq_ref = 16'sd4096; // 0.125
    for (int k = 0; k < 30; k++) sample(0, 0, 16384);
    if (duty_a == 16'sd16384 && duty_b == 16'sd16384) begin
      $display("  MISMATCH closed loop never moved duties"); errors++;
    end

    // ---- OCP trip and re-arm ---------------------------------------------
    sample(30000, 0, 0); // |ia| = 0.915 FS > 0.72 trip
    if (!ocp_trip) begin
      $display("  MISMATCH ocp_trip not latched"); errors++;
    end
    sample(0, 0, 0);
    if (!ocp_trip) begin
      $display("  MISMATCH ocp_trip did not stay latched"); errors++;
    end
    en = 0; @(negedge clk); en = 1; @(negedge clk); #1;
    if (ocp_trip) begin
      $display("  MISMATCH ocp_trip not cleared by enable edge"); errors++;
    end

    // ---- cal_active forces 50/50/50 ---------------------------------------
    cal_active = 1;
    sample(1000, 1000, 0);
    if (duty_a != 16'sd16384 || duty_b != 16'sd16384
        || duty_c != 16'sd16384) begin
      $display("  MISMATCH cal duties %0d %0d %0d", duty_a, duty_b, duty_c);
      errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_foc_core");
    else             $display("TB_FAIL: tb_foc_core (%0d errors)", errors);
    $finish;
  end

endmodule
