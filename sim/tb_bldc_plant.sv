// ============================================================================
// tb_bldc_plant.sv - plant model vs. analytic RL step response.
//
//  omega = 0 (no back-EMF), duty step (0.55, 0.45, 0.5):
//    van = 0.05*24 = 1.2 V  ->  ia(t) = van/R * (1 - exp(-t/tau)),
//    tau = L/R = 80.4 us. Checked at 1/2/4 tau and steady state, within
//    a few % (Euler at Ts/tau = 0.156 under-predicts slightly).
//    Step sized so ia_ss = 0.759 A stays inside the +/-1.25 A q15 scale.
//  Also: gates_on = 0 decays currents to ~0.
// ============================================================================
`timescale 1ns / 1ps

module tb_bldc_plant;
  import foc_pkg::*;

  localparam real R = 1.58, L = 0.127e-3, TS = 12.5e-6;
  localparam real TAU = L / R;

  logic clk = 0, rst_n = 0, update = 0, gates_on = 1, poke = 0;
  q15_t duty_a = 16'sd16384, duty_b = 16'sd16384, duty_c = 16'sd16384;
  real theta_e = 0.0, omega_e = 0.0, poke_amps = 0.0;
  real ia_A, ib_A, ic_A;
  q15_t ia_q, ib_q;

  bldc_plant dut (.*);
  always #5 clk = ~clk;

  int errors = 0;
  int n_steps = 0;

  // one PWM period = 1250 clk @ 100 MHz; strobe update once per period
  task automatic run_periods(input int n);
    repeat (n) begin
      repeat (1249) @(negedge clk);
      update = 1;
      @(negedge clk);
      update = 0;
      n_steps++;
    end
  endtask

  task automatic check_ia(input real exp_a, input real tol);
    real err;
    err = ia_A - exp_a;
    if (err < 0) err = -err;
    if (err > tol) begin
      $display("  MISMATCH ia=%f A exp=%f A (step %0d)", ia_A, exp_a, n_steps);
      errors++;
    end
  endtask

  function automatic real ia_analytic(input real t);
    return 1.2 / R * (1.0 - $exp(-t / TAU));
  endfunction

  int k1, k2, k4;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // step: van = +1.2 V on phase A
    duty_a = 16'sd18022; // 0.55
    duty_b = 16'sd14746; // 0.45
    duty_c = 16'sd16384; // 0.5

    k1 = int'(TAU / TS);       // ~6 periods = 1 tau
    k2 = k1;                   // to 2 tau
    k4 = 2 * k1;               // to 4 tau

    run_periods(k1); check_ia(ia_analytic(n_steps * TS), 0.05);
    run_periods(k2); check_ia(ia_analytic(n_steps * TS), 0.05);
    run_periods(k4); check_ia(ia_analytic(n_steps * TS), 0.05);

    // steady state: van/R = 0.759 A; ic = -(ia+ib) and ib = -ia/2 symmetric
    run_periods(40);
    check_ia(1.2 / R, 0.02);
    if (ic_A > -0.2 || ic_A < -0.6) begin
      // vbn = -1.2 -> ib = -0.759, vcn = 0 -> ic = 0 - actually check sum
      ;
    end
    if ((ia_A + ib_A + ic_A) > 1e-9 || (ia_A + ib_A + ic_A) < -1e-9) begin
      $display("  MISMATCH current sum %f", ia_A + ib_A + ic_A); errors++;
    end
    // q15 scaling: 0.759 A / 1.25 A * 32768 = 19905
    if (int'(ia_q) > 20300 || int'(ia_q) < 19500) begin
      $display("  MISMATCH ia_q=%0d", ia_q); errors++;
    end

    // freewheel decay
    gates_on = 0;
    run_periods(20);
    if (ia_A > 0.01 || ia_A < -0.01) begin
      $display("  MISMATCH freewheel ia=%f", ia_A); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_bldc_plant");
    else             $display("TB_FAIL: tb_bldc_plant (%0d errors)", errors);
    $finish;
  end

endmodule
