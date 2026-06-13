// ============================================================================
// tb_pi_controller.sv - PI controller vs. bit-exact golden model.
//
//  The golden model implements the documented difference equations
//  independently. Scenarios: pure P step, integrator ramp, random gain /
//  input sequences, and a windup test with an external output limiter -
//  after the limit releases, the output must come back without the huge
//  overshoot an unprotected integrator would show.
// ============================================================================
`timescale 1ns / 1ps

module tb_pi_controller;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0, clr = 0, strobe = 0;
  q15_t sp = 0, fb = 0, applied = 0;
  logic signed [15:0] kp = 0, ki = 0;
  logic out_valid;
  q15_t u;
  logic usat;

  pi_controller dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  // ---- golden model (independent coding of the documented equations) ----
  longint m_acc = 0;
  int     m_uprev = 0;
  localparam longint M_ACC_MAX = 64'd1 << 27;

  function automatic int clamp16(input longint x);
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return int'(x);
  endfunction

  function automatic int model_step(input int msp, input int mfb,
                                    input int mkp, input int mki,
                                    input int mapplied);
    int e;
    longint p, u32, corr;
    e    = clamp16(longint'(msp) - mfb);
    p    = longint'(mkp) * e;
    u32  = (p + m_acc + 2048) >>> 12;
    corr = (longint'(mapplied) - m_uprev) <<< 12;
    m_acc = m_acc + longint'(mki) * e + corr;
    if (m_acc >  M_ACC_MAX) m_acc =  M_ACC_MAX;
    if (m_acc < -M_ACC_MAX) m_acc = -M_ACC_MAX;
    m_uprev = clamp16(u32);
    return clamp16(u32);
  endfunction

  // ---- drive one strobe, wait for the 3-stage pipe, compare ----
  task automatic step(input int tsp, input int tfb, input int tapplied);
    int exp_u, t_out;
    exp_u = model_step(tsp, tfb, int'(kp), int'(ki), tapplied);
    @(negedge clk);
    sp = q15_t'(tsp); fb = q15_t'(tfb); applied = q15_t'(tapplied);
    strobe = 1;
    @(negedge clk);
    strobe = 0;
    t_out = 0;
    while (out_valid !== 1'b1 && t_out < 10) begin @(negedge clk); t_out++; end
    if (out_valid !== 1'b1) begin
      $display("  MISMATCH out_valid not set"); errors++;
    end
    if (int'(u) != exp_u) begin
      $display("  MISMATCH sp=%0d fb=%0d app=%0d: u=%0d exp=%0d",
               tsp, tfb, tapplied, u, exp_u);
      errors++;
    end
    @(negedge clk);
  endtask

  int app, lim, u_rel;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- pure P: kp = 1.0 -> u = err -------------------------------
    kp = 4096; ki = 0;
    step(8192, 0, 0);
    if (int'(u) != 8192) begin
      $display("  MISMATCH P step: u=%0d exp=8192", u); errors++;
    end
    step(-8192, 8192, int'(u)); // err = -0.5 -> u = -16384
    if (int'(u) != -16384) begin
      $display("  MISMATCH P step2: u=%0d exp=-16384", u); errors++;
    end

    // ---- integrator ramp: ki = 0.5, err = 0.25 -> du = 0.125/step ----
    rst_n = 0; m_acc = 0; m_uprev = 0; @(negedge clk); rst_n = 1;
    kp = 0; ki = 2048;
    step(8192, 0, 0);              // u = I_0 = 0
    step(8192, 0, int'(u));        // u = 4096
    if (int'(u) != 4096) begin
      $display("  MISMATCH I ramp: u=%0d exp=4096", u); errors++;
    end
    step(8192, 0, int'(u));        // u = 8192
    if (int'(u) != 8192) begin
      $display("  MISMATCH I ramp2: u=%0d exp=8192", u); errors++;
    end

    // ---- random sequences, applied = u (no external limiting) -------
    rst_n = 0; m_acc = 0; m_uprev = 0; @(negedge clk); rst_n = 1;
    app = 0;
    for (int t = 0; t < 300; t++) begin
      if (t % 50 == 0) begin
        kp = 16'($urandom_range(0, 8192));
        ki = 16'($urandom_range(0, 4096));
      end
      step($urandom_range(0, 32768) - 16384,
           $urandom_range(0, 32768) - 16384, app);
      app = int'(u);
    end

    // ---- windup: external limit 0.1 FS, then release ----------------
    rst_n = 0; m_acc = 0; m_uprev = 0; @(negedge clk); rst_n = 1;
    kp = 2048; ki = 1024; lim = 3277; app = 0;
    for (int t = 0; t < 40; t++) begin
      step(16384, 0, app);
      app = (int'(u) > lim) ? lim : ((int'(u) < -lim) ? -lim : int'(u));
    end
    // release: err -> 0; output must drop near/below the limit at once,
    // not unwind from a saturated integrator (which would sit at ~32767)
    step(0, 0, app);
    u_rel = int'(u);
    if (u_rel > lim + 4096) begin
      $display("  MISMATCH windup: post-release u=%0d (limit %0d)",
               u_rel, lim);
      errors++;
    end
    // and it must settle, not overshoot negative
    for (int t = 0; t < 10; t++) begin
      app = (int'(u) > lim) ? lim : ((int'(u) < -lim) ? -lim : int'(u));
      step(0, 0, app);
    end
    if (int'(u) < -1024) begin
      $display("  MISMATCH windup recovery overshoot: u=%0d", u); errors++;
    end

    // ---- clr: state held cleared, restart from zero ------------------
    kp = 0; ki = 2048;
    step(8192, 0, int'(u));
    step(8192, 0, int'(u));        // integrator non-zero now
    @(negedge clk);
    clr = 1;
    @(negedge clk);
    @(negedge clk);
    if (int'(u) != 0) begin
      $display("  MISMATCH clr: u=%0d exp=0", u); errors++;
    end
    clr = 0; m_acc = 0; m_uprev = 0;
    step(8192, 0, 0);              // first step after clr: u = I_0 = 0
    if (int'(u) != 0) begin
      $display("  MISMATCH post-clr restart: u=%0d exp=0", u); errors++;
    end
    step(8192, 0, int'(u));        // integrator ramps again from zero
    if (int'(u) != 4096) begin
      $display("  MISMATCH post-clr ramp: u=%0d exp=4096", u); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_pi_controller");
    else $display("TB_FAIL: tb_pi_controller (%0d errors)", errors);
    $finish;
  end

endmodule
