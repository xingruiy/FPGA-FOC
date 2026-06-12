// ============================================================================
// tb_foc_top.sv - closed-loop system test.
//
//  Assembly: TB-host uart_tx ──UART──> uart_rx -> cmd_telemetry ->
//  foc_core -> pwm_gen ──duties──> bldc_plant (RL + back-EMF, 24 V,
//  idealized angle source) ──currents @ cnt_peak──> current_offset_cal
//  -> foc_core.  The XADC itself is bypassed (verified standalone in
//  tb_xadc_iface; its analog stimulus file cannot close a loop), as is
//  the hall estimator (idealized angle per the plan's non-goals).
//
//  Safe state (the foc_top equation, combinational between pwm_gen and
//  the "pins"):  oe = enable & nfault & ~ocp_trip & ~wd_timeout.
//
//  Checks:
//   1. UART-injected iq_ref step: iq tracks (rise within ~1 ms for the
//      ~1.5 kHz design), id -> 0
//   2. saturating condition (high back-EMF): no windup - after stepping
//      back down, iq recovers without a large overshoot
//   3. forced overcurrent (plant poke): ocp_trip latches, gates die
//   4. UART silence: watchdog fires, iq_ref ramps to 0, gates die
//   5. nFAULT injection: gates low combinationally (same cycle)
// ============================================================================
`timescale 1ns / 1ps

module tb_foc_top;
  import foc_pkg::*;

  localparam int BAUD_DIV = 20;
  localparam int WD_CYC   = 1_500_000; // 15 ms
  localparam real PI2 = 6.28318530717958647;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  int errors = 0;

  // ---- host-side UART ---------------------------------------------------
  logic [7:0] h_data;
  logic h_valid = 0, h_ready, h_line;
  uart_tx #(.BAUD_DIV(BAUD_DIV)) u_host_tx
    (.clk, .rst_n, .tx_data(h_data), .tx_valid(h_valid),
     .tx_ready(h_ready), .tx(h_line));

  // ---- DUT-side UART + command/telemetry --------------------------------
  logic [7:0] rxb; logic rxb_v, rx_ferr;
  uart_rx #(.BAUD_DIV(BAUD_DIV)) u_rx
    (.clk, .rst_n, .rx(h_line), .rx_data(rxb), .rx_valid(rxb_v),
     .frame_err(rx_ferr));

  logic [7:0] txb; logic txb_v, txb_rdy;
  angle_t theta_codes;
  logic enable, cal_we, ocal_start, wd_timeout, ol_cmd;
  q15_t vq_ol_cmd;
  logic signed [15:0] ol_speed_cmd;
  q15_t iq_ref;
  logic signed [15:0] kp, ki;
  logic [3:0] cal_addr;
  angle_t cal_data;
  q15_t id_meas, iq_meas;
  logic signed [15:0] omega_tel = 0;

  cmd_telemetry #(.WD_CYC(WD_CYC), .TELEM_CYC(300_000),
                  .RAMP_INTERVAL(256), .RAMP_STEP(256)) u_cmd (
    .clk, .rst_n,
    .rx_data(rxb), .rx_valid(rxb_v),
    .tx_data(txb), .tx_valid(txb_v), .tx_ready(txb_rdy),
    .enable, .iq_ref, .kp, .ki,
    .cal_we, .cal_addr, .cal_data, .offset_cal_start(ocal_start),
    .ol_mode(ol_cmd), .vq_ol(vq_ol_cmd), .ol_speed(ol_speed_cmd),
    .wd_timeout,
    .id_meas, .iq_meas, .theta(theta_codes), .omega(omega_tel),
    .fault_flags('0), .status_flags('0), .err_flags('0));

  uart_tx #(.BAUD_DIV(BAUD_DIV)) u_dut_tx
    (.clk, .rst_n, .tx_data(txb), .tx_valid(txb_v), .tx_ready(txb_rdy),
     .tx());

  // ---- rotor (idealized angle source) -------------------------------------
  real theta_e = 0.0, omega_e = 500.0; // rad/s electrical
  always @(posedge clk) begin
    theta_e <= theta_e + omega_e * 10.0e-9;
    if (theta_e > PI2) theta_e <= theta_e + omega_e * 10.0e-9 - PI2;
  end
  assign theta_codes = angle_t'(int'(theta_e / PI2 * 65536.0));

  // ---- safe state (foc_top equation) -------------------------------------
  logic nfault = 1, ocp_trip;
  logic oe_all;
  assign oe_all = enable && nfault && !ocp_trip && !wd_timeout;

  // ---- PWM + combinational gate kill --------------------------------------
  q15_t duty_a, duty_b, duty_c;
  logic p_ah, p_al, p_bh, p_bl, p_ch, p_cl;
  logic [$clog2(PWM_ARR+1)-1:0] cnt;
  logic cnt_peak, update;

  pwm_gen u_pwm (
    .clk, .rst_n, .en(1'b1), .oe({3{oe_all}}),
    .duty_a, .duty_b, .duty_c,
    .pwm_ah(p_ah), .pwm_al(p_al), .pwm_bh(p_bh), .pwm_bl(p_bl),
    .pwm_ch(p_ch), .pwm_cl(p_cl),
    .cnt, .cnt_peak, .update);

  logic g_ah, g_al, g_bh, g_bl, g_ch, g_cl; // "pins"
  assign g_ah = p_ah & oe_all;  assign g_al = p_al & oe_all;
  assign g_bh = p_bh & oe_all;  assign g_bl = p_bl & oe_all;
  assign g_ch = p_ch & oe_all;  assign g_cl = p_cl & oe_all;

  // ---- plant ---------------------------------------------------------------
  logic poke = 0;
  real poke_amps = 0.0;
  real ia_A, ib_A, ic_A;
  q15_t ia_q, ib_q;

  bldc_plant u_plant (
    .clk, .rst_n, .update, .gates_on(oe_all),
    .duty_a, .duty_b, .duty_c,
    .theta_e, .omega_e, .poke, .poke_amps,
    .ia_A, .ib_A, .ic_A, .ia_q, .ib_q);

  // ---- sampling at cnt_peak -> offset cal -> foc_core ----------------------
  logic samp_v;
  q15_t ia_s, ib_s;
  always_ff @(posedge clk) begin
    samp_v <= cnt_peak; // 1-cycle "ADC" latency
    if (cnt_peak) begin
      ia_s <= ia_q;
      ib_s <= ib_q;
    end
  end

  logic cal_done, oc_ov;
  q15_t ia_c, ib_c, ic_c;
  current_offset_cal u_ocal (
    .clk, .rst_n, .in_valid(samp_v), .ia_raw(ia_s), .ib_raw(ib_s),
    .cal_start(ocal_start), .cal_done, .cal_busy(),
    .out_valid(oc_ov), .ia(ia_c), .ib(ib_c), .ic(ic_c));

  q15_t fduty_a, fduty_b, fduty_c;
  logic sat_any;
  foc_core u_core (
    .clk, .rst_n, .en(enable), .ol_mode(1'b0), .vd_ol(16'sd0), .vq_ol(16'sd0),
    .cal_active(1'b0),
    .sample_valid(oc_ov), .ia(ia_c), .ib(ib_c), .ic(ic_c),
    .theta(theta_codes),
    .iq_ref, .kp, .ki,
    .duty_a(fduty_a), .duty_b(fduty_b), .duty_c(fduty_c),
    .id_meas, .iq_meas, .ocp_trip, .sat_any);

  assign duty_a = fduty_a;
  assign duty_b = fduty_b;
  assign duty_c = fduty_c;

  // ---- host text-line sender (ASCII protocol) -------------------------------
  task automatic hbyte(input byte b);
    @(negedge clk);
    while (!h_ready) @(negedge clk);
    h_data = b; h_valid = 1;
    @(negedge clk);
    h_valid = 0;
  endtask

  task automatic htext(input string s);
    for (int i = 0; i < s.len(); i++) hbyte(byte'(s[i]));
    hbyte(8'h0A); // LF terminates the line
    repeat (12 * BAUD_DIV) @(negedge clk); // drain line
  endtask

  task automatic set_iq(input int v);
    htext($sformatf("iq %0d", v));
  endtask

  task automatic wait_periods(input int n);
    repeat (n) begin
      @(negedge clk);
      while (!update) @(negedge clk);
    end
  endtask

  int rise_p;
  q15_t iq_max;

  bit dbg_on = 0;
  int dbg_k = 0;
  always @(posedge clk) begin
    if (dbg_on && update) begin
      dbg_k++;
      if (dbg_k % 10 == 0)
        $display("  DBG k=%0d iq=%0d id=%0d uq=%0d ud=%0d vq=%0d vd=%0d accq=%0d",
                 dbg_k, iq_meas, id_meas, u_core.u_q, u_core.u_d,
                 u_core.vq_lim, u_core.vd_lim, u_core.u_pi_q.acc >>> 12);
    end
  end

  initial begin
    repeat (5) @(negedge clk);
    rst_n = 1;

    // gains + enable over UART
    htext("kp 850");
    htext("ki 130");
    htext("enable 1");
    if (!enable || kp != 16'sd850 || ki != 16'sd130) begin
      $display("  MISMATCH uart config enable=%b kp=%0d ki=%0d",
               enable, kp, ki);
      errors++;
    end

    // ---- 1: iq_ref step, track + rise time + id -> 0 ------------------
    set_iq(5243); // 0.16 FS = 0.2 A
    rise_p = 0;
    while (iq_meas < 16'sd4719 && rise_p < 200) begin // 90% of ref
      wait_periods(1);
      rise_p++;
    end
    if (rise_p >= 200) begin
      $display("  MISMATCH iq never reached 90%% (iq=%0d)", iq_meas); errors++;
    end else if (rise_p > 80) begin // > 1 ms
      $display("  MISMATCH slow rise: %0d periods", rise_p); errors++;
    end
    wait_periods(160); // settle 2 ms
    if (iq_meas > 5243 + 800 || iq_meas < 5243 - 800) begin
      $display("  MISMATCH iq_meas=%0d ref=5243", iq_meas); errors++;
    end
    if (id_meas > 800 || id_meas < -800) begin
      $display("  MISMATCH id_meas=%0d", id_meas); errors++;
    end

    // ---- 2: saturating condition, then recovery without windup ---------
    omega_e = 900.0;   // BEMF_q = 13.4 V, vmax*24 = 13.9 V: only ~0.13 FS
    set_iq(9830);      // of headroom left -> 0.3 FS is unreachable, vq clamps
    wait_periods(160); // integrator would wind up here without protection
    // step DOWN while still saturated: anti-windup must let iq fall to the
    // new reference promptly (a wound-up integrator would hold it high for
    // ~80 periods)
    // keep the unreachable reference and ramp the speed back down: as the
    // back-EMF falls the reference becomes reachable. A wound-up integrator
    // would overshoot far beyond the reference here; with anti-windup iq
    // just rises to the reference and stays.
    iq_max = -32768;
    for (int k = 0; k < 130; k++) begin
      wait_periods(1);
      if (omega_e > 500.0) omega_e = omega_e - 4.0;
      if (iq_meas > iq_max) iq_max = iq_meas;
      if (ocp_trip) begin
        // a wound-up integrator would drive ~2 A here and trip; with
        // anti-windup the current stays well inside the trip level
        $display("  MISMATCH OCP trip during desaturation (windup!)");
        errors++;
      end
    end
    omega_e = 500.0;
    wait_periods(60);
    if (iq_max > 9830 + 6554) begin // transient < 0.2 FS above ref
      $display("  MISMATCH windup overshoot iq_max=%0d ref=9830", iq_max);
      errors++;
    end
    if (iq_meas > 9830 + 800 || iq_meas < 9830 - 800) begin
      $display("  MISMATCH iq=%0d ref=9830 after desaturation", iq_meas);
      errors++;
    end
    // normal step-down at a comfortable operating point settles quickly
    set_iq(2621);
    wait_periods(40);
    if (iq_meas > 2621 + 800 || iq_meas < 2621 - 800) begin
      $display("  MISMATCH post-saturation iq=%0d ref=2621", iq_meas);
      errors++;
    end

    // ---- 3: forced overcurrent ------------------------------------------
    @(negedge clk);
    poke_amps = 1.2; poke = 1;
    @(negedge clk);
    poke = 0;
    wait_periods(3);
    if (!ocp_trip) begin
      $display("  MISMATCH ocp_trip not set after poke"); errors++;
    end
    #1;
    if (g_ah || g_al || g_bh || g_bl || g_ch || g_cl) begin
      $display("  MISMATCH gates alive after OCP"); errors++;
    end
    // re-arm: enable 0 -> 1
    htext("enable 0");
    htext("enable 1");
    set_iq(2621);
    wait_periods(160);
    if (ocp_trip) begin
      $display("  MISMATCH ocp_trip stuck after re-enable"); errors++;
    end
    if (iq_meas > 2621 + 800 || iq_meas < 2621 - 800) begin
      $display("  MISMATCH iq after re-arm=%0d", iq_meas); errors++;
    end

    // ---- 4: watchdog ramp-down on UART silence ----------------------------
    begin
      int t_out = 0;
      while (!wd_timeout && t_out < 2 * WD_CYC) begin
        @(negedge clk); t_out++;
      end
      if (!wd_timeout) begin
        $display("  MISMATCH watchdog never fired"); errors++;
      end
    end
    repeat (200_000) @(negedge clk); // let the ramp finish
    if (iq_ref != 0) begin
      $display("  MISMATCH iq_ref=%0d after watchdog ramp", iq_ref); errors++;
    end
    if (g_ah || g_al || g_bh || g_bl || g_ch || g_cl) begin
      $display("  MISMATCH gates alive during wd_timeout"); errors++;
    end

    // recover host link
    htext("ping");
    if (wd_timeout) begin
      $display("  MISMATCH wd_timeout stuck"); errors++;
    end

    // ---- 5: nFAULT combinational kill -------------------------------------
    set_iq(2621);
    wait_periods(60);
    // wait until some gate is high, then drop nfault and check same-cycle
    @(negedge clk);
    while (!(g_ah || g_bh || g_ch)) @(negedge clk);
    nfault = 0;
    #1;
    if (g_ah || g_al || g_bh || g_bl || g_ch || g_cl) begin
      $display("  MISMATCH gates alive with nFAULT low (comb path)"); errors++;
    end
    nfault = 1;

    if (errors == 0) $display("TB_PASS: tb_foc_top");
    else             $display("TB_FAIL: tb_foc_top (%0d errors)", errors);
    $finish;
  end

endmodule
