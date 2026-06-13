// ============================================================================
// tb_hall_angle_est.sv - hall_decode + hall_angle_est (PLL observer)
// chained against an emulated rotor.
//
//  A real-valued rotor angle th_ref advances at programmable speed; hall
//  signals are generated from physical boundary angles b[0..5]. The
//  observer ticks once per emulated PWM period (1250 clks).
//
//  Checks (PLL semantics - the old "theta never crosses the next edge"
//  guard is gone by design; the PLL crosses boundaries softly):
//   - cold start: theta = center of the current sector before any edge
//   - convergence: after >= 12 edges at constant speed, tick-aligned
//     tracking error within tolerance (the observer integrates per period,
//     so theta is compared at the tick instants its consumers sample it;
//     a small systematic lag ~ omega * ~0.5 period is part of the budget)
//   - no snap: per-tick |dtheta - omega| bounded (soft corrections only)
//   - omega magnitude/sign at constant speed, both directions, including
//     a non-integer codes-per-period speed (Q16.16 fractional omega)
//   - unequal physical sectors with the IDENTITY table: bounded error
//     (the PLL's tolerance to uncalibrated hall placement)
//   - calibrated unequal sectors: tight tracking, both directions
//   - bounce reject: decoder-accepted edge pairs closer than MIN_EDGE_CYC
//     do not disturb theta/omega
//   - stale: no edge for TIMEOUT_CYC -> omega = 0, moving = 0, theta
//     frozen at the CENTER of the current sector (STM32 semantics)
//   - direction reversal: omega is re-measured fresh (no blend across the
//     reversal), re-locks within ~12 edges
// ============================================================================
`timescale 1ns / 1ps

module tb_hall_angle_est;
  import foc_pkg::*;

  localparam int DEB        = 8;
  localparam int TIMEOUT    = 300000; // 3 ms (sim economy; 100 ms in HW)
  localparam int MIN_EDGE   = 100;    // 1 us (50 us in HW)
  localparam int PERIOD     = 1250;   // clks per PWM period

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // ---- tick generator (free-running PWM-period strobe) -----------------
  logic tick = 0;
  int   tick_cnt = 0;
  always @(posedge clk) begin
    if (tick_cnt == PERIOD - 1) begin
      tick_cnt <= 0;
      tick     <= 1;
    end else begin
      tick_cnt <= tick_cnt + 1;
      tick     <= 0;
    end
  end

  // ---- DUTs: decode -> estimator -------------------------------------
  logic [2:0] hall_i = 3'b001;
  logic [2:0] sector;
  logic sector_valid, edge_strobe, dir, illegal;

  hall_decode #(.DEBOUNCE_CYC(DEB)) u_dec (.*);

  logic   cal_we = 0;
  logic [3:0] cal_addr = 0;
  angle_t cal_data = 0;
  angle_t theta;
  logic signed [15:0] omega;
  logic moving;

  hall_angle_est #(.TIMEOUT_CYC(TIMEOUT), .MIN_EDGE_CYC(MIN_EDGE),
                   .OMEGA_MAX_CODES(8192)) u_est (
    .clk, .rst_n, .sector, .sector_valid, .edge_strobe, .dir, .tick,
    .cal_we, .cal_addr, .cal_data, .theta, .omega, .moving);

  // ---- rotor emulation -------------------------------------------------
  localparam logic [2:0] PAT [6] = '{3'b001, 3'b011, 3'b010,
                                     3'b110, 3'b100, 3'b101};
  int b [6]; // physical boundary angles: edge entering sector i (forward)

  real th_ref = 5461.0;  // start mid sector 0
  real om_ref = 0.0;     // codes per clk, signed

  // bounce injection override
  logic       force_b = 0;
  logic [2:0] force_pat = 3'b001;

  function automatic int wrap16(input int x);
    int r;
    r = x % 65536;
    return (r < 0) ? r + 65536 : r;
  endfunction

  function automatic int sector_of(input int th);
    for (int i = 0; i < 6; i++) begin
      int lo, wid;
      lo  = b[i];
      wid = wrap16(b[(i + 1) % 6] - b[i]);
      if (wrap16(th - lo) < wid) return i;
    end
    return 0;
  endfunction

  always @(posedge clk) begin
    th_ref <= th_ref + om_ref;
    if (th_ref >= 65536.0) th_ref <= th_ref + om_ref - 65536.0;
    if (th_ref < 0.0)      th_ref <= th_ref + om_ref + 65536.0;
    hall_i <= force_b ? force_pat
                      : PAT[sector_of(int'($floor(th_ref)) % 65536)];
  end

  // ---- continuous checkers (tick-aligned, gated) ---------------------------
  int errors = 0;
  bit chk_track = 0, chk_snap = 0;
  int track_tol = 400;

  // sample th_ref at the tick, compare a few clks later (theta_q settles
  // 4 clks after the tick; th_ref drift over those clks is < 3 codes)
  real th_ref_tick = 0.0;
  int  th_prev_s;
  bit  th_prev_v = 0;
  always @(posedge clk) begin
    if (tick) th_ref_tick <= th_ref;
    if (tick_cnt == 8) begin
      if (chk_track) begin
        int terr;
        terr = wrap16(int'(theta) - int'($floor(th_ref_tick)));
        if (terr > 32768) terr = 65536 - terr;
        if (terr > track_tol) begin
          $display("  MISMATCH tracking err=%0d (tol %0d) theta=%0d ref=%0d at %0t",
                   terr, track_tol, theta, int'(th_ref_tick), $time);
          errors++;
          chk_track = 0; // avoid error storms
        end
      end
      if (chk_snap && th_prev_v) begin
        int d, om_i, m;
        d = wrap16(int'(theta) - th_prev_s);
        if (d > 32768) d = d - 65536; // signed per-tick step
        om_i = int'(omega);
        // a soft correction is <= 0.3 * residual error; a hard snap would
        // jump by the full residual (or a full sector on a missed edge)
        m = 600 + ((om_i < 0 ? -om_i : om_i) >> 2);
        if (d - om_i > m || om_i - d > m) begin
          $display("  MISMATCH snap: dtheta=%0d omega=%0d at %0t",
                   d, om_i, $time);
          errors++;
          chk_snap = 0;
        end
      end
      th_prev_s = int'(theta);
      th_prev_v = 1;
    end
  end

  // ---- helpers --------------------------------------------------------------
  task automatic spin(input real om, input int cycles);
    om_ref = om;
    repeat (cycles) @(negedge clk);
  endtask

  // rotor inertia: speed changes are ramps, never steps (an abrupt flip
  // would let theta_q integrate the stale omega for up to a sector before
  // the first opposite edge - faithful to the STM32, but not physical)
  task automatic ramp_om(input real target);
    while (om_ref != target) begin
      if (om_ref < target)
        om_ref = (om_ref + 0.01 > target) ? target : om_ref + 0.01;
      else
        om_ref = (om_ref - 0.01 < target) ? target : om_ref - 0.01;
      repeat (1500) @(negedge clk);
    end
  endtask

  task automatic wait_edges(input real om, input int n);
    om_ref = om;
    repeat (n) begin
      @(negedge clk);
      while (!edge_strobe) @(negedge clk);
    end
  endtask

  task automatic check_omega(input real om);
    int exp_om, got;
    exp_om = int'(om * 1250.0);
    got    = int'(omega);
    if (got > exp_om + (exp_om < 0 ? -exp_om : exp_om) / 10 + 30 ||
        got < exp_om - (exp_om < 0 ? -exp_om : exp_om) / 10 - 30) begin
      $display("  MISMATCH omega got=%0d exp=%0d", got, exp_om);
      errors++;
    end
  endtask

  task automatic load_cal();
    // DUT cal: fwd entering s = b[s]; rev entering s = b[(s+1)%6]
    for (int s = 0; s < 6; s++) begin
      @(negedge clk);
      cal_we = 1; cal_addr = 4'(s); cal_data = angle_t'(b[s]);
      @(negedge clk);
      cal_we = 1; cal_addr = 4'(6 + s); cal_data = angle_t'(b[(s + 1) % 6]);
    end
    @(negedge clk);
    cal_we = 0;
  endtask

  // expected stale/cold theta: sector center per a boundary set c[]
  function automatic int center_of(input int c0, input int c1);
    return wrap16(c0 + wrap16(c1 - c0) / 2);
  endfunction

  task automatic check_stale(input int exp_center);
    int terr;
    if (moving) begin
      $display("  MISMATCH still 'moving' at standstill"); errors++;
    end
    if (omega != 0) begin
      $display("  MISMATCH omega=%0d at standstill", omega); errors++;
    end
    terr = wrap16(int'(theta) - exp_center);
    if (terr > 32768) terr = 65536 - terr;
    if (terr > 200) begin
      $display("  MISMATCH stale theta=%0d exp center=%0d", theta,
               exp_center); errors++;
    end
  endtask

  int s_now, om_pre;

  initial begin
    // identity boundaries
    b[0] = 0; b[1] = 10923; b[2] = 21845;
    b[3] = 32768; b[4] = 43691; b[5] = 54613;

    repeat (3) @(negedge clk);
    rst_n = 1;
    repeat (DEB + 6) @(negedge clk);

    // ---- cold start: theta = sector center before any edge -------------
    if (wrap16(int'(theta) - 5461) > 200 &&
        wrap16(5461 - int'(theta)) > 200) begin
      $display("  MISMATCH initial theta=%0d exp ~5461", theta); errors++;
    end

    // ---- constant forward speed: converge, then track -------------------
    // omega blends x0.7 per edge from 0 and the theta lock follows it (the
    // two corrections are coupled, so the effective lock is slower than
    // 0.7^k): allow a generous lock window before asserting
    ramp_om(0.2);
    wait_edges(0.2, 24);
    track_tol = 500;
    chk_track = 1; chk_snap = 1;
    wait_edges(0.2, 8);
    check_omega(0.2);

    // ---- acceleration: re-lock at 0.4 ------------------------------------
    chk_track = 0; chk_snap = 0; // tracking lags during accel; corrections
    ramp_om(0.4);                // are large-but-soft, so no-snap is only
    wait_edges(0.4, 20);         // meaningful in locked windows
    track_tol = 700;             // lag ~ omega * ~0.5 period grows w/ speed
    chk_track = 1; chk_snap = 1;
    wait_edges(0.4, 8);
    check_omega(0.4);

    // ---- low speed, non-integer codes/period (fractional omega) ---------
    chk_track = 0; chk_snap = 0;
    ramp_om(0.121);            // 151.25 codes/period
    wait_edges(0.121, 20);
    track_tol = 500;
    chk_track = 1; chk_snap = 1;
    wait_edges(0.121, 6);
    check_omega(0.121);

    // ---- stale: freeze at the current sector center ----------------------
    chk_track = 0; chk_snap = 0;
    spin(0.0, TIMEOUT + 20000);
    s_now = sector_of(int'($floor(th_ref)));
    check_stale(center_of(b[s_now], b[(s_now + 1) % 6]));
    spin(0.0, 3 * PERIOD); // stays frozen across further ticks
    check_stale(center_of(b[s_now], b[(s_now + 1) % 6]));

    // ---- bounce reject ----------------------------------------------------
    ramp_om(0.2);
    wait_edges(0.2, 24);       // re-lock after the stale period
    track_tol = 500;
    chk_track = 1; chk_snap = 1;
    om_pre = int'(omega);
    // shortly after a real edge, force the next sector's pattern long
    // enough for the decoder to accept it (> DEB), then revert - both
    // decoder edges land inside MIN_EDGE_CYC and must be ignored
    @(negedge clk);
    while (!edge_strobe) @(negedge clk);
    repeat (30) @(negedge clk);
    force_pat = PAT[(sector_of(int'($floor(th_ref))) + 1) % 6];
    force_b = 1;
    repeat (20) @(negedge clk);
    force_b = 0;
    spin(0.2, 3 * PERIOD);
    if (int'(omega) > om_pre + (om_pre >> 3) + 30 ||
        int'(omega) < om_pre - (om_pre >> 3) - 30) begin
      $display("  MISMATCH omega disturbed by bounce: %0d -> %0d",
               om_pre, omega); errors++;
    end
    wait_edges(0.2, 4);        // chk_track confirms theta undisturbed
    check_omega(0.2);

    // ---- unequal physical sectors, IDENTITY table -------------------------
    // (the PLL's raison d'etre: bounded error without calibration)
    chk_track = 0; chk_snap = 0;
    b[0] = 1500; b[1] = 9423; b[2] = 23345;
    b[3] = 31268; b[4] = 45191; b[5] = 53113;
    wait_edges(0.2, 20);
    track_tol = 4500;          // bounded ~3x max skew: the identity table
    chk_track = 1;             // also mis-measures per-sector travel, so
                               // omega wobbles sector to sector
    wait_edges(0.2, 10);
    chk_track = 0;

    // ---- calibrated unequal sectors: tight tracking ------------------------
    load_cal();
    wait_edges(0.2, 24);
    track_tol = 500;
    chk_track = 1; chk_snap = 1;
    wait_edges(0.2, 8);
    check_omega(0.2);

    // ---- direction reversal (calibrated table) -----------------------------
    chk_track = 0; chk_snap = 0; // large-but-soft corrections during re-lock
    ramp_om(-0.2);
    wait_edges(-0.2, 24);      // omega re-measured fresh after reversal
    track_tol = 500;
    chk_track = 1; chk_snap = 1;
    wait_edges(-0.2, 8);
    check_omega(-0.2);

    // ---- stale freeze with the calibrated table ----------------------------
    chk_track = 0; chk_snap = 0;
    spin(0.0, TIMEOUT + 20000);
    s_now = sector_of(int'($floor(th_ref)));
    check_stale(center_of(b[s_now], b[(s_now + 1) % 6]));

    if (errors == 0) $display("TB_PASS: tb_hall_angle_est");
    else $display("TB_FAIL: tb_hall_angle_est (%0d errors)", errors);
    $finish;
  end

endmodule
