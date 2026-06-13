// ============================================================================
// hall_angle_est.sv
//
//  PLL-style electrical angle / speed observer from hall edges (pole pairs
//  = 1, so the 6 hall edges are absolute over the full electrical
//  revolution). Port of the bench-proven STM32 observer (hall.c):
//
//    once per PWM period (tick):   theta += omega          (th += w*dt)
//    on each accepted hall edge:   theta += KP * wrap(theta_boundary - theta)
//                                  omega += KW * (omega_edge - omega)
//
//  with KP = KW = 0.3. wrap() is free: the signed 16-bit difference of two
//  angle codes is exactly the +/-180 deg wrap. The soft correction (instead
//  of snapping to the edge) tolerates unequal hall sectors, sensor
//  hysteresis and edge jitter.
//
//  - 12-entry calibrated edge table: entry (fwd, s) is the angle of the
//    edge ENTERING sector s moving forward (index s), entry (rev, s) the
//    angle entering s in reverse (index 6 + s). Identity default (60 deg
//    grid); loadable at runtime (UART `hall <idx> <ang>`).
//  - omega_edge = calibrated traveled angle / measured inter-edge time
//    (serial restoring divider, 32 clks, hidden in the >=MIN_EDGE_CYC edge
//    spacing). Deviation from the STM32 (which assumes 60 deg per edge):
//    strictly better with unequal sectors.
//  - Guards (mirroring hall.c): edges closer than MIN_EDGE_CYC are ignored
//    entirely (contact bounce); a non-adjacent sector jump updates the
//    tracking state but makes no PLL handoff; a direction reversal applies
//    the theta correction only and zeroes omega (deviation: the STM32
//    blends a speed measured ACROSS the reversal - hysteresis-dominated,
//    deliberately not ported); the first edge after reset or a stale
//    period is theta-correction-only (no rate history).
//  - Stale: no edge for TIMEOUT_CYC (100 ms) -> theta freezes at the CENTER
//    of the current sector, omega = 0, observer re-arms as at cold start
//    (STM32 semantics; the old interpolator held the last theta instead).
//  - Before the first edge: theta = center of the current hall sector.
//  - theta advances as a per-period staircase (max one PWM period of
//    staleness); foc_core samples it once per period at the same tick, so
//    consumers see no staircase. Like the STM32 (correction applied at the
//    next control ISR), the boundary error is measured up to one period
//    after the physical crossing: steady-state lag ~= omega * ~0.5 period
//    (about 110 codes = 0.6 deg at the motor's speed ceiling).
//
//  PLL gains are compile-time parameters (the STM32 shipped 0.3/0.3
//  untouched); a runtime `pllk` UART command is a noted extension point.
// ============================================================================

module hall_angle_est
  import foc_pkg::*;
#(
  parameter int unsigned TIMEOUT_CYC     = 10_000_000, // 100 ms @ 100 MHz
  parameter int unsigned MIN_EDGE_CYC    = 5_000,      // 50 us bounce reject
  parameter int unsigned OMEGA_MAX_CODES = 512,        // |omega| clamp,
                                                       // codes/period
  parameter logic [15:0] PLL_KP_Q16      = 16'd19661,  // 0.3 in Q0.16
  parameter logic [15:0] PLL_KW_Q16      = 16'd19661   // 0.3 in Q0.16
)(
  input  logic       clk,
  input  logic       rst_n,
  // from hall_decode
  input  logic [2:0] sector,
  input  logic       sector_valid,
  input  logic       edge_strobe,
  input  logic       dir,          // 1 = forward
  // observer tick, once per PWM period (pwm_gen cnt_peak)
  input  logic       tick,
  // calibration table write port (identity at reset)
  input  logic       cal_we,
  input  logic [3:0] cal_addr,     // 0..5 fwd-entering, 6..11 rev-entering
  input  angle_t     cal_data,
  // outputs
  output angle_t     theta,
  output logic signed [15:0] omega, // angle codes per PWM period
  output logic       moving
);

  localparam int unsigned PERIOD_CYC = 2 * PWM_ARR;
  // divider-output cap (codes/clk, Q16) so omega_edge stays inside the clamp
  localparam logic [31:0] INC_CAP =
      (32'(OMEGA_MAX_CODES) << 16) / PERIOD_CYC;
  localparam logic signed [31:0] OMEGA_LIM =
      $signed(32'(OMEGA_MAX_CODES) << 16);

  // ------------------------------------------------------------------
  // Calibration table, identity default (60-degree grid)
  // ------------------------------------------------------------------
  angle_t cal [12];

  localparam angle_t E0 = 16'd0,     E1 = 16'd10923, E2 = 16'd21845;
  localparam angle_t E3 = 16'd32768, E4 = 16'd43691, E5 = 16'd54613;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cal[0] <= E0; cal[1] <= E1; cal[2]  <= E2;  // fwd: lower boundary
      cal[3] <= E3; cal[4] <= E4; cal[5]  <= E5;
      cal[6] <= E1; cal[7] <= E2; cal[8]  <= E3;  // rev: upper boundary
      cal[9] <= E4; cal[10] <= E5; cal[11] <= E0;
    end else if (cal_we && cal_addr < 4'd12) begin
      cal[cal_addr] <= cal_data;
    end
  end

  function automatic logic [3:0] cal_idx(input logic d, input logic [2:0] s);
    return d ? {1'b0, s} : 4'd6 + 4'(s);
  endfunction

  function automatic logic [2:0] sec_inc(input logic [2:0] s);
    return (s == 3'd5) ? 3'd0 : s + 3'd1;
  endfunction
  function automatic logic [2:0] sec_dec(input logic [2:0] s);
    return (s == 3'd0) ? 3'd5 : s - 3'd1;
  endfunction

  // center of sector s per the forward table (cold start / stale freeze)
  function automatic angle_t sec_center(input logic [2:0] s);
    return cal[cal_idx(1'b1, s)]
         + (angle_t'(cal[cal_idx(1'b1, sec_inc(s))]
                     - cal[cal_idx(1'b1, s)]) >> 1);
  endfunction

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  logic [31:0]        theta_q;      // angle, Q16.16 codes (wraps mod 2^32)
  logic signed [31:0] omega_q;      // codes/period, Q16.16
  logic signed [31:0] omega_edge_q; // last measured edge speed, Q16.16
  angle_t             theta_edge;   // table angle of the last accepted edge
  angle_t             theta_bnd;    // boundary for the pending correction
  logic [2:0]         sec_q;        // sector after the last accepted edge
  logic               dir_q;        // direction at the last accepted edge
  logic               have_edge;
  logic               have_rate;
  logic [31:0]        t_cnt;        // clks since last accepted edge (sat.)
  logic               edge_pending; // omega + theta correction queued
  logic               theta_pending;// theta correction queued
  logic               rev_pending;  // zero omega at the next tick

  // divider state
  logic        div_busy, div_done, div_dir;
  logic [4:0]  div_i;
  logic [31:0] div_num, div_den, div_rem, div_quo;

  // tick micro-sequence state
  logic [1:0]         ph;
  logic               ph_run;
  logic signed [15:0] err_s;    // wrap(theta_bnd - theta)
  logic signed [31:0] domega;
  logic signed [32:0] corr;     // err * KP, Q16 codes
  logic signed [47:0] blend;    // domega * KW, >>>16 to apply
  logic               ap_edge, ap_theta;

  // edge-time combinational helpers
  angle_t theta_edge_new, traveled;
  logic   fwd_step, rev_step;
  assign theta_edge_new = cal[cal_idx(dir, sector)];
  assign traveled = dir ? angle_t'(theta_edge_new - theta_edge)
                        : angle_t'(theta_edge - theta_edge_new);
  assign fwd_step = (sector == sec_inc(sec_q));
  assign rev_step = (sector == sec_dec(sec_q));

  logic [31:0] quo_c;
  assign quo_c = (div_quo > INC_CAP) ? INC_CAP : div_quo;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      theta_q       <= {16'd5461, 16'h0}; // center of identity sector 0
      omega_q       <= '0;
      omega_edge_q  <= '0;
      theta_edge    <= '0;
      theta_bnd     <= '0;
      sec_q         <= '0;
      dir_q         <= 1'b1;
      have_edge     <= 1'b0;
      have_rate     <= 1'b0;
      t_cnt         <= '0;
      edge_pending  <= 1'b0;
      theta_pending <= 1'b0;
      rev_pending   <= 1'b0;
      div_busy      <= 1'b0;
      div_done      <= 1'b0;
      div_dir       <= 1'b1;
      div_i         <= '0;
      div_num       <= '0;
      div_den       <= 32'd1;
      div_rem       <= '0;
      div_quo       <= '0;
      ph            <= '0;
      ph_run        <= 1'b0;
      err_s         <= '0;
      domega        <= '0;
      corr          <= '0;
      blend         <= '0;
      ap_edge       <= 1'b0;
      ap_theta      <= 1'b0;
    end else begin
      // ---- inter-edge timer (saturating) -------------------------------
      if (t_cnt < TIMEOUT_CYC) t_cnt <= t_cnt + 1'b1;

      // ---- tick micro-sequence (3 clks, once per 1250-clk period) ------
      if (tick) begin
        ph_run <= 1'b1;
        ph     <= 2'd0;
      end
      if (ph_run) begin
        case (ph)
          2'd0: begin // error / delta terms + pending snapshot
            err_s    <= signed'(angle_t'(theta_bnd - theta_q[31:16]));
            domega   <= omega_edge_q - omega_q;
            ap_edge  <= edge_pending;
            ap_theta <= theta_pending;
            ph       <= 2'd1;
          end
          2'd1: begin // products
            corr  <= 33'(err_s) * $signed({1'b0, PLL_KP_Q16});
            blend <= 48'(domega) * $signed({1'b0, PLL_KW_Q16});
            ph    <= 2'd2;
          end
          default: begin // apply corrections + integrate
            ph_run <= 1'b0;
            if (t_cnt >= TIMEOUT_CYC || !have_edge) begin
              // stale / cold start: freeze at the current sector center,
              // re-arm as at cold start
              theta_q       <= {sec_center(sector), 16'h0};
              omega_q       <= '0;
              have_edge     <= 1'b0;
              have_rate     <= 1'b0;
              edge_pending  <= 1'b0;
              theta_pending <= 1'b0;
              rev_pending   <= 1'b0;
            end else begin
              logic signed [31:0] w;
              w = rev_pending ? 32'sd0 : omega_q;
              if (ap_edge) begin
                w = w + 32'(blend >>> 16);
                have_rate <= 1'b1;
              end
              if (w >  OMEGA_LIM) w =  OMEGA_LIM;
              if (w < -OMEGA_LIM) w = -OMEGA_LIM;
              omega_q <= w;
              theta_q <= theta_q + unsigned'(w)
                       + ((ap_edge || ap_theta) ? unsigned'(corr[31:0])
                                                : 32'h0);
              if (ap_edge)  edge_pending  <= 1'b0;
              if (ap_theta) theta_pending <= 1'b0;
              rev_pending <= 1'b0;
            end
          end
        endcase
      end

      // ---- serial restoring divider: traveled<<16 / t_cnt --------------
      if (div_busy) begin
        logic [31:0] r;
        r = {div_rem[30:0], div_num[31]};
        if (r >= div_den) begin
          div_rem <= r - div_den;
          div_quo <= {div_quo[30:0], 1'b1};
        end else begin
          div_rem <= r;
          div_quo <= {div_quo[30:0], 1'b0};
        end
        div_num <= {div_num[30:0], 1'b0};
        if (div_i == 5'd31) begin
          div_busy <= 1'b0;
          div_done <= 1'b1;
        end else div_i <= div_i + 1'b1;
      end else if (div_done) begin
        div_done     <= 1'b0;
        omega_edge_q <= div_dir ? $signed(quo_c * PERIOD_CYC)
                                : -$signed(quo_c * PERIOD_CYC);
        edge_pending <= 1'b1;
      end

      // ---- edge handling (after the FSM: a coincident set wins) --------
      if (edge_strobe && sector_valid) begin
        if (t_cnt >= MIN_EDGE_CYC) begin
          if (have_edge && dir == dir_q && (dir ? fwd_step : rev_step)
              && traveled != 0 && traveled <= 16'd16384) begin
            // fresh same-direction adjacent edge: rate + theta handoff
            // (both queued together at div_done via edge_pending, so a
            // tick landing inside the 32-clk divide can't apply the theta
            // correction twice)
            div_num   <= 32'(traveled) << 16;
            div_den   <= (t_cnt == 0) ? 32'd1 : t_cnt;
            div_rem   <= '0;
            div_quo   <= '0;
            div_i     <= '0;
            div_busy  <= 1'b1;
            div_done  <= 1'b0;
            div_dir   <= dir;
            theta_bnd <= theta_edge_new;
          end else if (have_edge && (fwd_step || rev_step)
                       && dir != dir_q) begin
            // reversal: theta correction only; omega is hysteresis-
            // dominated across a reversal -> zeroed, re-measured fresh
            theta_bnd     <= theta_edge_new;
            theta_pending <= 1'b1;
            rev_pending   <= 1'b1;
            edge_pending  <= 1'b0;
            have_rate     <= 1'b0;
            div_busy      <= 1'b0;
            div_done      <= 1'b0;
          end else if (!have_edge) begin
            // first edge (cold start / after stale): theta correction only
            theta_bnd     <= theta_edge_new;
            theta_pending <= 1'b1;
          end else begin
            // non-adjacent jump: track it, but no PLL handoff
            edge_pending  <= 1'b0;
            div_busy      <= 1'b0;
            div_done      <= 1'b0;
          end
          theta_edge <= theta_edge_new;
          sec_q      <= sector;
          dir_q      <= dir;
          have_edge  <= 1'b1;
          t_cnt      <= '0;
        end
        // else: bounce (< MIN_EDGE_CYC since the last accepted edge) -
        // ignored entirely, t_cnt keeps running
      end
    end
  end

  // ------------------------------------------------------------------
  // Outputs (theta_q/omega_q are registers; max one PWM period stale)
  // ------------------------------------------------------------------
  assign theta  = theta_q[31:16];
  assign omega  = 16'(omega_q >>> 16);
  assign moving = have_rate && (t_cnt < TIMEOUT_CYC);

endmodule
