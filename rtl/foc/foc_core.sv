// ============================================================================
// foc_core.sv
//
//  PWM-rate FOC scheduler: one pass through the math chain per current
//  sample (sample_valid, fired by the XADC path at the PWM counter peak).
//
//  Dataflow / timing (clk cycles after sample_valid; every multiply has
//  its own pipeline stage to close 100 MHz):
//    t0  sample_valid: ia/ib latched upstream; sincos_lut + clarke start
//    t2  clarke done; t3 sincos done -> park strobed
//    t5  park done -> both PI controllers strobe (id_ref = 0, iq_ref)
//    t8  PI outputs -> d/q vector limiter (reg, squares, serial isqrt)
//    t27 limiter done -> inv_park strobed with limited vd/vq
//    t29 inv_park done -> svpwm strobed
//    t32 duties registered (~0.3 us, noise against the 1250-clk period);
//        pwm_gen latches them into its shadow register at the next period
//        boundary => exactly ONE PWM period of transport delay from
//        sample to applied voltage. The default gains below assume it
//        (per-phase plant tau_e = Ls/Rs = 80 us, Ts = 12.5 us, delay = Ts).
//
//  Default tuning (target wc = 2*pi*1 kHz, checked against the RL model
//  in tb_foc_top): kp = 170 (Q4.12, = 0.0415), ki = 26 (= 0.0063).
//  Derivation in docs/foc.md (per-phase Rs = 1.58 ohm, Ls = 127 uH).
//
//  Simplifications (documented):
//   - SVPWM is normalized to the NOMINAL 24 V bus; the measured vbus is
//     telemetry-only in v1 (no dynamic bus-voltage compensation).
//   - id_ref is fixed at 0 (torque control only).
//
//  Open-loop mode (ol_mode): the PIs are frozen and (vd_ol, vq_ol) feed
//  inv_park directly - used for V/f spin and hall calibration.
//
//  cal_active forces 50/50/50 duties (current-offset calibration with
//  realistic switching common mode).
//
//  OCP: |ia|, |ib| or |ic| above OCP_TRIP_Q15 latches ocp_trip until
//  a rising edge of `en` clears it. This is the motor's protection.
// ============================================================================

module foc_core
  import foc_pkg::*;
(
  input  logic   clk,
  input  logic   rst_n,
  input  logic   en,            // closed-loop enable (rising edge clears OCP)
  input  logic   ol_mode,       // open-loop: vd_ol/vq_ol bypass the PIs
  input  q15_t   vd_ol,
  input  q15_t   vq_ol,
  input  logic   cal_active,    // force 50% duties (offset calibration)
  // sample stream (once per PWM period, at cnt_peak + ADC latency)
  input  logic   sample_valid,
  input  q15_t   ia,
  input  q15_t   ib,
  input  q15_t   ic,
  input  angle_t theta,
  // references and gains
  input  q15_t   iq_ref,
  input  logic signed [15:0] kp,
  input  logic signed [15:0] ki,
  // outputs
  output q15_t   duty_a,
  output q15_t   duty_b,
  output q15_t   duty_c,
  output q15_t   id_meas,
  output q15_t   iq_meas,
  output logic   ocp_trip,
  output logic   sat_any       // any stage saturated this sample (telemetry)
);

  // MAX_MOD/sqrt(3): the dq vector limiter must bind BEFORE the svpwm
  // per-phase MAX_MOD clamp, so the PI anti-windup (which sees the
  // limiter output) always sees the truly applied voltage. The full
  // hexagon inscribed circle would be 1/sqrt(3) = 18918.
  localparam q15_t VMAX_Q15 = 16'sd16459; // 0.87/sqrt(3), nominal bus

  // d-axis authority cap (~0.40 * VMAX, unchanged in volts): the d axis needs
  // R*id + wL*iq (< 1 V for this motor). Without the cap, a saturated
  // q axis lets the d integrator drift and - because the vector limiter
  // gives vd priority - eat the whole voltage circle (vq -> 0, a stable
  // wrong-way attractor seen in tb_foc_top).
  localparam q15_t VD_CAP_Q15 = 16'sd6622;

  // ------------------------------------------------------------------
  // Angle -> sin/cos
  // ------------------------------------------------------------------
  logic lut_ov;
  q15_t sin_t, cos_t;

  sincos_lut u_lut (
    .clk, .rst_n, .in_valid(sample_valid), .theta,
    .out_valid(lut_ov), .sin_o(sin_t), .cos_o(cos_t));

  // ------------------------------------------------------------------
  // Clarke -> Park
  // ------------------------------------------------------------------
  logic ck_ov, ck_sat;
  q15_t ialpha, ibeta;

  clarke u_clarke (
    .clk, .rst_n, .in_valid(sample_valid), .ia, .ib,
    .out_valid(ck_ov), .ialpha, .ibeta, .sat(ck_sat));

  logic pk_ov, pk_sat;
  q15_t id_q, iq_q;

  park u_park (
    .clk, .rst_n, .in_valid(lut_ov), // clarke (1 cyc) is stable by t2
    .ialpha, .ibeta, .sin_t, .cos_t,
    .out_valid(pk_ov), .id(id_q), .iq(iq_q), .sat(pk_sat));

  assign id_meas = id_q;
  assign iq_meas = iq_q;

  // ------------------------------------------------------------------
  // PI controllers (strobed at park done), with anti-windup from the
  // APPLIED (post-limiter) outputs of the previous sample
  // ------------------------------------------------------------------
  logic pi_strobe;
  assign pi_strobe = pk_ov && en && !ol_mode;

  q15_t vd_app, vq_app; // applied (limited) values, registered
  q15_t u_d, u_q;
  logic pid_ov, piq_ov, pid_sat, piq_sat;

  pi_controller u_pi_d (
    .clk, .rst_n, .clr(!en), .strobe(pi_strobe),
    .sp(16'sd0), .fb(id_q), .kp, .ki, .applied(vd_app),
    .out_valid(pid_ov), .u(u_d), .usat(pid_sat));

  pi_controller u_pi_q (
    .clk, .rst_n, .clr(!en), .strobe(pi_strobe),
    .sp(iq_ref), .fb(iq_q), .kp, .ki, .applied(vq_app),
    .out_valid(piq_ov), .u(u_q), .usat(piq_sat));

  // ------------------------------------------------------------------
  // d/q vector limiter, SEQUENTIAL: a combinational dq_limit() (16-step
  // isqrt) is ~77 logic levels and misses 100 MHz by 50 ns. The serial
  // version below takes 18 clks - noise against the 1250-clk period.
  // Semantics match foc_pkg::dq_limit (vd priority), plus the VD_CAP.
  // ------------------------------------------------------------------
  localparam logic [31:0] VMAX2 = 32'd270898681; // VMAX_Q15^2

  // start: PI out_valid in closed loop (3 clks after strobe); in open
  // loop the PIs are frozen, so a delayed park-done event is used instead
  logic        pk_ov_d1, lim_go, lim_prep, lim_v, lim_busy;
  logic [4:0]  lim_i;
  logic        lim_clamped;
  q15_t        vd_lim, vq_lim, vq_pend;
  logic [31:0] lim_rem, lim_vq2;
  logic [31:0] sq_op, sq_res, sq_one;

  assign lim_go = ol_mode ? pk_ov_d1 : pid_ov;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pk_ov_d1    <= 1'b0;
      lim_prep    <= 1'b0;
      lim_v       <= 1'b0;
      lim_busy    <= 1'b0;
      lim_i       <= '0;
      vd_lim      <= '0;
      vq_lim      <= '0;
      vq_pend     <= '0;
      vd_app      <= '0;
      vq_app      <= '0;
      lim_clamped <= 1'b0;
      lim_rem     <= '0;
      lim_vq2     <= '0;
      sq_op       <= '0;
      sq_res      <= '0;
      sq_one      <= '0;
    end else begin
      pk_ov_d1 <= pk_ov;
      lim_v    <= 1'b0;
      lim_prep <= 1'b0;

      if (lim_go) begin
        // stage 0: register the clamped inputs (no arithmetic yet)
        q15_t vd_in, vq_in;
        vd_in = ol_mode ? vd_ol : u_d;
        if (!ol_mode) begin // cap closed-loop d-axis authority
          if      (vd_in >  VD_CAP_Q15) vd_in =  VD_CAP_Q15;
          else if (vd_in < -VD_CAP_Q15) vd_in = -VD_CAP_Q15;
        end
        vq_in = ol_mode ? vq_ol : u_q;
        lim_clamped <= 1'b0;
        if (vd_in > VMAX_Q15) begin
          vd_in = VMAX_Q15; lim_clamped <= 1'b1;
        end else if (vd_in < -VMAX_Q15) begin
          vd_in = -VMAX_Q15; lim_clamped <= 1'b1;
        end
        vd_lim   <= vd_in;
        vq_pend  <= vq_in;
        lim_prep <= 1'b1;
      end else if (lim_prep) begin
        // stage 1: squares from registered values, init the serial sqrt
        lim_rem  <= VMAX2 - unsigned'(32'(vd_lim) * 32'(vd_lim));
        lim_vq2  <= unsigned'(32'(vq_pend) * 32'(vq_pend));
        sq_op    <= VMAX2 - unsigned'(32'(vd_lim) * 32'(vd_lim));
        sq_res   <= '0;
        sq_one   <= 32'h4000_0000;
        lim_i    <= '0;
        lim_busy <= 1'b1;
      end else if (lim_busy) begin
        if (lim_i < 5'd16) begin
          // one isqrt iteration per clk (non-restoring, as isqrt32)
          if (sq_op >= sq_res + sq_one) begin
            sq_op  <= sq_op - (sq_res + sq_one);
            sq_res <= (sq_res >> 1) + sq_one;
          end else sq_res <= sq_res >> 1;
          sq_one <= sq_one >> 2;
          lim_i  <= lim_i + 1'b1;
        end else begin
          // finish: apply vq = min(|vq|, sqrt(rem)) with sign
          if (lim_vq2 > lim_rem) begin
            vq_lim <= (vq_pend < 0) ? q15_t'(-sq_res[15:0])
                                    : q15_t'(sq_res[15:0]);
            lim_clamped <= 1'b1;
          end else vq_lim <= vq_pend;
          vd_app   <= vd_lim; // PI anti-windup sees what was applied
          vq_app   <= (lim_vq2 > lim_rem)
                      ? ((vq_pend < 0) ? q15_t'(-sq_res[15:0])
                                       : q15_t'(sq_res[15:0]))
                      : vq_pend;
          lim_v    <= 1'b1;   // strobe inv_park
          lim_busy <= 1'b0;
        end
      end

      // disabled: duties are forced to 50/50/50, so NOTHING is applied -
      // the anti-windup feedback must say so, or the first PI step after
      // re-enable back-injects the stale pre-disable voltage into the
      // freshly cleared integrator (applied - u_prev = old vq in one step)
      if (!en) begin
        vd_app <= '0;
        vq_app <= '0;
      end
    end
  end

  // ------------------------------------------------------------------
  // Inverse Park -> SVPWM -> duty registers
  // ------------------------------------------------------------------
  logic ip_ov, ip_sat;
  q15_t valpha, vbeta;

  inv_park u_ip (
    .clk, .rst_n, .in_valid(lim_v),
    .vd(vd_lim), .vq(vq_lim), .sin_t, .cos_t,
    .out_valid(ip_ov), .valpha, .vbeta, .sat(ip_sat));

  logic sv_ov, sv_sat;
  q15_t da_w, db_w, dc_w;

  svpwm u_svpwm (
    .clk, .rst_n, .in_valid(ip_ov), .valpha, .vbeta,
    .out_valid(sv_ov), .da(da_w), .db(db_w), .dc(dc_w), .sat(sv_sat));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      duty_a <= 16'sd16384;
      duty_b <= 16'sd16384;
      duty_c <= 16'sd16384;
    end else if (cal_active || !en) begin
      duty_a <= 16'sd16384; // 50/50/50: zero average voltage
      duty_b <= 16'sd16384;
      duty_c <= 16'sd16384;
    end else if (sv_ov) begin
      duty_a <= da_w;
      duty_b <= db_w;
      duty_c <= dc_w;
    end
  end

  // ------------------------------------------------------------------
  // Overcurrent trip (latched; cleared by a rising edge of en)
  // ------------------------------------------------------------------
  logic en_q;

  function automatic logic over(input q15_t x);
    return (x > OCP_TRIP_Q15) || (x < -OCP_TRIP_Q15);
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ocp_trip <= 1'b0;
      en_q     <= 1'b0;
    end else begin
      en_q <= en;
      if (en && !en_q)      ocp_trip <= 1'b0; // re-arm on enable edge
      else if (sample_valid && (over(ia) || over(ib) || over(ic)))
        ocp_trip <= 1'b1;
    end
  end

  // saturation telemetry (sticky per sample window, exported live)
  assign sat_any = ck_sat || pk_sat || ip_sat || sv_sat
                || pid_sat || piq_sat || lim_clamped;

endmodule
