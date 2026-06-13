// ============================================================================
// foc_top.sv
//
//  Top level: I/O map + safe state. All blocks are individually verified;
//  this file only wires them and implements the safety equation
//
//      oe = (enable | cal_busy) & nFAULT_sync & ~ocp_trip & ~wd_timeout
//           & drv_cfg_done & ~(enable & ~ol_mode & hall_illegal)
//
//  which gates the six gate pins COMBINATIONALLY (between pwm_gen and the
//  pads) and asserts DRVOFF in parallel. The closed-loop sample path is
//  XADC (simultaneous S/H on VAUX1/VAUX9 @ cnt_peak, unipolar) -> offset cal ->
//  foc_core; the angle comes from the hall estimator, or from an internal
//  ramp in open-loop mode (V/f spin / hall calibration, commanded over UART).
// ============================================================================

module foc_top
  import foc_pkg::*;
(
  input  logic clk100,       // board oscillator
  input  logic ck_rstn,      // board reset button, active low
  // motor hall sensors
  input  logic [2:0] hall,
  // host UART
  input  logic uart_rx_i,
  output logic uart_tx_o,
  // DRV8316 SPI + control
  output logic drv_sclk,
  output logic drv_mosi,
  input  logic drv_miso,
  output logic drv_csn,
  output logic drv_off,      // high = driver outputs Hi-Z
  input  logic drv_nfault,
  // gate signals to DRV8316 (6x PWM mode)
  output logic gate_ah, gate_al,
  output logic gate_bh, gate_bl,
  output logic gate_ch, gate_cl,
  // analog: phase A/B shunt amps (VAUX1/VAUX9 pair, outer header, board divider)
  input  logic xa_p, xa_n,
  input  logic xb_p, xb_n
);

  // ------------------------------------------------------------------
  // Clock / reset
  // ------------------------------------------------------------------
  logic clk, rst_n;
  clk_rst_gen u_clk (.clk_in(clk100), .rstn_in(ck_rstn), .clk, .rst_n);

  // ------------------------------------------------------------------
  // Host link
  // ------------------------------------------------------------------
  logic [7:0] rxb;  logic rxb_v, rx_ferr;
  uart_rx u_rx (.clk, .rst_n, .rx(uart_rx_i),
                .rx_data(rxb), .rx_valid(rxb_v), .frame_err(rx_ferr));

  logic [7:0] txb;  logic txb_v, txb_rdy;
  uart_tx u_tx (.clk, .rst_n, .tx_data(txb), .tx_valid(txb_v),
                .tx_ready(txb_rdy), .tx(uart_tx_o));

  logic   enable, cal_we, ocal_start, wd_timeout, ol_mode;
  q15_t   iq_ref, vq_ol;
  logic signed [15:0] kp, ki, ol_speed;
  logic [3:0] cal_addr;
  angle_t cal_data;
  q15_t   id_meas, iq_meas;
  angle_t theta_mux;
  logic signed [15:0] omega;
  logic   ocp_trip, sat_any, cal_busy;
  logic   cfg_done, cfg_err;
  logic [7:0] ic_stat, stat1;
  logic   nfault_s;
  logic   hall_illegal;
  logic   ferr_sticky, hill_sticky; // diagnostic latches (clear on reset)

  cmd_telemetry u_cmd (
    .clk, .rst_n,
    .rx_data(rxb), .rx_valid(rxb_v),
    .tx_data(txb), .tx_valid(txb_v), .tx_ready(txb_rdy),
    .enable, .iq_ref, .kp, .ki,
    .cal_we, .cal_addr, .cal_data,
    .offset_cal_start(ocal_start),
    .ol_mode, .vq_ol, .ol_speed,
    .wd_timeout,
    .id_meas, .iq_meas, .theta(theta_mux), .omega,
    .fault_flags(ic_stat),
    .status_flags({cfg_done, cfg_err, ocp_trip, wd_timeout,
                   enable, cal_busy, sat_any, nfault_s}),
    .err_flags({5'b0, ferr_sticky, hill_sticky, hall_illegal}));

  // sticky diagnostics: UART frame errors and illegal hall codes are rare
  // but indicate wiring/level trouble - latch until reset so a 100 ms
  // telemetry snapshot cannot miss them (bit0 is the LIVE illegal level)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ferr_sticky <= 1'b0;
      hill_sticky <= 1'b0;
    end else begin
      if (rx_ferr)      ferr_sticky <= 1'b1;
      if (hall_illegal) hill_sticky <= 1'b1;
    end
  end

  // ------------------------------------------------------------------
  // DRV8316 configuration + fault poll (auto-start shortly after reset)
  // ------------------------------------------------------------------
  logic [15:0] start_cnt;
  logic        drv_start;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start_cnt <= '0;
      drv_start <= 1'b0;
    end else begin
      drv_start <= 1'b0;
      if (start_cnt != 16'hFFFF) begin
        start_cnt <= start_cnt + 1'b1;
        if (start_cnt == 16'hFFFE) drv_start <= 1'b1; // ~0.65 ms after rst
      end
    end
  end

  drv8316_spi u_drv (
    .clk, .rst_n, .start(drv_start),
    .sclk(drv_sclk), .mosi(drv_mosi), .cs_n(drv_csn), .miso(drv_miso),
    .cfg_done, .cfg_err, .ic_stat, .stat1, .poll_valid());

  // nFAULT synchronizer (active low from the driver)
  logic nf_m;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      nf_m     <= 1'b0;
      nfault_s <= 1'b0;
    end else begin
      nf_m     <= drv_nfault;
      nfault_s <= nf_m;
    end
  end

  // ------------------------------------------------------------------
  // Hall decode + angle estimation
  // ------------------------------------------------------------------
  logic [2:0] sector;
  logic sector_valid, edge_strobe, dir;

  hall_decode u_hd (
    .clk, .rst_n, .hall_i(hall),
    .sector, .sector_valid, .edge_strobe, .dir, .illegal(hall_illegal));

  angle_t theta_hall;
  logic   moving;
  logic   cnt_peak, update; // from pwm_gen below; cnt_peak ticks the observer

  hall_angle_est u_ha (
    .clk, .rst_n, .sector, .sector_valid, .edge_strobe, .dir,
    .tick(cnt_peak),
    .cal_we, .cal_addr, .cal_data,
    .theta(theta_hall), .omega, .moving);

  // open-loop angle ramp (V/f spin, hall calibration sweeps)
  angle_t ol_theta;

  // ------------------------------------------------------------------
  // PWM
  // ------------------------------------------------------------------
  q15_t duty_a, duty_b, duty_c;
  logic p_ah, p_al, p_bh, p_bl, p_ch, p_cl;
  logic oe;

  pwm_gen u_pwm (
    .clk, .rst_n, .en(1'b1), .oe({3{oe}}),
    .duty_a, .duty_b, .duty_c,
    .pwm_ah(p_ah), .pwm_al(p_al), .pwm_bh(p_bh), .pwm_bl(p_bl),
    .pwm_ch(p_ch), .pwm_cl(p_cl),
    .cnt(), .cnt_peak, .update);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)      ol_theta <= '0;
    else if (update) ol_theta <= ol_theta + angle_t'(ol_speed);
  end

  assign theta_mux = ol_mode ? ol_theta : theta_hall;

  // ------------------------------------------------------------------
  // Sensing: XADC -> offset calibration
  // ------------------------------------------------------------------
  q15_t ia_raw, ib_raw;
  logic adc_valid;

  xadc_iface u_adc (
    .clk, .rst_n, .trigger(cnt_peak),
    .vauxp({6'b0, xb_p, 1'b0, 6'b0, xa_p, 1'b0}), // bit9=VAUX9(phB) bit1=VAUX1(phA)
    .vauxn({6'b0, xb_n, 1'b0, 6'b0, xa_n, 1'b0}),
    .ia_raw, .ib_raw,
    .valid(adc_valid), .adc_busy());

  logic oc_ov, cal_done;
  q15_t ia_c, ib_c, ic_c;

  current_offset_cal u_ocal (
    .clk, .rst_n, .in_valid(adc_valid), .ia_raw, .ib_raw,
    .cal_start(ocal_start && !enable), // only honored while disabled
    .cal_done, .cal_busy,
    .out_valid(oc_ov), .ia(ia_c), .ib(ib_c), .ic(ic_c));

  // ------------------------------------------------------------------
  // FOC core
  // ------------------------------------------------------------------
  foc_core u_core (
    .clk, .rst_n,
    .en(enable), .ol_mode, .vd_ol(16'sd0), .vq_ol,
    .cal_active(cal_busy),
    .sample_valid(oc_ov), .ia(ia_c), .ib(ib_c), .ic(ic_c),
    .theta(theta_mux),
    .iq_ref, .kp, .ki,
    .duty_a, .duty_b, .duty_c,
    .id_meas, .iq_meas, .ocp_trip, .sat_any);

  // ------------------------------------------------------------------
  // Safe state: COMBINATIONAL between pwm_gen and the pads
  // ------------------------------------------------------------------
  // a live illegal hall code (broken wire / no sensor power) kills the
  // gates whenever the hall angle is actually in use (closed loop);
  // open-loop spin and offset calibration do not depend on the halls
  assign oe = (enable || cal_busy) && nfault_s && !ocp_trip
              && !wd_timeout && cfg_done
              && !(enable && !ol_mode && hall_illegal);

  assign gate_ah = p_ah & oe;
  assign gate_al = p_al & oe;
  assign gate_bh = p_bh & oe;
  assign gate_bl = p_bl & oe;
  assign gate_ch = p_ch & oe;
  assign gate_cl = p_cl & oe;
  assign drv_off = ~oe;

endmodule
