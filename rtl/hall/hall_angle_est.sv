// ============================================================================
// hall_angle_est.sv
//
//  Electrical angle / speed estimator from hall edges (pole pairs = 1, so
//  the 6 hall edges are absolute over the full electrical revolution).
//
//  - 12-entry calibrated edge table: entry (fwd, s) is the angle of the
//    edge ENTERING sector s moving forward (index s), entry (rev, s) the
//    angle entering s in reverse (index 6 + s). Identity default (60 deg
//    grid); loadable at runtime (UART) to absorb hall placement error,
//    which maps 1:1 into electrical angle at Np = 1.
//  - Inter-edge speed: traveled table angle divided by measured inter-edge
//    time (serial restoring divider, 32 clks, hidden in the >>1000-clk
//    edge spacing).
//  - Intra-sector interpolation: theta = theta_edge +/- acc; acc is rate-
//    limited and CLAMPED to the table distance to the next expected edge,
//    so theta never crosses an edge angle before the edge arrives (the
//    interpolation guard).
//  - Standstill: no edge for TIMEOUT_CYC -> omega = 0, theta holds.
//  - Direction reversal resets the rate; one fresh same-direction interval
//    is measured before interpolation resumes.
//  - omega: signed, angle codes per PWM period (2*PWM_ARR clks).
//  - Before the first edge: theta = center of the current hall sector.
// ============================================================================

module hall_angle_est
  import foc_pkg::*;
#(
  parameter int unsigned TIMEOUT_CYC = 1 << 22 // ~42 ms @ 100 MHz
)(
  input  logic       clk,
  input  logic       rst_n,
  // from hall_decode
  input  logic [2:0] sector,
  input  logic       sector_valid,
  input  logic       edge_strobe,
  input  logic       dir,          // 1 = forward
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
  localparam logic [31:0] INC_MAX = 32'h0010_0000; // 16 codes/clk sanity cap

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

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  angle_t      theta_edge;  // table angle of the last accepted edge
  logic        dir_q;       // direction at the last edge
  logic        have_edge;
  logic        have_rate;
  logic [31:0] t_cnt;       // clks since last edge (saturating)
  logic [31:0] acc;         // interpolated advance, Q16
  logic [31:0] inc;         // rate, codes/clk in Q16
  logic [31:0] dist_q16;    // guard limit: table distance to next edge, Q16

  // divider state
  logic        div_busy, div_done;
  logic [4:0]  div_i;
  logic [31:0] div_num, div_den, div_rem, div_quo;

  // edge-time combinational helpers
  angle_t theta_edge_new, traveled, dist_new;
  assign theta_edge_new = cal[cal_idx(dir, sector)];
  assign traveled = dir ? angle_t'(theta_edge_new - theta_edge)
                        : angle_t'(theta_edge - theta_edge_new);
  assign dist_new = dir
      ? angle_t'(cal[cal_idx(1'b1, sec_inc(sector))] - theta_edge_new)
      : angle_t'(theta_edge_new - cal[cal_idx(1'b0, sec_dec(sector))]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      theta_edge <= '0;
      dir_q      <= 1'b1;
      have_edge  <= 1'b0;
      have_rate  <= 1'b0;
      t_cnt      <= '0;
      acc        <= '0;
      inc        <= '0;
      dist_q16   <= 32'(E1) << 16;
      div_busy   <= 1'b0;
      div_done   <= 1'b0;
      div_i      <= '0;
      div_num    <= '0;
      div_den    <= 32'd1;
      div_rem    <= '0;
      div_quo    <= '0;
    end else begin
      // ---- inter-edge timer ------------------------------------------
      if (t_cnt < TIMEOUT_CYC) t_cnt <= t_cnt + 1'b1;

      // ---- interpolation with guard ------------------------------------
      if (have_rate && t_cnt < TIMEOUT_CYC) begin
        if (33'(acc) + 33'(inc) < 33'(dist_q16)) acc <= acc + inc;
        else                                     acc <= dist_q16;
      end

      // ---- serial restoring divider -------------------------------------
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
        div_done <= 1'b0;
        inc      <= (div_quo > INC_MAX) ? INC_MAX : div_quo;
      end

      // ---- edge handling ---------------------------------------------------
      if (edge_strobe && sector_valid) begin
        if (have_edge && dir == dir_q
            && traveled != 0 && traveled <= 16'd16384) begin
          div_num   <= 32'(traveled) << 16;
          div_den   <= (t_cnt == 0) ? 32'd1 : t_cnt;
          div_rem   <= '0;
          div_quo   <= '0;
          div_i     <= '0;
          div_busy  <= 1'b1;
          div_done  <= 1'b0;
          have_rate <= 1'b1;
        end else begin
          have_rate <= 1'b0; // first edge / reversal / odd jump: no rate
          inc       <= '0;
          div_busy  <= 1'b0;
          div_done  <= 1'b0;
        end
        theta_edge <= theta_edge_new;
        dist_q16   <= 32'(dist_new) << 16;
        dir_q      <= dir;
        have_edge  <= 1'b1;
        acc        <= '0;
        t_cnt      <= '0;
      end
    end
  end

  // ------------------------------------------------------------------
  // Outputs (registered: one clk of angle staleness = 10 ns, irrelevant,
  // and it keeps the multiply/adders out of downstream paths)
  // ------------------------------------------------------------------
  assign moving = have_rate && (t_cnt < TIMEOUT_CYC);

  logic [31:0] om32;
  assign om32 = (inc * PERIOD_CYC) >> 16; // <= INC_MAX*1250>>16 = 20000

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      theta <= angle_t'(E1 >> 1);
      omega <= '0;
    end else if (!have_edge) begin
      theta <= cal[cal_idx(1'b1, sector)]
             + (angle_t'(cal[cal_idx(1'b1, sec_inc(sector))]
                         - cal[cal_idx(1'b1, sector)]) >> 1);
      omega <= '0;
    end else begin
      theta <= dir_q ? angle_t'(theta_edge + angle_t'(acc >> 16))
                     : angle_t'(theta_edge - angle_t'(acc >> 16));
      omega <= moving ? (dir_q ? 16'(om32) : -16'(om32)) : 16'sd0;
    end
  end

endmodule
