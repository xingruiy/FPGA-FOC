// ============================================================================
// cmd_telemetry.sv  –  ASCII text command parser + on-demand telemetry
// ============================================================================
//  Host → FPGA: ASCII lines, terminated by CR, LF, or CR+LF.
//    Decimal integers. Optional leading '-' for signed fields.
//    Every known command echoes "OK\r\n" and kicks the watchdog.
//    Unknown keywords echo "?\r\n" without changing state.
//    ESC (0x1B) stops telemetry streaming.
//
//    Commands (full keyword required, case-insensitive - there is no
//    checksum, so partial matches are never acted on):
//      enable [0|1]        enable/disable drive (bare = enable 1)
//      disable             alias for enable 0
//      iq  <int16>         iq_ref, Q1.15 raw integer (signed, ±32767)
//      kp  <uint16>        proportional gain Q4.12
//      ki  <uint16>        integral gain Q4.12
//      cal                 offset-calibration strobe (while disabled)
//      ping                watchdog kick only
//      tele                start telemetry streaming (100 ms per line)
//      ol  <0|1>           open-loop mode
//      vq  <int16>         open-loop Vq (Q1.15, signed)
//      speed <int16>       open-loop angle codes/period (signed)
//      hall <idx> <uint16> hall edge table: idx 0-11, angle 0-65535
//
//  FPGA → Host:
//    "OK\r\n"  after every accepted command
//    "?\r\n"   on unknown keyword
//    While tele_en, every TELEM_CYC clks:
//      "id=XXXX iq=XXXX th=XXXX om=XXXX f=XX s=XX e=XX\r\n"  (48 bytes, hex)
//
//  Watchdog: no accepted command for WD_CYC clks (100 ms default) →
//    wd_timeout asserts; iq_ref ramps to 0 (RAMP_STEP per RAMP_INTERVAL).
//    Recovery: any accepted command clears wd_timeout; fresh iq needed.
// ============================================================================

module cmd_telemetry
  import foc_pkg::*;
#(
  parameter int unsigned WD_CYC        = 10_000_000, // 100 ms @ 100 MHz
  parameter int unsigned TELEM_CYC     = 10_000_000, // 100 ms
  parameter int unsigned RAMP_INTERVAL = 1024,
  parameter int unsigned RAMP_STEP     = 64
)(
  input  logic       clk,
  input  logic       rst_n,
  // UART byte interfaces
  input  logic [7:0] rx_data,
  input  logic       rx_valid,
  output logic [7:0] tx_data,
  output logic       tx_valid,
  input  logic       tx_ready,
  // control outputs
  output logic       enable,
  output q15_t       iq_ref,
  output logic signed [15:0] kp,
  output logic signed [15:0] ki,
  output logic       cal_we,
  output logic [3:0] cal_addr,
  output angle_t     cal_data,
  output logic       offset_cal_start,
  output logic       ol_mode,
  output q15_t       vq_ol,
  output logic signed [15:0] ol_speed,
  output logic       wd_timeout,
  // telemetry inputs
  input  q15_t       id_meas,
  input  q15_t       iq_meas,
  input  angle_t     theta,
  input  logic signed [15:0] omega,
  input  logic [7:0] fault_flags,
  input  logic [7:0] status_flags,
  input  logic [7:0] err_flags     // sticky link/sensor errors (foc_top)
);

  // ------------------------------------------------------------------
  // RX text line parser
  // ------------------------------------------------------------------
  typedef enum logic [1:0] {T_IDLE, T_KEY, T_ARG1, T_ARG2} tstate_t;

  tstate_t     tstate;
  logic [7:0]  kbuf [7:0]; // keyword buffer, up to 8 chars
  logic [2:0]  klen;
  logic [15:0] arg1, arg2;
  logic        arg1_neg, arg2_neg;
  logic [2:0]  arg1_ndig, arg2_ndig;
  logic        cmd_ok; // strobe: line complete, dispatch may apply

  function automatic logic is_alpha(input logic [7:0] c);
    return (c >= 8'h41 && c <= 8'h5A) || (c >= 8'h61 && c <= 8'h7A);
  endfunction

  function automatic logic is_digit(input logic [7:0] c);
    return c >= 8'h30 && c <= 8'h39;
  endfunction

  function automatic logic is_eol(input logic [7:0] c);
    return c == 8'h0D || c == 8'h0A;
  endfunction

  // Decimal accumulate: v*10 + digit  (synthesises as shifts + add)
  function automatic logic [15:0] dec_push(
      input logic [15:0] v, input logic [7:0] d);
    return (v << 3) + (v << 1) + {8'h0, d - 8'h30};
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tstate    <= T_IDLE;
      klen      <= '0;
      arg1      <= '0; arg1_neg <= 1'b0; arg1_ndig <= '0;
      arg2      <= '0; arg2_neg <= 1'b0; arg2_ndig <= '0;
      cmd_ok    <= 1'b0;
      for (int i = 0; i < 8; i++) kbuf[i] <= '0;
    end else begin
      cmd_ok <= 1'b0;

      if (rx_valid) begin
        if (rx_data == 8'h1B) begin // ESC: abort current line
          tstate <= T_IDLE;
          klen   <= '0;
        end else begin
          unique case (tstate)
            T_IDLE: begin
              if (is_alpha(rx_data)) begin
                for (int i = 0; i < 8; i++) kbuf[i] <= '0;
                kbuf[0] <= rx_data;
                klen    <= 3'd1;
                tstate  <= T_KEY;
              end
            end

            T_KEY: begin
              if (is_alpha(rx_data) && klen < 3'd7) begin
                kbuf[klen] <= rx_data;
                klen       <= klen + 1'b1;
              end else if (rx_data == 8'h20) begin
                arg1      <= '0; arg1_neg <= 1'b0; arg1_ndig <= '0;
                tstate    <= T_ARG1;
              end else if (is_eol(rx_data)) begin
                cmd_ok <= 1'b1;
                tstate <= T_IDLE;
                klen   <= '0;
              end
            end

            T_ARG1: begin
              if (rx_data == 8'h20 && arg1_ndig == '0) begin
                ; // skip leading spaces
              end else if (rx_data == 8'h2D && arg1_ndig == '0) begin
                arg1_neg <= 1'b1;
              end else if (is_digit(rx_data) && arg1_ndig < 3'd5) begin
                arg1      <= dec_push(arg1, rx_data);
                arg1_ndig <= arg1_ndig + 1'b1;
              end else if (rx_data == 8'h20 && arg1_ndig > '0) begin
                arg2      <= '0; arg2_neg <= 1'b0; arg2_ndig <= '0;
                tstate    <= T_ARG2;
              end else if (is_eol(rx_data)) begin
                cmd_ok <= 1'b1;
                tstate <= T_IDLE;
                klen   <= '0;
              end
            end

            T_ARG2: begin
              if (rx_data == 8'h20 && arg2_ndig == '0) begin
                ; // skip leading spaces
              end else if (rx_data == 8'h2D && arg2_ndig == '0) begin
                arg2_neg <= 1'b1;
              end else if (is_digit(rx_data) && arg2_ndig < 3'd5) begin
                arg2      <= dec_push(arg2, rx_data);
                arg2_ndig <= arg2_ndig + 1'b1;
              end else if (is_eol(rx_data)) begin
                cmd_ok <= 1'b1;
                tstate <= T_IDLE;
                klen   <= '0;
              end
            end
          endcase
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // Command dispatch + watchdog + iq ramp-down
  // ------------------------------------------------------------------
  logic [31:0] wd_cnt;
  logic [31:0] ramp_cnt;
  logic        tele_en;
  logic        resp_ok_req;  // strobe → TX: queue "OK\r\n"
  logic        resp_err_req; // strobe → TX: queue "?\r\n"

  function automatic logic signed [15:0] to_signed(
      input logic [15:0] v, input logic neg);
    return neg ? (-$signed(v)) : $signed(v);
  endfunction

  // Exact keyword match: the whole zero-padded buffer is compared, so a
  // prefix ("en") or an extension ("enabled") is rejected with '?'. This
  // protocol has no checksum - never act on a partial match.
  function automatic logic [7:0] lc(input logic [7:0] c);
    return (c >= 8'h41 && c <= 8'h5A) ? (c | 8'h20) : c;
  endfunction

  logic [63:0] kw; // kbuf lower-cased, kbuf[0] in the top byte
  always_comb begin
    for (int i = 0; i < 8; i++)
      kw[8*(7-i) +: 8] = lc(kbuf[i]);
  end

  localparam logic [63:0] KW_ENABLE  = {"enable",  16'h0};
  localparam logic [63:0] KW_DISABLE = {"disable",  8'h0};
  localparam logic [63:0] KW_IQ      = {"iq",      48'h0};
  localparam logic [63:0] KW_KP      = {"kp",      48'h0};
  localparam logic [63:0] KW_KI      = {"ki",      48'h0};
  localparam logic [63:0] KW_CAL     = {"cal",     40'h0};
  localparam logic [63:0] KW_PING    = {"ping",    32'h0};
  localparam logic [63:0] KW_TELE    = {"tele",    32'h0};
  localparam logic [63:0] KW_OL      = {"ol",      48'h0};
  localparam logic [63:0] KW_VQ      = {"vq",      48'h0};
  localparam logic [63:0] KW_SPEED   = {"speed",   24'h0};
  localparam logic [63:0] KW_HALL    = {"hall",    32'h0};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      enable           <= 1'b0;
      iq_ref           <= '0;
      // Default current-loop gains: wc = 2*pi*1 kHz against the per-phase
      // plant (Rs = 1.58 ohm, Ls = 127 uH). kp = wc*Ls * (1.25/24) * 4096,
      // ki = wc*Rs*Ts * (1.25/24) * 4096. See docs/foc.md "PI tuning".
      kp               <= 16'sd170;
      ki               <= 16'sd26;
      cal_we           <= 1'b0;
      cal_addr         <= '0;
      cal_data         <= '0;
      offset_cal_start <= 1'b0;
      ol_mode          <= 1'b0;
      vq_ol            <= '0;
      ol_speed         <= '0;
      wd_timeout       <= 1'b0;
      wd_cnt           <= '0;
      ramp_cnt         <= '0;
      tele_en          <= 1'b0;
      resp_ok_req      <= 1'b0;
      resp_err_req     <= 1'b0;
    end else begin
      cal_we           <= 1'b0;
      offset_cal_start <= 1'b0;
      resp_ok_req      <= 1'b0;
      resp_err_req     <= 1'b0;

      // ESC stops telemetry
      if (rx_valid && rx_data == 8'h1B)
        tele_en <= 1'b0;

      // iq ramp while watchdog timed out
      if (wd_timeout) begin
        if (ramp_cnt == RAMP_INTERVAL - 1) begin
          ramp_cnt <= '0;
          if      (iq_ref >  q15_t'(RAMP_STEP)) iq_ref <= iq_ref - q15_t'(RAMP_STEP);
          else if (iq_ref < -q15_t'(RAMP_STEP)) iq_ref <= iq_ref + q15_t'(RAMP_STEP);
          else                                   iq_ref <= '0;
        end else ramp_cnt <= ramp_cnt + 1'b1;
      end else ramp_cnt <= '0;

      // Watchdog counter + command dispatch (exact keyword match)
      if (cmd_ok) begin
        if (kw == KW_ENABLE) begin
          enable      <= (arg1_ndig == '0) ? 1'b1 : arg1[0];
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_DISABLE) begin
          enable      <= 1'b0;
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_IQ) begin
          if (!wd_timeout)
            iq_ref <= q15_t'(to_signed(arg1, arg1_neg));
          wd_cnt <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_KP) begin
          kp          <= to_signed(arg1, 1'b0);
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_KI) begin
          ki          <= to_signed(arg1, 1'b0);
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_CAL) begin
          offset_cal_start <= 1'b1;
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_PING) begin
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_TELE) begin
          tele_en     <= 1'b1;
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_OL) begin
          ol_mode     <= arg1[0];
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_VQ) begin
          vq_ol       <= q15_t'(to_signed(arg1, arg1_neg));
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_SPEED) begin
          ol_speed    <= to_signed(arg1, arg1_neg);
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else if (kw == KW_HALL) begin
          cal_addr    <= arg1[3:0];
          cal_data    <= arg2;
          cal_we      <= 1'b1;
          wd_cnt      <= '0; wd_timeout <= 1'b0; resp_ok_req <= 1'b1;
        end else begin
          resp_err_req <= 1'b1;
        end
      end else begin
        // No command this cycle: advance watchdog
        if (wd_cnt >= WD_CYC) wd_timeout <= 1'b1;
        else                  wd_cnt <= wd_cnt + 1'b1;
      end
    end
  end

  // ------------------------------------------------------------------
  // Telemetry byte builder
  // ------------------------------------------------------------------
  localparam int unsigned N_TELE = 48; // "id=XXXX iq=XXXX th=XXXX om=XXXX f=XX s=XX e=XX\r\n"

  q15_t   l_id, l_iq;
  angle_t l_th;
  logic signed [15:0] l_om;
  logic [7:0]  l_fault, l_stat, l_err;

  function automatic logic [7:0] nibble_to_hex(input logic [3:0] n);
    return (n < 4'd10) ? (8'h30 + {4'h0, n}) : (8'h37 + {4'h0, n});
  endfunction

  function automatic logic [7:0] tele_byte(input logic [5:0] i);
    unique case (i)
      6'd0:  return "i";
      6'd1:  return "d";
      6'd2:  return "=";
      6'd3:  return nibble_to_hex(l_id[15:12]);
      6'd4:  return nibble_to_hex(l_id[11:8]);
      6'd5:  return nibble_to_hex(l_id[7:4]);
      6'd6:  return nibble_to_hex(l_id[3:0]);
      6'd7:  return " ";
      6'd8:  return "i";
      6'd9:  return "q";
      6'd10: return "=";
      6'd11: return nibble_to_hex(l_iq[15:12]);
      6'd12: return nibble_to_hex(l_iq[11:8]);
      6'd13: return nibble_to_hex(l_iq[7:4]);
      6'd14: return nibble_to_hex(l_iq[3:0]);
      6'd15: return " ";
      6'd16: return "t";
      6'd17: return "h";
      6'd18: return "=";
      6'd19: return nibble_to_hex(l_th[15:12]);
      6'd20: return nibble_to_hex(l_th[11:8]);
      6'd21: return nibble_to_hex(l_th[7:4]);
      6'd22: return nibble_to_hex(l_th[3:0]);
      6'd23: return " ";
      6'd24: return "o";
      6'd25: return "m";
      6'd26: return "=";
      6'd27: return nibble_to_hex(l_om[15:12]);
      6'd28: return nibble_to_hex(l_om[11:8]);
      6'd29: return nibble_to_hex(l_om[7:4]);
      6'd30: return nibble_to_hex(l_om[3:0]);
      6'd31: return " ";
      6'd32: return "f";
      6'd33: return "=";
      6'd34: return nibble_to_hex(l_fault[7:4]);
      6'd35: return nibble_to_hex(l_fault[3:0]);
      6'd36: return " ";
      6'd37: return "s";
      6'd38: return "=";
      6'd39: return nibble_to_hex(l_stat[7:4]);
      6'd40: return nibble_to_hex(l_stat[3:0]);
      6'd41: return " ";
      6'd42: return "e";
      6'd43: return "=";
      6'd44: return nibble_to_hex(l_err[7:4]);
      6'd45: return nibble_to_hex(l_err[3:0]);
      6'd46: return 8'h0D; // CR
      6'd47: return 8'h0A; // LF
      default: return " ";
    endcase
  endfunction

  // ------------------------------------------------------------------
  // TX: responses take priority over telemetry
  // ------------------------------------------------------------------
  logic [7:0] resp_buf [3:0]; // max 4 bytes: "OK\r\n"
  logic [1:0] resp_len_m1;    // length - 1
  logic [1:0] resp_ridx;
  logic       resp_pending;

  logic [31:0] tele_cnt;
  logic [5:0]  tidx;
  logic        sending;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_valid     <= 1'b0;
      tx_data      <= '0;
      resp_pending <= 1'b0;
      resp_ridx    <= '0;
      resp_len_m1  <= '0;
      resp_buf[0] <= '0; resp_buf[1] <= '0;
      resp_buf[2] <= '0; resp_buf[3] <= '0;
      tele_cnt    <= '0;
      tidx        <= '0;
      sending     <= 1'b0;
      l_id <= '0; l_iq <= '0; l_th <= '0; l_om <= '0;
      l_fault <= '0; l_stat <= '0; l_err <= '0;
    end else begin
      tx_valid <= 1'b0;

      // Latch new response (resp_ok/err_req are 1-cycle strobes from dispatch)
      if (!resp_pending) begin
        if (resp_ok_req) begin
          resp_buf[0] <= "O"; resp_buf[1] <= "K";
          resp_buf[2] <= 8'h0D; resp_buf[3] <= 8'h0A;
          resp_len_m1 <= 2'd3;
          resp_ridx   <= '0;
          resp_pending <= 1'b1;
        end else if (resp_err_req) begin
          resp_buf[0] <= "?";
          resp_buf[1] <= 8'h0D; resp_buf[2] <= 8'h0A;
          resp_len_m1 <= 2'd2;
          resp_ridx   <= '0;
          resp_pending <= 1'b1;
        end
      end

      if (tx_ready && !tx_valid) begin
        if (resp_pending) begin
          // Send response byte
          tx_data  <= resp_buf[resp_ridx];
          tx_valid <= 1'b1;
          if (resp_ridx == resp_len_m1) begin
            resp_pending <= 1'b0;
            resp_ridx    <= '0;
          end else
            resp_ridx <= resp_ridx + 1'b1;
        end else if (sending) begin
          // Send telemetry byte
          tx_data  <= tele_byte(tidx);
          tx_valid <= 1'b1;
          if (tidx == 6'(N_TELE - 1)) begin
            sending <= 1'b0;
            tidx    <= '0;
          end else
            tidx <= tidx + 1'b1;
        end else begin
          // Idle: advance telemetry timer
          if (tele_en) begin
            if (tele_cnt >= TELEM_CYC - 1) begin
              tele_cnt <= '0;
              sending  <= 1'b1;
              tidx     <= '0;
              l_id <= id_meas; l_iq <= iq_meas; l_th <= theta;
              l_om <= omega;
              l_fault <= fault_flags; l_stat <= status_flags;
              l_err <= err_flags;
            end else
              tele_cnt <= tele_cnt + 1'b1;
          end else
            tele_cnt <= '0;
        end
      end
    end
  end

endmodule
