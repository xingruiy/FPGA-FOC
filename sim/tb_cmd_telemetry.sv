// ============================================================================
// tb_cmd_telemetry.sv - text command parser / telemetry / watchdog tests
//
//  - text commands apply (enable, iq, kp, ki, hall, cal, ol, vq, speed)
//  - unknown keyword echoes '?' without changing state
//  - watchdog fires after WD_CYC silence; ramps iq_ref to 0; ping recovers
//  - telemetry streams after "tele" command; ESC stops it
// ============================================================================
`timescale 1ns / 1ps

module tb_cmd_telemetry;
  import foc_pkg::*;

  localparam int WD_CYC     = 20000;
  localparam int TELEM_CYC  = 30000;
  localparam int RAMP_INT   = 64;
  localparam int RAMP_STEP  = 512;
  localparam int N_TELE_TB  = 48;
  localparam int TX_SETTLE  = 300; // cycles — enough for any response to complete

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic [7:0] rx_data = 0;
  logic rx_valid = 0;
  logic [7:0] tx_data;
  logic tx_valid;
  logic tx_ready = 1;
  logic enable, cal_we, wd_timeout, offset_cal_start, ol_mode;
  q15_t vq_ol;
  logic signed [15:0] ol_speed;
  q15_t iq_ref;
  logic signed [15:0] kp, ki;
  logic [3:0] cal_addr;
  angle_t cal_data;

  q15_t id_meas = 16'sd1111, iq_meas = -16'sd2222;
  angle_t theta = 16'd33333;
  logic signed [15:0] omega = -16'sd4444;
  logic [7:0] fault_flags = 8'h5A, status_flags = 8'hC3, err_flags = 8'h07;

  cmd_telemetry #(
    .WD_CYC(WD_CYC), .TELEM_CYC(TELEM_CYC),
    .RAMP_INTERVAL(RAMP_INT), .RAMP_STEP(RAMP_STEP)
  ) dut (.*);

  int errors = 0;

  // ---- TX capture --------------------------------------------------------
  byte tx_q [$];
  always @(posedge clk) if (tx_valid) tx_q.push_back(byte'(tx_data));
  int ocal_seen = 0;
  always @(posedge clk) if (offset_cal_start) ocal_seen++;
  // Capture the most recent cal_we strobe for comparison
  int      cal_we_seen = 0;
  logic [3:0]  cal_addr_lat;
  angle_t      cal_data_lat;
  always @(posedge clk) if (cal_we) begin
    cal_we_seen++;
    cal_addr_lat <= cal_addr;
    cal_data_lat <= cal_data;
  end

  // ---- module-level temporaries -----------------------------------------
  q15_t iq_now, iq_prev;
  int   tele_idx;
  int   found_tele;
  logic [15:0] rx_id_v, rx_iq_v;
  logic [7:0]  rx_f_v, rx_s_v, rx_e_v;

  // ---- helpers -----------------------------------------------------------
  function automatic logic [3:0] hex_nibble(input byte c);
    if (c >= "0" && c <= "9") return 4'(c - "0");
    else if (c >= "A" && c <= "F") return 4'(c - "A" + 10);
    else if (c >= "a" && c <= "f") return 4'(c - "a" + 10);
    else return 4'hF;
  endfunction

  // Scan queue for "OK\r\n"
  function automatic int find_ok(ref byte q[$]);
    for (int i = 0; i + 3 < q.size(); i++)
      if (q[i]=="O" && q[i+1]=="K" && q[i+2]==8'h0D && q[i+3]==8'h0A)
        return 1;
    return 0;
  endfunction

  // Scan queue for "?\r\n"
  function automatic int find_err(ref byte q[$]);
    for (int i = 0; i + 2 < q.size(); i++)
      if (q[i]=="?" && q[i+1]==8'h0D && q[i+2]==8'h0A)
        return 1;
    return 0;
  endfunction

  task automatic send_byte(input byte b);
    @(negedge clk);
    rx_data = b; rx_valid = 1;
    @(negedge clk);
    rx_valid = 0;
    repeat (3) @(negedge clk);
  endtask

  task automatic send_text(input string s);
    for (int i = 0; i < s.len(); i++)
      send_byte(byte'(s[i]));
    repeat (TX_SETTLE) @(negedge clk); // let response bytes fully transmit
  endtask

  task automatic drain_tx();
    repeat (TX_SETTLE) @(negedge clk);
    tx_q.delete();
  endtask

  // ---- test body ---------------------------------------------------------
  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- basic commands (decimal args) -----------------------------------
    drain_tx();
    send_text("enable 1\n");
    if (enable !== 1'b1) begin
      $display("  MISMATCH enable not set"); errors++;
    end
    if (!find_ok(tx_q)) begin
      $display("  MISMATCH no OK after enable"); errors++;
    end

    // iq_ref = 16384 = 0x4000
    drain_tx();
    send_text("iq 16384\n");
    if (iq_ref !== 16'sh4000) begin
      $display("  MISMATCH iq_ref=%h (exp 4000)", iq_ref); errors++;
    end
    if (!find_ok(tx_q)) begin
      $display("  MISMATCH no OK after iq"); errors++;
    end

    // kp = 4660 = 0x1234,  ki = 2765 = 0x0ACD
    send_text("kp 4660\n");
    send_text("ki 2765\n");
    if (kp !== 16'sh1234 || ki !== 16'sh0ACD) begin
      $display("  MISMATCH gains kp=%h ki=%h", kp, ki); errors++;
    end

    // hall 7 48879  (idx=7, angle=0xBEEF=48879)
    send_text("hall 7 48879\n");
    if (cal_we_seen == 0 || cal_addr_lat !== 4'd7 || cal_data_lat !== 16'hBEEF) begin
      $display("  MISMATCH cal we=%0d addr=%0d data=%h",
               cal_we_seen, cal_addr_lat, cal_data_lat);
      errors++;
    end

    // offset-cal strobe
    send_text("cal\n");
    if (ocal_seen == 0) begin
      $display("  MISMATCH offset_cal_start never strobed"); errors++;
    end

    // open-loop: vq = 2048 = 0x0800, speed = 20
    send_text("ol 1\n");
    send_text("vq 2048\n");
    send_text("speed 20\n");
    if (ol_mode !== 1'b1 || vq_ol !== 16'sh0800 || ol_speed !== 16'sd20) begin
      $display("  MISMATCH OL cmds mode=%b vq=%h spd=%0d",
               ol_mode, vq_ol, ol_speed);
      errors++;
    end
    send_text("ol 0\n");
    if (ol_mode !== 1'b0) begin
      $display("  MISMATCH OL off"); errors++;
    end

    // negative iq_ref: iq -1000
    send_text("iq -1000\n");
    if (iq_ref !== -16'sd1000) begin
      $display("  MISMATCH negative iq_ref=%0d (exp -1000)", $signed(iq_ref));
      errors++;
    end
    send_text("iq 8192\n"); // = 0x2000

    // ---- unknown keyword → '?' response, state unchanged ----------------
    drain_tx();
    send_text("garbage\n");
    if (!find_err(tx_q)) begin
      $display("  MISMATCH no '?' after unknown keyword"); errors++;
    end
    if (iq_ref !== 16'sh2000) begin
      $display("  MISMATCH iq_ref changed after garbage: %h", iq_ref); errors++;
    end

    // ---- exact keyword match: prefixes/extensions must NOT dispatch -------
    // (safety: there is no checksum, so "exit" must never act as "enable")
    send_text("disable\n");
    if (enable !== 1'b0) begin
      $display("  MISMATCH disable not applied"); errors++;
    end
    drain_tx();
    send_text("exit\n");          // starts with 'e' like "enable"
    if (enable !== 1'b0) begin
      $display("  MISMATCH 'exit' enabled the drive"); errors++;
    end
    if (!find_err(tx_q)) begin
      $display("  MISMATCH no '?' after 'exit'"); errors++;
    end
    drain_tx();
    send_text("en 1\n");          // prefix of "enable"
    send_text("enabled 1\n");     // extension of "enable"
    send_text("iqx 5\n");         // extension of "iq"
    if (enable !== 1'b0 || iq_ref !== 16'sh2000) begin
      $display("  MISMATCH partial keyword changed state en=%b iq=%h",
               enable, iq_ref);
      errors++;
    end
    if (!find_err(tx_q)) begin
      $display("  MISMATCH no '?' for partial keywords"); errors++;
    end
    send_text("ENABLE 1\n");      // case-insensitive full keyword
    if (enable !== 1'b1) begin
      $display("  MISMATCH upper-case enable rejected"); errors++;
    end

    // ---- watchdog + ramp ------------------------------------------------
    begin
      automatic int t_out = 0;
      while (!wd_timeout && t_out < 2 * WD_CYC) begin @(negedge clk); t_out++; end
      if (!wd_timeout) begin
        $display("  MISMATCH watchdog never fired"); errors++;
      end
    end
    iq_prev = iq_ref;
    for (int k = 0; k < 40; k++) begin
      repeat (RAMP_INT) @(negedge clk);
      iq_now = iq_ref;
      if (iq_now > iq_prev) begin
        $display("  MISMATCH ramp not monotone: %0d -> %0d", iq_prev, iq_now);
        errors++;
      end
      iq_prev = iq_now;
    end
    if (iq_ref !== 16'sd0) begin
      $display("  MISMATCH iq_ref=%0d after ramp", iq_ref); errors++;
    end

    // recovery: ping clears timeout; fresh iq required
    send_text("ping\n");
    if (wd_timeout) begin
      $display("  MISMATCH wd_timeout stuck after ping"); errors++;
    end
    if (iq_ref !== 16'sd0) begin
      $display("  MISMATCH iq_ref nonzero after recovery"); errors++;
    end
    send_text("iq 4096\n");
    if (iq_ref !== 16'sh1000) begin
      $display("  MISMATCH iq_ref after recovery=%h", iq_ref); errors++;
    end

    // ---- telemetry: start + format check --------------------------------
    // Drain any leftover bytes, then start telemetry and wait for a full
    // line (OK\r\n = 4 bytes + 48 tele bytes = 52 minimum).
    drain_tx();
    send_byte("t"); send_byte("e"); send_byte("l"); send_byte("e"); send_byte("\n");
    begin
      automatic int t_out = 0;
      automatic int need = N_TELE_TB + 8; // OK(4) + telemetry(48) + margin
      while (tx_q.size() < need && t_out < 4 * TELEM_CYC) begin
        @(negedge clk); t_out++;
      end
      if (tx_q.size() < need) begin
        $display("  MISMATCH telemetry not seen (%0d bytes after %0d cyc)",
                 tx_q.size(), t_out);
        errors++;
      end
    end

    // Locate "id=" in tx_q (skip OK\r\n at start)
    tele_idx = -1;
    for (int i = 0; i + 2 < tx_q.size(); i++) begin
      if (tx_q[i]=="i" && tx_q[i+1]=="d" && tx_q[i+2]=="=") begin
        tele_idx = i; break;
      end
    end
    if (tele_idx < 0) begin
      $display("  MISMATCH telemetry 'id=' prefix not found"); errors++;
    end else if (tele_idx + N_TELE_TB > tx_q.size()) begin
      $display("  MISMATCH telemetry line truncated (idx=%0d size=%0d)",
               tele_idx, tx_q.size());
      errors++;
    end else begin
      // Check label structure at fixed offsets from tele_idx
      if (tx_q[tele_idx+7]  != " "   || tx_q[tele_idx+8]  != "i" ||
          tx_q[tele_idx+9]  != "q"   || tx_q[tele_idx+10] != "=" ||
          tx_q[tele_idx+15] != " "   || tx_q[tele_idx+16] != "t" ||
          tx_q[tele_idx+17] != "h"   || tx_q[tele_idx+18] != "=" ||
          tx_q[tele_idx+23] != " "   || tx_q[tele_idx+24] != "o" ||
          tx_q[tele_idx+25] != "m"   || tx_q[tele_idx+26] != "=" ||
          tx_q[tele_idx+31] != " "   || tx_q[tele_idx+32] != "f" ||
          tx_q[tele_idx+33] != "="   || tx_q[tele_idx+36] != " " ||
          tx_q[tele_idx+37] != "s"   || tx_q[tele_idx+38] != "=" ||
          tx_q[tele_idx+41] != " "   || tx_q[tele_idx+42] != "e" ||
          tx_q[tele_idx+43] != "="   ||
          tx_q[tele_idx+46] != 8'h0D || tx_q[tele_idx+47] != 8'h0A) begin
        $display("  MISMATCH telemetry frame structure"); errors++;
      end else begin
        // Decode id field (hex nibbles at offsets 3..6)
        rx_id_v = {hex_nibble(tx_q[tele_idx+3]),  hex_nibble(tx_q[tele_idx+4]),
                   hex_nibble(tx_q[tele_idx+5]),  hex_nibble(tx_q[tele_idx+6])};
        if (rx_id_v !== 16'(id_meas)) begin
          $display("  MISMATCH telemetry id=%h (exp %h)", rx_id_v, id_meas);
          errors++;
        end
        // Decode iq field (offsets 11..14)
        rx_iq_v = {hex_nibble(tx_q[tele_idx+11]), hex_nibble(tx_q[tele_idx+12]),
                   hex_nibble(tx_q[tele_idx+13]), hex_nibble(tx_q[tele_idx+14])};
        if (rx_iq_v !== 16'(iq_meas)) begin
          $display("  MISMATCH telemetry iq=%h (exp %h)", rx_iq_v, iq_meas);
          errors++;
        end
        // Decode fault (34-35), status (39-40) and err (44-45)
        rx_f_v = {hex_nibble(tx_q[tele_idx+34]), hex_nibble(tx_q[tele_idx+35])};
        rx_s_v = {hex_nibble(tx_q[tele_idx+39]), hex_nibble(tx_q[tele_idx+40])};
        rx_e_v = {hex_nibble(tx_q[tele_idx+44]), hex_nibble(tx_q[tele_idx+45])};
        if (rx_f_v !== fault_flags || rx_s_v !== status_flags
            || rx_e_v !== err_flags) begin
          $display("  MISMATCH telemetry f=%h s=%h e=%h (exp %h %h %h)",
                   rx_f_v, rx_s_v, rx_e_v, fault_flags, status_flags,
                   err_flags);
          errors++;
        end
      end
    end

    // ---- ESC stops telemetry -------------------------------------------
    tx_q.delete();
    send_byte(8'h1B); // ESC
    begin
      automatic int t_out = 0;
      while (t_out < 2 * TELEM_CYC) begin @(negedge clk); t_out++; end
    end
    found_tele = 0;
    for (int i = 0; i + 2 < tx_q.size(); i++) begin
      if (tx_q[i]=="i" && tx_q[i+1]=="d" && tx_q[i+2]=="=")
        found_tele = 1;
    end
    if (found_tele) begin
      $display("  MISMATCH telemetry continued after ESC"); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_cmd_telemetry");
    else $display("TB_FAIL: tb_cmd_telemetry (%0d errors)", errors);
    $finish;
  end

endmodule
