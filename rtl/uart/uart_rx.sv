// ============================================================================
// uart_rx.sv - 8N1 UART receiver with mid-bit sampling.
//
//  2-FF input synchronizer; the start-bit edge arms a counter and every
//  bit is sampled at its center, which tolerates a few percent of baud
//  mismatch (verified to +/-2 % in the TB). frame_err strobes on a bad
//  stop bit (the byte is discarded).
// ============================================================================

module uart_rx #(
  parameter int unsigned BAUD_DIV = 868
)(
  input  logic       clk,
  input  logic       rst_n,
  input  logic       rx,
  output logic [7:0] rx_data,
  output logic       rx_valid,
  output logic       frame_err
);

  // 2-FF synchronizer
  logic rx_m, rx_s;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_m <= 1'b1; rx_s <= 1'b1; end
    else        begin rx_m <= rx;   rx_s <= rx_m; end
  end

  logic [$clog2(BAUD_DIV + BAUD_DIV/2)-1:0] baud_cnt;
  logic [3:0] bit_idx; // 0 = start verify, 1..8 data, 9 stop
  logic [7:0] sh;
  logic       busy;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy      <= 1'b0;
      baud_cnt  <= '0;
      bit_idx   <= '0;
      sh        <= '0;
      rx_data   <= '0;
      rx_valid  <= 1'b0;
      frame_err <= 1'b0;
    end else begin
      rx_valid  <= 1'b0;
      frame_err <= 1'b0;
      if (!busy) begin
        if (!rx_s) begin // start edge: sample center of start bit first
          busy     <= 1'b1;
          bit_idx  <= '0;
          baud_cnt <= '0;
        end
      end else if (bit_idx == 0 && baud_cnt == BAUD_DIV/2 - 1) begin
        // middle of the start bit: must still be low, else glitch
        baud_cnt <= '0;
        if (rx_s) busy <= 1'b0;
        else      bit_idx <= 4'd1;
      end else if (bit_idx != 0 && baud_cnt == BAUD_DIV - 1) begin
        baud_cnt <= '0;
        if (bit_idx <= 4'd8) begin
          sh      <= {rx_s, sh[7:1]}; // LSB first
          bit_idx <= bit_idx + 1'b1;
        end else begin // stop bit
          busy <= 1'b0;
          if (rx_s) begin
            rx_data  <= sh;
            rx_valid <= 1'b1;
          end else frame_err <= 1'b1;
        end
      end else baud_cnt <= baud_cnt + 1'b1;
    end
  end

endmodule
