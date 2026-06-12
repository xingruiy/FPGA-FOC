// ============================================================================
// uart_tx.sv - 8N1 UART transmitter.
//
//  BAUD_DIV = clk / baud (868 -> 115200 at 100 MHz). Handshake: assert
//  tx_valid with tx_data; the byte is accepted when tx_ready is high.
// ============================================================================

module uart_tx #(
  parameter int unsigned BAUD_DIV = 868
)(
  input  logic       clk,
  input  logic       rst_n,
  input  logic [7:0] tx_data,
  input  logic       tx_valid,
  output logic       tx_ready,
  output logic       tx
);

  logic [$clog2(BAUD_DIV)-1:0] baud_cnt;
  logic [3:0] bit_idx;   // 0 start, 1..8 data, 9 stop
  logic [9:0] shifter;
  logic       busy;

  assign tx_ready = !busy;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx       <= 1'b1;
      busy     <= 1'b0;
      baud_cnt <= '0;
      bit_idx  <= '0;
      shifter  <= '1;
    end else if (!busy) begin
      tx <= 1'b1;
      if (tx_valid) begin
        shifter  <= {1'b1, tx_data, 1'b0}; // stop, data LSB-first, start
        busy     <= 1'b1;
        bit_idx  <= '0;
        baud_cnt <= '0;
        tx       <= 1'b0;                  // start bit right away
      end
    end else if (baud_cnt == BAUD_DIV - 1) begin
      baud_cnt <= '0;
      if (bit_idx == 4'd9) begin
        busy <= 1'b0;
        tx   <= 1'b1;
      end else begin
        bit_idx <= bit_idx + 1'b1;
        tx      <= shifter[bit_idx + 1];
      end
    end else baud_cnt <= baud_cnt + 1'b1;
  end

endmodule
