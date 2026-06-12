// ============================================================================
// tb_uart.sv - UART TX -> RX loopback plus baud-tolerance tests.
//
//  - loopback: 64 random bytes through uart_tx into uart_rx at the same
//    BAUD_DIV, all received intact, no frame errors
//  - tolerance: TB bit-bangs the rx line at +2 % and -2 % bit period:
//    bytes must still be received; a broken stop bit must raise frame_err
//    and not produce a byte
// ============================================================================
`timescale 1ns / 1ps

module tb_uart;

  localparam int BAUD_DIV = 100; // small divider keeps the sim short

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // TX -> RX loopback wiring
  logic [7:0] tx_data;
  logic tx_valid = 0, tx_ready;
  logic line;
  logic [7:0] rx_data;
  logic rx_valid, frame_err;
  logic rx_drive = 1'b1;   // TB override for bit-banging
  logic use_tb_drive = 0;

  uart_tx #(.BAUD_DIV(BAUD_DIV)) u_tx
    (.clk, .rst_n, .tx_data, .tx_valid, .tx_ready, .tx(line));
  logic rx_line;
  assign rx_line = use_tb_drive ? rx_drive : line;
  uart_rx #(.BAUD_DIV(BAUD_DIV)) u_rx
    (.clk, .rst_n, .rx(rx_line), .rx_data, .rx_valid, .frame_err);

  int errors = 0;
  int ferr_cnt = 0;
  byte rx_q [$];
  always @(posedge clk) begin
    if (rx_valid)  rx_q.push_back(byte'(rx_data));
    if (frame_err) ferr_cnt++;
  end

  task automatic send_byte(input byte b);
    @(negedge clk);
    while (!tx_ready) @(negedge clk);
    tx_data = b; tx_valid = 1;
    @(negedge clk);
    tx_valid = 0;
  endtask

  // bit-bang a byte on rx with a scaled bit period (in clk cycles)
  task automatic bang_byte(input byte b, input int bit_cyc,
                           input bit good_stop);
    rx_drive = 0;                       // start
    repeat (bit_cyc) @(negedge clk);
    for (int i = 0; i < 8; i++) begin
      rx_drive = b[i];
      repeat (bit_cyc) @(negedge clk);
    end
    rx_drive = good_stop;               // stop
    repeat (bit_cyc) @(negedge clk);
    rx_drive = 1;
    repeat (bit_cyc) @(negedge clk);    // idle gap
  endtask

  byte exp_q [$];
  byte b;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- loopback -----------------------------------------------------
    for (int i = 0; i < 64; i++) begin
      b = byte'($urandom());
      exp_q.push_back(b);
      send_byte(b);
    end
    // drain
    repeat (12 * BAUD_DIV) @(negedge clk);
    if (rx_q.size() != 64) begin
      $display("  MISMATCH loopback got %0d/64 bytes", rx_q.size()); errors++;
    end else begin
      for (int i = 0; i < 64; i++) begin
        if (rx_q[i] != exp_q[i]) begin
          $display("  MISMATCH byte %0d: %h != %h", i, rx_q[i], exp_q[i]);
          errors++;
        end
      end
    end
    if (ferr_cnt != 0) begin
      $display("  MISMATCH loopback frame errors: %0d", ferr_cnt); errors++;
    end

    // ---- baud tolerance: +2% and -2% ------------------------------------
    use_tb_drive = 1; rx_q.delete();
    repeat (4 * BAUD_DIV) @(negedge clk);
    bang_byte(8'hA7, BAUD_DIV + 2, 1'b1);  // +2 %
    bang_byte(8'h31, BAUD_DIV - 2, 1'b1);  // -2 %
    repeat (4 * BAUD_DIV) @(negedge clk);
    if (rx_q.size() != 2 || rx_q[0] != 8'hA7 || rx_q[1] != 8'h31) begin
      $display("  MISMATCH baud tolerance: got %0d bytes", rx_q.size());
      errors++;
    end

    // ---- broken stop bit -> frame_err, no byte ---------------------------
    rx_q.delete(); ferr_cnt = 0;
    bang_byte(8'h5C, BAUD_DIV, 1'b0);
    repeat (4 * BAUD_DIV) @(negedge clk);
    if (ferr_cnt == 0) begin
      $display("  MISMATCH no frame_err on broken stop"); errors++;
    end
    if (rx_q.size() != 0) begin
      $display("  MISMATCH byte accepted despite broken stop"); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_uart");
    else             $display("TB_FAIL: tb_uart (%0d errors)", errors);
    $finish;
  end

endmodule
