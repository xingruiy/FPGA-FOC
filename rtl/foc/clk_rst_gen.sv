// ============================================================================
// clk_rst_gen.sv
//
//  Single-clock clocking: 100 MHz board oscillator through IBUF -> BUFG
//  (no MMCM/PLL, no CDC anywhere in the design). Reset from the board
//  button: asynchronous assert, synchronous (2-FF) deassert.
// ============================================================================

module clk_rst_gen (
  input  logic clk_in,   // 100 MHz oscillator pin
  input  logic rstn_in,  // active-low reset button
  output logic clk,
  output logic rst_n
);

  logic clk_ibuf;

  IBUF u_ibuf (.I(clk_in), .O(clk_ibuf));
  BUFG u_bufg (.I(clk_ibuf), .O(clk));

  // max_fanout: let synthesis replicate the final stage so the reset
  // deassert recovery path is not one register spraying ~1300 loads
  logic r1;
  (* max_fanout = 50 *) logic r2;

  always_ff @(posedge clk or negedge rstn_in) begin
    if (!rstn_in) begin
      r1 <= 1'b0;
      r2 <= 1'b0;
    end else begin
      r1 <= 1'b1;
      r2 <= r1;
    end
  end

  assign rst_n = r2;

endmodule
