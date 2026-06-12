// ============================================================================
// clarke.sv
//
//  Amplitude-invariant Clarke transform from two measured phase currents
//  (ic is implied by ia + ib + ic = 0, reconstructed upstream):
//
//      i_alpha = ia
//      i_beta  = (ia + 2*ib) / sqrt(3)
//
//  I/O is Q1.15; the (ia + 2*ib) intermediate spans +/-3.0 so the math is
//  done full width and only the output is saturated (sat flag exported).
//  Latency: 2 clk (product registered for 100 MHz), in_valid -> out_valid.
// ============================================================================

module clarke
  import foc_pkg::*;
(
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,
  input  q15_t ia,
  input  q15_t ib,
  output logic out_valid,
  output q15_t ialpha,
  output q15_t ibeta,
  output logic sat
);

  localparam logic signed [15:0] INV_SQRT3_Q15 = 16'sd18919; // 1/sqrt(3)

  logic signed [17:0] s;     // ia + 2*ib, scale 2^15, range +/-3.0
  logic signed [31:0] p_r;   // registered product, scale 2^30
  q15_t               ia_d;
  logic signed [31:0] beta32;

  assign s = 18'(ia) + (18'(ib) <<< 1);

  always_ff @(posedge clk) begin
    // stage 1: multiply
    p_r  <= 32'(s) * 32'(INV_SQRT3_Q15);
    ia_d <= ia;
  end

  assign beta32 = rnd_shr(p_r, 15);

  always_ff @(posedge clk) begin
    // stage 2: round + saturate
    ialpha <= ia_d;
    ibeta  <= sat16(beta32);
    sat    <= (beta32 > 32'sd32767) || (beta32 < -32'sd32768);
  end

  logic v1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v1        <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      v1        <= in_valid;
      out_valid <= v1;
    end
  end

endmodule
