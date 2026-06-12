// ============================================================================
// park.sv
//
//  Park transform (stationary alpha/beta -> rotating d/q):
//
//      id =  i_alpha * cos(theta) + i_beta * sin(theta)
//      iq = -i_alpha * sin(theta) + i_beta * cos(theta)
//
//  I/O is Q1.15 (sin/cos from sincos_lut). Each output is a sum of two
//  Q2.30 products (range +/-sqrt(2) for in-range vectors) computed full
//  width; saturation only at the output, flag exported.
//  Latency: 2 clk (products registered for 100 MHz), in_valid->out_valid.
// ============================================================================

module park
  import foc_pkg::*;
(
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,
  input  q15_t ialpha,
  input  q15_t ibeta,
  input  q15_t sin_t,
  input  q15_t cos_t,
  output logic out_valid,
  output q15_t id,
  output q15_t iq,
  output logic sat
);

  // stage 1: the four products, registered (DSP outputs)
  logic signed [31:0] p_ac, p_bs, p_bc, p_as;

  always_ff @(posedge clk) begin
    p_ac <= 32'(ialpha) * 32'(cos_t);
    p_bs <= 32'(ibeta)  * 32'(sin_t);
    p_bc <= 32'(ibeta)  * 32'(cos_t);
    p_as <= 32'(ialpha) * 32'(sin_t);
  end

  // stage 2: sums, round Q2.30 -> Q1.15, saturate
  logic signed [32:0] d_acc, q_acc;
  logic signed [31:0] d32, q32;

  assign d_acc = 33'(p_ac) + 33'(p_bs);
  assign q_acc = 33'(p_bc) - 33'(p_as);
  assign d32 = 32'((d_acc + 33'sd16384) >>> 15);
  assign q32 = 32'((q_acc + 33'sd16384) >>> 15);

  always_ff @(posedge clk) begin
    id  <= sat16(d32);
    iq  <= sat16(q32);
    sat <= (d32 > 32'sd32767) || (d32 < -32'sd32768) ||
           (q32 > 32'sd32767) || (q32 < -32'sd32768);
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
