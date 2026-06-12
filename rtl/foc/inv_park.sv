// ============================================================================
// inv_park.sv
//
//  Inverse Park transform (rotating d/q -> stationary alpha/beta):
//
//      v_alpha = vd * cos(theta) - vq * sin(theta)
//      v_beta  = vd * sin(theta) + vq * cos(theta)
//
//  I/O is Q1.15. Sums of two Q2.30 products computed full width;
//  saturation only at the output, flag exported.
//  Latency: 2 clk (products registered for 100 MHz), in_valid->out_valid.
// ============================================================================

module inv_park
  import foc_pkg::*;
(
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,
  input  q15_t vd,
  input  q15_t vq,
  input  q15_t sin_t,
  input  q15_t cos_t,
  output logic out_valid,
  output q15_t valpha,
  output q15_t vbeta,
  output logic sat
);

  // stage 1: the four products, registered (DSP outputs)
  logic signed [31:0] p_dc, p_qs, p_ds, p_qc;

  always_ff @(posedge clk) begin
    p_dc <= 32'(vd) * 32'(cos_t);
    p_qs <= 32'(vq) * 32'(sin_t);
    p_ds <= 32'(vd) * 32'(sin_t);
    p_qc <= 32'(vq) * 32'(cos_t);
  end

  // stage 2: sums, round, saturate
  logic signed [32:0] a_acc, b_acc;
  logic signed [31:0] a32, b32;

  assign a_acc = 33'(p_dc) - 33'(p_qs);
  assign b_acc = 33'(p_ds) + 33'(p_qc);
  assign a32 = 32'((a_acc + 33'sd16384) >>> 15);
  assign b32 = 32'((b_acc + 33'sd16384) >>> 15);

  always_ff @(posedge clk) begin
    valpha <= sat16(a32);
    vbeta  <= sat16(b32);
    sat    <= (a32 > 32'sd32767) || (a32 < -32'sd32768) ||
              (b32 > 32'sd32767) || (b32 < -32'sd32768);
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
