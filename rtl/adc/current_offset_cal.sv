// ============================================================================
// current_offset_cal.sv
//
//  Phase-current offset calibration and third-phase reconstruction.
//
//  - On cal_start, averages CAL_SAMPLES (64) consecutive raw samples per
//    channel at zero average current (gates at 50/50/50 duty) and stores
//    the result as the channel offset.
//  - In normal operation subtracts the offsets and reconstructs
//    ic = -(ia + ib), everything saturating Q1.15.
//  - out_valid tracks in_valid with one cycle latency; cal_done is a
//    level flag (clears while a calibration is running).
// ============================================================================

module current_offset_cal
  import foc_pkg::*;
#(
  parameter int unsigned CAL_SAMPLES = 64 // power of two
)(
  input  logic clk,
  input  logic rst_n,
  input  logic in_valid,    // one strobe per XADC sample pair
  input  q15_t ia_raw,
  input  q15_t ib_raw,
  input  logic cal_start,
  output logic cal_done,
  output logic cal_busy,
  output logic out_valid,
  output q15_t ia,
  output q15_t ib,
  output q15_t ic
);

  localparam int unsigned CAL_SHIFT = $clog2(CAL_SAMPLES);

  logic signed [15 + CAL_SHIFT:0] acc_a, acc_b;
  logic [CAL_SHIFT:0] n;
  logic               calib;
  q15_t               off_a, off_b;

  assign cal_busy = calib;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      calib <= 1'b0;
      cal_done <= 1'b0;
      acc_a <= '0;
      acc_b <= '0;
      n     <= '0;
      off_a <= '0;
      off_b <= '0;
    end else begin
      if (cal_start && !calib) begin
        calib    <= 1'b1;
        cal_done <= 1'b0;
        acc_a    <= '0;
        acc_b    <= '0;
        n        <= '0;
      end else if (calib && in_valid) begin
        acc_a <= acc_a + ia_raw;
        acc_b <= acc_b + ib_raw;
        if (n == (CAL_SHIFT + 1)'(CAL_SAMPLES - 1)) begin
          // this sample completes the set: fold it in and average
          off_a    <= q15_t'((acc_a + ia_raw) >>> CAL_SHIFT);
          off_b    <= q15_t'((acc_b + ib_raw) >>> CAL_SHIFT);
          calib    <= 1'b0;
          cal_done <= 1'b1;
        end
        n <= n + 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_valid <= 1'b0;
      ia <= '0;
      ib <= '0;
      ic <= '0;
    end else begin
      out_valid <= in_valid && !calib;
      if (in_valid) begin
        ia <= sat16(32'(ia_raw) - 32'(off_a));
        ib <= sat16(32'(ib_raw) - 32'(off_b));
        ic <= sat16(-(32'(ia_raw) - 32'(off_a)) - (32'(ib_raw) - 32'(off_b)));
      end
    end
  end

endmodule
