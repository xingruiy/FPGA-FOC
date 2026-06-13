// ============================================================================
// pi_controller.sv
//
//  Generic sampled PI controller for the d/q current loops.
//
//  Equations (one step per `strobe`, all bit-exact, documented for the
//  golden model):
//      e_k   = sat16(sp - fb)                              Q1.15
//      u_k   = sat16( round_12( kp*e_k + I_k ) )           Q1.15
//      I_k+1 = clamp( I_k + ki*e_k + ((applied_k-1 - u_k-1) << 12) )
//
//  - Gains kp, ki are Q4.12 (range +/-8); ki has the sample time folded in
//    (ki = Ki_cont * Ts in the same scale).
//  - Integrator I is Q5.27 in 32 bits, clamped to +/-1.0 so it can drive
//    the output to full scale but no further.
//  - Anti-windup is back-calculation from the *applied* output: `applied`
//    must be driven with the post-limiter value of the previous output
//    sample (wire u -> external d/q limiter -> applied). When nothing
//    clamps, applied == previous u and the correction term is zero.
//
//  Latency: 3 clks from strobe (error, gain multiplies, sum/update are
//  separate stages for 100 MHz); out_valid marks the updated output.
//  The difference equations above are computed exactly as written -
//  sp/fb are sampled at the strobe, acc/u update at stage 3.
// ============================================================================

module pi_controller
  import foc_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        clr,      // hold the controller state cleared
                                // (loop disabled - STM32 foc_enable
                                // semantics: re-enable starts fresh)
  input  logic        strobe,   // one PI step per pulse (PWM rate)
  input  q15_t        sp,       // setpoint
  input  q15_t        fb,       // feedback
  input  logic signed [15:0] kp, // Q4.12
  input  logic signed [15:0] ki, // Q4.12
  input  q15_t        applied,  // previous output after external limiting
  output logic        out_valid,
  output q15_t        u,
  output logic        usat      // output left Q1.15 before saturation
);

  localparam logic signed [31:0] ACC_MAX = 32'sd1 <<< 27; // +1.0 in Q5.27

  logic signed [31:0] acc;     // integrator, Q5.27
  q15_t               u_prev;  // previous unclamped-by-us output

  // stage 1 (strobe): sample the error
  q15_t err_r;
  logic s1, s2;

  // stage 2: gain multiplies, registered (DSP outputs)
  logic signed [31:0] p_r, i_r;

  // stage 3: output sum + integrator update
  logic signed [31:0] aw_corr, acc_next, u32;

  assign aw_corr = (32'(applied) - 32'(u_prev)) <<< 12;   // Q5.27
  assign u32     = rnd_shr(p_r + acc, 12);                // -> Q1.15

  always_comb begin
    acc_next = acc + i_r + aw_corr;
    if      (acc_next >  ACC_MAX) acc_next =  ACC_MAX;
    else if (acc_next < -ACC_MAX) acc_next = -ACC_MAX;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      err_r     <= '0;
      s1        <= 1'b0;
      s2        <= 1'b0;
      p_r       <= '0;
      i_r       <= '0;
      acc       <= '0;
      u_prev    <= '0;
      u         <= '0;
      usat      <= 1'b0;
      out_valid <= 1'b0;
    end else if (clr) begin
      s1        <= 1'b0;
      s2        <= 1'b0;
      out_valid <= 1'b0;
      acc       <= '0;
      u_prev    <= '0;
      u         <= '0;
      usat      <= 1'b0;
    end else begin
      s1        <= strobe;
      s2        <= s1;
      out_valid <= s2;
      if (strobe) err_r <= sat16(32'(sp) - 32'(fb));
      if (s1) begin
        p_r <= 32'(kp) * 32'(err_r); // Q5.27
        i_r <= 32'(ki) * 32'(err_r); // Q5.27
      end
      if (s2) begin
        u      <= sat16(u32);
        usat   <= (u32 > 32'sd32767) || (u32 < -32'sd32768);
        u_prev <= sat16(u32);
        acc    <= acc_next;
      end
    end
  end

endmodule
