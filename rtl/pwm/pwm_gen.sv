// ============================================================================
// pwm_gen.sv
//
//  3-phase center-aligned complementary PWM with dead-time insertion.
//
//  - One shared up/down counter: 0 .. ARR .. 0, period = 2*ARR clk cycles
//  - Duties are Q1.15 in [0, 1); compare values ccr = round(duty * ARR)
//    are latched into a per-phase shadow register at the period boundary
//    (update), so mid-period duty changes are glitch-free. Callers must
//    hold duty_* stable across that boundary (registered, same clock).
//  - Active level: high-side on while (cnt < ccr), so the high-side pulse
//    is centered on the counter trough and the low-side conduction window
//    is centered on the counter PEAK, where `cnt_peak` strobes - that is
//    the XADC sampling trigger for low-side CSAs.
//  - Dead time: after either gate switches off, the complementary gate
//    waits exactly DT cycles with both off before switching on.
//  - oe[x] = 0 forces both gates of that phase low within 1 clk and
//    re-arms the dead-time timers. en = 0 stops the counter and all gates.
// ============================================================================

module pwm_gen
  import foc_pkg::*;
#(
  parameter int unsigned ARR = PWM_ARR,
  parameter int unsigned DT  = DEADTIME_CYC
)(
  input  logic clk,
  input  logic rst_n,
  input  logic en,
  input  logic [2:0] oe,          // per-phase output enable {c, b, a}
  input  q15_t duty_a,
  input  q15_t duty_b,
  input  q15_t duty_c,
  output logic pwm_ah, pwm_al,
  output logic pwm_bh, pwm_bl,
  output logic pwm_ch, pwm_cl,
  output logic [$clog2(ARR + 1) - 1:0] cnt,
  output logic cnt_peak,          // 1-clk strobe at counter peak
  output logic update             // 1-clk strobe at period boundary
);

  localparam int unsigned CNT_W = $clog2(ARR + 1);
  localparam int unsigned DT_W  = $clog2(DT + 1);

  // ------------------------------------------------------------------
  // Up/down counter
  // ------------------------------------------------------------------
  logic down;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cnt  <= '0;
      down <= 1'b0;
    end else if (!en) begin
      cnt  <= '0;
      down <= 1'b0;
    end else if (!down) begin
      if (cnt == CNT_W'(ARR - 1)) down <= 1'b1;
      cnt <= cnt + 1'b1;
    end else begin
      if (cnt == CNT_W'(1)) down <= 1'b0;
      cnt <= cnt - 1'b1;
    end
  end

  assign cnt_peak = en && (cnt == CNT_W'(ARR));
  assign update   = en && (cnt == '0);

  // ------------------------------------------------------------------
  // Duty -> compare value, latched into a shadow register at the period boundary
  // ------------------------------------------------------------------
  function automatic logic [CNT_W-1:0] ccr_of(input q15_t d);
    logic signed [31:0] p;
    if (d <= 0) return '0;
    p = (32'(d) * 32'(ARR) + 32'sd16384) >>> 15;
    return CNT_W'(p);
  endfunction

  logic [CNT_W-1:0] ccr [3];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ccr[0] <= '0; ccr[1] <= '0; ccr[2] <= '0;
    end else if (!en || cnt == '0) begin // track while stopped, else at update
      ccr[0] <= ccr_of(duty_a);
      ccr[1] <= ccr_of(duty_b);
      ccr[2] <= ccr_of(duty_c);
    end
  end

  // ------------------------------------------------------------------
  // Per-phase complementary outputs with dead time
  // ------------------------------------------------------------------
  logic [2:0] raw, h, l;

  assign raw[0] = en && (cnt < ccr[0]);
  assign raw[1] = en && (cnt < ccr[1]);
  assign raw[2] = en && (cnt < ccr[2]);

  for (genvar i = 0; i < 3; i++) begin : g_phase
    logic [DT_W-1:0] t_h, t_l;

    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        h[i] <= 1'b0;  l[i] <= 1'b0;
        t_h  <= DT_W'(DT);
        t_l  <= DT_W'(DT);
      end else if (!en || !oe[i]) begin
        h[i] <= 1'b0;  l[i] <= 1'b0;
        t_h  <= DT_W'(DT);
        t_l  <= DT_W'(DT);
      end else begin
        // high side: on only after raw has been high for DT cycles
        if (!raw[i]) begin
          h[i] <= 1'b0;
          t_h  <= DT_W'(DT);
        end else if (t_h != '0) t_h <= t_h - 1'b1;
        else                    h[i] <= 1'b1;
        // low side: complementary, same dead-time rule
        if (raw[i]) begin
          l[i] <= 1'b0;
          t_l  <= DT_W'(DT);
        end else if (t_l != '0) t_l <= t_l - 1'b1;
        else                    l[i] <= 1'b1;
      end
    end
  end

  assign {pwm_ah, pwm_al} = {h[0], l[0]};
  assign {pwm_bh, pwm_bl} = {h[1], l[1]};
  assign {pwm_ch, pwm_cl} = {h[2], l[2]};

endmodule
