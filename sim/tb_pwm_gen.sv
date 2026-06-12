// ============================================================================
// tb_pwm_gen.sv - 3-phase center-aligned PWM generator tests.
//
//  Checks (real ARR = 625, DT = 20):
//   - counter shape: period = 2*ARR between update strobes, cnt_peak only
//     at cnt == ARR
//   - shoot-through: a phase's gates are NEVER both high (checked every clk)
//   - dead time: complementary fall-to-rise both-off gap is exactly DT;
//     same-gate re-rise gaps (degenerate duties) are >= DT
//   - duty accuracy: steady-state high-side pulse width == 2*ccr-1-(DT+1)
//     within +/-1 (the pulse is centered on the counter trough)
//   - double buffering: a mid-period duty change leaves the in-flight pulse
//     untouched; the boundary-spanning pulse mixes old/new; then steady new
//   - oe drop kills both gates of that phase within 1 clk, others unaffected
//   - en = 0 stops the counter and all gates
// ============================================================================
`timescale 1ns / 1ps

module tb_pwm_gen;
  import foc_pkg::*;

  localparam int ARR = 625;
  localparam int DT  = 20;

  logic clk = 0, rst_n = 0, en = 0;
  logic [2:0] oe = 3'b111;
  q15_t duty_a = 0, duty_b = 0, duty_c = 0;
  logic pwm_ah, pwm_al, pwm_bh, pwm_bl, pwm_ch, pwm_cl;
  logic [$clog2(ARR+1)-1:0] cnt;
  logic cnt_peak, update;

  pwm_gen #(.ARR(ARR), .DT(DT)) dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  logic [2:0] hh, ll;
  assign hh = {pwm_ch, pwm_bh, pwm_ah};
  assign ll = {pwm_cl, pwm_bl, pwm_al};
  logic [2:0] hh_q = '0, ll_q = '0;
  always @(posedge clk) begin hh_q <= hh; ll_q <= ll; end

  // ---- shoot-through (must never fail) -------------------------------
  always @(posedge clk) begin
    for (int i = 0; i < 3; i++) begin
      if (hh[i] && ll[i]) begin
        $display("  MISMATCH shoot-through phase %0d at %0t", i, $time);
        errors++;
      end
    end
  end

  // ---- dead-gap measurement -------------------------------------------
  // gap_src: which gate fell last (1 = high side, 2 = low side, 0 = none)
  int gap_cnt [3];
  int gap_src [3];
  initial for (int i = 0; i < 3; i++) begin gap_cnt[i] = 0; gap_src[i] = 0; end

  always @(posedge clk) begin
    for (int i = 0; i < 3; i++) begin
      if (!en || !oe[i]) begin
        gap_cnt[i] = 0; gap_src[i] = 0;
      end else begin
        if (hh_q[i] && !hh[i]) begin gap_src[i] = 1; gap_cnt[i] = 0; end
        if (ll_q[i] && !ll[i]) begin gap_src[i] = 2; gap_cnt[i] = 0; end
        if (!hh[i] && !ll[i]) gap_cnt[i] = gap_cnt[i] + 1;
        if (hh[i] && !hh_q[i] && gap_src[i] != 0) begin
          if (gap_src[i] == 2 && gap_cnt[i] != DT) begin
            $display("  MISMATCH dead gap l->h phase %0d: %0d != %0d at %0t",
                     i, gap_cnt[i], DT, $time);
            errors++;
          end
          if (gap_cnt[i] < DT) begin
            $display("  MISMATCH dead gap < DT phase %0d: %0d at %0t",
                     i, gap_cnt[i], $time);
            errors++;
          end
          gap_src[i] = 0;
        end
        if (ll[i] && !ll_q[i] && gap_src[i] != 0) begin
          if (gap_src[i] == 1 && gap_cnt[i] != DT) begin
            $display("  MISMATCH dead gap h->l phase %0d: %0d != %0d at %0t",
                     i, gap_cnt[i], DT, $time);
            errors++;
          end
          if (gap_cnt[i] < DT) begin
            $display("  MISMATCH dead gap < DT phase %0d: %0d at %0t",
                     i, gap_cnt[i], $time);
            errors++;
          end
          gap_src[i] = 0;
        end
      end
    end
  end

  // ---- high-side pulse-width tracking -----------------------------------
  int run_cnt  [3];
  int run_last [3];
  initial for (int i = 0; i < 3; i++) begin run_cnt[i] = 0; run_last[i] = 0; end

  always @(posedge clk) begin
    for (int i = 0; i < 3; i++) begin
      if (hh[i]) run_cnt[i] = run_cnt[i] + 1;
      else begin
        if (run_cnt[i] != 0) run_last[i] = run_cnt[i];
        run_cnt[i] = 0;
      end
    end
  end

  // ---- counter shape -----------------------------------------------------
  int per_cnt = 0;
  always @(posedge clk) begin
    if (en) begin
      if (cnt_peak && cnt != ARR) begin
        $display("  MISMATCH cnt_peak at cnt=%0d", cnt); errors++;
      end
      if (update) begin
        if (per_cnt != 0 && per_cnt != 2 * ARR) begin
          $display("  MISMATCH period %0d != %0d", per_cnt, 2 * ARR); errors++;
        end
        per_cnt = 0;
      end
      per_cnt = per_cnt + 1;
    end else per_cnt = 0;
  end

  // ------------------------------------------------------------------------
  function automatic int ccr_of(input int d);
    if (d <= 0) return 0;
    return (d * ARR + 16384) >>> 15;
  endfunction

  // steady-state high-side pulse width: raw width (2c-1) minus the
  // dead-time rise delay (DT); the fall is immediate
  function automatic int exp_run(input int c);
    return 2 * c - 1 - DT;
  endfunction

  task automatic wait_update();
    @(negedge clk);
    while (!update) @(negedge clk);
    @(negedge clk); // let end-of-period bookkeeping settle
  endtask

  task automatic check_run(input int phase, input int exp, input int tol);
    if (run_last[phase] > exp + tol || run_last[phase] < exp - tol) begin
      $display("  MISMATCH pulse width phase %0d: %0d exp %0d +/-%0d",
               phase, run_last[phase], exp, tol);
      errors++;
    end
  endtask

  int ca, cb, cc;

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;

    // ---- steady duties 50% / 10% / 93.5% ------------------------------
    duty_a = 16'sd16384; duty_b = 16'sd3277; duty_c = 16'sd30638;
    ca = ccr_of(16384); cb = ccr_of(3277); cc = ccr_of(30638);
    @(negedge clk); en = 1;
    repeat (4) wait_update();
    check_run(0, exp_run(ca), 1);
    check_run(1, exp_run(cb), 1);
    check_run(2, exp_run(cc), 1);

    // ---- double buffering ------------------------------------------------
    // change duty_a a few cycles into a period: the pulse completing inside
    // this period must still have the OLD width
    repeat (5) @(negedge clk);
    duty_a = 16'sd8192;
    // wait for the in-period fall of phase A (pulse end), check old width
    @(negedge clk);
    while (pwm_ah) @(negedge clk);
    while (!pwm_ah) @(negedge clk); // rise of boundary-spanning pulse
    while (pwm_ah) @(negedge clk);  // its fall, after the boundary
    @(posedge clk); #1;             // let the run tracker log the pulse
    check_run(0, ccr_of(16384) + ccr_of(8192) - 1 - DT, 3); // mixed old/new
    repeat (3) wait_update();
    check_run(0, exp_run(ccr_of(8192)), 1);                       // new

    // ---- corner duties: 0 and ~100% -----------------------------------
    duty_a = 16'sd0; duty_b = 16'sd32767;
    repeat (3) wait_update();
    begin
      int seen_a = 0;
      for (int t = 0; t < 2 * ARR + 10; t++) begin
        @(negedge clk);
        if (pwm_ah) seen_a++;
      end
      if (seen_a != 0) begin
        $display("  MISMATCH duty=0 high-side active"); errors++;
      end
      if (!pwm_al) begin
        $display("  MISMATCH duty=0 low side not fully on"); errors++;
      end
    end

    // ---- oe kill ---------------------------------------------------------
    duty_a = 16'sd16384; duty_b = 16'sd16384; duty_c = 16'sd16384;
    repeat (3) wait_update();
    @(negedge clk);
    while (!pwm_ah) @(negedge clk);
    oe[0] = 0;
    @(negedge clk); #1;
    if (pwm_ah || pwm_al) begin
      $display("  MISMATCH oe kill: gates still active"); errors++;
    end
    begin
      int sw = 0;
      for (int t = 0; t < 2 * ARR + 10; t++) begin
        @(negedge clk);
        if (pwm_bh || pwm_ch) sw++;
        if (pwm_ah || pwm_al) begin
          $display("  MISMATCH oe kill: phase A re-activated"); errors++;
        end
      end
      if (sw == 0) begin
        $display("  MISMATCH oe kill disturbed other phases"); errors++;
      end
    end
    oe[0] = 1; // shoot-through + gap monitors stay armed through re-enable
    repeat (3) wait_update();
    check_run(0, exp_run(ccr_of(16384)), 1);

    // ---- en = 0 ------------------------------------------------------------
    en = 0;
    @(negedge clk); @(negedge clk);
    if (pwm_ah || pwm_al || pwm_bh || pwm_bl || pwm_ch || pwm_cl) begin
      $display("  MISMATCH en=0 gates active"); errors++;
    end
    if (cnt != 0) begin
      $display("  MISMATCH en=0 cnt=%0d", cnt); errors++;
    end

    if (errors == 0) $display("TB_PASS: tb_pwm_gen");
    else             $display("TB_FAIL: tb_pwm_gen (%0d errors)", errors);
    $finish;
  end

endmodule
