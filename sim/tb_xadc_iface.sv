// ============================================================================
// tb_xadc_iface.sv - xadc_iface against the real UNISIM XADC model.
//
//  Analog inputs come from sim/xadc_stim.txt (SIM_MONITOR_FILE, staged into
//  the run directory by scripts/simulate.sh). Channels VAUX1 (phase A) and
//  VAUX9 (phase B), both unipolar (INIT_4D=0x0000). Absolute voltages at
//  the XADC pin (after board 0-3.3V -> 0-1V divider); 0A -> ~0.5V mid-bias.
//    t = 0 ns      : A = 0.60 V (+0.10 V above mid), B = 0.40 V (-0.10 V)
//    t = 50 us     : A = 0.45 V (-0.05 V),            B = 0.55 V (+0.05 V)
//    t = 100 us    : A = 0.50 V (mid-bias, 0 A),      B = 0.50 V
//    t = 150 us    : A = 0.70 V (+0.20 V),             B = 0.30 V (-0.20 V)
//
//  Expected codes: unipolar left-aligned, converted offset-binary -> two's
//  complement by the DUT (MSB inverted): code = int(V * 65536) - 32768, so
//  mid-bias 0.5V reads ~0 and below-mid voltages read negative.
//  Checks trigger alignment, channel mapping (A vs B), and sign handling.
//  Tolerance covers model quantization.
// ============================================================================
`timescale 1ns / 1ps

module tb_xadc_iface;
  import foc_pkg::*;

  logic clk = 0, rst_n = 0;
  logic trigger = 0;
  logic [15:0] vauxp = '0, vauxn = '0; // sim values come from the stim file
  q15_t ia_raw, ib_raw;
  logic valid, adc_busy;

  xadc_iface dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  // Code as it appears in ia_raw/ib_raw after the DUT's offset-binary ->
  // two's-complement conversion: mid-bias 0.5V -> 0, below-mid -> negative.
  function automatic int uni_code(input real v_abs);
    return int'(v_abs * 65536.0) - 32768;
  endfunction

  localparam int TOL = 4 * 16; // 4 LSB12 of model/sampling slack

  task automatic sample_and_check(input real va, input real vb);
    int exp_a, exp_b, t_out;
    exp_a = uni_code(va); exp_b = uni_code(vb);

    @(negedge clk); trigger = 1;
    @(negedge clk); trigger = 0;

    t_out = 0;
    while (!valid && t_out < 100000) begin @(negedge clk); t_out++; end
    if (!valid) begin
      $display("  MISMATCH no valid after trigger (timeout)"); errors++;
      return;
    end
    if (int'(ia_raw) > exp_a + TOL || int'(ia_raw) < exp_a - TOL) begin
      $display("  MISMATCH ia=%0d exp=%0d", ia_raw, exp_a); errors++;
    end
    if (int'(ib_raw) > exp_b + TOL || int'(ib_raw) < exp_b - TOL) begin
      $display("  MISMATCH ib=%0d exp=%0d", ib_raw, exp_b); errors++;
    end
  endtask

  initial begin
    repeat (10) @(negedge clk);
    rst_n = 1;
    // let the XADC model initialize/calibrate
    repeat (2000) @(negedge clk);

    // discard one conversion burst: the model's first pass after init
    // returns stale data (real hardware: same flush after FPGA config)
    @(negedge clk); trigger = 1;
    @(negedge clk); trigger = 0;
    begin
      int t_out = 0;
      while (!valid && t_out < 100000) begin @(negedge clk); t_out++; end
    end

    // window 1 (t < 50 us): A=0.60V (+0.10 above mid), B=0.40V (-0.10)
    sample_and_check(0.60, 0.40);

    // move into window 2 (50..100 us): A=0.45V (-0.05), B=0.55V (+0.05)
    while ($time < 60_000) @(negedge clk);
    sample_and_check(0.45, 0.55);

    // window 3 (100..150 us): both at mid-bias 0.50V (zero current)
    while ($time < 110_000) @(negedge clk);
    sample_and_check(0.50, 0.50);

    // window 4 (>150 us): A=0.70V (+0.20), B=0.30V (-0.20)
    while ($time < 160_000) @(negedge clk);
    sample_and_check(0.70, 0.30);

    // no spurious valid without trigger
    begin
      int v_cnt = 0;
      repeat (5000) begin
        @(negedge clk);
        if (valid) v_cnt++;
      end
      if (v_cnt != 0) begin
        $display("  MISMATCH %0d spurious valid strobes", v_cnt); errors++;
      end
    end

    if (errors == 0) $display("TB_PASS: tb_xadc_iface");
    else             $display("TB_FAIL: tb_xadc_iface (%0d errors)", errors);
    $finish;
  end

endmodule
