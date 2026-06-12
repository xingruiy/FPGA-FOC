// ============================================================================
// tb_drv8316_spi.sv - SPI master + config FSM against a DRV8316 slave model.
//
//  The slave model resolves MOSI on SCLK falling edges and launches MISO
//  on rising edges (DRV8316 SPI mode 1: data launched on the rising edge,
//  resolved on the falling edge), checks even parity on every frame, and
//  commits writes at CS rise.
//  Tests:
//   - config sequence: CTRL1/CTRL2/CTRL5/CTRL4 written with the foc_pkg
//     values, each readback-verified, cfg_done asserted
//   - injected readback mismatch (slave corrupts the first CTRL5 write):
//     master must rewrite and still finish with cfg_done, no cfg_err
//   - every master frame has correct parity
//   - fault poll: IC_STAT/STAT1 reads on the POLL_CYC cadence, outputs
//     reflect the slave register values
// ============================================================================
`timescale 1ns / 1ps

module tb_drv8316_spi;
  import foc_pkg::*;

  localparam int SCLK_DIV = 4;
  localparam int POLL_CYC = 3000;

  logic clk = 0, rst_n = 0, start = 0;
  logic sclk, mosi, cs_n, miso;
  logic cfg_done, cfg_err, poll_valid;
  logic [7:0] ic_stat, stat1;

  drv8316_spi #(.SCLK_DIV(SCLK_DIV), .POLL_CYC(POLL_CYC)) dut (.*);
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // DRV8316 slave model
  // ------------------------------------------------------------------
  logic [7:0] regs [64];
  initial for (int i = 0; i < 64; i++) regs[i] = 8'h00;

  logic [15:0] sr;          // shifted-in frame
  int          nbits;       // rising edges seen this frame
  bit          corrupt_arm = 0; // corrupt the next CTRL5 write
  int          ctrl5_writes = 0;
  int          parity_errs = 0;
  realtime     icstat_reads [$];

  // resolve MOSI on SCLK falling edges (mode 1); launch MISO on rising
  always @(negedge sclk) begin
    if (!cs_n) begin
      sr    <= {sr[14:0], mosi};
      nbits <= nbits + 1;
    end
  end

  logic rw_bit;
  logic [5:0] addr_bits;
  always_comb begin
    rw_bit    = sr[15];     // valid once nbits == 16; during frame use live
    addr_bits = sr[14:9];
  end

  // MISO: for read frames, launch reg data bits on the rising edges of
  // frame bits 7..0 so the master resolves them on the falling edges.
  // RW/addr are latched once the first 8 bits (RW, A5..A0, PARITY) have
  // been captured, because sr keeps shifting afterwards.
  logic       f_rw;
  logic [5:0] f_addr;
  always @(posedge sclk) begin
    if (!cs_n && nbits >= 8 && nbits <= 15) begin
      if (nbits == 8) begin
        f_rw   = sr[7];
        f_addr = sr[6:1];
      end
      if (f_rw) miso <= regs[f_addr][7 - (nbits - 8)];
      else      miso <= 1'b0;
    end else miso <= 1'b0;
  end

  // frame end: commit writes, check parity
  always @(posedge cs_n) begin
    if (nbits == 16) begin
      if (^sr != 1'b0) begin
        parity_errs++;
        $display("  MISMATCH parity error in frame %h at %0t", sr, $time);
      end
      if (!sr[15]) begin // write
        logic [7:0] wdata;
        wdata = sr[7:0];
        if (sr[14:9] == DRV_REG_CTRL5) begin
          ctrl5_writes++;
          if (corrupt_arm) begin
            wdata = wdata ^ 8'hFF; // inject readback mismatch once
            corrupt_arm = 0;
          end
        end
        regs[sr[14:9]] = wdata;
      end else begin
        if (sr[14:9] == DRV_REG_IC_STAT) icstat_reads.push_back($realtime);
      end
    end else if (nbits != 0) begin
      $display("  MISMATCH frame with %0d bits at %0t", nbits, $time);
      errors++;
    end
    nbits = 0;
  end
  always @(negedge cs_n) nbits = 0;

  // ------------------------------------------------------------------
  initial begin
    repeat (5) @(negedge clk);
    rst_n = 1;
    corrupt_arm = 1; // first CTRL5 write gets corrupted

    @(negedge clk); start = 1;
    @(negedge clk); start = 0;

    // wait for config completion
    begin
      int t_out = 0;
      while (!cfg_done && !cfg_err && t_out < 200000) begin
        @(negedge clk); t_out++;
      end
    end
    if (!cfg_done || cfg_err) begin
      $display("  MISMATCH cfg_done=%b cfg_err=%b", cfg_done, cfg_err);
      errors++;
    end

    // registers hold the configured values
    if (regs[DRV_REG_CTRL1] != DRV_CTRL1_CFG ||
        regs[DRV_REG_CTRL2] != DRV_CTRL2_CFG ||
        regs[DRV_REG_CTRL5] != DRV_CTRL5_CFG ||
        regs[DRV_REG_CTRL4] != DRV_CTRL4_CFG) begin
      $display("  MISMATCH configured regs: %h %h %h %h",
               regs[DRV_REG_CTRL1], regs[DRV_REG_CTRL2],
               regs[DRV_REG_CTRL5], regs[DRV_REG_CTRL4]);
      errors++;
    end
    // the corrupted write forced a retry
    if (ctrl5_writes < 2) begin
      $display("  MISMATCH no rewrite after corrupted readback (%0d writes)",
               ctrl5_writes);
      errors++;
    end

    // ---- fault poll ----------------------------------------------------
    regs[DRV_REG_IC_STAT] = 8'hA5;
    regs[DRV_REG_STAT1]   = 8'h3C;
    begin
      int polls = 0, t_out = 0;
      while (polls < 3 && t_out < 10 * POLL_CYC) begin
        @(negedge clk); t_out++;
        if (poll_valid) polls++;
      end
      if (polls < 3) begin
        $display("  MISMATCH only %0d polls seen", polls); errors++;
      end
    end
    if (ic_stat != 8'hA5 || stat1 != 8'h3C) begin
      $display("  MISMATCH poll data ic_stat=%h stat1=%h", ic_stat, stat1);
      errors++;
    end
    // cadence: consecutive IC_STAT reads spaced ~POLL_CYC clks (10 ns each)
    if (icstat_reads.size() >= 2) begin
      realtime dt;
      dt = icstat_reads[icstat_reads.size()-1]
         - icstat_reads[icstat_reads.size()-2];
      if (dt < (POLL_CYC - 200) * 10.0 || dt > (POLL_CYC + 2000) * 10.0) begin
        $display("  MISMATCH poll cadence %.0f ns", dt); errors++;
      end
    end else begin
      $display("  MISMATCH too few IC_STAT reads"); errors++;
    end

    if (parity_errs != 0) errors += parity_errs;

    if (errors == 0) $display("TB_PASS: tb_drv8316_spi");
    else             $display("TB_FAIL: tb_drv8316_spi (%0d errors)", errors);
    $finish;
  end

endmodule
