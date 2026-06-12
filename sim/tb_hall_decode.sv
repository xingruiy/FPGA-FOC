// ============================================================================
// tb_hall_decode.sv - hall front-end tests.
//
//  Spun-rotor stimulus in both directions, glitch injection shorter than
//  the debounce window, and illegal-state (000/111) detection.
// ============================================================================
`timescale 1ns / 1ps

module tb_hall_decode;
  import foc_pkg::*;

  localparam int DEB = 8;

  logic clk = 0, rst_n = 0;
  logic [2:0] hall_i = 3'b001;
  logic [2:0] sector;
  logic sector_valid, edge_strobe, dir, illegal;

  hall_decode #(.DEBOUNCE_CYC(DEB)) dut (.*);
  always #5 clk = ~clk;

  int errors = 0;
  int edges_seen = 0;
  always @(posedge clk) if (edge_strobe) edges_seen++;

  // hall pattern per sector: 001 011 010 110 100 101
  localparam logic [2:0] PAT [6] = '{3'b001, 3'b011, 3'b010,
                                     3'b110, 3'b100, 3'b101};

  task automatic expect_sector(input int s, input bit d);
    if (sector != 3'(s) || !sector_valid) begin
      $display("  MISMATCH sector got=%0d exp=%0d at %0t", sector, s, $time);
      errors++;
    end
    if (dir !== d) begin
      $display("  MISMATCH dir got=%b exp=%b at %0t", dir, d, $time);
      errors++;
    end
  endtask

  initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;
    repeat (DEB + 4) @(negedge clk);
    if (sector != 0 || !sector_valid) begin
      $display("  MISMATCH initial sector %0d", sector); errors++;
    end

    // ---- forward spin: 2 revolutions ---------------------------------
    for (int rev = 0; rev < 2; rev++) begin
      for (int s = 1; s <= 6; s++) begin
        hall_i = PAT[s % 6];
        repeat (DEB + 5) @(negedge clk);
        expect_sector(s % 6, 1'b1);
      end
    end
    if (edges_seen != 12) begin
      $display("  MISMATCH fwd edges %0d != 12", edges_seen); errors++;
    end

    // ---- reverse spin ---------------------------------------------------
    for (int s = 5; s >= 0; s--) begin
      hall_i = PAT[s];
      repeat (DEB + 5) @(negedge clk);
      expect_sector(s, 1'b0);
    end

    // ---- glitch shorter than debounce: must be ignored -----------------
    edges_seen = 0;
    hall_i = PAT[3]; // 3-cycle glitch
    repeat (3) @(negedge clk);
    hall_i = PAT[0];
    repeat (DEB + 5) @(negedge clk);
    if (edges_seen != 0) begin
      $display("  MISMATCH glitch caused %0d edges", edges_seen); errors++;
    end
    expect_sector(0, 1'b0);

    // ---- illegal codes -----------------------------------------------------
    hall_i = 3'b000;
    repeat (DEB + 5) @(negedge clk);
    if (!illegal) begin
      $display("  MISMATCH illegal(000) not flagged"); errors++;
    end
    if (sector != 0) begin
      $display("  MISMATCH sector changed on illegal code"); errors++;
    end
    hall_i = 3'b111;
    repeat (DEB + 5) @(negedge clk);
    if (!illegal) begin
      $display("  MISMATCH illegal(111) not flagged"); errors++;
    end
    // recovery
    hall_i = PAT[1];
    repeat (DEB + 5) @(negedge clk);
    if (illegal) begin
      $display("  MISMATCH illegal stuck after recovery"); errors++;
    end
    expect_sector(1, 1'b1); // 0 -> 1 is a forward step

    if (errors == 0) $display("TB_PASS: tb_hall_decode");
    else             $display("TB_FAIL: tb_hall_decode (%0d errors)", errors);
    $finish;
  end

endmodule
