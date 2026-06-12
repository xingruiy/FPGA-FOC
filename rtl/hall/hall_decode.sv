// ============================================================================
// hall_decode.sv
//
//  Hall sensor front end: 2-FF synchronizer, debounce filter, 6-step sector
//  decode, direction detection and illegal-state flagging.
//
//  Hall code convention ({C,B,A}, 120 deg placement) and sector mapping:
//      001 -> 0, 011 -> 1, 010 -> 2, 110 -> 3, 100 -> 4, 101 -> 5
//  Forward rotation walks sector 0,1,2,3,4,5,0,...  000 and 111 are
//  illegal (broken wire / no power) and never update the sector.
//
//  - A new hall code must be stable for DEBOUNCE_CYC clks to be accepted.
//  - edge_strobe pulses once per accepted legal sector change; dir updates
//    only on single-step transitions (1 = forward).
//  - illegal is a level flag while the debounced code is 000/111.
// ============================================================================

module hall_decode
  import foc_pkg::*;
#(
  parameter int unsigned DEBOUNCE_CYC = 16
)(
  input  logic       clk,
  input  logic       rst_n,
  input  logic [2:0] hall_i,       // async from the motor {C,B,A}
  output logic [2:0] sector,       // 0..5, last legal
  output logic       sector_valid, // at least one legal code seen
  output logic       edge_strobe,  // 1-clk pulse on accepted sector change
  output logic       dir,          // 1 = forward
  output logic       illegal       // debounced code is 000 or 111
);

  // ------------------------------------------------------------------
  // 2-FF synchronizer + debounce
  // ------------------------------------------------------------------
  logic [2:0] h_m, h_s, h_db;
  logic [$clog2(DEBOUNCE_CYC + 1) - 1:0] db_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      h_m  <= '0;
      h_s  <= '0;
      h_db <= '0;
      db_cnt <= '0;
    end else begin
      h_m <= hall_i;
      h_s <= h_m;
      if (h_s == h_db) db_cnt <= '0;
      else if (db_cnt == DEBOUNCE_CYC - 1) begin
        h_db   <= h_s;
        db_cnt <= '0;
      end else db_cnt <= db_cnt + 1'b1;
    end
  end

  // ------------------------------------------------------------------
  // Sector decode
  // ------------------------------------------------------------------
  function automatic logic [2:0] sector_of(input logic [2:0] h);
    unique case (h)
      3'b001: return 3'd0;
      3'b011: return 3'd1;
      3'b010: return 3'd2;
      3'b110: return 3'd3;
      3'b100: return 3'd4;
      3'b101: return 3'd5;
      default: return 3'd7; // illegal marker
    endcase
  endfunction

  logic [2:0] sec_new;
  assign sec_new = sector_of(h_db);
  assign illegal = (sec_new == 3'd7);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sector       <= '0;
      sector_valid <= 1'b0;
      edge_strobe  <= 1'b0;
      dir          <= 1'b1;
    end else begin
      edge_strobe <= 1'b0;
      if (!illegal) begin
        if (!sector_valid) begin
          sector       <= sec_new;
          sector_valid <= 1'b1;
        end else if (sec_new != sector) begin
          sector      <= sec_new;
          edge_strobe <= 1'b1;
          if (sec_new == ((sector == 3'd5) ? 3'd0 : sector + 3'd1))
            dir <= 1'b1;
          else if (sec_new == ((sector == 3'd0) ? 3'd5 : sector - 3'd1))
            dir <= 1'b0;
          // multi-step jumps keep the previous dir (glitch tolerance)
        end
      end
    end
  end

endmodule
