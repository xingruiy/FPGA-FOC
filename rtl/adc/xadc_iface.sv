// ============================================================================
// xadc_iface.sv
//
//  Raw XADC primitive wrapper (no wizard IP) for phase-current sampling
//  on the Arty S7-50.
//
//  Configuration (INIT_xx, see UG480):
//   - Event-driven sampling (EC = 1): conversions start on CONVST, which
//    this module pulses on `trigger` (= pwm_gen cnt_peak, the center of
//    the low-side conduction window).
//   - Simultaneous sampling sequencer (SEQ = 0100): VAUX1 and VAUX9
//     convert at the same instant on the two ADCs - phase A and phase B
//     shunt amp outputs MUST be wired to that fixed pair (A1/A2, outer
//     header, both have the on-board 0-3.3V->0-1V resistor divider).
//   - VAUX1/VAUX9 unipolar: the CSA mid-bias (~0.5V at the XADC pin after
//     the board divider) appears as a DC offset that current_offset_cal
//     removes; no external common-mode-shift network required.
//   - DCLK divider 4 -> ADCCLK 25 MHz; one pair conversion ~ 1.1 us,
//     comfortably inside the 12.5 us period.
//
//  Outputs are left-aligned 12-bit codes converted from unipolar
//  offset-binary to two's complement (MSB inverted): mid-bias 0.5V -> ~0,
//  0V -> Q15_MIN, 1V -> ~Q15_MAX. The conversion keeps the zero-current
//  operating point away from the signed wrap, so current_offset_cal can
//  average and subtract the (small) residual offset in plain signed
//  arithmetic. `valid` strobes once per trigger after both reads.
// ============================================================================

module xadc_iface
  import foc_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        trigger,       // cnt_peak strobe
  input  logic [15:0] vauxp,
  input  logic [15:0] vauxn,
  output q15_t        ia_raw,        // VAUX1 - VAUX9 pair, simultaneous
  output q15_t        ib_raw,
  output logic        valid,         // ia/ib updated
  output logic        adc_busy
);

  // ------------------------------------------------------------------
  // XADC primitive
  // ------------------------------------------------------------------
  logic [6:0]  daddr;
  logic        den, drdy;
  logic [15:0] do_bus;
  logic        eoc, eos, busy;
  logic        convst;

  XADC #(
    .INIT_40(16'h0200), // Config0: event-driven (EC), no averaging
    .INIT_41(16'h4F0F), // Config1: SEQ=0100 simultaneous, alarms disabled
    .INIT_42(16'h0400), // Config2: DCLK divider = 4
    .INIT_48(16'h0000), // no internal channels in the sequence
    .INIT_49(16'h0002), // VAUX1 (bit 1) + VAUX9 (bit 9) simultaneous pair
    .INIT_4A(16'h0000), // no averaging
    .INIT_4B(16'h0000),
    .INIT_4C(16'h0000),
    .INIT_4D(16'h0000), // unipolar (DC offset removed by current_offset_cal)
    .INIT_4E(16'h0000), // no acquisition-time extension
    .INIT_4F(16'h0000),
    .SIM_MONITOR_FILE("xadc_stim.txt")
  ) u_xadc (
    .DCLK    (clk),
    .RESET   (~rst_n),
    .DADDR   (daddr),
    .DEN     (den),
    .DI      (16'h0000),
    .DWE     (1'b0),
    .DO      (do_bus),
    .DRDY    (drdy),
    .CONVST  (convst),
    .CONVSTCLK(1'b0),
    .VAUXP   (vauxp),
    .VAUXN   (vauxn),
    .VP      (1'b0),
    .VN      (1'b0),
    .EOC     (eoc),
    .EOS     (eos),
    .BUSY    (busy),
    .CHANNEL (),
    .ALM     (),
    .OT      (),
    .MUXADDR (),
    .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED()
  );

  assign adc_busy = busy;

  // ------------------------------------------------------------------
  // Trigger / DRP readout FSM
  //   trigger -> CONVST (pair VAUX1/VAUX9) -> EOC -> read 0x11, 0x19 -> done
  // ------------------------------------------------------------------
  typedef enum logic [2:0] {
    IDLE, CONV1, WAIT1, RD_A_W, RD_B_W
  } st_t;

  st_t st;
  logic [3:0] convst_cnt;

  localparam logic [6:0] ADDR_VAUX1 = 7'h11;
  localparam logic [6:0] ADDR_VAUX9 = 7'h19;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st         <= IDLE;
      convst     <= 1'b0;
      convst_cnt <= '0;
      den        <= 1'b0;
      daddr      <= '0;
      ia_raw     <= '0;
      ib_raw     <= '0;
      valid      <= 1'b0;
    end else begin
      valid <= 1'b0;
      den   <= 1'b0;

      unique case (st)
        IDLE: begin
          if (trigger) begin
            convst     <= 1'b1;
            convst_cnt <= 4'd8;
            st         <= CONV1;
          end
        end

        CONV1: begin // hold CONVST a few DCLKs
          if (convst_cnt != 0) convst_cnt <= convst_cnt - 1'b1;
          else begin
            convst <= 1'b0;
            st     <= WAIT1;
          end
        end

        WAIT1: if (eoc) begin
          daddr <= ADDR_VAUX1;
          den   <= 1'b1;
          st    <= RD_A_W;
        end

        RD_A_W: if (drdy) begin
          // offset-binary -> two's complement (mid-scale 0x8000 -> 0)
          ia_raw <= q15_t'(do_bus ^ 16'h8000);
          daddr  <= ADDR_VAUX9;
          den    <= 1'b1;
          st     <= RD_B_W;
        end

        RD_B_W: if (drdy) begin
          ib_raw <= q15_t'(do_bus ^ 16'h8000);
          valid  <= 1'b1;
          st     <= IDLE;
        end

        default: st <= IDLE;
      endcase
    end
  end

endmodule
