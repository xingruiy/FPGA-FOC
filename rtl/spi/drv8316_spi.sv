// ============================================================================
// drv8316_spi.sv
//
//  SPI master + configuration / fault-poll FSM for the DRV8316.
//
//  Frame (16 bit, MSB first):  {RW, A5..A0, PARITY, D7..D0}
//    RW = 1 read, 0 write; PARITY makes the total number of 1s in the
//    frame even. SPI mode 1 (CPOL=0, CPHA=1) per the DRV8316 datasheet:
//    SCLK idles low, both sides LAUNCH data on the rising edge and the
//    receiver RESOLVES it on the falling edge - this master changes MOSI
//    on the rising edge and samples MISO on the falling edge. CS is held
//    low for half an SCLK period after the last falling edge (CS hold
//    time). SCLK = clk / (2*SCLK_DIV).
//
//  Startup sequence (after `start`):
//    CTRL1 <- unlock, CTRL2 <- 6x PWM + slew, CTRL5 <- CSA gain 1.2 V/A,
//    CTRL4 <- OCP config (driver-level protection only).
//    Every write is READBACK-VERIFIED; a mismatch retries up to 3 times,
//    then cfg_err latches (cfg_done stays low).
//
//  After config: IC_STAT (0x00) and STAT1 (0x01) are polled every
//  POLL_CYC clks; poll_valid strobes when both refresh.
// ============================================================================

module drv8316_spi
  import foc_pkg::*;
#(
  parameter int unsigned SCLK_DIV = 10,     // 100 MHz / (2*10) = 5 MHz SCLK
  parameter int unsigned POLL_CYC = 100_000 // 1 ms fault poll
)(
  input  logic clk,
  input  logic rst_n,
  input  logic start,
  // SPI pins
  output logic sclk,
  output logic mosi,
  output logic cs_n,
  input  logic miso,
  // status
  output logic cfg_done,
  output logic cfg_err,
  output logic [7:0] ic_stat,
  output logic [7:0] stat1,
  output logic poll_valid
);

  // ------------------------------------------------------------------
  // Bit-level SPI engine: one 16-bit transfer per xfer_go pulse
  // ------------------------------------------------------------------
  logic        xfer_go, xfer_busy, xfer_done;
  logic [15:0] tx_frame, rx_frame;

  logic [$clog2(SCLK_DIV)-1:0] div_cnt;
  logic [4:0]  bit_idx; // 15 .. 0
  logic        phase;   // 0 = sclk low half, 1 = sclk high half
  logic        ending;  // CS hold half-period after the last falling edge

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk      <= 1'b0;
      mosi      <= 1'b0;
      cs_n      <= 1'b1;
      xfer_busy <= 1'b0;
      xfer_done <= 1'b0;
      div_cnt   <= '0;
      bit_idx   <= '0;
      phase     <= 1'b0;
      ending    <= 1'b0;
      rx_frame  <= '0;
    end else begin
      xfer_done <= 1'b0;
      if (!xfer_busy) begin
        if (xfer_go) begin
          xfer_busy <= 1'b1;
          cs_n      <= 1'b0;
          bit_idx   <= 5'd15;
          phase     <= 1'b0;
          ending    <= 1'b0;
          div_cnt   <= '0;
          mosi      <= tx_frame[15]; // early, re-launched at the 1st rise
          sclk      <= 1'b0;
        end
      end else if (div_cnt != SCLK_DIV - 1) begin
        div_cnt <= div_cnt + 1'b1;
      end else begin
        div_cnt <= '0;
        if (ending) begin
          // CS hold elapsed: release the bus
          cs_n      <= 1'b1;
          mosi      <= 1'b0;
          ending    <= 1'b0;
          xfer_busy <= 1'b0;
          xfer_done <= 1'b1;
        end else if (!phase) begin
          // rising edge: launch the current bit (slave resolves MOSI on
          // the falling edge; the slave's SDO launches here too)
          sclk  <= 1'b1;
          phase <= 1'b1;
          mosi  <= tx_frame[bit_idx];
        end else begin
          // falling edge: resolve MISO, advance or start the CS hold
          sclk     <= 1'b0;
          phase    <= 1'b0;
          rx_frame <= {rx_frame[14:0], miso};
          if (bit_idx == 0) ending  <= 1'b1;
          else              bit_idx <= bit_idx - 1'b1;
        end
      end
    end
  end

  // frame builder: even parity over the whole 16-bit frame
  function automatic logic [15:0] mk_frame(input logic rw,
                                           input logic [5:0] addr,
                                           input logic [7:0] data);
    logic par;
    par = ^{rw, addr, data}; // parity bit makes total XOR = 0
    return {rw, addr, par, data};
  endfunction

  // ------------------------------------------------------------------
  // Config + poll sequencer
  // ------------------------------------------------------------------
  typedef struct packed {
    logic [5:0] addr;
    logic [7:0] data;
  } cfg_t;

  localparam int unsigned N_CFG = 4;

  // (function + concat cast: xsim elaborates localparam arrays of structs
  // with assignment patterns to zero - do not convert this back)
  function automatic cfg_t cfg_at(input logic [1:0] i);
    unique case (i)
      2'd0:    return cfg_t'({DRV_REG_CTRL1, DRV_CTRL1_CFG});
      2'd1:    return cfg_t'({DRV_REG_CTRL2, DRV_CTRL2_CFG});
      2'd2:    return cfg_t'({DRV_REG_CTRL5, DRV_CTRL5_CFG});
      default: return cfg_t'({DRV_REG_CTRL4, DRV_CTRL4_CFG});
    endcase
  endfunction

  typedef enum logic [2:0] {
    S_IDLE, S_WR, S_WR_WAIT, S_RD, S_RD_WAIT, S_POLL_WAIT, S_POLL
  } st_t;

  st_t st;
  logic [1:0]  cfg_i;
  logic [1:0]  retry;
  logic [31:0] poll_cnt;
  logic        poll_sel; // 0 = IC_STAT, 1 = STAT1

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st         <= S_IDLE;
      cfg_i      <= '0;
      retry      <= '0;
      cfg_done   <= 1'b0;
      cfg_err    <= 1'b0;
      xfer_go    <= 1'b0;
      tx_frame   <= '0;
      poll_cnt   <= '0;
      poll_sel   <= 1'b0;
      ic_stat    <= '0;
      stat1      <= '0;
      poll_valid <= 1'b0;
    end else begin
      xfer_go    <= 1'b0;
      poll_valid <= 1'b0;

      unique case (st)
        S_IDLE: if (start && !cfg_err) begin
          cfg_i <= '0;
          retry <= '0;
          st    <= S_WR;
        end

        S_WR: begin
          tx_frame <= mk_frame(1'b0, cfg_at(cfg_i).addr, cfg_at(cfg_i).data);
          xfer_go  <= 1'b1;
          st       <= S_WR_WAIT;
        end

        S_WR_WAIT: if (xfer_done) st <= S_RD;

        S_RD: begin
          tx_frame <= mk_frame(1'b1, cfg_at(cfg_i).addr, 8'h00);
          xfer_go  <= 1'b1;
          st       <= S_RD_WAIT;
        end

        S_RD_WAIT: if (xfer_done) begin
          if (rx_frame[7:0] == cfg_at(cfg_i).data) begin
            retry <= '0;
            if (cfg_i == 2'(N_CFG - 1)) begin
              cfg_done <= 1'b1;
              poll_cnt <= '0;
              st       <= S_POLL_WAIT;
            end else begin
              cfg_i <= cfg_i + 1'b1;
              st    <= S_WR;
            end
          end else if (retry == 2'd3) begin
            cfg_err <= 1'b1;
            st      <= S_IDLE;
          end else begin
            retry <= retry + 1'b1;
            st    <= S_WR; // rewrite and re-verify
          end
        end

        S_POLL_WAIT: begin
          if (poll_cnt == POLL_CYC - 1) begin
            poll_cnt <= '0;
            poll_sel <= 1'b0;
            st       <= S_POLL;
            tx_frame <= mk_frame(1'b1, DRV_REG_IC_STAT, 8'h00);
            xfer_go  <= 1'b1;
          end else poll_cnt <= poll_cnt + 1'b1;
        end

        S_POLL: if (xfer_done) begin
          if (!poll_sel) begin
            ic_stat  <= rx_frame[7:0];
            poll_sel <= 1'b1;
            tx_frame <= mk_frame(1'b1, DRV_REG_STAT1, 8'h00);
            xfer_go  <= 1'b1;
          end else begin
            stat1      <= rx_frame[7:0];
            poll_valid <= 1'b1;
            st         <= S_POLL_WAIT;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
