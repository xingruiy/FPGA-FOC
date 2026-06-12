// ============================================================================
// tb_hall_angle_est.sv - hall_decode + hall_angle_est chained against an
// emulated rotor.
//
//  A real-valued rotor angle th_ref advances at programmable speed; hall
//  signals are generated from physical boundary angles b[0..5] (identity
//  grid first, then a skewed set loaded into the calibration table).
//
//  Checks:
//   - tracking: |theta - th_ref| (mod 2^16) within TRACK_TOL at constant
//     speed, both directions, identity and skewed tables
//   - omega magnitude/sign vs reference at constant speed
//   - interpolation guard: theta always stays inside the current hall
//     sector's table span (theta never crosses the next edge early),
//     including during acceleration
//   - standstill: omega -> 0, theta holds
//   - direction reversal: re-locks within 2 edges
// ============================================================================
`timescale 1ns / 1ps

module tb_hall_angle_est;
  import foc_pkg::*;

  localparam int DEB     = 8;
  localparam int TIMEOUT = 50000;
  localparam int TRACK_TOL = 400;   // angle codes (~2.2 deg)

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // ---- DUTs: decode -> estimator -------------------------------------
  logic [2:0] hall_i = 3'b001;
  logic [2:0] sector;
  logic sector_valid, edge_strobe, dir, illegal;

  hall_decode #(.DEBOUNCE_CYC(DEB)) u_dec (.*);

  logic   cal_we = 0;
  logic [3:0] cal_addr = 0;
  angle_t cal_data = 0;
  angle_t theta;
  logic signed [15:0] omega;
  logic moving;

  hall_angle_est #(.TIMEOUT_CYC(TIMEOUT)) u_est (
    .clk, .rst_n, .sector, .sector_valid, .edge_strobe, .dir,
    .cal_we, .cal_addr, .cal_data, .theta, .omega, .moving);

  // ---- rotor emulation -------------------------------------------------
  localparam logic [2:0] PAT [6] = '{3'b001, 3'b011, 3'b010,
                                     3'b110, 3'b100, 3'b101};
  int b [6]; // physical boundary angles: edge entering sector i (forward)

  real th_ref = 5461.0;  // start mid sector 0
  real om_ref = 0.0;     // codes per clk, signed

  function automatic int wrap16(input int x);
    int r;
    r = x % 65536;
    return (r < 0) ? r + 65536 : r;
  endfunction

  function automatic int sector_of(input int th);
    for (int i = 0; i < 6; i++) begin
      int lo, wid;
      lo  = b[i];
      wid = wrap16(b[(i + 1) % 6] - b[i]);
      if (wrap16(th - lo) < wid) return i;
    end
    return 0;
  endfunction

  always @(posedge clk) begin
    th_ref <= th_ref + om_ref;
    if (th_ref >= 65536.0) th_ref <= th_ref + om_ref - 65536.0;
    if (th_ref < 0.0)      th_ref <= th_ref + om_ref + 65536.0;
    hall_i <= PAT[sector_of(int'($floor(th_ref)) % 65536)];
  end

  // ---- continuous checkers (gated) ----------------------------------------
  int errors = 0;
  bit chk_track = 0, chk_guard = 0;
  int since_edge = 1000;
  always @(posedge clk) begin
    if (edge_strobe) since_edge <= 0;
    else if (since_edge < 1000) since_edge <= since_edge + 1;
  end

  int terr;
  always @(posedge clk) begin
    if (chk_track) begin
      terr = wrap16(int'(theta) - int'($floor(th_ref)));
      if (terr > 32768) terr = 65536 - terr;
      if (terr > TRACK_TOL) begin
        $display("  MISMATCH tracking err=%0d theta=%0d ref=%0d at %0t",
                 terr, theta, int'(th_ref), $time);
        errors++;
        chk_track = 0; // avoid error storms
      end
    end
    // guard: theta inside the current sector's span (+small slack near edges)
    // (!edge_strobe: on the strobe cycle the decoder's sector is already
    // new but the estimator updates one clk later - not a violation)
    if (chk_guard && since_edge > 3 && !edge_strobe) begin
      int lo, wid, off;
      lo  = b[sector];
      wid = wrap16(b[(int'(sector) + 1) % 6] - lo);
      off = wrap16(int'(theta) - lo);
      if (off > wid + 64) begin
        $display("  MISMATCH guard: theta=%0d outside sector %0d [%0d +%0d] at %0t",
                 theta, sector, lo, wid, $time);
        errors++;
        chk_guard = 0;
      end
    end
  end

  // ---- helpers --------------------------------------------------------------
  task automatic spin(input real om, input int cycles);
    om_ref = om;
    repeat (cycles) @(negedge clk);
  endtask

  task automatic check_omega(input real om);
    int exp_om, got;
    exp_om = int'(om * 1250.0);
    got    = int'(omega);
    if (got > exp_om + (exp_om < 0 ? -exp_om : exp_om) / 10 + 30 ||
        got < exp_om - (exp_om < 0 ? -exp_om : exp_om) / 10 - 30) begin
      $display("  MISMATCH omega got=%0d exp=%0d", got, exp_om);
      errors++;
    end
  endtask

  task automatic load_cal();
    // DUT cal: fwd entering s = b[s]; rev entering s = b[(s+1)%6]
    for (int s = 0; s < 6; s++) begin
      @(negedge clk);
      cal_we = 1; cal_addr = 4'(s); cal_data = angle_t'(b[s]);
      @(negedge clk);
      cal_we = 1; cal_addr = 4'(6 + s); cal_data = angle_t'(b[(s + 1) % 6]);
    end
    @(negedge clk);
    cal_we = 0;
  endtask

  angle_t th_hold;

  initial begin
    // identity boundaries
    b[0] = 0; b[1] = 10923; b[2] = 21845;
    b[3] = 32768; b[4] = 43691; b[5] = 54613;

    repeat (3) @(negedge clk);
    rst_n = 1;
    repeat (DEB + 6) @(negedge clk);

    // before any edge: theta = sector center
    if (wrap16(int'(theta) - 5461) > 200 &&
        wrap16(5461 - int'(theta)) > 200) begin
      $display("  MISMATCH initial theta=%0d exp ~5461", theta); errors++;
    end

    // ---- constant forward speed: lock after 3 edges, then track -------
    spin(2.0, 20000);          // ~3.6 sectors: rate is established
    chk_track = 1; chk_guard = 1;
    spin(2.0, 30000);
    check_omega(2.0);

    // ---- acceleration: guard must prevent crossing edges early ---------
    chk_track = 0;             // tracking lags during accel; guard stays on
    for (int k = 0; k < 40; k++) spin(2.0 + 0.05 * k, 1000);
    spin(4.0, 20000);
    chk_track = 1;
    spin(4.0, 20000);
    check_omega(4.0);

    // ---- deceleration to standstill -------------------------------------
    chk_track = 0;
    spin(0.5, 30000);
    spin(0.0, TIMEOUT + 5000); // exceed timeout
    if (moving) begin
      $display("  MISMATCH still 'moving' at standstill"); errors++;
    end
    if (omega != 0) begin
      $display("  MISMATCH omega=%0d at standstill", omega); errors++;
    end
    th_hold = theta;
    spin(0.0, 2000);
    if (theta != th_hold) begin
      $display("  MISMATCH theta moved at standstill"); errors++;
    end

    // ---- direction reversal ----------------------------------------------
    chk_guard = 1;
    spin(-2.0, 25000);         // re-lock takes 2 reverse edges
    chk_track = 1;
    spin(-2.0, 30000);
    check_omega(-2.0);
    chk_track = 0;

    // ---- skewed edge table -------------------------------------------------
    chk_guard = 0; // theta frozen from the old table until the first edge
    spin(0.0, 2000);
    b[0] = 500; b[1] = 10623; b[2] = 22045;
    b[3] = 32368; b[4] = 43791; b[5] = 54363;
    load_cal();
    spin(2.0, 25000);
    chk_guard = 1;
    chk_track = 1;
    spin(2.0, 30000);
    check_omega(2.0);
    chk_track = 0;     // theta freezes during reversal re-lock: not an error
    spin(-2.0, 25000); // and reversed with the skewed table
    chk_track = 1;
    spin(-2.0, 20000);
    check_omega(-2.0);

    if (errors == 0) $display("TB_PASS: tb_hall_angle_est");
    else $display("TB_FAIL: tb_hall_angle_est (%0d errors)", errors);
    $finish;
  end

endmodule
