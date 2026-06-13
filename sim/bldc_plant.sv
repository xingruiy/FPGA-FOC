// ============================================================================
// bldc_plant.sv  (simulation only)
//
//  Electrical model of the Moons ECU16052H24-S002 on a 24 V bus:
//  per-phase RL + sinusoidal back-EMF, driven by per-PWM-period averaged
//  inverter voltages (duty inputs). The rotor angle/speed are EXTERNAL
//  inputs (idealized angle source per the plan's non-goals).
//
//    R = 1.58 ohm, L = 0.127 mH (per phase; datasheet line-to-line / 2),
//    Ke = 0.01485 V*s/rad (= 643 rpm/V)
//
//  Euler integration once per `update` (Ts = 12.5 us, tau_e/Ts = 6.4).
//  Back-EMF convention matches the RTL transforms: e_d = 0,
//  e_q = omega*Ke, i.e.  e_a = -w*Ke*sin(th), e_b/e_c shifted -/+120 deg.
//
//  Outputs both ampere reals and Q1.15 codes on the +/-1.25 A XADC scale.
//  `poke` adds poke_amps to phase A (overcurrent-trip testing).
// ============================================================================
`timescale 1ns / 1ps

module bldc_plant
  import foc_pkg::*;
#(
  parameter real R_OHM = 1.58,
  parameter real L_H   = 0.127e-3,
  parameter real KE    = 0.01485,
  parameter real VBUS  = 24.0,
  parameter real TS    = 12.5e-6
)(
  input  logic clk,
  input  logic rst_n,
  input  logic update,        // once per PWM period
  input  logic gates_on,      // 0: inverter floating -> currents decay to 0
  input  q15_t duty_a,
  input  q15_t duty_b,
  input  q15_t duty_c,
  input  real  theta_e,       // electrical angle, rad
  input  real  omega_e,       // electrical speed, rad/s
  input  logic poke,
  input  real  poke_amps,
  output real  ia_A,
  output real  ib_A,
  output real  ic_A,
  output q15_t ia_q,
  output q15_t ib_q
);

  localparam real PI = 3.14159265358979323846;
  localparam real I_FS = 1.25;

  real ia_r = 0.0, ib_r = 0.0;
  real da, db, dc, davg, van, vbn, vcn, ea, eb, ec;

  function automatic q15_t amps_to_q15(input real a);
    real c;
    c = a / I_FS * 32768.0;
    if (c > 32767.0)  return 16'sd32767;
    if (c < -32768.0) return -16'sd32768;
    return q15_t'(int'(c));
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      ia_r = 0.0;
      ib_r = 0.0;
    end else begin
      if (poke) ia_r = ia_r + poke_amps;
      if (update) begin
        if (!gates_on) begin
          // freewheel approximation: fast decay through body diodes
          ia_r = ia_r * 0.5;
          ib_r = ib_r * 0.5;
        end else begin
          da = real'(duty_a) / 32768.0;
          db = real'(duty_b) / 32768.0;
          dc = real'(duty_c) / 32768.0;
          davg = (da + db + dc) / 3.0;
          van = (da - davg) * VBUS;
          vbn = (db - davg) * VBUS;
          vcn = (dc - davg) * VBUS;
          ea = -omega_e * KE * $sin(theta_e);
          eb = -omega_e * KE * $sin(theta_e - 2.0 * PI / 3.0);
          ec = -omega_e * KE * $sin(theta_e + 2.0 * PI / 3.0);
          ia_r = ia_r + TS / L_H * (van - ea - R_OHM * ia_r);
          ib_r = ib_r + TS / L_H * (vbn - eb - R_OHM * ib_r);
        end
      end
    end
  end

  assign ia_A = ia_r;
  assign ib_A = ib_r;
  assign ic_A = -ia_r - ib_r;
  assign ia_q = amps_to_q15(ia_r);
  assign ib_q = amps_to_q15(ib_r);

endmodule
