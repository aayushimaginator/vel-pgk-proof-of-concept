# RESULTS.md - PGK Power & Control Simulation Campaign
Repo: VEL/pgk_power_sim | branch: power | Last updated: v7.5 (2026-09-30)
Supersedes all prior entries. Owner: power/control subsystem.

## 0. How to read this log
- Deterministic = single NOMINAL run. MC = Monte Carlo with jitter: V0 +/-2%,
  canard CL +/-20%, drag +/-10%, crosswind 3 m/s 1-sigma, gyro noise 0.005 rad/s.
- Authority = directed lateral correction from dwell events. Direction-honest
  from v7.2 on (counts only arrested-hold time, |w_nose| < 2 rad/s).
- resid95 = 95th-percentile |crosswind + sensor residual| after correction.
  CEP PROXY ONLY: excludes wind shear, muzzle-V spread beyond +/-2%, IMU bias
  drift, cant error. Full dispersion CEP pending 6-DoF + guidance-in-loop.
- Acceptance (current): authority mean >= 3x resid95; vsc_min >= 12 V soft /
  >= 9.5 V hard (wide-Vin buck floor); boot < 3 s; wake < 2 s; range sanity
  17-20 km with flat-atmosphere caveat (see 6).

## 1. Baseline index
| Config | Tag / commit        | Aero               | Power arch                    | MC N | Mean auth | resid95 | Vsc worst |
| v6.6   | sim-v6.6-mc-baseline| 680-only, pgk-ref  | commanded harvest, 3 F cap    | 200  | 78.3 m    | 5.9 m   | 23.58 V   |
| v7.4   | sim-v7.4-final c914790 | 680-only, body-ref | PWM-connect buck, 1.5 F, 2.2 R | 1300 | 51.4 m  | 6.2 m   | 11.87 V   |
| v7.5   | pending sim-v7.5-aero3table | 3-table + F5 corr | same as v7.4             | 1300 | 36.8 m    | 6.2 m   | 15.09 V   |

## 2. v7.5 - CURRENT (3-table CFD aero + F5 normalization correction)
Deterministic NOMINAL:
- wake/despin 0.88 s | connect 0.88 s | boot 1.99 s | gate 3.48 s
- schedule 4.9 / 16.9 / 31.9 s | dwell 0.50 / 0.50 / 0.90 s (arrest-time-aware)
- q at events 150 / 57 / 33 kPa | Vsc 18.99 / 20.16 / 18.96 V
- authority 28.1 / 7.4 / 3.5 m = 39.0 m
- harvest 2792 J | brake 1 J | peak fin 10210 mNm (canard damping at lock)
- peak harvest 280 mNm (burst clamp only) | peak duty 1.00
- supercap final 16.98 V | range 15.9 km (flat-atmo confound, see 6)
MC N=1300 (13 workers x 100):
- mean 36.8 m | std 6.8 m | resid95 6.2 m | Vsc worst 15.09 V -> PASS
Aero data state:
- Cd0 corrected: 0.391 @M2 / 0.461 @M1.2 / 0.230 @M0.8.
- F5 RESOLVED IN SOFTWARE: 270/410 tables were post-processed with q fixed at
  680 m/s; correction factor (680/V)^2 active (opts.f5_fix=true).
- Acceptance test for aero source fix: when they confirm refs fixed at source,
  set f5_fix=false; det + MC must reproduce the v7.5 bands above.
- Data defects logged: LUT_for_GAZEBO_410_changed DP0 row P4=270 typo; DP0
  coefficient blanks in all low-V tables (loader drops NaN rows); PGK_LUT.csv
  superseded by LUT_for_GAZEBO_680.csv (archive it).

## 3. v7.4 / v7.3 - validated 680-only baseline (tag sim-v7.4-final)
Deterministic: wake 0.88 | connect 0.88 | boot 1.98 | gate 3.48 s;
schedule 4.9/16.9/31.9; dwell 0.50 x3; q 171/94/58 kPa; Vsc 18.99/19.89/17.57 V;
authority 29.9/12.5/5.1 = 47.6 m; brake 20 J; final 14.65 V; range 19.6 km.
MC N=1300: mean 51.4 m | std 4.0 m | resid95 6.2 m | Vsc worst 11.87 V.
- 11.87 V misses 12 V soft by 0.13 V; PASSES hard floor 9.5 V with 2.37 V
  margin. Accepted and documented; tail case = high drag + late wake.
- v7.3 introduced arrest-time-aware dwell + burst hysteresis (16-19 V band);
  v7.4 = same physics + 1300-run validation.

## 4. v7.0-v7.2 - architecture transition (superseded, traceability only)
- v7.0: physical shell-fin ODE (2 x 357 mm2 @ 7.5 deg cant, R=32.5 mm ->
  tau_spin ~150 s VALIDATED) + nose canard roll-damping despin (c ~7.2e-3
  N.m.s -> 10.2 N.m at lock, despin ~0.9 s); cap CHARGE integration (Qsc)
  fixed the V=0 Euler degeneracy that pinned the bus at 0 V.
- v7.1: commanded steer deflection (1.0 deg) separated from actuator limit
  (15 deg); guidance hold-time dwell replaced obsolete pi/|w| law.
- v7.2: PWM-connect FET = buck harvest (duty = T_cmd/T_line) restored the
  downward torque channel (passive bridge alone cannot arrest the nose);
  held_gate made authority direction-honest; exposed event-3 arrest failure
  at low q (fixed in v7.3).

## 5. v6.8-v6.9 - power-architecture crisis and resolution (superseded)
- v6.8: body-reference drag fix (range 33 km pgk-ref -> 19.6 km body-ref;
  brochure 18 km). Authority halved vs v6.6 because q became realistic.
- ngspice ps_burst: inrush 17.9 A @0.53 ms through 0.3 ohm NTC -> NTC/precharge
  mandatory. R_BURST sweep 1.5/2.2/3.3/4.7 ohm -> v_bus 8.8/10.0/11.0/11.7 V,
  v_cap@7.5s 11.0/13.0/14.7/16.0 V: passive bridge cannot sustain a 120 mNm
  charge burst into 3 F -> triggered A+ package.
- A+ PACKAGE (locked): C_sc 1.5 F; R_brk 2.2 ohm; wide-Vin buck 9-30 V (boot
  latch 9.5 V); bus-ready gate 16 V; NTC 0.3 ohm bypassed 1 s post-connect;
  connect switch closes post-despin at V_oc >= 18 V.
- v6.9 first-run deadlock (documented failure): connect before despin ->
  short-circuit harvest torque 200-280 mNm froze despin -> cap never charged.
  Fix = despin-gated connect latch (carried in all later versions).

## 6. Known limitations and queued work
- Flat sea-level atmosphere: v7.5 range 15.9 km vs brochure 18 km; gap is
  missing altitude density relief, not aero-table error (bracket: 680-only
  19.6 km / 3-table 15.9 km / truth 18 km). v7.6 = ISA rho(h) + 3-DoF
  point-mass queued. q at event times shifts <5% with ISA -> current MC
  remains valid for authority and power.
- CEP proxy caveat (see 0). Full dispersion CEP pending 6-DoF + guidance loop.
- PLL validated deterministic-only; MC workers use noisy direct rate.
- Open aero: F8 reference values for P7/P8/P9 (email sent, awaiting reply);
  F6 spin axis (all tables at 1361.36 rad/s locked; spin scaling = assumption);
  F9 dual-spin (locked vs de-spun) point for hold-torque truth.
- Open mech: J_nose from CAD (1.4e-3 estimate); slip-clutch chatter/wear under
  ~10 N.m slip-through during despin; canard root ~90 N transient FEA.
- Machine parameters analytical (Ke 0.0116 V.s/rad, R 0.006 ohm, L 250 uH):
  FEA/bench validation pending.

## 7. Decision record (the numbers that decided each item)
- Passive diode bridge + PWM-connect FET over active buck-boost: no switcher
  in the 100%-power path (reliability). Reopen triggers: load >45 W, boot <2 s
  requirement, regen into cap, terminal w_rel <894 rad/s sustained.
- Pulsed dwell over continuous hold: continuous = 2-3.3 kJ vs 324 J usable cap
  store; 18x disturbance exposure; 16 deg gyro-bias pointing error over 55 s.
- 1.5 F cap over 3 F: 3 F never reaches the 16 V gate before event 1 under
  load-line charge; ride-through need is only ~20-100 J.
- Shell fins (2 x 47 mm2 projection @7.5 deg) = spin decay only; nose despin =
  steering-canard roll damping. 7.5 deg cant on the NOSE would demand 0.9-1.9
  N.m continuous hold = 1.2-2.6 kW electrical: impossible, architecture rejected.
- Canard actuator: +/-15 deg travel (3-sigma 9 m/s gust -> 4.5 deg demand at
  terminal q); 0.1 deg resolution (1-LSB miss <=0.05 m vs 5-6 m CEP; below the
  0.14 deg gyro-noise integration floor).
- vsc acceptance redefined at v7.4: bus is regulated 16-19 V, so criterion is
  vs buck floor (>=12 soft / >=9.5 hard), NOT vs a 24 V rail. Old 23.6 V
  numbers are obsolete by design - do not red-flag 14-17 V against them.

## 8. Artifact index
CURRENT: pgk_flight_sim.m (v7.5 label); pgk_aero_model.m (3-table + F5, robust
loader); combine_mc.m (globs mc_slice_*.mat); run_mc_v73_13w.sh (13 workers);
mc_corr_hist.png + mc_vsc_hist.png (v7.5 N=1300); v7x_*_6panel.png
(deterministic); ps_burst.cir (NTC bypass + R_BURST sweep).
STALE / DO NOT CIRCULATE: fin_wake_vs_stall.png (obsolete premise);
fin_corr_vs_wake.png, fin_corr_vs_t1.png (v6.x magnitudes); mc_slice1/2.mat
(v6.6); PGK_LUT.csv (superseded); any histogram titled N=200 mean 78.3 (v6.6).
SIMULINK HANDOFF: ICD frozen - 5 inputs (va, vb, vc, w_nose_gyro, cmd_hold),
12 outputs (duty_brake, duty_harvest, connect_sw, ntc_bypass, rpm_rel,
rpm_nose, v_bus_est, T_em_nose, T_brake_est, T_harvest_est, bus_ready,
dwell_active); sample time 1e-4 s; sign convention +T_em_nose = toward shell
spin (matches sim Ttot = -T_aero + T_em_nose).

## 9. v7.6 (2-DoF lofted trajectory + ISA) - CURRENT baseline
- elev 45 deg | apogee 5.16 km | impact 63.5 s | downrange 15.2 km
  (OPEN: transonic Cd / L=0 point-mass; aero M1.0 table requested to close 15.2 vs 18 km)
- det: authority 25.0/9.5/0.0 = 34.5 m (event 3 q-starved: h=5.1 km, q=19 kPa)
- MC N=1300: mean 35.9 m | std 6.1 m | resid95 5.7 m | Vsc worst 16.00 V
- Vsc worst = 16.00 V by construction (burst hysteresis floor); 6.5 V margin over 9.5 V buck floor
- brake 0 J | harvest 3284 J | Vsc final 18.17 V
- GUIDANCE NOTE: max-range loft = 2 effective steer events; event 3 viable only
  at q >= 30 kPa (flatter charges) or with 2.5 s dwell cap

## 10. v7.6.1 (burst-priority fix + 2.5 s dwell cap) - CURRENT baseline
- det: authority 25.0/9.5/5.3 = 39.8 m (event 3 arrests at h=5.1 km, q=19 kPa)
- Apogee hold energy: 2.5 s x 34.1 W = 85 J from cap -> Vsc dips to 13.06 V
  (>= 12 V soft, >= 9.5 V hard: PASS; abort guard not tripped)
- Harvest-during-hold is physically forbidden at low q (harvest torque breaks
  the arrest); late-event holds are cap-funded by design
- MC N=1300: mean [ ] std [ ] resid95 [ ] Vsc worst [ ] ; abort-tail count [ ]

## 11. v7.6.1 (hold preempts burst + 2.5 s dwell cap) - FINAL sim baseline
- det: authority 25.0/9.5/5.3 = 39.8 m; apogee hold dips bus to 13.06 V
- MC N=1300: mean 41.7 m | std 4.1 m | resid95 5.7 m | Vsc worst 12.33 V
  (>=12 V soft by 0.33 V; >=9.5 V hard by 2.83 V; no dwell aborts observed)
- Vsc histogram bimodal: 16.0 V spike = regulator-pinned runs; 12.3-16 V shelf
  = coast-then-drain runs (apogee hold energy 85 J on 1.5 F)
- Mitigation if >=13 V worst ever required: C_sc 1.5 -> 2.0 F (boot +0.5 s)
- Open: range anchor 15.2 vs 18 km (aero Cd@340 m/s requested); pitch-ring
  dispersion (zeta~0.006) queued for v7.7; 270-table P5 low-AoA rows suspect

## 12. Python GNC Integration Interface (ICD v3)
The GNC 6-DoF stack integrates with the validated power/control physics via a single, self-contained roll channel module (`pgk_roll_channel.py`). 

**Inputs to Roll Channel (from GNC/FSW):**
- `pwm_harvest` (0.0–1.0): PWM duty cycle for the connect FET.
- `pwm_brake` (0.0–1.0): PWM duty cycle for the brake resistor (overflow only).
- `connect_sw` (0.0 or 1.0): Main contactor state.
- `ntc_bypass` (0.0 or 1.0): Precharge bypass relay state.
- `omega_shell` (rad/s): Body spin rate (from 6-DoF dynamics).
- `q` (Pa), `V` (m/s): Local dynamic pressure and velocity (for canard damping).
- `w_nose_gyro` (rad/s): Noisy gyro measurement of nose spin.
- `theta_init_flag` (0.0 or 1.0): When high, resets dead-reckoned theta to 0.0.

**Outputs from Roll Channel (to GNC/FSW):**
- `theta` (rad): Dead-reckoned nose orientation marker `[0, 2π)`.
- `omega_nose` (rad/s): True nose spin rate.
- `v_bus` (V): Supercapacitor/bus voltage.
- `T_em` (N·m): Net electromagnetic torque applied to the nose (for 6-DoF dynamics).

**Control Law Summary (v7.6.1 parity):**
1. **Despin:** `pwm=0.0`. Canard damping dominates, pushing nose toward 0 rad/s.
2. **Dwell (Hold):** `pwm=0.0`. Nose settles to < 2.0 rad/s (`held_gate`). Bus sags safely (e.g., 19V → 13V) on 1.5 F cap.
3. **Recharge:** `pwm=1.0`. Bus rapidly recovers to 19.0 V hysteresis ceiling. Nose finds stable cruise equilibrium (~15–25 rad/s) between steering events.
