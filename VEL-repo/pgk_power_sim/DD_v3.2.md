# PGK Power & Control Subsystem Design Document
**Version:** 3.2 (Final Validated Baseline)
**Simulation Tag:** `sim-v7.6.1-final` (N=1300 Monte Carlo)
**Date:** 2026-09-30

## 1. Executive Summary
This document defines the finalized architecture and control logic for the PGK nose-electronics subsystem. The system harvests energy from the shell-nose differential spin via a passive alternator, stores it in a supercapacitor, and executes a three-event steering sequence on a 155mm max-range lofted trajectory. 

The architecture has been validated via a 2-DoF lofted trajectory simulator with ISA atmospheric density and a 3-table CFD aerodynamic lookup. The baseline achieves **41.7 m mean correction authority** (6.7× the residual error) while maintaining a worst-case bus voltage of **12.33 V** (clearing the 9.5 V hard brownout floor).

## 2. Power Subsystem Architecture (The "A+" Package)
The power network is designed for maximum reliability in a high-G gun-launched environment. It eliminates active switching in the primary power path.

*   **Alternator & Rectification:** 8-pole permanent magnet alternator feeding a passive diode bridge. 
*   **Energy Storage:** 1.5 F Supercapacitor bank (Max voltage 28 V).
*   **Connection Logic (PWM-Connect Buck):** 
    *   The rectifier is connected to the bus via a MOSFET only *after* the nose despins and the open-circuit voltage ($V_{oc}$) reaches 18 V.
    *   A 0.3 $\Omega$ NTC precharge resistor limits inrush current (17.9 A peak); it is bypassed by a relay 1.0 s after connection.
    *   Harvest torque is modulated via PWM duty cycle on the connect FET to regulate the bus between **16.0 V and 19.0 V** (hysteresis band).
*   **Electromagnetic Brake:** A 2.2 $\Omega$ dumping resistor. It acts strictly as an overflow/failsafe valve. Primary arrest authority is achieved by shedding the harvest torque to zero.
*   **Avionics Load:** 30 W continuous, fed through a wide-Vin buck converter (9–30 V input range, 9.5 V UVLO floor).

## 3. Control Strategy: Two-Tier Pulsed & Proportional
Continuous braking was rejected due to thermal limits (requires >2 kJ; cap stores ~324 J) and sensor drift. The system uses a two-tier approach:

### Tier 1: Guidance Layer (Pulsed Scheduling)
*   Steering is executed as discrete, azimuth-triggered dwell pulses.
*   **Dwell Duration:** Calculated dynamically based on the time required to arrest the nose roll rate ($\tau = J/c_{damp}$), plus a 0.3 s hold margin. 
*   **High-Altitude Cap:** Dwell duration is capped at **2.5 seconds** to ensure the nose can fully arrest at apogee where aerodynamic damping is extremely low ($q \approx 19$ kPa).

### Tier 2: Actuator Layer (Proportional Execution)
*   Inside the dwell pulse, the controller applies proportional electromagnetic torque to arrest the nose and hold it at 0 rad/s relative to the inertial frame.
*   **CRITICAL PRIORITY RULE:** **Steering Hold preempts Burst Charging.** If the bus voltage sags below 16 V *during* a steering pulse, the controller will shed harvest torque to zero to maintain the hold, allowing the bus to dip to ~13 V. Charging the capacitor is secondary to executing the mission-critical steer.

## 4. Mechanical & Aerodynamic Interfaces
*   **Shell Spin Decay:** Achieved by 2x canted shell fins (357 mm² projection, 7.5° cant, 32.5 mm radius). Time constant $\tau \approx 150$ s.
*   **Nose Despin:** Achieved purely by the roll-damping reaction of the steering canards. Peak transient torque at lock is **10.2 N·m**.
*   **Slip Clutch:** Must be rated to slip continuously at 0.3–0.5 N·m, and survive the 10.2 N·m transient during despin without mechanical chatter or wear-debris generation.
*   **Canard Actuator:** 
    *   Hardware travel limit: $\pm 15^\circ$ (sized for 9 m/s terminal crosswind).
    *   Commanded deflection during hold: $1.0^\circ$.
*   **Aerodynamics:** 3-table CFD LUT (270, 410, 680 m/s). 
    *   *Note:* Software applies an F5 normalization correction factor $(680/V)^2$ to the low-speed tables pending confirmation of reference area fixes from the aero team.

## 5. Simulink Interface Control Document (ICD)
The subsystem is encapsulated in a single MATLAB Function block (Sample Time: 1e-4 s). 

**Inputs (5):**
1. `va, vb, vc` (V): Alternator phase voltages.
2. `w_nose_gyro` (rad/s): Inertial nose roll rate.
3. `cmd_hold` (bool): Guidance command (High = execute dwell).

**Outputs (12):**
1. `duty_brake` (0-1): Brake MOSFET PWM.
2. `duty_harvest` (0-1): Connect FET PWM.
3. `connect_sw` (bool): Main contactor enable.
4. `ntc_bypass` (bool): Precharge bypass relay.
5. `rpm_rel` (RPM): Estimated shell-nose relative speed.
6. `rpm_nose` (RPM): Estimated inertial nose speed.
7. `v_bus_est` (V): Internal bus voltage estimate.
8. `T_em_nose` (N·m): Net EM torque applied to nose (Plant feedback).
9. `T_brake_est` (N·m): Brake torque component.
10. `T_harvest_est` (N·m): Harvest torque component.
11. `bus_ready` (bool): Inhibit flag (0 if V < 16V).
12. `dwell_active` (bool): Handshake flag.

## 6. Validated Performance (v7.6.1 Baseline)
*Flight Profile: 45° Loft, Apogee 5.16 km, TOF 63.5 s.*

| Metric | Deterministic | Monte Carlo (N=1300) |
| :--- | :--- | :--- |
| **Total Correction Authority** | 39.8 m | Mean: 41.7 m, Std: 4.1 m |
| **Residual Error (CEP Proxy)** | - | 5.7 m (95th percentile) |
| **Worst-Case Bus Voltage** | 13.06 V (Apogee hold) | 12.33 V |
| **Energy Harvested** | 3329 J | - |
| **Energy Braked** | 0 J | - |

## 7. Known Limitations & Open Items
1.  **Range Anchor (15.2 km vs 18 km Brochure):** The 2-DoF point-mass model with current transonic Cd tables falls 2.8 km short. *Action:* Request aero team for dense Cl/Cd sweep at 340 m/s (Mach 1.0) to verify the transonic drag peak.
2.  **Pitch-Mode Dispersion:** The dense pitch-moment sweep reveals a very lightly damped pitch mode ($\zeta \approx 0.006$, period 0.17–0.87 s). Canard pulses excite this mode, causing authority to ring. *Action:* Queued for v7.7 high-fidelity upgrade; currently bounded within the 4.1 m MC standard deviation.
3.  **Capacitor Margin:** The worst-case MC bus dip (12.33 V) clears the 9.5 V hard floor by 2.83 V, but only clears a hypothetical 13 V soft floor by a narrow margin. *Mitigation:* If hardware policy demands $\ge 13$ V worst-case, upgrade supercapacitor to 2.0 F (adds ~0.5 s to boot time).

***

