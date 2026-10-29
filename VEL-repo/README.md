# PGK On-Flight Power Generation & Dwell-Brake Control System
## Technical Design Document

| Field | Value |
|---|---|
| **Document ID** | PGK-PWR-DD-001 |
| **Version** | 2.0 |
| **Date** | 2026-09-06 |
| **Author** | Electrical / Power Electronics Lead |
| **Status** | For Team Review |
| **Revision** | Nose spin −500 RPM → **−80 RPM** |
| **Distribution** | Guidance · Mechanical · Aerodynamics · GNC · Program · Safety |

---

## 1. Executive Summary

This document defines the **alternator and power-electronics subsystem** for a Precision Guidance Kit (PGK). The subsystem has two integrated functions:

1. **On-flight power generation** — harvest ~$30\text{ W}$ continuous from the relative rotation between the spinning shell and the counter-rotating nose ring, powering the GPS, IMU, MCU, and sensors without a primary battery.
2. **Dwell-brake steering** — use the same alternator as a PWM-controlled electromagnetic brake to hold the canard fins at a commanded azimuth, generating directional lift to counter the Magnus effect and steer the projectile.

Both functions share a single hardware set: a permanent-magnet alternator, a 3-phase rectifier, a DC bus, and one PWM-controlled MOSFET. No battery is used; a small supercapacitor bridges the terminal-phase power gap.

> **Design basis:** This architecture mirrors the de-spin/roll-control scheme disclosed in the original PGK lineage (ATK/Orbital patent US8552349B1: *"Despinning is accomplished via a brake… by balancing the torque"*), and the fixed-canard counter-rotation concept from the NDIA PGK physics briefing.

---

## 2. System Overview

```text
                155 mm Spinning Projectile
   +----------------------------------------------------+
   |  Shell (Rotor) --- 18,500 RPM ------------+        |
   |                                           |        |
   |        +-------------------------+        |        |
   |        |  PMSM ALTERNATOR (gap)  |<-------+        |
   |        |  8-pole, relative motion|                 |
   |        +-----------+-------------+                 |
   |  Nose (Stator) - -80 RPM -|                        |
   |                           | 3-phase AC             |
   |                           v                        |
   |                  +------------------+              |
   |                  |   Diode Bridge   |              |
   |                  +--------+---------+              |
   |                           v                        |
   |                     DC BUS (~24 V)                 |
   |        +---------+--------+--------+--------+      |
   |        v         v                 v        v      |
   |    Supercap  Shunt/Brake      DC-DC Buck  (spare)  |
   |    2.5 F     MOSFET (PWM)     24->5 V              |
   |        |         |                 |               |
   |        +---------+--------+--------+               |
   |                           v                        |
   |              MCU . GPS . IMU . Sensors             |
   +----------------------------------------------------+
```

**Operating principle:**
- The alternator sees only the **relative speed** ($\omega_{\text{rel}}$) between shell and nose.
- Continuous light loading (harvest) produces a fixed **$20\text{ mNm}$** drag torque → baseline power.
- Intermittent heavy loading (brake) produces up to **~$\text{224 mNm}$** drag torque → holds the nose azimuth for steering.
- The mechanical **de-spin clutch/bearing** resists the combined electromagnetic torque and must never be exceeded ($T_{\text{clutch,max}} = 300\text{ mNm}$).

---

## 3. Operating Parameters (Revised)

### 3.1 Speeds
| Quantity | Value | Note |
|---|---|---|
| Shell (rotor) muzzle speed $\omega_{\text{shell}}$ | $18,500\text{ RPM} = 1937\text{ rad/s}$ | SPEC |
| Nose (stator) equilibrium speed $\omega_{\text{nose}}$ | **$-80\text{ RPM} = -8.38\text{ rad/s}$** | SPEC (revised) |
| Relative speed at muzzle $\omega_{\text{rel,0}}$ | **$18,580\text{ RPM} = 1945.5\text{ rad/s}$** | CALC |
| Relative speed at terminal ($t = 60\text{ s}$) | **$12,481\text{ RPM} = 1307\text{ rad/s}$** | CALC |
| Shell spin decay constant $\tau$ | $150\text{ s}$ | EST |

### 3.2 Electrical (8-pole machine)
| Quantity | Muzzle | Terminal | Note |
|---|---|---|---|
| Electrical frequency $f_e$ | $1238.7\text{ Hz}$ | $832.1\text{ Hz}$ | CALC |
| Back-EMF peak / phase $V_{\text{emf}}$ | $14.20\text{ V}$ | $9.54\text{ V}$ | CALC |
| Open-circuit DC bus voltage $V_{\text{bus,oc}}$ | $\sim 23.5\text{ V}$ | $\sim 15.8\text{ V}$ | CALC |

---

## 4. Physics & Mathematical Foundation

### 4.1 Torque–Current (braking authority)

$$
T_{\text{em}} = \frac{3}{2} p \lambda I_q
$$

Torque $T_{\text{em}}$ is controlled by regulating stator quadrature current $I_q$ via the electrical load.

### 4.2 Back-EMF (speed sensing)

$$
V_{\text{emf}} = K_e \omega_{\text{rel}}
$$

The alternator acts as a self-tachometer: electrical frequency $f_e \propto \omega_{\text{rel}}$.

### 4.3 Power

$$
P = T_{\text{em}} \omega_{\text{rel}}
$$

At fixed harvest torque ($T_{\text{harvest}} = 20\text{ mNm}$), generated power $P$ decreases as relative speed $\omega_{\text{rel}}$ decays.

### 4.4 Maximum short-circuit brake torque

$$
T_{\text{brake,max}} = \frac{3}{2} p \lambda I_{\text{sc}} \frac{R}{Z}, \qquad Z = \sqrt{R^2 + (\omega_e L)^2}
$$

### 4.5 Nose roll dynamics

$$
J \dot{\omega}_{\text{nose}} = T_{\text{aero}} + T_{\text{harvest}} + T_{\text{brake}}
$$

---

## 5. Architecture

### 5.1 Hardware (minimal footprint)
| Block | Implementation | Purpose |
|---|---|---|
| Alternator | 8-pole PMSM, SmCo magnets | Power source + actuator |
| Rectifier | 3-phase Schottky bridge | AC $\rightarrow$ DC (frequency-agnostic) |
| DC bus cap | $22\text{ }\mu\text{F}$ MLCC bank | Voltage ripple buffering |
| Brake/Shunt | Single N-MOSFET, $20\text{ kHz}$ PWM | Voltage regulation + steering brake |
| Energy buffer | $2.5\text{ F}$ supercap @ $25\text{ V}$ | Terminal-phase power bridge |
| Regulator | Wide-input buck ($24 \rightarrow 5\text{ V}$) | MCU/sensor power supply |

### 5.2 Why no VFD / no frequency control
Output frequency varies ($1239 \rightarrow 832\text{ Hz}$) with RPM decay. AC power is immediately rectified to DC, making frequency irrelevant downstream. Only the DC bus voltage $V_{\text{bus}}$ is actively regulated.

---

## 6. Parameter Register

**Provenance tags:** `PUBLIC` = NG/Army/patent; `SPEC` = team requirement; `CALC` = derived; `EST` = engineering estimate; `CHOSEN` = design choice.

| ID | Parameter | Value | Tag |
|---|---|---|---|
| A1 | Shell muzzle speed $\omega_{\text{shell}}$ | $18,500\text{ RPM}$ | PUBLIC |
| A2 | Shell spin decay constant $\tau$ | $150\text{ s}$ | EST |
| A3 | Nose equilibrium speed $\omega_{\text{nose}}$ | $-80\text{ RPM}$ | SPEC |
| A4 | Nose moment of inertia $J$ | $1.4 \times 10^{-3}\text{ kg}\cdot\text{m}^2$ | EST |
| A5 | Canard counter-rotation drive | $0.16\text{ Nm}$ @ $q_0$ | EST |
| A6 | Nose roll damping $c$ | **$0.0191\text{ N}\cdot\text{m}\cdot\text{s}$** | CALC (revised) |
| A7 | Dynamic pressure $q_0$ | $115\text{ kPa}$ | EST |
| A8 | Launch setback acceleration | $15,000\text{--}20,000\text{ g}$ | PUBLIC |
| A9 | Clutch/bearing torque rating $T_{\text{clutch,max}}$ | $300\text{ mNm}$ | EST |
| B1 | Pole count $p$ | $8$ | CHOSEN |
| B2 | Back-EMF constant $K_e$ | $0.0073\text{ V}\cdot\text{s/rad}$ | CALC |
| B3 | Phase resistance $R$ | $0.5\text{ }\Omega$ | EST |
| B4 | Phase inductance $L$ | $40\text{ }\mu\text{H}$ | CHOSEN |
| B7 | Baseline harvest torque $T_{\text{harvest}}$ | $20\text{ mNm}$ | SPEC |
| B8 | Max brake torque $T_{\text{brake,max}}$ | $\sim 224\text{ mNm}$ (muzzle) / $\sim 178\text{ mNm}$ (terminal) | CALC |
| C1 | Rectifier topology | 3-phase Schottky bridge | CHOSEN |
| C2 | DC bus capacitor $C_{\text{bus}}$ | $22\text{ }\mu\text{F}$ | CHOSEN |
| C3 | Supercapacitor capacity $C_{\text{super}}$ | $2.5\text{ F @ } 25\text{ V}$ | CALC |
| C4 | Bus regulation target $V_{\text{bus,target}}$ | $24\text{ V}$ | CHOSEN |
| C5 | Bus droop floor $V_{\text{bus,min}}$ | $14\text{ V}$ | CHOSEN |
| C6 | Brake switch | N-MOSFET, $20\text{ kHz}$ PWM | CHOSEN |
| C7 | Continuous avionics load $P_{\text{load}}$ | $30\text{ W}$ | EST |
| D1 | Brake-inhibit window | $t < 4\text{ s}$ | PUBLIC |
| D2 | Azimuth deadband | $\pm 5^\circ$ | CHOSEN |
| D3 | Steering control law | Bang-bang roll-angle hold | PUBLIC |
| D4 | Sample steering windows | $t = 8, 20, 35\text{ s}$ (dwell $0.6\text{ s}$) | SPEC |
| D5 | Max brake condition | $T_{\text{brake}} \le T_{\text{clutch,max}} - T_{\text{harvest}}$ | SPEC |

---

## 7. Torque Constraints

$$
T_{\text{harvest}} + T_{\text{brake}} < T_{\text{clutch,max}}
$$

| State | Torque | Power Equivalent | System Objective |
|---|---|---|---|
| Free-spin harvest | $20\text{ mNm}$ | $\sim 39\text{ W} \rightarrow \sim 26\text{ W}$ | Power continuous avionics |
| Dwell brake (peak) | $\sim 150\text{--}224\text{ mNm}$ | Up to $\sim 430\text{ W}$ (intermittent) | Hold nose azimuth for steering |
| **Clutch slip limit** | **$300\text{ mNm}$** | — | **Absolute mechanical constraint** |

**Brake-inhibit rule:** Braking is inhibited for $t < 4\text{ s}$. The canards free-despin the nose at high muzzle dynamic pressure ($q$); braking begins only after $q$ decays, maintaining total torque stress below $T_{\text{clutch,max}} = 300\text{ mNm}$.

---

## 8. Flight Profile & Power Budget

| Phase | Time $t$ | Rel. Speed $\omega_{\text{rel}}$ | Harvest $P$ | Brake $P$ | Supercapacitor Status |
|---|---|---|---|---|---|
| Muzzle | $0\text{ s}$ | $18,580\text{ RPM}$ | $\sim 39\text{ W}$ | Inhibited | Charging |
| Mid-course | $30\text{ s}$ | $15,270\text{ RPM}$ | $\sim 32\text{ W}$ | Active steering | Float |
| Terminal | $60\text{ s}$ | $12,481\text{ RPM}$ | $\sim 26\text{ W}$ | Active steering | **Discharging** |

- Surplus power at muzzle ($\sim 9\text{ W}$) charges the supercapacitor and dissipates excess energy as heat.
- Terminal deficit ($\sim 4\text{ W}$ gap below $30\text{ W}$ load) is supplemented by the supercapacitor.
- Supercapacitor energy buffer sizing equation:

$$
C = \frac{2E}{V_1^2 - V_2^2} \approx 1.3\text{ F} \implies \mathbf{2.5\text{ F @ 25 V}}
$$

---

## 9. Control Logic

### 9.1 Dwell-brake steering (bang-bang roll-angle hold)
- Electromagnetic brake torque $T_{\text{brake}}$ pulls the nose ring **toward** shell spin (+ direction); aerodynamic torque $T_{\text{aero}}$ drives it **back** (− direction).
- **Hold phase:** $T_{\text{brake}}$ balances $T_{\text{aero}}$ at the commanded steering azimuth.
- **Release phase:** $T_{\text{aero}}$ re-accelerates the nose assembly to its equilibrium speed $\omega_{\text{nose}} = -80\text{ RPM}$.
- Electrical response latency ($\sim 600\text{ }\mu\text{s}$) is negligible compared to the roll response window ($\sim 230\text{ ms}$).

### 9.2 Energy Management Loop
- When $V_{\text{bus}} > 24\text{ V}$, the PWM shunt MOSFET dumps surplus power into heat.
- When $V_{\text{bus}} < 14\text{ V}$, the supercapacitor discharges into the $5\text{ V}$ buck converter rail.

---

## 10. Edge Cases & Failure Modes (FMEA)

| ID | Failure | Consequence | Mitigation |
|---|---|---|---|
| C1 | Brake MOSFET fails short (ON) | Continuous max brake $\rightarrow$ clutch slip | Redundant MOSFET in series + DC bus fuse |
| C2 | Supercapacitor failure | Terminal phase power loss | Parallel cap bank / thermal battery backup |
| C3 | EMI coupling into IMU | False guidance signals | Mu-metal shielding, twisted pair, CM choke |
| C4 | Clutch thermal derating | Torque slip at elevated temperature | Firmware thermal budget + dynamic power derating |
| H1 | Magnet demagnetization | Reduced $V_{\text{emf}}$ and lower brake torque | SmCo magnets + firmware current limits |
| H2 | Bearing EDM pitting | Bearing race failure | Shaft grounding / ceramic bearing races |
| M1 | Winding insulation breakdown | Inter-turn short circuit | Kapton wire + epoxy encapsulation |

---

## 11. Simulation Toolchain & File Manifest

**Toolchain:** Lepton (schematic) · ngspice (circuit) · Octave (control/analysis)

| File | Tool | Purpose |
|---|---|---|
| `pgk_flight_sim.m` | Octave | Coupled electromechanical flight dynamics model |
| `pmsm_model.cir` | ngspice | Stator & rotor PMSM alternator subcircuit |
| `power_stage.cir` | ngspice | 3-phase rectifier + brake MOSFET + bus + load |
| `analyze_results.m` | Octave | Data extraction & metrics post-processing |
| `run_simulation.sh` | Bash | Automated simulation pipeline execution script |

---

## 12. Document Revision History

| Ver | Date | Change | Author |
|---|---|---|---|
| 1.0 | — | Initial architecture (nose equilibrium $\omega_{\text{nose}} = -500\text{ RPM}$) | Elec. Lead |
| 2.0 | 2026-09-06 | Revised nose equilibrium $-500 \rightarrow -80\text{ RPM}$; relative speed $19,000 \rightarrow 18,580\text{ RPM}$; damping $c = 0.00306 \rightarrow 0.0191\text{ N}\cdot\text{m}\cdot\text{s}$; updated MathJax standards | Elec. Lead |

---

*End of document.*
