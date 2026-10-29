# PGK Power & Control System Design Document v3.1
**Status:** Validated via Monte Carlo Digital Twin
**Tag:** `sim-v6.6-mc-baseline`

## 1. System Overview
* **Host Round:** 155mm M107 (685 m/s muzzle velocity)
* **Rifling:** 1:20 twist (Shell spin = 13,260 RPM at muzzle)
* **Alternator Form Factor:** 60 mm OD, 40 mm stack length (Minimum form factor)
* **Architecture:** Path A (Electromagnetic Braking via Alternator + Mechanical Despin Fins)

## 2. Electromagnetic Alternator (60 mm PMSM)
* **Poles / Slots:** 8 poles / 12 slots (Fractional Slot Concentrated Winding)
* **Parallel Paths:** 4 (1 coil per path)
* **Back-EMF Constant ($K_e$):** 0.0116 V·s/rad
* **Phase Resistance ($R_{ph}$):** 0.006 $\Omega$
* **Phase Inductance ($L_{ph}$):** 250 $\mu$H
* **Electrical Frequency:** 889 Hz (muzzle) decaying to 615 Hz (terminal)

## 3. Power Electronics Architecture
* **DC Bus Target:** 24 V (Rectified from 3-phase Schottky bridge)
* **Energy Storage:** 3.0 F Supercapacitor (Starts at 0V, charges via burst harvest)
* **Dump Resistor:** 2.5 $\Omega$ (Pulse rated for 230W during steering dwells)
* **Harvest Law:** Charge-then-cruise. 120 mNm burst until $V_{sc} \ge 21.6$V, then power-tracking cruise (~45W electrical / 30 mNm mechanical).

## 4. Control & Sensing
* **Speed/Angle Sensor:** Synchronous Reference Frame PLL (SRF-PLL) on 3-phase sense winding. 100 Hz bandwidth, 20 kHz update rate.
* **Nose Rate Sensor:** Tactical MEMS Gyro (0.005 rad/s resolution).
* **Steering Schedule:** Wake-relative adaptive dwell. Events trigger at `t_wake + [4, 16, 31]` seconds. Dwell duration adapts to instantaneous nose rate ($\pi / |\omega_{nose}|$).
* **Dwell Control:** v6.7 Bang-Bang EM torque command. Net EM torque perfectly cancels aerodynamic fin drive and arrests nose at 0 rad/s (inertial space) for directional fidelity.

## 5. Validated Performance (Monte Carlo N=200)
* **CEP Proxy (95th %ile Residual Error):** 5.9 m (Passes <10m requirement)
* **Mean Correction Authority:** 78.3 m
* **Worst-Case Terminal Supercap Voltage:** 23.58 V (Massive margin above 14V buck floor)
* **Peak Fin Torque Required:** 491 mNm (Mechanical spec locked)
