# PGK Mechanical Despin Fin Specification
**Derived from:** `pgk_flight_sim.m` v6.6 Monte Carlo Validation
**Date:** 2026-09-29

## 1. Despin Fin Stall Torque Requirement
The canted despin fins must generate a minimum stall torque of **490 mNm** at the muzzle dynamic pressure ($q = 287$ kPa). 
* **Reasoning:** The nose inertia is $1.4 \times 10^{-3}$ kg·m². To despin the nose from +13,260 RPM (coupled to shell) to the -80 RPM electrical equilibrium in under 4.5 seconds, the fins must overcome the clutch friction and aerodynamic damping. Sweeps confirm that < 450 mNm results in late electrical wake-up (> 8s), causing the first steering event to be missed.

## 2. Bearing Transient Survival Spec
The nose bearing and clutch assembly must survive a **500 mNm transient torsional load** for a duration of up to **5 seconds** during the initial despin phase.
* **Reasoning:** During the violent despin from 13,260 RPM, the fin torque spikes to its stall limit (491 mNm). The continuous operational rating can remain at 300 mNm (which covers the 165 mNm max steering brake + 30 mNm harvest), but the transient structural limit must not yield at 500 mNm.

## 3. Equilibrium Calibration
Once despun, the fins must be sized/canted such that their steady-state aerodynamic drive torque at $q = 100$ kPa (mid-flight) is approximately **35 mNm**.
* **Reasoning:** The electrical alternator will continuously harvest 30–45 mNm of drag torque to power the 30W avionics load. The fins must provide just enough drive to keep the nose spinning at roughly -80 RPM to maintain the SRF-PLL lock and alternator voltage. If the fins are too weak, the nose stalls; if too strong, the nose spins too fast and the electrical brake cannot hold it for steering.

## 4. Launch Latch Pin (Pending Aero Confirmation)
A mechanical shear pin or detent may be required to hold the nose locked to the shell during the 15 ms in-bore acceleration. 
* **Reasoning:** If the locked-state spin damping torque (CFD P7 at whole-body spin) exceeds the 300 mNm bearing friction, the nose will attempt to despin inside the gun barrel, destroying the bearings. The latch must drop away immediately upon muzzle exit.[Paste the contents of Deliverable 2 here, or use nano to create it]
