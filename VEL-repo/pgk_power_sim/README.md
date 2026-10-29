# PGK Electromechanical and Power Simulation v7.6.1

## Scope

This repository contains the validated electromechanical and power simulation baseline for the dual-spin Projected Guided Kit concept. The submission is limited to:

- GNU Octave flight-dynamics and authority models.
- SPICE netlists for the PMSM alternator and power-conditioning electronics.
- Design documentation, mechanical fin specification, and results summary.
- Execution scripts for nominal trajectory and Monte Carlo analysis.
- Pre-generated validation figures.

This submission does not include Python GNC source, intermediate Monte Carlo slice files, solver raw files, log files, or cached workspace files.

## Reported Validation Summary

The following values correspond to the v7.6.1 validation baseline:

- Monte Carlo sample size: N = 1300
- Mean guidance authority: 51.4 m
- Authority standard deviation: 4.0 m
- Despin time: approximately 0.9 to 1.3 seconds to reach below 5 rad/s
- Regulated bus operating band: 16.0 V to 19.0 V
- Dwell capability: 2.5-second apogee dwell without falling below the 12.0 V undervoltage lockout floor

## Repository Contents

### Documentation

- `README.md`  
  Repository overview, execution instructions, and submission manifest.

- `DD_v3.1.md`  
  Prior revision of the design document, retained for traceability.

- `DD_v3.2.md`  
  Current master design document. Includes architecture, control-law updates, hold-preempt-burst behavior, and the 2.5-second apogee dwell cap.

- `MECH_FIN_SPEC.md`  
  Mechanical fin specification and physical authority constraints.

- `RESULTS.md`  
  Executive summary of the Monte Carlo validation results.

### Octave Flight-Dynamics and Analysis Sources

- `pgk_flight_sim.m`  
  Primary 6-degree-of-freedom dual-spin flight simulation.

- `pgk_aero_model.m`  
  Aerodynamic model using the corrected three-table CFD representation.

- `fin_sweep_study.m`  
  Fin authority, wake, and stall-related sweep study.

- `combine_mc.m`  
  Aggregates individual Monte Carlo slice results.

- `mc_aggregate.m`  
  Computes aggregate Monte Carlo statistics.

- `analyze_results.m`  
  Post-processing and reporting script for simulation outputs.

### SPICE Netlists

#### Machine and Power Stage

- `pmsm_model.cir`  
  Eight-pole permanent magnet synchronous machine alternator model.

- `power_stage.cir`  
  Baseline power-conditioning stage, including rectification, switching device, supercapacitor bus, and load path.

#### Dwell-Event Validation

- `power_stage_dwell_ac.cir`  
  AC-side dwell brake specification and validation case.

- `power_stage_dwell_macro.cir`  
  Macro-scale dwell transient model.

- `power_stage_dwell_micro.cir`  
  Micro-scale dwell transient model for switching and local energy behavior.

#### Flight-Phase Power Snapshots

- `ps_muzzle.cir`  
  Power-stage condition at muzzle exit.

- `ps_burst.cir`  
  Power-stage condition during bus burst-charge phase.

- `ps_mid.cir`  
  Power-stage condition during midflight cruise.

- `ps_terminal.cir`  
  Power-stage condition during terminal phase.

### Shell Execution Scripts

- `run_simulation.sh`  
  Runs the nominal trajectory.

- `run_mc_v73.sh`  
  Monte Carlo driver retained for reference or single-threaded execution.

- `run_mc_v73_13w.sh`  
  Monte Carlo driver used for the reported parallel validation campaign.

- `run_mc_parallel.sh`  
  General parallel Monte Carlo wrapper.

- `run_final.sh`  
  Convenience script that checks for Octave and runs the nominal trajectory only. It does not run the full Monte Carlo campaign.

### Validation Figures

- `v76_nominal_6panel.png`  
  Nominal v7.6.1 trajectory and state summary.

- `fin_corr_vs_t1.png`  
  Fin correlation result versus first timing parameter.

- `fin_corr_vs_wake.png`  
  Fin correlation result versus wake condition.

- `fin_wake_vs_stall.png`  
  Wake and stall relationship plot.

- `mc_corr_hist.png`  
  Monte Carlo correction histogram.

- `mc_correction_hist.png`  
  Monte Carlo correction distribution histogram.

- `mc_vsc_hist.png`  
  Monte Carlo supercapacitor voltage or related bus metric histogram.

## Prerequisites

Required:

- GNU Octave, version 6.0 or newer recommended.
- Bash-compatible shell for the provided scripts.

Optional:

- NGspice or LTspice, only if the SPICE netlists are to be rerun.
- A multi-core host for the full Monte Carlo campaign.

## Execution Instructions

### 1. Nominal Trajectory

To run the nominal 6-degree-of-freedom trajectory:

```bash
./run_simulation.sh
```

Equivalent direct Octave invocation:

```bash
octave --no-gui --eval "pgk_flight_sim"
```

Expected outcome:

- Nominal trajectory outputs are generated.
- `v76_nominal_6panel.png` is available for review if the script produces figures in the current working directory.

### 2. Full Monte Carlo and Statistical Analysis

To reproduce the reported Monte Carlo validation campaign:

```bash
./run_mc_v73_13w.sh
```

After the Monte Carlo slices are complete, aggregate and analyze the results:

```bash
octave --no-gui --eval "combine_mc; mc_aggregate; analyze_results"
```

Expected outcome:

- Monte Carlo slice files are generated during execution.
- Aggregate statistics are computed.
- Histograms and summary metrics are produced.
- The slice files themselves are not part of the final submission and should remain excluded by `.gitignore`.

Alternative Monte Carlo drivers:

```bash
./run_mc_v73.sh
```

or, where appropriate:

```bash
./run_mc_parallel.sh
```

Use `run_mc_v73_13w.sh` for the reported parallel validation configuration.

### 3. Convenience Nominal Runner

For a quick dependency check and nominal trajectory execution:

```bash
./run_final.sh
```

This script performs only the nominal trajectory. It intentionally does not launch the full Monte Carlo campaign.

## Submission Manifest

The final submission root directory should contain the following files:

```text
.gitignore
DD_v3.1.md
DD_v3.2.md
MECH_FIN_SPEC.md
README.md
RESULTS.md
analyze_results.m
combine_mc.m
fin_corr_vs_t1.png
fin_corr_vs_wake.png
fin_sweep_study.m
fin_wake_vs_stall.png
mc_aggregate.m
mc_corr_hist.png
mc_correction_hist.png
mc_vsc_hist.png
pgk_aero_model.m
pgk_flight_sim.m
pmsm_model.cir
power_stage.cir
power_stage_dwell_ac.cir
power_stage_dwell_macro.cir
power_stage_dwell_micro.cir
ps_burst.cir
ps_mid.cir
ps_muzzle.cir
ps_terminal.cir
run_final.sh
run_mc_parallel.sh
run_mc_v73.sh
run_mc_v73_13w.sh
run_simulation.sh
v76_nominal_6panel.png
```

## Excluded Files

The following should not be included in the final submission:

```text
*.py
*.mat
*.raw
*.dat
*.log
__pycache__/
gnc/
gnu/
octave-workspace
v71_nominal_6panel.png
v72_nominal_6panel.png
```

These files are either implementation scaffolding, intermediate solver output, logs, superseded figures, or non-submission Python material.

## Verification Before Upload

From the repository root, run:

```bash
find . -maxdepth 1 -type f | sort
```

Compare the output with the submission manifest above. No subdirectories should be present unless explicitly required by the Octave scripts.

To confirm that the nominal runner is executable:

```bash
test -x run_final.sh && echo "run_final.sh is executable"
```

To perform a final nominal run before packaging:

```bash
./run_final.sh
```

## Packaging

If uploading as a compressed archive:

```bash
cd ..
tar -czf pgk_power_sim_v7.6.1.tar.gz pgk_power_sim
```

If uploading through Git:

```bash
cd ~/VEL-repo/pgk_power_sim
git add .
git status
git commit -m "Release v7.6.1 electromechanical and power simulation baseline"
git tag -a v7.6.1 -m "Validated electromechanical and power simulation release"
```

Push the branch and tag to the remote repository if applicable:

```bash
git push origin HEAD
git push origin v7.6.1
```
