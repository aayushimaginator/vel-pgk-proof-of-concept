<codebase>
<project_structure>
.
├── __pycache__
│   ├── generate_figures.cpython-313.pyc
│   ├── pgk_aero.cpython-313.pyc
│   ├── pgk_canard_aero.cpython-313.pyc
│   ├── pgk_control.cpython-313.pyc
│   ├── pgk_control_budget.cpython-313.pyc
│   ├── pgk_forces.cpython-313.pyc
│   ├── pgk_fuze.cpython-313.pyc
│   ├── pgk_guidance.cpython-313.pyc
│   ├── pgk_imu.cpython-313.pyc
│   ├── pgk_math.cpython-313.pyc
│   ├── pgk_modes.cpython-313.pyc
│   ├── pgk_montecarlo.cpython-313.pyc
│   ├── pgk_nav.cpython-313.pyc
│   ├── pgk_params.cpython-313.pyc
│   ├── pgk_sensors.cpython-313.pyc
│   ├── pgk_simulator.cpython-313.pyc
│   ├── pgk_validation.cpython-313.pyc
│   └── test_fuze.cpython-313.pyc
├── generate_figures.py
├── m1156_pgk_final.py
├── m1156_pgk_upgraded.py
├── pgk_aero.py
├── pgk_canard_aero.py
├── pgk_control.py
├── pgk_control_budget.py
├── pgk_forces.py
├── pgk_fuze.py
├── pgk_gnc_doc.py
├── pgk_guidance.py
├── pgk_hold_metrics.py
├── pgk_imu.py
├── pgk_main.py
├── pgk_math.py
├── pgk_modes.py
├── pgk_montecarlo.py
├── pgk_nav.py
├── pgk_params.py
├── pgk_phase_id.py
├── pgk_run_nextlevel.py
├── pgk_sensors.py
├── pgk_simulator.py
├── pgk_stress_cep.py
├── pgk_validation.py
├── run_all.py
└── test_fuze.py

1 directories, 45 files
</project_structure>

<file src="generate_figures.py">
#!/usr/bin/env python3
"""
Regenerate PGK result plots and animations from the current simulation.

Usage:
  python generate_figures.py              # all current-data plots
  python generate_figures.py --suite      # open-loop / guided suite only
  python generate_figures.py --paper      # paper-style trajectories + angular
  python generate_figures.py --gif        # canards local GIF only
  python generate_figures.py --motion     # pgk_real_motion_full.png
  python generate_figures.py --workflow   # calibration workflow panel

Outputs are written to the current directory (or --out DIR).
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_validation import open_loop_summary, compare_to_ballpark
from pgk_guidance import Guidance


def _quiet_params(**kwargs) -> Params:
    p = Params()
    p.dt = kwargs.get("dt", 0.0025)
    p.gps_enable = kwargs.get("gps_enable", False)
    p.use_kalman = kwargs.get("use_kalman", False)
    p.use_met_layers = False
    p.wind = np.zeros(3)
    p.fail_safe_miss = 5000.0
    for k, v in kwargs.items():
        if hasattr(p, k) and k not in ("dt", "gps_enable", "use_kalman"):
            setattr(p, k, v)
    return p


def run_open_loop(log_every: int = 100):
    p = _quiet_params(arm_time=9999.0, azimuth_bias_deg=0.0, dt=0.0025, control_dt=0.03)
    log = Simulator(p).run(log_every=log_every)
    s = open_loop_summary(log, p)
    return p, log, s


def run_guided(s_ol: dict, truth: bool = True, log_every: int = 80):
    p = _quiet_params(
        target_x=s_ol["impact_x"] + 100.0,
        target_y=s_ol["impact_y"] - 70.0,
        azimuth_bias_deg=0.35,
        guidance_phase_deg=0.0,
        gps_enable=not truth,
        use_kalman=not truth,
        dt=0.0025,
        control_dt=0.03,
        fail_safe_miss=5000.0,
    )
    if not truth:
        p.gps_noise_sigma = 2.5
        p.gps_dropout_prob = 0.0
    log = Simulator(p).run(log_every=log_every)
    tgt = np.array([p.target_x, p.target_y])
    imp = np.array([log["x"][-1], log["y"][-1]])
    miss = float(np.linalg.norm(imp - tgt))
    ol_miss = float(np.hypot(100.0, 70.0))
    return p, log, miss, ol_miss, tgt, imp


def fig_suite(out: Path):
    print("=== Suite: open-loop + guided ===")
    _, log_ol, s = run_open_loop()
    compare_to_ballpark(s, Params())
    _, log_t, miss_t, ol_miss, tgt, _ = run_guided(s, truth=True)
    _, log_g, miss_g, _, _, _ = run_guided(s, truth=False)

    fig = plt.figure(figsize=(12, 7))
    ax = fig.add_subplot(1, 2, 1)
    ax.plot(log_ol["x"], log_ol["y"], "k-", lw=1.5, label="OL")
    ax.plot(log_t["x"], log_t["y"], "C0-", lw=1.5, label="Truth (%.0fm)" % miss_t)
    ax.plot(log_g["x"], log_g["y"], "C2-", lw=1.3, label="GPS (%.0fm)" % miss_g)
    ax.scatter(tgt[0], tgt[1], c="m", marker="*", s=100, label="Target")
    ax.axis("equal")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title("Ground track")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    ax = fig.add_subplot(1, 2, 2)
    ax.axis("off")
    txt = (
        "Open-loop range   %.0f m\n"
        "Open-loop miss    %.1f m\n"
        "Guided truth nav  %.1f m\n"
        "Guided GPS+KF     %.1f m\n\n"
        "CN ~ 0.019  a_max = 0.85 m/s²\n"
        "PGK threshold < 50 m\n"
        "PGK objective  < 30 m"
    ) % (s["range_m"], ol_miss, miss_t, miss_g)
    ax.text(
        0.05,
        0.9,
        txt,
        transform=ax.transAxes,
        fontsize=11,
        va="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightyellow"),
    )
    plt.tight_layout()
    path = out / "pgk_full_suite.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)
    return s


def fig_paper(out: Path, s_ol: dict | None = None):
    print("=== Paper-style trajectories + angular ===")
    if s_ol is None:
        _, log_ol, s_ol = run_open_loop(log_every=55)
    else:
        p_ol = _quiet_params(arm_time=9999.0, azimuth_bias_deg=0.0)
        log_ol = Simulator(p_ol).run(log_every=55)
        s_ol = open_loop_summary(log_ol, p_ol)

    p_t, log_t, miss_t, _, tgt, _ = run_guided(s_ol, truth=True, log_every=55)

    # Fixed-canard cases
    class FixedGuid(Guidance):
        def __init__(self, p, deg, t0=25.0):
            super().__init__(p)
            self.fixed = np.radians(deg)
            self.t0 = t0
            self.theta_c = self.fixed
            self.free_spin = True

        def update(self, t, r, v, dt, F_can_current=None, **kwargs):
            if t < self.t0:
                self.free_spin = True
            else:
                self.free_spin = False
                self.theta_c = self.fixed

    fixed_logs = {}
    for deg in (0, 90, 180, 270):
        p = _quiet_params(azimuth_bias_deg=0.0, arm_time=0.0)
        sim = Simulator(p)
        sim.guid = FixedGuid(p, deg, t0=25.0)
        fixed_logs[deg] = sim.run(log_every=60)

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Paper-style trajectories (dual-spin PGK)", fontsize=13, fontweight="bold")

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(log_ol["x"], log_ol["y"], log_ol["z"], "k-", lw=1.2, label="OL")
    ax.plot(log_t["x"], log_t["y"], log_t["z"], "C0-", lw=1.2, label="Guided")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3-D CM paths")
    ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(log_ol["x"], log_ol["y"], "k-", lw=1.4, label="OL")
    ax.plot(log_t["x"], log_t["y"], "C0-", lw=1.4, label="Guided (%.0fm)" % miss_t)
    for deg, lg in fixed_logs.items():
        ax.plot(lg["x"], lg["y"], lw=1.0, alpha=0.75, label="Canard %d°" % deg)
    ax.scatter(tgt[0], tgt[1], c="m", marker="*", s=80)
    ax.set_title("Ground track")
    ax.axis("equal")
    ax.legend(fontsize=6)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(log_ol["t"], log_ol["speed"], "k-", label="OL")
    ax.plot(log_t["t"], log_t["speed"], "C0-", label="Guided")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("V (m/s)")
    ax.set_title("Speed")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(log_t["t"], log_t["p_rps"], label="Body rps")
    ax.plot(log_t["t"], np.abs(log_t["omega_nose_rps"]), label="|Nose| rps")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rps")
    ax.set_title("Dual-spin rates")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_paper_trajectories.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)

    # Angular
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle("Angular motion / AoA", fontsize=13, fontweight="bold")
    ax = fig.add_subplot(2, 2, 1)
    ax.plot(log_t["t"], log_t["aoa"], label="AoA")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("deg")
    ax.set_title("Angle of attack")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(log_t["t"], log_t["pitch"], label="pitch")
    ax.plot(log_t["t"], log_t["yaw"], label="yaw")
    ax.legend(fontsize=8)
    ax.set_title("Attitude (euler)")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(log_t["t"], log_t["theta_c_deg"], label="θ_c")
    ax.plot(log_t["t"], log_t["phi_nose_deg"], label="φ_nose", alpha=0.85)
    ax.legend(fontsize=8)
    ax.set_title("Canard command vs nose")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(log_t["t"], log_t["zem"], label="ZEM")
    ax.axhline(Params().zem_deadband, color="r", ls="--", label="deadband")
    ax.legend(fontsize=8)
    ax.set_title("Predicted ZEM")
    ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_paper_angular.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)


def fig_workflow(out: Path):
    print("=== Workflow summary panel ===")
    _, log_ol, s = run_open_loop()
    _, log_t, miss_t, ol_miss, tgt, _ = run_guided(s, truth=True)
    _, log_g, miss_g, _, _, _ = run_guided(s, truth=False)

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle("PGK calibration workflow (realistic canard authority)", fontsize=13, fontweight="bold")

    ax = fig.add_subplot(2, 2, 1)
    ax.plot(log_ol["x"], log_ol["y"], "k-", lw=1.5, label="Open-loop")
    ax.plot(log_t["x"], log_t["y"], "C0-", lw=1.5, label="Guided truth (%.0fm)" % miss_t)
    ax.plot(log_g["x"], log_g["y"], "C2-", lw=1.3, label="Guided GPS (%.0fm)" % miss_g)
    ax.scatter(tgt[0], tgt[1], c="m", marker="*", s=100, label="Target")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Ground track")
    ax.axis("equal")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 2)
    ax.bar([0, 1, 2], [ol_miss, miss_t, miss_g], color=["0.4", "C0", "C2"])
    ax.axhline(50, color="r", ls="--", label="Threshold 50 m")
    ax.axhline(30, color="orange", ls="--", label="Objective 30 m")
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(["OL", "Truth", "GPS"])
    ax.set_ylabel("Miss (m)")
    ax.set_title("Guided miss")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(log_t["t"], log_t["p_rps"], label="Body")
    ax.plot(log_t["t"], np.abs(log_t["omega_nose_rps"]), label="Nose")
    ax.set_title("Dual-spin")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.axis("off")
    p = Params()
    cn = p.canard_Cl_max * p.canard_authority * p.canard_area / p.ref_area
    txt = (
        "WORKFLOW RESULTS\n"
        "─────────────────────────────\n"
        "OL range     %.0f m\n"
        "OL drift     %.0f m\n"
        "OL miss      %.1f m\n"
        "Guided truth %.1f m\n"
        "Guided GPS   %.1f m\n"
        "\n"
        "CN ≈ %.3f   a_max = %.2f m/s²\n"
        "phase = %.0f°   arm = %.0f s\n"
        "Force from phi_nose\n"
    ) % (
        s["range_m"],
        s["drift_m"],
        ol_miss,
        miss_t,
        miss_g,
        cn,
        p.canard_a_max,
        p.guidance_phase_deg,
        p.arm_time,
    )
    ax.text(
        0.05,
        0.95,
        txt,
        transform=ax.transAxes,
        fontsize=10,
        va="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="lightyellow"),
    )
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_workflow_complete.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)


def _anim_log(log_every: int = 30):
    _, _, s = run_open_loop(log_every=60)
    p, log, miss, _, tgt, imp = run_guided(s, truth=True, log_every=log_every)
    return {
        "t": np.array(log["t"]),
        "x": np.array(log["x"]),
        "y": np.array(log["y"]),
        "z": np.array(log["z"]),
        "p_rps": np.array(log["p_rps"]),
        "nose_rps": np.array(log["omega_nose_rps"]),
        "free": np.array(log["free_spin"]),
        "aoa": np.array(log["aoa"]),
        "speed": np.array(log["speed"]),
        "phi": np.array(log["phi_nose_deg"]),
        "target": np.array([tgt[0], tgt[1], 0.0]),
        "impact": np.array([imp[0], imp[1], 0.0]),
        "miss": miss,
    }


def gif_canards_local(out: Path):
    print("=== GIF: canards local (spin / freeze) ===")
    data = _anim_log(log_every=30)
    t = data["t"]
    x, y, z = data["x"], data["y"], data["z"]
    p_rps = data["p_rps"]
    nose_rps = data["nose_rps"]
    free = data["free"]
    speed = data["speed"]
    phi_deg = data["phi"]
    tgt = data["target"]

    n_frames = 80
    idx = np.linspace(0, len(t) - 1, n_frames).astype(int)
    dt_f = np.gradient(t[idx])
    body_vis = np.cumsum(
        np.clip(np.abs(p_rps[idx]) / max(float(p_rps.max()), 1.0), 0, 1) * 2.5 * dt_f * 8
    )
    nose_vis = np.zeros(n_frames)
    acc = 0.0
    for k in range(n_frames):
        i = idx[k]
        if free[i] > 0.5:
            acc += float(nose_rps[i]) * float(dt_f[k]) * 0.35
            nose_vis[k] = acc
        else:
            nose_vis[k] = np.radians(float(phi_deg[i]))
            acc = nose_vis[k]

    from matplotlib.patches import Circle

    fig = plt.figure(figsize=(12, 6.2))
    fig.suptitle(
        "Local view: canards SPIN when free-spin, FREEZE when guiding",
        fontsize=12,
        fontweight="bold",
    )
    ax = fig.add_subplot(1, 2, 1)
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-2.2, 2.2)
    ax.set_aspect("equal")
    ax.set_title("View along flight direction\n(canard roll plane)", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    body_circ = Circle((0, 0), 1.0, facecolor="steelblue", edgecolor="k", lw=1.5, alpha=0.9)
    ax.add_patch(body_circ)
    body_stripe, = ax.plot([], [], "w-", lw=3)
    nose_circ = Circle((0, 0), 0.55, facecolor="orange", edgecolor="k", lw=1.5, alpha=0.95, zorder=5)
    ax.add_patch(nose_circ)
    canard_lines = [ax.plot([], [], "-", lw=4, solid_capstyle="round", zorder=6)[0] for _ in range(4)]
    mode_label = ax.text(0, -2.0, "", ha="center", fontsize=11, fontweight="bold")

    ax3 = fig.add_subplot(2, 2, 2, projection="3d")
    ax3.plot(x, y, z, color="0.65", lw=0.7, alpha=0.5)
    trail, = ax3.plot([], [], [], "b-", lw=1.8)
    ax3.scatter([0], [0], [z[0]], c="g", s=30)
    ax3.scatter([tgt[0]], [tgt[1]], [0], c="m", marker="*", s=60)
    ax3.set_title("Trajectory context", fontsize=9)

    axr = fig.add_subplot(2, 2, 4)
    axr.set_title("Spin rates", fontsize=9)
    axr.set_xlim(float(t[0]), float(t[-1]))
    axr.set_ylim(0, max(float(p_rps.max()) * 1.05, 20))
    axr.set_ylabel("rps")
    axr.grid(alpha=0.3)
    sb, = axr.plot([], [], "C0-", lw=1.5, label="Body")
    sn, = axr.plot([], [], "C1-", lw=1.5, label="Nose")
    sc = axr.axvline(0, color="k", ls="--", lw=0.8)
    axr.legend(fontsize=7)

    hud = fig.text(
        0.02,
        0.02,
        "",
        fontsize=9,
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.9),
    )

    def update(frame):
        i = int(idx[frame])
        ang_b = body_vis[frame]
        body_stripe.set_data(
            [0.15 * np.cos(ang_b), 0.95 * np.cos(ang_b)],
            [0.15 * np.sin(ang_b), 0.95 * np.sin(ang_b)],
        )
        ang_n = nose_vis[frame]
        colors = ["crimson", "darkred", "crimson", "darkred"]
        for k, line in enumerate(canard_lines):
            a = ang_n + k * np.pi / 2
            line.set_data([0.5 * np.cos(a), 1.85 * np.cos(a)], [0.5 * np.sin(a), 1.85 * np.sin(a)])
            line.set_color(colors[k])
        if free[i] > 0.5:
            nose_circ.set_facecolor("orange")
            mode_label.set_text("FREE-SPIN — canards rotating (lift cancels)")
            mode_label.set_color("darkorange")
        else:
            nose_circ.set_facecolor("#e74c3c")
            mode_label.set_text("GUIDING — canards HELD (steady side force)")
            mode_label.set_color("crimson")
        trail.set_data(x[: i + 1], y[: i + 1])
        trail.set_3d_properties(z[: i + 1])
        sb.set_data(t[: i + 1], p_rps[: i + 1])
        sn.set_data(t[: i + 1], np.abs(nose_rps[: i + 1]))
        sc.set_xdata([t[i], t[i]])
        hud.set_text(
            "t=%5.1fs  V=%4.0f m/s  h=%4.0fm  |  body=%5.0f rps  nose=%5.1f rps  |  %s"
            % (
                t[i],
                speed[i],
                z[i],
                p_rps[i],
                nose_rps[i],
                "FREE-SPIN" if free[i] > 0.5 else "GUIDING",
            )
        )
        return body_stripe, *canard_lines, trail, sb, sn, mode_label, hud

    anim = FuncAnimation(fig, update, frames=n_frames, interval=100, blit=False)
    path = out / "pgk_canards_local.gif"
    anim.save(str(path), writer=PillowWriter(fps=10), dpi=100)
    plt.close()
    print("  wrote", path)



def fig_real_motion_full(out: Path):
    """Static multi-panel: 3D path, ground track, dual-spin, free-spin/hold, AoA."""
    print("=== Real motion full panel ===")
    data = _anim_log(log_every=80)
    t = data["t"]
    x, y, z = data["x"], data["y"], data["z"]
    p_rps = data["p_rps"]
    nose_rps = data["nose_rps"]
    free = data["free"]
    speed = data["speed"]
    aoa = data["aoa"]
    tgt = data["target"]
    impact = data["impact"]
    miss = data["miss"]

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(
        "PGK real motion overview (dual-spin free-spin / hold)  |  miss = %.1f m" % miss,
        fontsize=13,
        fontweight="bold",
    )

    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(x, y, z, "C0-", lw=1.5)
    ax.scatter([0], [0], [z[0]], c="g", s=40, label="Gun")
    ax.scatter([tgt[0]], [tgt[1]], [0], c="m", marker="*", s=80, label="Target")
    ax.scatter([impact[0]], [impact[1]], [0], c="r", marker="x", s=50, label="Impact")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("3-D trajectory"); ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(x, y, "C0-", lw=1.5, label="Path")
    ax.scatter([0], [0], c="g", s=40, label="Gun")
    ax.scatter([tgt[0]], [tgt[1]], c="m", marker="*", s=90, label="Target")
    ax.scatter([impact[0]], [impact[1]], c="r", marker="x", s=60, label="Impact")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Ground track"); ax.axis("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, p_rps, "C0-", lw=1.4, label="Body spin")
    ax.plot(t, np.abs(nose_rps), "C1-", lw=1.4, label="|Nose| spin")
    ax.set_xlabel("t (s)"); ax.set_ylabel("rps")
    ax.set_title("Dual-spin rates"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, free, "C2-", lw=1.2, label="free_spin (1=free)")
    ax.plot(t, aoa / (np.max(np.abs(aoa)) + 1e-6), "C3-", alpha=0.8, label="AoA (norm)")
    ax.set_xlabel("t (s)")
    ax.set_title("Free-spin / hold mode + AoA")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_real_motion_full.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)



def fig_ji_style(out: Path):
    """Paper-style (Ji et al.): complex AoA, spin, CY proxy, free-spin/hold."""
    print("=== Ji-style angular / aero panels ===")
    data = _anim_log(log_every=70)
    t = data["t"]
    z = data["z"]
    p_rps = data["p_rps"]
    nose_rps = data["nose_rps"]
    free = data["free"]
    aoa = data["aoa"]
    speed = data["speed"]
    # Approximate sideslip proxy from yaw rate of path
    y = data["y"]
    x = data["x"]
    # path azimuth change as crude beta proxy
    psi = np.arctan2(np.gradient(y, t), np.gradient(x, t) + 1e-9)
    beta_deg = np.degrees(psi - psi[0]) * 0.15  # scaled visual proxy
    # CY proxy: lateral force order ~ mass * lateral accel when holding
    vx = np.gradient(x, t); vy = np.gradient(y, t)
    ax = np.gradient(vx, t); ay = np.gradient(vy, t)
    # body-approx lateral accel magnitude in horizontal plane
    a_lat = np.sqrt(ax**2 + ay**2)
    rho0 = 1.225
    S = Params().ref_area
    qdyn = 0.5 * rho0 * np.maximum(speed, 1.0)**2
    Cy_proxy = (Params().mass * a_lat) / (qdyn * S + 1e-6)
    Cy_proxy = np.clip(Cy_proxy, 0, 0.5)

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(
        "Ji-style dual-spin diagnostics (AoA, spin, CY proxy, mode)  miss=%.1fm" % data["miss"],
        fontsize=13, fontweight="bold",
    )

    ax = fig.add_subplot(2, 2, 1)
    ax.plot(t, aoa, "C0-", label=r"$\alpha$ (deg)")
    ax.plot(t, beta_deg, "C1-", alpha=0.85, label=r"$\beta$ proxy (deg)")
    ax.set_xlabel("t (s)"); ax.set_ylabel("deg")
    ax.set_title("Angle of attack / sideslip proxy (complex AoA idea)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(aoa, beta_deg, "C0-", lw=1.0, alpha=0.85)
    ax.scatter([aoa[0]], [beta_deg[0]], c="g", s=40, label="start")
    ax.scatter([aoa[-1]], [beta_deg[-1]], c="r", s=40, label="end")
    ax.set_xlabel(r"$\alpha$ (deg)"); ax.set_ylabel(r"$\beta$ proxy (deg)")
    ax.set_title(r"Complex AoA plane ($\alpha$–$\beta$)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, p_rps, label="aft body $p_a$")
    ax.plot(t, nose_rps, label="forebody $p_f$")
    ax.set_xlabel("t (s)"); ax.set_ylabel("rps")
    ax.set_title("Dual-spin rates (aft / forebody)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, Cy_proxy, "C3-", label=r"$C_Y$ proxy from path accel")
    ax.plot(t, free * np.nanmax(Cy_proxy) * 0.2, "C2--", alpha=0.7, label="free_spin (scaled)")
    ax.set_xlabel("t (s)"); ax.set_ylabel(r"$C_Y$ proxy")
    ax.set_title("Lateral force proxy + HOLD/FREE mode")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_ji_style.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)




def fig_motion_explained(out: Path):
    """How dual-spin PGK moves: free-spin vs hold, path, spin."""
    print("=== Motion explained ===")
    data = _anim_log(log_every=90)
    t_ = data["t"]
    x, y, z = data["x"], data["y"], data["z"]
    free = data["free"]
    p_rps = data["p_rps"]
    nose = data["nose_rps"]
    tgt = data["target"]
    miss = data["miss"]

    fig = plt.figure(figsize=(12, 8))
    fig.suptitle("How a dual-spin PGK moves (miss=%.1f m)" % miss, fontsize=13, fontweight="bold")

    ax = fig.add_subplot(2, 2, 1)
    ax.plot(x, y, "C0-", lw=1.5)
    ax.scatter([tgt[0]], [tgt[1]], c="m", marker="*", s=90, label="Target")
    ax.set_title("Ground track"); ax.axis("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(t_, z, "C0-")
    ax.set_xlabel("t (s)"); ax.set_ylabel("h (m)"); ax.set_title("Altitude"); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t_, free, "C2-", label="free_spin=1")
    ax.fill_between(t_, 0, free, alpha=0.25, color="C2")
    ax.set_xlabel("t (s)"); ax.set_title("FREE-SPIN vs HOLD")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t_, p_rps, label="body")
    ax.plot(t_, np.abs(nose), label="|nose|")
    ax.set_xlabel("t (s)"); ax.set_ylabel("rps")
    ax.set_title("Dual-spin"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    path = out / "pgk_motion_explained.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)


def fig_paper_motion(out: Path):
    """Compact motion panel used as pgk_paper_motion.png."""
    print("=== Paper motion panel ===")
    data = _anim_log(log_every=90)
    t_ = data["t"]
    x, y, z = data["x"], data["y"], data["z"]
    p_rps = data["p_rps"]
    nose = data["nose_rps"]
    aoa = data["aoa"]
    free = data["free"]
    tgt = data["target"]

    fig = plt.figure(figsize=(12, 5))
    fig.suptitle("Paper motion panel (trajectory + spin + AoA)", fontsize=12, fontweight="bold")
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    ax.plot(x, y, z, lw=1.2)
    ax.scatter([tgt[0]], [tgt[1]], [0], c="m", marker="*", s=50)
    ax.set_title("3-D path")

    ax = fig.add_subplot(1, 3, 2)
    ax.plot(t_, p_rps, label="body")
    ax.plot(t_, np.abs(nose), label="nose")
    ax.plot(t_, free * np.nanmax(p_rps) * 0.05, "--", alpha=0.6, label="free(scaled)")
    ax.set_title("Spin"); ax.legend(fontsize=7); ax.grid(alpha=0.3)

    ax = fig.add_subplot(1, 3, 3)
    ax.plot(t_, aoa)
    ax.set_title("AoA"); ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    path = out / "pgk_paper_motion.png"
    plt.savefig(path, dpi=140)
    plt.close()
    print("  wrote", path)



def main():
    ap = argparse.ArgumentParser(description="Regenerate PGK figures and GIFs")
    ap.add_argument("--out", type=str, default=".", help="Output directory")
    ap.add_argument("--suite", action="store_true")
    ap.add_argument("--paper", action="store_true")
    ap.add_argument("--workflow", action="store_true")
    ap.add_argument("--gif", action="store_true", help="Canards local GIF only")
    ap.add_argument("--motion", action="store_true", help="pgk_real_motion_full.png")
    ap.add_argument("--ji", action="store_true", help="Ji-style AoA/spin/CY panels")
    ap.add_argument("--all", action="store_true", help="Everything (default if no flags)")
    args = ap.parse_args()

    flags = args.suite or args.paper or args.workflow or args.gif or args.motion or getattr(args,'ji',False)
    do_all = args.all or not flags

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    os.chdir(out if out.is_absolute() else Path.cwd())
    # keep relative writes under out
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Default (no flags / --all): regenerate full current-data set
    if do_all or args.suite:
        fig_suite(out)
    if do_all or args.paper:
        fig_paper(out)
        fig_paper_motion(out)
    if do_all or args.workflow:
        fig_workflow(out)
    if do_all or args.motion or do_all:
        fig_real_motion_full(out)
        fig_motion_explained(out)
    if do_all or getattr(args, "ji", False):
        fig_ji_style(out)
    if do_all or args.gif:
        gif_canards_local(out)
    # Optional fuze plot if test_fuze available
    if do_all:
        try:
            from test_fuze import main as fuze_main
            import sys
            sys.argv = ["test_fuze.py", "--mode", "all", "--out", str(out / "fuze_test.png")]
            fuze_main()
        except Exception as e:
            print("  fuze_test skipped:", e)

    print("Done. Outputs in", out.resolve())


if __name__ == "__main__":
    main()
</file>

<file src="m1156_pgk_final.py">
#!/usr/bin/env python3
"""
M1156-style Fixed-Canard Dual-Spin PGK – Final upgraded demo
=============================================================
Key upgrades vs original educational skeleton:

1. Dual-spin
   - Body keeps high spin for gyroscopic stability
   - Nose/canard assembly has its own roll angle + rate
   - Brake torque de-spins the nose and then holds commanded orientation
   - Mild reaction torque on body (scaled)

2. Aerodynamics
   - Mild Mach dependence on Cd, Clα, Cmα, damping, Magnus
   - Induced drag from body lift
   - More physical (but still simplified) canard force

3. Environment
   - ISA-like density + speed of sound
   - Gravity varies with altitude
   - Coriolis acceleration

4. Guidance
   - Predictor includes frozen estimate of current canard force
   - Fail-safe inhibit if predicted miss stays excessive
   - Rate-limited orientation command

5. Hygiene
   - Exact control interval
   - Quaternion renormalisation + rate clamps
   - Dual-spin states logged

Still a research/demo model. Open-loop range should be checked against
real firing tables before trusting quantitative CEP numbers.
"""

from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


# =====================================================================
#  Parameters
# =====================================================================

@dataclass
class Params:
    # ----- Mass properties (approx. 155 mm HE + PGK) -----
    mass: float = 43.2
    Ixx: float = 0.15
    Iyy: float = 1.85
    Izz: float = 1.85
    diameter: float = 0.155
    ref_area: float = 0.01887
    cg_to_nose: float = 0.46
    Ixx_nose: float = 0.0040

    # ----- Atmosphere / Earth -----
    rho0: float = 1.225
    T0: float = 288.15
    lapse: float = 0.0065
    g0: float = 9.80665                 # overwritten by latitude
    latitude_deg: float = 30.0
    R_earth: float = 6371000.0
    omega_earth: float = 7.292115e-5
    wind_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # ----- Body aero (base values; mild Mach dependence applied) -----
    Cd0: float = 0.27
    Cl_alpha: float = 1.55
    max_Cl: float = 0.95
    Cmq: float = -3.8
    Clp: float = -0.045
    Cm_alpha: float = -2.1
    Cn_pa: float = 0.12
    Cm_pa: float = -0.008

    # ----- Fixed canards -----
    canard_area: float = 0.00115
    canard_Cl: float = 2.5
    canard_authority: float = 0.40
    canard_min_speed: float = 75.0

    # ----- Dual-spin brake -----
    brake_Kp: float = 5.0
    brake_Kd: float = 0.20
    brake_max_torque: float = 4.0
    residual_nose_rate: float = 0.3

    # ----- Launch -----
    muzzle_velocity: float = 605.0
    elevation_deg: float = 42.0
    azimuth_bias_deg: float = 0.35
    initial_spin_rps: float = 170.0
    launch_height: float = 4.0

    # ----- Target -----
    target_x: float = 17500.0
    target_y: float = 300.0
    target_z: float = 0.0

    # ----- Guidance -----
    arm_time: float = 16.0
    zem_deadband: float = 15.0
    fail_safe_miss: float = 220.0
    orientation_slew_tau: float = 0.25
    max_slew_rate_deg: float = 100.0
    K_orient: float = 4.0
    predictor_dt: float = 0.15
    predictor_horizon: float = 45.0

    # ----- Integration -----
    dt: float = 0.004
    control_dt: float = 0.016
    tmax: float = 65.0

    # ----- Optional noise -----
    gps_pos_sigma: float = 0.0
    gps_vel_sigma: float = 0.0


def g0_at_latitude(lat_deg: float) -> float:
    lat = np.radians(lat_deg)
    g_fts2 = 32.174 * (1.0 - 0.0026 * np.cos(2.0 * lat))
    return g_fts2 * 0.3048


# =====================================================================
#  Atmosphere / gravity / Coriolis
# =====================================================================

def atmosphere(h: float, p: Params) -> Tuple[float, float]:
    h = max(h, 0.0)
    T = max(p.T0 - p.lapse * h, 216.65)
    rho = p.rho0 * (T / p.T0) ** (g0_at_latitude(p.latitude_deg) / (p.lapse * 287.05) - 1.0)
    a = np.sqrt(1.4 * 287.05 * T)
    return rho, a


def gravity(h: float, p: Params) -> float:
    return p.g0 * (p.R_earth / (p.R_earth + max(h, 0.0))) ** 2


def coriolis(v: np.ndarray, lat_deg: float, omega_e: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    Omega = omega_e * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    return -2.0 * np.cross(Omega, v)


# =====================================================================
#  Quaternion helpers
# =====================================================================

def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else q


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def q_dot(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    p, qr, r = omega
    Om = np.array([
        [0, -p, -qr, -r],
        [p,  0,  r, -qr],
        [qr, -r, 0,  p],
        [r,  qr, -p, 0],
    ])
    return 0.5 * Om @ q


def euler_to_q(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(-pitch/2), np.sin(-pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return q_normalize(np.array([w, x, y, z]))


def q_to_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


# =====================================================================
#  Mild Mach dependence
# =====================================================================

def aero_scale(Mach: float) -> dict:
    M = max(Mach, 0.1)
    # smooth transonic bump for drag
    if M < 0.95:
        k_cd = 1.0 + 0.12 * M**2
    elif M < 1.25:
        k_cd = 1.15 + 0.6 * ((M - 0.95) / 0.3)**2
    else:
        k_cd = 1.25 * (1.0 + 0.05 / M)
    k_cl = 1.0 - 0.08 * max(M - 0.8, 0.0)
    k_cm = 1.0 - 0.15 * max(M - 0.9, 0.0)
    return {"Cd": k_cd, "Cl": k_cl, "Cm": k_cm}


# =====================================================================
#  Guidance
# =====================================================================

class Guidance:
    def __init__(self, p: Params):
        self.p = p
        self.theta_c = 0.0
        self.free_spin = True
        self.inhibited = False
        self.target = np.array([p.target_x, p.target_y, p.target_z])
        self.last_F_can = np.zeros(3)

    def predict_impact(self, r: np.ndarray, v: np.ndarray,
                       F_can_est: np.ndarray = None) -> Tuple[np.ndarray, float]:
        p = self.p
        dt = p.predictor_dt
        rr, vv = r.copy(), v.copy()
        t = 0.0
        F_can = np.zeros(3) if F_can_est is None else F_can_est.copy()

        while rr[2] > 0.0 and t < p.predictor_horizon:
            speed = np.linalg.norm(vv)
            rho, a = atmosphere(rr[2], p)
            Mach = speed / max(a, 1.0)
            sc = aero_scale(Mach)
            qdyn = 0.5 * rho * speed * speed
            if speed > 1.0:
                a_drag = -p.Cd0 * sc["Cd"] * qdyn * p.ref_area / p.mass * (vv / speed)
            else:
                a_drag = np.zeros(3)
            g = gravity(rr[2], p)
            a = a_drag + F_can / p.mass + np.array([0.0, 0.0, -g])
            a += coriolis(vv, p.latitude_deg, p.omega_earth)
            vv = vv + a * dt
            rr = rr + vv * dt
            t += dt

        impact = rr.copy()
        impact[2] = 0.0
        return impact, max(t, 0.3)

    def update(self, t: float, r: np.ndarray, v: np.ndarray, dt: float,
               F_can_current: np.ndarray):
        self.last_F_can = F_can_current.copy()
        if t < self.p.arm_time or self.inhibited:
            self.free_spin = True
            return

        r_m = r + self.p.gps_pos_sigma * np.random.randn(3)
        v_m = v + self.p.gps_vel_sigma * np.random.randn(3)
        impact, _ = self.predict_impact(r_m, v_m, F_can_est=self.last_F_can)
        zem_vec = self.target[:2] - impact[:2]
        zem = np.linalg.norm(zem_vec)

        if zem > self.p.fail_safe_miss and t > self.p.arm_time + 4.0:
            self.inhibited = True
            self.free_spin = True
            return

        if zem < self.p.zem_deadband:
            self.free_spin = True
            return

        self.free_spin = False
        desired = np.arctan2(zem_vec[1], zem_vec[0])
        err = (desired - self.theta_c + np.pi) % (2 * np.pi) - np.pi
        max_rate = np.radians(self.p.max_slew_rate_deg)
        rate = np.clip(self.p.K_orient * err, -max_rate, max_rate)
        alpha = dt / (self.p.orientation_slew_tau + dt)
        self.theta_c = (self.theta_c + alpha * rate * self.p.orientation_slew_tau) % (2 * np.pi)


# =====================================================================
#  Forces & moments
# =====================================================================

def forces_moments(r, v, q, omega_b, theta_nose, omega_nose,
                   free_spin, theta_c, p: Params):
    R = q_to_dcm(q)
    body_x = R[:, 0]
    v_air = v - p.wind_ned
    speed = np.linalg.norm(v_air)
    rho, a = atmosphere(r[2], p)
    Mach = speed / max(a, 1.0)
    sc = aero_scale(Mach)
    qdyn = 0.5 * rho * speed ** 2
    S, d = p.ref_area, p.diameter

    # Body drag
    if speed > 1.0:
        F_drag = -p.Cd0 * sc["Cd"] * qdyn * S * (v_air / speed)
    else:
        F_drag = np.zeros(3)

    # Body lift + overturning + Magnus
    F_lift = np.zeros(3)
    M_over = np.zeros(3)
    M_mag = np.zeros(3)
    F_mag = np.zeros(3)
    alpha = 0.0

    if speed > 35.0:
        vhat = v_air / speed
        ca = np.clip(np.dot(body_x, vhat), -1.0, 1.0)
        alpha = np.arccos(ca)
        if alpha > 1e-4:
            ldir = body_x - ca * vhat
            n = np.linalg.norm(ldir)
            if n > 1e-9:
                ldir /= n
                Cl = np.clip(p.Cl_alpha * sc["Cl"] * alpha, -p.max_Cl, p.max_Cl)
                F_lift = Cl * qdyn * S * ldir
                # induced drag
                F_drag = F_drag - (Cl**2) / (np.pi * 3.5) * qdyn * S * vhat

                axis2 = np.cross(body_x, vhat)
                n2 = np.linalg.norm(axis2)
                if n2 > 1e-9:
                    axis2 /= n2
                    M_over_mag = -p.Cm_alpha * sc["Cm"] * qdyn * S * d * alpha
                    M_over = R.T @ (M_over_mag * axis2)

                    pbar = omega_b[0] * d / (2.0 * speed)
                    F_mag = (p.Cn_pa * qdyn * S * pbar * alpha) * axis2
                    M_mag = R.T @ ((p.Cm_pa * qdyn * S * d * pbar * alpha) * axis2)

    # Canards on de-spun nose
    F_can = np.zeros(3)
    M_can = np.zeros(3)
    tau_aero_n = 0.0

    if speed > p.canard_min_speed:
        vhat = v_air / speed
        desired = np.array([np.cos(theta_nose), np.sin(theta_nose), 0.0])
        force_dir = desired - np.dot(desired, vhat) * vhat
        n = np.linalg.norm(force_dir)
        if n > 1e-6:
            force_dir /= n
            Fmag = p.canard_authority * qdyn * p.canard_area * p.canard_Cl
            F_can = Fmag * force_dir
            r_c = np.array([p.cg_to_nose, 0.0, 0.0])
            M_can = np.cross(r_c, R.T @ F_can)
            # de-spin aero torque on nose
            tau_aero_n = -0.06 * qdyn * p.canard_area * d * np.sign(omega_nose) \
                         - 0.003 * qdyn * p.canard_area * omega_nose

    # Brake
    if free_spin:
        tau_brake = -0.03 * omega_nose + tau_aero_n
    else:
        err = (theta_c - theta_nose + np.pi) % (2 * np.pi) - np.pi
        rate_err = -omega_nose
        tau_cmd = p.brake_Kp * err + p.brake_Kd * rate_err
        tau_brake = np.clip(tau_cmd, -p.brake_max_torque, p.brake_max_torque) + tau_aero_n

    # Damping
    pr, qr, rr = omega_b
    L_damp = p.Clp * qdyn * S * d * (d / (2*speed + 1e-3)) * pr
    damp = p.Cmq * qdyn * S * d * (d / (2*speed + 1e-3))
    M_damp = np.array([L_damp, damp * qr, damp * rr])

    F = F_drag + F_lift + F_can + F_mag
    M = M_can + M_damp + M_over + M_mag
    M[0] -= 0.12 * tau_brake          # mild reaction

    return F, M, tau_brake, F_can, alpha


# =====================================================================
#  Simulator
# =====================================================================

class Simulator:
    def __init__(self, p: Params):
        self.p = p
        p.g0 = g0_at_latitude(p.latitude_deg)
        self.guid = Guidance(p)
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])
        self.Iinv = np.linalg.inv(self.I)

    def deriv(self, t, s, free_spin, theta_c):
        r, v = s[0:3], s[3:6]
        q = q_normalize(s[6:10])
        omega = s[10:13]
        theta_n, omega_n = s[13], s[14]

        F, M, tau_b, F_can, _ = forces_moments(
            r, v, q, omega, theta_n, omega_n, free_spin, theta_c, self.p)

        g = gravity(r[2], self.p)
        a = F / self.p.mass + np.array([0.0, 0.0, -g])
        a += coriolis(v, self.p.latitude_deg, self.p.omega_earth)

        wdot = self.Iinv @ (M - np.cross(omega, self.I @ omega))
        omega_n_dot = tau_b / self.p.Ixx_nose

        ds = np.zeros(15)
        ds[0:3] = v
        ds[3:6] = a
        ds[6:10] = q_dot(q, omega)
        ds[10:13] = wdot
        ds[13] = omega_n
        ds[14] = omega_n_dot
        return ds, F_can

    def step(self, t, s, dt, free_spin, theta_c):
        k1, F_can = self.deriv(t, s, free_spin, theta_c)
        k2, _ = self.deriv(t + dt/2, s + dt/2*k1, free_spin, theta_c)
        k3, _ = self.deriv(t + dt/2, s + dt/2*k2, free_spin, theta_c)
        k4, _ = self.deriv(t + dt,   s + dt*k3,   free_spin, theta_c)
        ns = s + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        ns[6:10] = q_normalize(ns[6:10])
        ns[10:13] = np.clip(ns[10:13], -2000.0, 2000.0)
        ns[14] = np.clip(ns[14], -600.0, 600.0)
        ns[13] = (ns[13] + np.pi) % (2*np.pi) - np.pi
        return ns, F_can

    def run(self, verbose=True):
        p = self.p
        elev = np.radians(p.elevation_deg)
        az = np.arctan2(p.target_y, p.target_x) + np.radians(p.azimuth_bias_deg)
        vm = p.muzzle_velocity

        s = np.zeros(15)
        s[0:3] = [0.0, 0.0, p.launch_height]
        s[3:6] = [vm*np.cos(elev)*np.cos(az),
                  vm*np.cos(elev)*np.sin(az),
                  vm*np.sin(elev)]
        s[6:10] = euler_to_q(0.0, elev, az)
        s[10] = 2*np.pi * p.initial_spin_rps
        s[13] = 0.0
        s[14] = 2*np.pi * p.initial_spin_rps * 0.12

        log = {k: [] for k in [
            "t", "x", "y", "z", "speed", "pitch", "yaw",
            "p_rps", "nose_rps", "theta_c_deg", "theta_nose_deg",
            "free_spin", "aoa", "zem", "inhibited"
        ]}

        t = 0.0
        sub = max(int(round(p.control_dt / p.dt)), 1)
        n = int(np.ceil(p.tmax / p.dt))
        F_can = np.zeros(3)

        for i in range(n + 1):
            if i % sub == 0:
                self.guid.update(t, s[0:3], s[3:6], sub * p.dt, F_can)
            if i % 6 == 0:
                self._log(log, t, s)
            if s[2] < 0.0 and t > 1.0:
                break
            s, F_can = self.step(t, s, p.dt, self.guid.free_spin, self.guid.theta_c)
            t += p.dt

        if verbose:
            impact = np.array([log["x"][-1], log["y"][-1]])
            miss = np.linalg.norm(impact - np.array([p.target_x, p.target_y]))
            print("=== M1156-style Dual-Spin Fixed-Canard PGK (final) ===")
            print(f"Target     : ({p.target_x:.0f}, {p.target_y:.0f}) m")
            print(f"Impact     : ({impact[0]:.1f}, {impact[1]:.1f}) m")
            print(f"Miss       : {miss:.1f} m")
            print(f"Apogee     : {max(log['z']):.0f} m")
            print(f"ToF        : {log['t'][-1]:.1f} s")
            print(f"Max |AoA|  : {max(np.abs(log['aoa'])):.1f}°")
            print(f"Final body spin : {log['p_rps'][-1]:.1f} rps")
            print(f"Final nose spin : {log['nose_rps'][-1]:.2f} rps")
            print(f"Inhibited  : {bool(log['inhibited'][-1])}")
        return log

    def _log(self, log, t, s):
        r, v = s[0:3], s[3:6]
        q = q_normalize(s[6:10])
        omega = s[10:13]
        eul = q_to_euler(q)
        speed = np.linalg.norm(v)
        R = q_to_dcm(q)
        bx = R[:, 0]
        aoa = 0.0
        if speed > 1.0:
            ca = np.clip(np.dot(bx, v/speed), -1.0, 1.0)
            aoa = np.degrees(np.arccos(ca))
        impact, _ = self.guid.predict_impact(r, v, self.guid.last_F_can)
        zem = np.linalg.norm(self.guid.target[:2] - impact[:2])

        log["t"].append(t)
        log["x"].append(r[0]); log["y"].append(r[1]); log["z"].append(r[2])
        log["speed"].append(speed)
        log["pitch"].append(np.degrees(eul[1]))
        log["yaw"].append(np.degrees(eul[2]))
        log["p_rps"].append(omega[0]/(2*np.pi))
        log["nose_rps"].append(s[14]/(2*np.pi))
        log["theta_c_deg"].append(np.degrees(self.guid.theta_c))
        log["theta_nose_deg"].append(np.degrees(s[13]))
        log["free_spin"].append(1.0 if self.guid.free_spin else 0.0)
        log["aoa"].append(aoa)
        log["zem"].append(zem)
        log["inhibited"].append(1.0 if self.guid.inhibited else 0.0)


# =====================================================================
#  Plots + main
# =====================================================================

def plot_results(log, log_ol, p, impact):
    fig = plt.figure(figsize=(15, 10))

    ax = fig.add_subplot(2, 3, (1, 4), projection="3d")
    ax.plot(log["x"], log["y"], log["z"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], log_ol["z"], lw=1.0, ls="--", alpha=0.7, label="Open-loop")
    ax.scatter(0, 0, p.launch_height, c="g", s=40, label="Gun")
    ax.scatter(p.target_x, p.target_y, 0, c="m", marker="*", s=90, label="Target")
    ax.scatter(impact[0], impact[1], 0, c="r", s=40, label="Impact")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("3-D Trajectory (Dual-Spin PGK)")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(log["t"], log["z"], label="Guided")
    ax.plot(log_ol["t"], log_ol["z"], ls="--", alpha=0.7, label="Open-loop")
    ax.set_title("Altitude"); ax.set_xlabel("t (s)"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 3)
    ax.plot(log["t"], log["speed"])
    ax.set_title("Speed"); ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 3, 5)
    ax.plot(log["t"], log["theta_c_deg"], label="θ_c")
    ax.plot(log["t"], log["theta_nose_deg"], label="θ_nose", alpha=0.8)
    ax.plot(log["t"], np.array(log["free_spin"])*40, label="free-spin×40")
    ax2 = ax.twinx()
    ax2.plot(log["t"], log["zem"], color="C3", alpha=0.7, label="ZEM")
    ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=7); ax2.legend(loc="upper right", fontsize=7)

    ax = fig.add_subplot(2, 3, 6)
    ax.plot(log["t"], log["pitch"], label="pitch")
    ax.plot(log["t"], log["yaw"], label="yaw")
    ax.plot(log["t"], log["p_rps"], label="body rps")
    ax.plot(log["t"], log["nose_rps"], label="nose rps")
    ax.set_title("Attitude & Spin"); ax.set_xlabel("t (s)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("m1156_pgk_final.png", dpi=140)
    plt.close()

    fig2, ax = plt.subplots(figsize=(8, 5))
    ax.plot(log["x"], log["y"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], lw=1.0, ls="--", alpha=0.7, label="Open-loop")
    ax.scatter(0, 0, c="g", marker="s", s=50, label="Gun")
    ax.scatter(p.target_x, p.target_y, c="m", marker="*", s=90, label="Target")
    ax.scatter(impact[0], impact[1], c="r", marker="x", s=60, label="Impact")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Ground Track – Dual-Spin PGK")
    ax.axis("equal"); ax.legend(); ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("m1156_pgk_final_track.png", dpi=140)
    plt.close()
    print("\nSaved: m1156_pgk_final.png , m1156_pgk_final_track.png")


def main():
    p = Params()
    log = Simulator(p).run(verbose=True)

    p_ol = Params()
    p_ol.arm_time = 9999.0
    log_ol = Simulator(p_ol).run(verbose=False)
    miss_ol = np.hypot(log_ol["x"][-1] - p_ol.target_x, log_ol["y"][-1] - p_ol.target_y)
    print(f"Open-loop miss (no guidance): {miss_ol:.0f} m")

    impact = np.array([log["x"][-1], log["y"][-1]])
    plot_results(log, log_ol, p, impact)


if __name__ == "__main__":
    main()

</file>

<file src="m1156_pgk_upgraded.py">
#!/usr/bin/env python3
"""
M1156-style Fixed-Canard Dual-Spin PGK – Upgraded 6-DOF Simulation
===================================================================
Improvements over the original educational skeleton:

1. Dual-spin dynamics
   - High-spin aft body (gyroscopic stability)
   - Mechanically de-spun nose/canard assembly with relative roll rate
   - Brake torque that de-spins the nose and then holds commanded orientation
   - Reaction torque on the body

2. Mach-dependent aerodynamics
   - Cd(M, α), Cl_α(M), Cm_α(M), Magnus, roll/pitch damping
   - Induced drag from body lift + canard force
   - More physical canard force model with authority limit

3. Atmosphere / Earth
   - Density + speed-of-sound from a simple ISA-like model
   - Gravity varies with altitude
   - Coriolis acceleration (rotating Earth)

4. Guidance & control
   - Drag + current-control-force-aware impact predictor
   - Fail-safe (inhibit guidance if predicted miss remains excessive)
   - First-order orientation command with realistic rate/torque limits
   - Continuous free-spin ↔ guided transition

5. Numerical hygiene
   - Exact control interval handling
   - Quaternion renormalisation every step
   - Clean logging of dual-spin states, AoA, ZEM, nose rate, etc.

Still a research/demo model – not a certified engineering tool.
Validate open-loop range/drift against real firing tables before trusting
guided CEP numbers.
"""

from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


# =====================================================================
#  Parameters
# =====================================================================

@dataclass
class Params:
    # ----- Projectile mass properties (approx. 155 mm HE + PGK) -----
    mass: float = 43.5                  # kg  (slightly higher with PGK)
    Ixx: float = 0.148                  # kg·m²  (roll – body)
    Iyy: float = 1.82                   # kg·m²  (pitch)
    Izz: float = 1.82                   # kg·m²  (yaw)
    diameter: float = 0.155             # m
    ref_area: float = 0.01887           # m²   (π d²/4)
    cg_to_nose: float = 0.48            # m    (canard force lever arm)
    # Nose / canard assembly inertia about spin axis (small)
    Ixx_nose: float = 0.0045            # kg·m²

    # ----- Atmosphere / gravity / Earth -----
    rho0: float = 1.225                 # kg/m³ sea-level
    T0: float = 288.15                  # K
    lapse: float = 0.0065               # K/m
    g0: float = 9.80665                 # m/s² (overwritten by latitude)
    latitude_deg: float = 30.0
    R_earth: float = 6371000.0          # m
    omega_earth: float = 7.292115e-5    # rad/s
    # Optional constant wind (NED, m/s) – set to zero for calm day
    wind_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # ----- Aerodynamics (body) – base values; Mach dependence applied in aero() -----
    Cd0_sub: float = 0.25
    Cd0_sup: float = 0.32
    Cl_alpha0: float = 1.55             # 1/rad at low Mach
    max_Cl: float = 0.95
    Cmq0: float = -4.5
    Clp0: float = -0.050
    Cm_alpha0: float = -2.4             # negative = stabilizing
    Cn_pa0: float = 0.10                # Magnus force slope
    Cm_pa0: float = -0.007              # Magnus moment slope (placeholder)

    # ----- Fixed canards (M1156-style) -----
    canard_area: float = 0.00085        # m² total effective area of control pair
    canard_Cl: float = 2.0              # effective lift slope
    canard_stall_alpha: float = np.radians(12.0)
    canard_authority: float = 0.28      # fraction of ideal force retained (conservative)
    canard_min_speed: float = 85.0      # m/s

    # ----- Dual-spin brake / de-spin -----
    # When free-spin the brake is off → nose freewheels (driven only by aero torque)
    # When guided the brake applies torque to hold θ_nose ≈ θ_c
    brake_Kp: float = 6.0               # N·m / rad   (orientation error gain)
    brake_Kd: float = 0.22              # N·m / (rad/s)
    brake_max_torque: float = 5.0       # N·m
    residual_nose_rate: float = 0.4     # rad/s  (small residual spin of “de-spun” nose)

    # ----- Launch conditions -----
    muzzle_velocity: float = 605.0      # m/s
    elevation_deg: float = 42.0
    azimuth_bias_deg: float = 0.40      # intentional small error
    initial_spin_rps: float = 175.0     # body spin
    launch_height: float = 4.0          # m

    # ----- Target -----
    target_x: float = 17500.0           # m
    target_y: float = 320.0             # m
    target_z: float = 0.0

    # ----- Guidance / control -----
    arm_time: float = 15.0              # s
    zem_deadband: float = 18.0          # m
    fail_safe_miss: float = 280.0       # m  (if still larger after arm → inhibit)
    orientation_slew_tau: float = 0.30  # s
    max_slew_rate_deg: float = 90.0     # deg/s
    K_orient: float = 3.5
    predictor_dt: float = 0.10          # s
    predictor_horizon: float = 45.0     # s

    # ----- Integration -----
    dt: float = 0.003                   # s   (slightly coarser for speed)
    control_dt: float = 0.012           # s
    tmax: float = 70.0                  # s

    # ----- Optional noise (set sigma=0 for deterministic) -----
    gps_pos_sigma: float = 0.0          # m  (1-σ white noise on measured position)
    gps_vel_sigma: float = 0.0          # m/s


def g0_at_latitude(lat_deg: float) -> float:
    """McCoy-style latitude-dependent sea-level gravity → m/s²."""
    lat = np.radians(lat_deg)
    g_fts2 = 32.174 * (1.0 - 0.0026 * np.cos(2.0 * lat))
    return g_fts2 * 0.3048


# =====================================================================
#  Atmosphere helpers
# =====================================================================

def atmosphere(h: float, p: Params) -> Tuple[float, float, float]:
    """
    Simple ISA-like model up to ~11 km.
    Returns (density, speed_of_sound, temperature).
    """
    h = max(h, 0.0)
    T = p.T0 - p.lapse * h
    if T < 216.65:                      # tropopause approx
        T = 216.65
    # density ratio (isothermal approximation above tropopause ignored for simplicity)
    rho = p.rho0 * (T / p.T0) ** (g0_at_latitude(p.latitude_deg) / (p.lapse * 287.05) - 1.0)
    a = np.sqrt(1.4 * 287.05 * T)
    return rho, a, T


def gravity(h: float, p: Params) -> float:
    """Inverse-square gravity."""
    return p.g0 * (p.R_earth / (p.R_earth + max(h, 0.0))) ** 2


def coriolis_acceleration(v: np.ndarray, lat_deg: float, omega_e: float) -> np.ndarray:
    """
    a = -2 Ω × v   in a local North-East-Down frame approximation.
    Ω = ω_e * [cos(lat), 0, -sin(lat)]  (North, East, Down)
    """
    lat = np.radians(lat_deg)
    Omega = omega_e * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    return -2.0 * np.cross(Omega, v)


# =====================================================================
#  Quaternion helpers (scalar-first, body +X = nose)
# =====================================================================

def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else q


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body → inertial DCM."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def q_dot(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    p, qr, r = omega
    Om = np.array([
        [0, -p, -qr, -r],
        [p,  0,  r, -qr],
        [qr, -r, 0,  p],
        [r,  qr, -p, 0],
    ])
    return 0.5 * Om @ q


def euler_to_q(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Pitch sign chosen so positive elevation → nose up."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(-pitch/2), np.sin(-pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return q_normalize(np.array([w, x, y, z]))


def q_to_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


# =====================================================================
#  Mach-dependent aerodynamic coefficients
# =====================================================================

def aero_coeffs(Mach: float, alpha: float, p: Params) -> dict:
    """
    Smooth approximations of typical 155 mm spin-stabilized coefficients.
    Not flight-test data – only shape-correct trends for demo purposes.
    """
    M = max(Mach, 0.05)

    # Zero-lift drag – rises through transonic, settles
    if M < 0.9:
        Cd0 = p.Cd0_sub * (1.0 + 0.15 * M**2)
    elif M < 1.2:
        # crude transonic bump
        Cd0 = p.Cd0_sub + (p.Cd0_sup - p.Cd0_sub) * ((M - 0.9) / 0.3)**2 * 1.8
    else:
        Cd0 = p.Cd0_sup * (1.0 + 0.08 / M)

    # Lift slope softens slightly at high Mach
    Cl_alpha = p.Cl_alpha0 * (1.0 - 0.12 * max(M - 0.8, 0.0))

    # Overturning moment (more stable subsonic, softens transonic)
    Cm_alpha = p.Cm_alpha0 * (1.0 - 0.25 * max(M - 0.9, 0.0))

    # Pitch damping
    Cmq = p.Cmq0 * (1.0 + 0.1 * M)

    # Roll damping
    Clp = p.Clp0 * (1.0 + 0.05 * M)

    # Magnus (very approximate – real data vary strongly)
    Cn_pa = p.Cn_pa0 * (1.0 - 0.3 * max(M - 1.0, 0.0))
    Cm_pa = p.Cm_pa0 * (1.0 - 0.2 * max(M - 1.0, 0.0))

    return {
        "Cd0": Cd0,
        "Cl_alpha": Cl_alpha,
        "Cm_alpha": Cm_alpha,
        "Cmq": Cmq,
        "Clp": Clp,
        "Cn_pa": Cn_pa,
        "Cm_pa": Cm_pa,
    }


# =====================================================================
#  Guidance computer
# =====================================================================

class Guidance:
    def __init__(self, p: Params):
        self.p = p
        self.theta_c = 0.0              # commanded nose orientation [rad]
        self.free_spin = True
        self.inhibited = False          # fail-safe flag
        self.target = np.array([p.target_x, p.target_y, p.target_z])
        self.last_F_can = np.zeros(3)   # frozen control force for predictor

    def predict_impact(self, r: np.ndarray, v: np.ndarray,
                       F_can_est: np.ndarray = None) -> Tuple[np.ndarray, float]:
        """
        Short-horizon forward propagation including gravity, drag and a
        frozen estimate of the current canard force.  Lift/Magnus are
        still omitted because they depend on instantaneous attitude.
        """
        p = self.p
        dt_pred = p.predictor_dt
        rr = r.copy()
        vv = v.copy()
        t = 0.0
        F_can = np.zeros(3) if F_can_est is None else F_can_est.copy()

        while rr[2] > 0.0 and t < p.predictor_horizon:
            speed = np.linalg.norm(vv)
            rho, a, _ = atmosphere(rr[2], p)
            Mach = speed / max(a, 1.0)
            coeffs = aero_coeffs(Mach, 0.0, p)
            qdyn = 0.5 * rho * speed * speed
            if speed > 1.0:
                a_drag = -coeffs["Cd0"] * qdyn * p.ref_area / p.mass * (vv / speed)
            else:
                a_drag = np.zeros(3)
            g = gravity(rr[2], p)
            a = a_drag + F_can / p.mass + np.array([0.0, 0.0, -g])
            # simple Coriolis (constant latitude)
            a += coriolis_acceleration(vv, p.latitude_deg, p.omega_earth)
            vv = vv + a * dt_pred
            rr = rr + vv * dt_pred
            t += dt_pred

        tgo = max(t, 0.3)
        impact = rr.copy()
        impact[2] = 0.0
        return impact, tgo

    def update(self, t: float, r: np.ndarray, v: np.ndarray, dt: float,
               F_can_current: np.ndarray):
        self.last_F_can = F_can_current.copy()

        if t < self.p.arm_time or self.inhibited:
            self.free_spin = True
            return

        # Optional measurement noise
        r_meas = r + self.p.gps_pos_sigma * np.random.randn(3)
        v_meas = v + self.p.gps_vel_sigma * np.random.randn(3)

        impact, _ = self.predict_impact(r_meas, v_meas, F_can_est=self.last_F_can)
        zem_vec = self.target[:2] - impact[:2]
        zem = np.linalg.norm(zem_vec)

        # Fail-safe: if still wildly off after arming, inhibit
        if zem > self.p.fail_safe_miss and t > self.p.arm_time + 3.0:
            self.inhibited = True
            self.free_spin = True
            return

        if zem < self.p.zem_deadband:
            self.free_spin = True
            return

        self.free_spin = False
        desired = np.arctan2(zem_vec[1], zem_vec[0])

        # First-order slew toward desired orientation with rate limit
        err = (desired - self.theta_c + np.pi) % (2 * np.pi) - np.pi
        max_rate = np.radians(self.p.max_slew_rate_deg)
        rate = np.clip(self.p.K_orient * err, -max_rate, max_rate)
        alpha = dt / (self.p.orientation_slew_tau + dt)
        self.theta_c = (self.theta_c + alpha * rate * self.p.orientation_slew_tau) % (2 * np.pi)


# =====================================================================
#  Forces & moments (body + dual-spin nose)
# =====================================================================

def forces_moments(r, v, q, omega_body, theta_nose, omega_nose,
                   free_spin, theta_c, p: Params):
    """
    Returns:
        F_inertial, M_body, tau_brake
    tau_brake is the torque applied by the brake on the nose (positive
    accelerates nose in the positive body-x sense).  Reaction on body is -tau.
    """
    R = q_to_dcm(q)                     # body → inertial
    body_x = R[:, 0]
    speed = np.linalg.norm(v)
    # relative air velocity (simple constant wind)
    v_air = v - p.wind_ned
    speed_air = np.linalg.norm(v_air)
    rho, a, _ = atmosphere(r[2], p)
    Mach = speed_air / max(a, 1.0)
    qdyn = 0.5 * rho * speed_air ** 2
    S, d = p.ref_area, p.diameter

    # ----- Body drag -----
    if speed_air > 1.0:
        coeffs = aero_coeffs(Mach, 0.0, p)
        F_drag = -coeffs["Cd0"] * qdyn * S * (v_air / speed_air)
    else:
        coeffs = aero_coeffs(0.1, 0.0, p)
        F_drag = np.zeros(3)

    # ----- Body lift + overturning + Magnus -----
    F_lift = np.zeros(3)
    M_over_body = np.zeros(3)
    M_magnus_body = np.zeros(3)
    F_magnus = np.zeros(3)
    alpha = 0.0

    if speed_air > 30.0:
        vhat = v_air / speed_air
        ca = np.clip(np.dot(body_x, vhat), -1.0, 1.0)
        alpha = np.arccos(ca)
        if alpha > 1e-4:
            ldir = body_x - ca * vhat
            n = np.linalg.norm(ldir)
            if n > 1e-9:
                ldir /= n
                Cl = np.clip(coeffs["Cl_alpha"] * alpha, -p.max_Cl, p.max_Cl)
                F_lift = Cl * qdyn * S * ldir

                # induced drag (simple)
                Cd_i = (Cl ** 2) / (np.pi * 3.5)
                F_drag = F_drag - Cd_i * qdyn * S * vhat

                axis2_raw = np.cross(body_x, vhat)
                axis2_norm = np.linalg.norm(axis2_raw)
                if axis2_norm > 1e-9:
                    axis2 = axis2_raw / axis2_norm

                    # Overturning moment (stabilizing when Cm_alpha < 0)
                    M_over_mag = -coeffs["Cm_alpha"] * qdyn * S * d * alpha
                    M_over_inertial = M_over_mag * axis2
                    M_over_body = R.T @ M_over_inertial

                    # Magnus force & moment
                    pbar = omega_body[0] * d / (2.0 * speed_air)
                    F_magnus_mag = coeffs["Cn_pa"] * qdyn * S * pbar * alpha
                    F_magnus = F_magnus_mag * axis2
                    M_magnus_mag = coeffs["Cm_pa"] * qdyn * S * d * pbar * alpha
                    M_magnus_inertial = M_magnus_mag * axis2
                    M_magnus_body = R.T @ M_magnus_inertial

    # ----- Fixed canards on de-spun nose -----
    F_can = np.zeros(3)
    M_can = np.zeros(3)
    tau_aero_nose = 0.0                 # aero torque on nose about body-x

    if speed_air > p.canard_min_speed:
        vhat = v_air / speed_air
        # Absolute orientation of the nose assembly (theta_nose is inertial roll of nose)
        # Desired force direction in horizontal plane from nose angle
        desired = np.array([np.cos(theta_nose), np.sin(theta_nose), 0.0])
        # Keep force perpendicular to velocity (pure side force)
        force_dir = desired - np.dot(desired, vhat) * vhat
        n = np.linalg.norm(force_dir)
        if n > 1e-6:
            force_dir /= n
            # Effective α of canards ≈ geometric incidence (simplified)
            # Authority limit + soft stall
            alpha_can = min(alpha + 0.05, p.canard_stall_alpha)
            Cl_can = p.canard_Cl * (alpha_can / p.canard_stall_alpha) \
                     if alpha_can < p.canard_stall_alpha else p.canard_Cl * 0.7
            Fmag = p.canard_authority * qdyn * p.canard_area * Cl_can
            F_can = Fmag * force_dir

            # Moment about CG (force applied at nose)
            r_c_body = np.array([p.cg_to_nose, 0.0, 0.0])
            F_can_body = R.T @ F_can
            M_can = np.cross(r_c_body, F_can_body)

            # Aero torque that de-spins the nose (canard drag / alternator loading)
            # Stronger when |omega_nose| is high – approximates real PGK de-spin
            tau_aero_nose = -0.08 * qdyn * p.canard_area * p.diameter * np.sign(omega_nose) \
                            - 0.004 * qdyn * p.canard_area * omega_nose

    # ----- Brake torque (dual-spin control) -----
    if free_spin:
        # Brake off – residual viscous friction + aero de-spin torque
        tau_brake = -0.04 * omega_nose + tau_aero_nose
    else:
        # Active orientation hold
        err = (theta_c - theta_nose + np.pi) % (2 * np.pi) - np.pi
        # Also drive rate toward a small residual
        rate_err = p.residual_nose_rate * np.sign(err) - omega_nose if abs(err) > 0.05 \
                   else -omega_nose
        tau_cmd = p.brake_Kp * err + p.brake_Kd * rate_err
        tau_brake = np.clip(tau_cmd, -p.brake_max_torque, p.brake_max_torque)
        tau_brake += tau_aero_nose

    # ----- Damping moments on body -----
    p_rate, q_rate, r_rate = omega_body
    L_damp = coeffs["Clp"] * qdyn * S * d * (d / (2 * speed_air + 1e-3)) * p_rate
    damp_coef = coeffs["Cmq"] * qdyn * S * d * (d / (2 * speed_air + 1e-3))
    M_damp = np.array([L_damp, damp_coef * q_rate, damp_coef * r_rate])

    # Total force (inertial) and body moment
    F = F_drag + F_lift + F_can + F_magnus
    M = M_can + M_damp + M_over_body + M_magnus_body
    # Reaction of brake torque on body roll axis.
    # Scaled down: real bearing friction is small relative to body spin energy.
    M[0] -= 0.15 * tau_brake

    return F, M, tau_brake, F_can, alpha


# =====================================================================
#  Simulator
# =====================================================================

class Simulator:
    def __init__(self, p: Params):
        self.p = p
        p.g0 = g0_at_latitude(p.latitude_deg)
        self.guid = Guidance(p)
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])
        self.Iinv = np.linalg.inv(self.I)

    def deriv(self, t, s, free_spin, theta_c):
        """
        State layout (15):
          0:3   position (inertial)
          3:6   velocity
          6:10  quaternion (body)
          10:13 body rates ω
          13    theta_nose (inertial roll of de-spun assembly)
          14    omega_nose
        """
        r       = s[0:3]
        v       = s[3:6]
        q       = q_normalize(s[6:10])
        omega_b = s[10:13]
        theta_n = s[13]
        omega_n = s[14]

        F, M, tau_brake, F_can, _ = forces_moments(
            r, v, q, omega_b, theta_n, omega_n, free_spin, theta_c, self.p)

        # Translational acceleration
        g = gravity(r[2], self.p)
        a = F / self.p.mass + np.array([0.0, 0.0, -g])
        a += coriolis_acceleration(v, self.p.latitude_deg, self.p.omega_earth)

        # Body angular acceleration
        wdot = self.Iinv @ (M - np.cross(omega_b, self.I @ omega_b))

        # Nose dynamics (1-DOF about body x-axis)
        # Ixx_nose * omega_n_dot = tau_brake
        omega_n_dot = tau_brake / self.p.Ixx_nose

        ds = np.zeros(15)
        ds[0:3]   = v
        ds[3:6]   = a
        ds[6:10]  = q_dot(q, omega_b)
        ds[10:13] = wdot
        ds[13]    = omega_n                 # theta_nose_dot = omega_nose
        ds[14]    = omega_n_dot
        return ds, F_can

    def step(self, t, s, dt, free_spin, theta_c):
        # RK4 – F_can from first stage is sufficient for guidance freeze
        k1, F_can = self.deriv(t, s, free_spin, theta_c)
        k2, _     = self.deriv(t + dt/2, s + dt/2 * k1, free_spin, theta_c)
        k3, _     = self.deriv(t + dt/2, s + dt/2 * k2, free_spin, theta_c)
        k4, _     = self.deriv(t + dt,   s + dt   * k3, free_spin, theta_c)
        ns = s + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        ns[6:10] = q_normalize(ns[6:10])
        # hard limits to keep the integration stable
        ns[10:13] = np.clip(ns[10:13], -2500.0, 2500.0)   # body rates rad/s
        ns[14] = np.clip(ns[14], -800.0, 800.0)            # nose rate
        # wrap nose angle
        ns[13] = (ns[13] + np.pi) % (2 * np.pi) - np.pi
        return ns, F_can

    def run(self, verbose: bool = True) -> Dict[str, List[float]]:
        p = self.p
        elev = np.radians(p.elevation_deg)
        az   = np.arctan2(p.target_y, p.target_x) + np.radians(p.azimuth_bias_deg)
        vm   = p.muzzle_velocity

        s = np.zeros(15)
        s[0:3]  = [0.0, 0.0, p.launch_height]
        s[3:6]  = [vm * np.cos(elev) * np.cos(az),
                   vm * np.cos(elev) * np.sin(az),
                   vm * np.sin(elev)]
        s[6:10] = euler_to_q(0.0, elev, az)
        s[10]   = 2 * np.pi * p.initial_spin_rps          # body spin
        s[13]   = 0.0                                     # nose angle
        s[14]   = 2 * np.pi * p.initial_spin_rps * 0.15   # nose starts partially spinning

        log = {k: [] for k in [
            "t", "x", "y", "z", "speed", "pitch", "yaw",
            "p_rps", "nose_rps", "theta_c_deg", "theta_nose_deg",
            "free_spin", "aoa", "zem", "inhibited"
        ]}

        t = 0.0
        sub = max(int(round(p.control_dt / p.dt)), 1)
        n = int(np.ceil(p.tmax / p.dt))
        F_can = np.zeros(3)

        for i in range(n + 1):
            if i % sub == 0:
                self.guid.update(t, s[0:3], s[3:6], sub * p.dt, F_can)

            if i % 8 == 0:          # log every ~24 ms
                self._log(log, t, s)

            if s[2] < 0.0 and t > 1.0:
                break

            s, F_can = self.step(t, s, p.dt, self.guid.free_spin, self.guid.theta_c)
            t += p.dt

        if verbose:
            impact = np.array([log["x"][-1], log["y"][-1]])
            target = np.array([p.target_x, p.target_y])
            miss = np.linalg.norm(impact - target)
            print("=== M1156-style Dual-Spin Fixed-Canard PGK (upgraded) ===")
            print(f"Target     : ({p.target_x:.0f}, {p.target_y:.0f}) m")
            print(f"Impact     : ({impact[0]:.1f}, {impact[1]:.1f}) m")
            print(f"Miss       : {miss:.1f} m")
            print(f"Apogee     : {max(log['z']):.0f} m")
            print(f"ToF        : {log['t'][-1]:.1f} s")
            print(f"Max |AoA|  : {max(np.abs(log['aoa'])):.1f}°")
            print(f"Final body spin : {log['p_rps'][-1]:.1f} rps")
            print(f"Final nose spin : {log['nose_rps'][-1]:.2f} rps")
            print(f"Inhibited  : {bool(log['inhibited'][-1])}")

        return log

    def _log(self, log, t, s):
        r, v = s[0:3], s[3:6]
        q = q_normalize(s[6:10])
        omega = s[10:13]
        eul = q_to_euler(q)
        speed = np.linalg.norm(v)

        R = q_to_dcm(q)
        bx = R[:, 0]
        aoa = 0.0
        if speed > 1.0:
            ca = np.clip(np.dot(bx, v / speed), -1.0, 1.0)
            aoa = np.degrees(np.arccos(ca))

        impact, _ = self.guid.predict_impact(r, v, F_can_est=self.guid.last_F_can)
        zem = np.linalg.norm(self.guid.target[:2] - impact[:2])

        log["t"].append(t)
        log["x"].append(r[0])
        log["y"].append(r[1])
        log["z"].append(r[2])
        log["speed"].append(speed)
        log["pitch"].append(np.degrees(eul[1]))
        log["yaw"].append(np.degrees(eul[2]))
        log["p_rps"].append(omega[0] / (2 * np.pi))
        log["nose_rps"].append(s[14] / (2 * np.pi))
        log["theta_c_deg"].append(np.degrees(self.guid.theta_c))
        log["theta_nose_deg"].append(np.degrees(s[13]))
        log["free_spin"].append(1.0 if self.guid.free_spin else 0.0)
        log["aoa"].append(aoa)
        log["zem"].append(zem)
        log["inhibited"].append(1.0 if self.guid.inhibited else 0.0)


# =====================================================================
#  Main + plots
# =====================================================================

def plot_results(log: dict, log_ol: dict, p: Params, impact: np.ndarray):
    fig = plt.figure(figsize=(15, 10))

    # 3-D trajectory
    ax = fig.add_subplot(2, 3, (1, 4), projection="3d")
    ax.plot(log["x"], log["y"], log["z"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], log_ol["z"], lw=1.0, ls="--", alpha=0.7, label="Open-loop")
    ax.scatter(0, 0, p.launch_height, c="g", s=40, label="Gun")
    ax.scatter(p.target_x, p.target_y, 0, c="m", marker="*", s=90, label="Target")
    ax.scatter(impact[0], impact[1], 0, c="r", s=40, label="Impact")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3-D Trajectory (Dual-Spin PGK)")
    ax.legend(fontsize=8)

    # Altitude
    ax = fig.add_subplot(2, 3, 2)
    ax.plot(log["t"], log["z"], label="Guided")
    ax.plot(log_ol["t"], log_ol["z"], ls="--", alpha=0.7, label="Open-loop")
    ax.set_title("Altitude")
    ax.set_xlabel("t (s)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # Speed
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(log["t"], log["speed"])
    ax.set_title("Speed")
    ax.set_xlabel("t (s)")
    ax.grid(alpha=0.3)

    # Guidance activity
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(log["t"], log["theta_c_deg"], label="θ_c (cmd)")
    ax.plot(log["t"], log["theta_nose_deg"], label="θ_nose", alpha=0.8)
    ax.plot(log["t"], np.array(log["free_spin"]) * 40, label="free-spin (scaled)")
    ax.set_ylabel("deg")
    ax2 = ax.twinx()
    ax2.plot(log["t"], log["zem"], color="C3", alpha=0.7, label="ZEM")
    ax2.set_ylabel("ZEM (m)")
    ax.set_xlabel("t (s)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=7)
    ax2.legend(loc="upper right", fontsize=7)

    # Attitude + dual-spin rates
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(log["t"], log["pitch"], label="pitch")
    ax.plot(log["t"], log["yaw"], label="yaw")
    ax.plot(log["t"], log["p_rps"], label="body spin (rps)")
    ax.plot(log["t"], log["nose_rps"], label="nose spin (rps)")
    ax.set_title("Attitude & Spin")
    ax.set_xlabel("t (s)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("m1156_pgk_upgraded.png", dpi=140)
    plt.close()

    # Ground track
    fig2, ax = plt.subplots(figsize=(8, 5))
    ax.plot(log["x"], log["y"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], lw=1.0, ls="--", alpha=0.7, label="Open-loop")
    ax.scatter(0, 0, c="g", marker="s", s=50, label="Gun")
    ax.scatter(p.target_x, p.target_y, c="m", marker="*", s=90, label="Target")
    ax.scatter(impact[0], impact[1], c="r", marker="x", s=60, label="Impact")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Ground Track – Dual-Spin PGK")
    ax.axis("equal")
    ax.legend()
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("m1156_pgk_upgraded_track.png", dpi=140)
    plt.close()

    print("\nSaved: m1156_pgk_upgraded.png , m1156_pgk_upgraded_track.png")


def main():
    p = Params()
    sim = Simulator(p)
    log = sim.run(verbose=True)

    # Open-loop reference (guidance never arms)
    p_ol = Params()
    p_ol.arm_time = 9999.0
    log_ol = Simulator(p_ol).run(verbose=False)
    miss_ol = np.hypot(log_ol["x"][-1] - p_ol.target_x,
                       log_ol["y"][-1] - p_ol.target_y)
    print(f"Open-loop miss (no guidance): {miss_ol:.0f} m")

    impact = np.array([log["x"][-1], log["y"][-1]])
    plot_results(log, log_ol, p, impact)


if __name__ == "__main__":
    main()

</file>

<file src="pgk_aero.py">
"""Mach/alpha aero tables + atmosphere + MET wind layers + gravity/Coriolis."""

import numpy as np
from pgk_params import Params


def _interp_table(M, M_nodes, vals):
    return float(np.interp(M, M_nodes, vals))


_M_NODES = np.array([0.5, 0.8, 0.95, 1.05, 1.2, 1.5, 2.0, 2.5])
_CD0 = np.array([0.17, 0.19, 0.32, 0.40, 0.34, 0.27, 0.24, 0.23])
_CL_A = np.array([1.70, 1.68, 1.55, 1.45, 1.50, 1.55, 1.50, 1.45])
_CM_A = np.array([-2.4, -2.35, -2.1, -1.95, -2.15, -2.3, -2.25, -2.2])
_CLP = np.array([-0.04, -0.042, -0.05, -0.055, -0.05, -0.048, -0.045, -0.043])
_CMQ = np.array([-3.8, -4.0, -4.5, -4.6, -4.3, -4.1, -4.0, -3.9])
_CN_PA = np.array([0.04, 0.05, 0.07, 0.08, 0.06, 0.05, 0.04, 0.035])
_CM_PA = np.array([-0.003, -0.0035, -0.005, -0.0055, -0.0045, -0.004, -0.0035, -0.003])


def mach_from_speed(speed: float, h: float, p: Params) -> float:
    T = max(p.T0 - p.lapse * max(h, 0.0), 200.0)
    a = np.sqrt(1.4 * 287.05 * T)
    return speed / a


def body_aero_coeffs(M: float, p: Params):
    Cd0 = _interp_table(M, _M_NODES, _CD0) * (p.Cd0_ref / 0.215)
    Cl_alpha = _interp_table(M, _M_NODES, _CL_A) * (p.Cl_alpha_ref / 1.65)
    Cm_alpha = _interp_table(M, _M_NODES, _CM_A) * (p.Cm_alpha_ref / -2.35)
    Clp = _interp_table(M, _M_NODES, _CLP) * (p.Clp_ref / -0.048)
    Cmq = _interp_table(M, _M_NODES, _CMQ) * (p.Cmq_ref / -4.2)
    Cn_pa = _interp_table(M, _M_NODES, _CN_PA) * (p.Cn_pa_ref / 0.055)
    Cm_pa = _interp_table(M, _M_NODES, _CM_PA) * (p.Cm_pa_ref / -0.004)
    return Cd0, Cl_alpha, Cm_alpha, Clp, Cmq, Cn_pa, Cm_pa


def density(h: float, p: Params) -> float:
    h = max(h, 0.0)
    T = max(p.T0 - p.lapse * h, 210.0)
    expo = 9.80665 / (287.05 * p.lapse) - 1.0
    rho = p.rho0 * (T / p.T0) ** expo
    return max(rho, 0.04)


def gravity_vector(r: np.ndarray, p: Params) -> np.ndarray:
    h = r[2]
    g_mag = p.g0 * (p.earth_radius / (p.earth_radius + max(h, 0.0))) ** 2
    return np.array([0.0, 0.0, -g_mag])


def coriolis_acceleration(v: np.ndarray, p: Params) -> np.ndarray:
    lat = np.radians(p.latitude_deg)
    Om = p.omega_earth * np.array([0.0, np.cos(lat), np.sin(lat)])
    return -2.0 * np.cross(Om, v)


_MET_H = np.array([0.0, 500.0, 1000.0, 2000.0, 3000.0, 5000.0, 8000.0])
_MET_WX = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
_MET_WY = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])


def wind_at_altitude(h: float, p: Params) -> np.ndarray:
    if not p.use_met_layers:
        return p.wind.copy()
    h = max(h, 0.0)
    wx = float(np.interp(h, _MET_H, _MET_WX))
    wy = float(np.interp(h, _MET_H, _MET_WY))
    return np.array([wx, wy, 0.0]) + p.wind

</file>

<file src="pgk_canard_aero.py">
"""
Reduced-order canard lateral aero (Ji-inspired tables).

Not CFD-accurate; shapes lateral force with alpha, Mach, and nose rate so
HOLD (p_f~0) is the only meaningful steer mode and FREE-SPIN averages out.
"""

from __future__ import annotations

import numpy as np
from pgk_params import Params


def cy_canard_table(alpha_rad: float, mach: float, omega_nose: float, free_spin: bool,
                    p: Params) -> float:
    """
    Effective lateral force coefficient referenced to ref_area:
        F_lat ≈ cy * qdyn * S_ref * direction

    Built as: base_hold_CN * f_alpha * f_mach * f_spin
    """
    # Base CN (canard area / ref) * Cl * authority — same order as before
    CN0 = (p.canard_Cl_max * p.canard_authority * p.canard_area / p.ref_area)

    # Alpha shape (Ji Fig.5-like growth, soft stall)
    a = float(np.clip(abs(alpha_rad), 0.0, p.canard_stall_alpha))
    a_deg = np.degrees(a)
    # Hold geometry still produces force at small body AoA (canard incidence)
    f_alpha = 0.88 + 0.012 * a_deg - 0.00015 * a_deg ** 2
    f_alpha = float(np.clip(f_alpha, 0.85, 1.25))
    if a >= 0.95 * p.canard_stall_alpha:
        f_alpha *= 0.7

    # Mach: slightly less efficient high subsonic / transonic (crude)
    if mach < 0.7:
        f_mach = 1.0
    elif mach < 1.1:
        f_mach = 1.0 - 0.15 * (mach - 0.7) / 0.4
    else:
        f_mach = 0.82
    f_mach = float(np.clip(f_mach, 0.75, 1.05))

    # Nose-rate attenuation: only fully effective when de-spun
    if free_spin:
        f_spin = 0.02  # residual; mean force nearly cancels
    else:
        # rad/s: full at |w|<5, dead by |w|>40
        w = abs(float(omega_nose))
        f_spin = float(np.clip(1.0 - (w / 40.0) ** 2, 0.0, 1.0))

    return CN0 * f_alpha * f_mach * f_spin


def canard_force_inertial(
    phi_nose: float,
    v_air: np.ndarray,
    speed: float,
    qdyn: float,
    alpha_rad: float,
    mach: float,
    omega_nose: float,
    free_spin: bool,
    p: Params,
) -> tuple[np.ndarray, float]:
    """
    Returns (F_can_inertial [N], cy_used).
    Direction from physical nose attitude; magnitude from table + a_max clamp.
    """
    if free_spin or speed < p.canard_min_speed:
        return np.zeros(3), 0.0
    if abs(omega_nose) >= 40.0:
        return np.zeros(3), 0.0

    cy = cy_canard_table(alpha_rad, mach, omega_nose, free_spin, p)
    Fmag = cy * qdyn * p.ref_area
    # Safety clamp — not the primary design knob
    Fmag = min(Fmag, p.canard_a_max * p.mass)

    c_phi, s_phi = np.cos(phi_nose), np.sin(phi_nose)
    desired = np.array([c_phi, s_phi, 0.0])
    vhat = v_air / (speed + 1e-9)
    force_dir = desired - np.dot(desired, vhat) * vhat
    n = np.linalg.norm(force_dir)
    if n < 1e-6:
        return np.zeros(3), cy
    force_dir /= n
    return Fmag * force_dir, cy
</file>

<file src="pgk_control.py">
"""
Flight control for dual-spin fixed-canard PGK (no motorized canard servo).

Real PGK-class control:
  - FREE-SPIN: nose/canards rotate → average lift ~ 0
  - HOLD: brake orients de-spun nose to commanded roll theta_c
  - Control "actuator" is brake/hold torque on nose inertia, not a fin servo

Outer loop: ZEM → theta_c (guidance)
Inner loop: phi_meas → brake torque command (this module)
"""

from __future__ import annotations

import numpy as np
from pgk_params import Params


class DualSpinFlightControl:
    """
    Inner-loop hold of nose roll using measured phi (sensor), not truth.

    Outputs:
      theta_c     — commanded inertial roll of canard plane
      free_spin   — True if free-spin mode
      tau_cmd_extra — optional additive torque (0 default; dynamics use full law in forces)
    """

    def __init__(self, p: Params):
        self.p = p
        self.theta_c = 0.0
        self.free_spin = True
        self.phase = np.radians(p.guidance_phase_deg)

    def set_phase_from_fixed_canard_id(self, phase_deg: float):
        """Freeze design phase after fixed-canard identification / MC."""
        self.phase = np.radians(phase_deg)
        self.p.guidance_phase_deg = float(phase_deg)

    def command_from_zem(self, zem_vec_xy: np.ndarray, dt: float):
        """Outer guidance → slew theta_c."""
        if self.free_spin:
            return self.theta_c
        desired = np.arctan2(zem_vec_xy[1], zem_vec_xy[0]) + self.phase
        err = (desired - self.theta_c + np.pi) % (2 * np.pi) - np.pi
        max_rate = np.radians(self.p.max_slew_rate_deg)
        rate = np.clip(self.p.K_orient * err, -max_rate, max_rate)
        alpha = dt / (self.p.orientation_slew_tau + dt)
        self.theta_c = (self.theta_c + alpha * rate * self.p.orientation_slew_tau) % (2 * np.pi)
        return self.theta_c

    def hold_error(self, phi_meas: float) -> float:
        """Roll tracking error for logging / brake law (wrap to [-pi, pi])."""
        return (self.theta_c - phi_meas + np.pi) % (2 * np.pi) - np.pi

    def set_mode(self, free_spin: bool):
        self.free_spin = bool(free_spin)


def guidance_phase_design_rule() -> str:
    return (
        "Guidance phase is frozen from fixed-canard identification: "
        "hold canards at 0/90/180/270°, measure impact shifts, choose phase "
        "that maximizes correlation of force direction with desired correction, "
        "then confirm with guided single-shot sweep. Current baseline: 0 deg."
    )

</file>

<file src="pgk_control_budget.py">
"""Control effort / authority budget from a trajectory log."""

from __future__ import annotations

import numpy as np


def control_budget(log, arm_time: float = 7.0, mass: float = 43.0) -> dict:
    t = np.asarray(log["t"], float)
    if len(t) < 3:
        return {}
    free = np.asarray(log["free_spin"], float)
    aoa = np.asarray(log["aoa"], float)
    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1] if len(dt) > 1 else 0.05
    post = t >= arm_time
    hold = post & (free < 0.5)

    if "can_a" in log and len(log["can_a"]) == len(t):
        a_can = np.asarray(log["can_a"], float)
        dV = float(np.sum(a_can[hold] * dt[hold])) if hold.any() else 0.0
        mean_a = float(a_can[hold].mean()) if hold.any() else 0.0
        max_a = float(a_can[hold].max()) if hold.any() else 0.0
        src = "canard |F|/m"
    else:
        # fallback path lateral (overcounts geometry) — marked
        x = np.asarray(log["x"], float)
        y = np.asarray(log["y"], float)
        ax = np.gradient(np.gradient(x, t), t)
        ay = np.gradient(np.gradient(y, t), t)
        a_lat = np.sqrt(ax**2 + ay**2)
        dV = float(np.sum(a_lat[hold] * dt[hold])) if hold.any() else 0.0
        mean_a = float(a_lat[hold].mean()) if hold.any() else 0.0
        max_a = float(a_lat[hold].max()) if hold.any() else 0.0
        src = "path proxy (overcount)"

    free_frac = float(free[post].mean()) if post.any() else 1.0
    return {
        "dV_lat_ms": dV,
        "hold_frac_after_arm": 1.0 - free_frac,
        "free_frac_after_arm": free_frac,
        "mean_a_lat_hold": mean_a,
        "max_a_lat_hold": max_a,
        "max_aoa_deg": float(np.max(np.abs(aoa))),
        "t_hold_s": float(np.sum(dt[hold])) if hold.any() else 0.0,
        "source": src,
    }


def print_control_budget(log, arm_time: float = 7.0, mass: float = 43.0):
    b = control_budget(log, arm_time, mass)
    print("=== Control budget (realism) ===")
    print(f"  Source                  : {b.get('source')}")
    print(f"  Lateral ΔV (HOLD)       : {b.get('dV_lat_ms', 0):.1f} m/s")
    print(f"  HOLD time / fraction    : {b.get('t_hold_s', 0):.1f} s  ({100*b.get('hold_frac_after_arm',0):.0f}%)")
    print(f"  Mean |a_can| while HOLD : {b.get('mean_a_lat_hold', 0):.3f} m/s²")
    print(f"  Max  |a_can| while HOLD : {b.get('max_a_lat_hold', 0):.3f} m/s²")
    print(f"  Max |AoA|               : {b.get('max_aoa_deg', 0):.1f} deg")
    return b
</file>

<file src="pgk_forces.py">
"""Forces, moments, dual-spin brake/alternator, physical canard authority."""

import numpy as np
from pgk_params import Params
from pgk_aero import body_aero_coeffs, density, mach_from_speed, wind_at_altitude
from pgk_math import q_to_dcm
from pgk_canard_aero import canard_force_inertial


def forces_moments(r, v, q, omega_body, phi_nose, omega_nose,
                   free_spin, theta_c, p: Params):
    R = q_to_dcm(q)
    body_x = R[:, 0]
    wind = wind_at_altitude(r[2], p)
    v_air = v - wind
    speed = np.linalg.norm(v_air)
    h = r[2]
    rho = density(h, p)
    qdyn = 0.5 * rho * speed * speed
    S, d = p.ref_area, p.diameter
    M_mach = mach_from_speed(speed, h, p)
    Cd0, Cl_alpha, Cm_alpha, Clp, Cmq, Cn_pa, Cm_pa = body_aero_coeffs(M_mach, p)

    if speed > 1.0:
        F_drag = -Cd0 * qdyn * S * (v_air / speed)
    else:
        F_drag = np.zeros(3)

    F_lift = np.zeros(3)
    M_over_body = np.zeros(3)
    M_magnus_body = np.zeros(3)
    F_magnus = np.zeros(3)
    aoa = 0.0

    if speed > 35.0:
        vhat = v_air / speed
        ca = np.clip(np.dot(body_x, vhat), -1.0, 1.0)
        aoa = np.arccos(ca)
        if aoa > 1e-4:
            ldir = body_x - ca * vhat
            n = np.linalg.norm(ldir)
            if n > 1e-9:
                ldir /= n
                Cl = np.clip(Cl_alpha * aoa, -p.max_Cl, p.max_Cl)
                F_lift = Cl * qdyn * S * ldir
                AR, e = 5.2, 0.75
                Cd_i = (Cl * Cl) / (np.pi * e * AR)
                F_drag = F_drag - Cd_i * qdyn * S * (v_air / speed)
                axis2_raw = np.cross(body_x, vhat)
                axis2_norm = np.linalg.norm(axis2_raw)
                if axis2_norm > 1e-9:
                    axis2 = axis2_raw / axis2_norm
                    M_over_mag = -Cm_alpha * qdyn * S * d * aoa
                    M_over_body = R.T @ (M_over_mag * axis2)
                    pbar = omega_body[0] * d / (2.0 * speed)
                    F_magnus_mag = Cn_pa * qdyn * S * pbar * aoa
                    F_magnus = F_magnus_mag * axis2
                    M_magnus_mag = Cm_pa * qdyn * S * d * pbar * aoa
                    M_magnus_body = R.T @ (M_magnus_mag * axis2)

    # Canards: force from physical nose attitude; CY table (alpha, Mach, nose rate)
    F_can = np.zeros(3)
    M_can_body = np.zeros(3)
    cy_used = 0.0
    F_can, cy_used = canard_force_inertial(
        phi_nose, v_air, speed, qdyn, aoa, M_mach, omega_nose, free_spin, p)
    if np.linalg.norm(F_can) > 1e-9:
        Fmag = float(np.linalg.norm(F_can))
        F_lift = F_lift * (1.0 - p.canard_interference * min(Fmag / (qdyn * S + 1e-6), 1.0))
        r_c_body = np.array([p.cg_to_nose, 0.0, 0.0])
        F_can_body = R.T @ F_can
        M_can_body = np.cross(r_c_body, F_can_body)
        Cd_can_i = 0.05 * (Fmag / (qdyn * S + 1e-6)) ** 2
        F_drag = F_drag - Cd_can_i * qdyn * S * (v_air / speed)

    F = F_drag + F_lift + F_can + F_magnus

    p_rate = float(np.clip(omega_body[0], -3000, 3000))
    q_rate = float(np.clip(omega_body[1], -50, 50))
    r_rate = float(np.clip(omega_body[2], -50, 50))
    if speed > 5.0 and np.isfinite(qdyn):
        L_damp = Clp * qdyn * S * d * (d / (2.0 * speed)) * p_rate
        damp_coef = Cmq * qdyn * S * d * (d / (2.0 * speed))
        M_damp = np.array([L_damp, damp_coef * q_rate, damp_coef * r_rate])
    else:
        M_damp = np.zeros(3)

    # Dual-spin: aero de-spin + alternator load + brake
    omega_rel = omega_nose - omega_body[0]
    tau_aero = 0.0
    if speed > 40.0:
        tau_aero = -p.aero_despin_coeff * qdyn * np.sign(omega_nose) * min(abs(omega_nose) / 80.0, 1.0)
        if abs(omega_nose) < p.free_spin_rel_rate:
            tau_aero *= 0.2
    tau_alt = -p.alternator_torque_coeff * omega_rel  # load when relative spin

    if free_spin:
        tau_cmd = tau_aero + tau_alt
    else:
        # Hold mode: suppress aero de-spin so brake can point the nose
        # err_phi = theta_c - phi_nose: positive => need +omega_nose to catch up
        err_phi = (theta_c - phi_nose + np.pi) % (2 * np.pi) - np.pi
        tau_cmd = (0.15 * tau_aero + tau_alt
                   - p.K_brake * omega_nose
                   + p.K_phi_hold * err_phi)

    tau_brake = float(np.clip(tau_cmd, -p.brake_torque_max, p.brake_torque_max))
    tau_brake -= p.bearing_visc * omega_rel

    M_body = M_can_body + M_damp + M_over_body + M_magnus_body
    M_body[0] -= tau_brake

    return F, M_body, tau_brake, F_can, np.degrees(aoa)
</file>

<file src="pgk_fuze.py">
"""
Multi-mode electronic fuze (functional model for GNC integration).

Modes (problem statement + PGK-class):
  - Proximity (height of burst)
  - Time (clock from setback — extra vs baseline M1156 PD+Prox)
  - Impact (point detonating)

PGK-style detonation inhibit (public M1156 concept):
  Early after launch, if predicted miss to intended target is very large
  (order ~150 m), the kit withholds detonation so a failed guide does not
  function on an unintended area. Guidance may free-spin; fuze stays safe.

Safe–arm: setback + safe separation delay.
Programming: pre-fire mode + time/prox parameters (EPIAFS-style API stub).
"""

from __future__ import annotations

from enum import IntEnum
from pgk_params import Params


class FuzeMode(IntEnum):
    IMPACT = 0
    TIME = 1
    PROXIMITY = 2


class MultiModeFuze:
    def __init__(self, p: Params):
        self.p = p
        self.mode = FuzeMode.IMPACT
        self.time_s = 60.0
        self.prox_height_m = 10.0
        self.armed = False
        self.fired = False
        self.t_setback = None
        self.safe_delay_s = 0.8

        # PGK-style detonation inhibit
        self.detonate_inhibit = False
        self.inhibit_checked = False
        # Public-order threshold (~150 m); overridable
        self.inhibit_miss_m = float(getattr(p, "fuze_inhibit_miss_m", 150.0))
        self.inhibit_check_time_s = float(getattr(p, "fuze_inhibit_check_time_s", 5.0))
        self.last_pred_miss_m = None

    def program(self, mode: FuzeMode, time_s: float = 60.0, prox_height_m: float = 10.0):
        """Pre-fire programming interface."""
        if self.t_setback is not None:
            return False
        self.mode = mode
        self.time_s = float(time_s)
        self.prox_height_m = float(prox_height_m)
        return True

    def on_setback(self, t: float):
        if self.t_setback is None:
            self.t_setback = t

    def evaluate_inhibit(self, t: float, predicted_miss_m: float | None):
        """
        Early guidance-quality check (PGK-like).
        If predicted miss >> inhibit_miss_m, freeze detonate_inhibit=True for the flight.
        """
        if self.inhibit_checked or self.fired:
            return
        t0 = self.t_setback if self.t_setback is not None else 0.0
        if (t - t0) < self.inhibit_check_time_s:
            return
        self.inhibit_checked = True
        if predicted_miss_m is None:
            return
        self.last_pred_miss_m = float(predicted_miss_m)
        if predicted_miss_m > self.inhibit_miss_m:
            self.detonate_inhibit = True
            # Stay safe: do not allow armed detonation path
            self.armed = False

    def step(
        self,
        t: float,
        h: float,
        impact_contact: bool,
        predicted_miss_m: float | None = None,
    ) -> dict:
        """
        Update fuze. Optional predicted_miss_m enables PGK inhibit logic.
        """
        if self.t_setback is None and t > 0.05:
            self.on_setback(t)

        if self.fired:
            return self._status(t, h)

        # Early inhibit decision (once)
        self.evaluate_inhibit(t, predicted_miss_m)

        t0 = self.t_setback if self.t_setback is not None else t
        if (t - t0) >= self.safe_delay_s and not self.detonate_inhibit:
            self.armed = True

        if self.detonate_inhibit:
            # Explicitly never fire
            self.armed = False
            return self._status(t, h)

        if not self.armed:
            return self._status(t, h)

        if self.mode == FuzeMode.IMPACT:
            if impact_contact or h <= 0.0:
                self.fired = True
        elif self.mode == FuzeMode.TIME:
            if (t - t0) >= self.time_s:
                self.fired = True
        elif self.mode == FuzeMode.PROXIMITY:
            if h <= self.prox_height_m and (t - t0) > self.safe_delay_s + 1.0:
                self.fired = True
            if impact_contact or h <= 0.0:
                self.fired = True

        return self._status(t, h)

    def _status(self, t, h):
        return {
            "mode": int(self.mode),
            "mode_name": self.mode.name,
            "armed": self.armed,
            "fired": self.fired,
            "detonate_inhibit": self.detonate_inhibit,
            "inhibit_checked": self.inhibit_checked,
            "last_pred_miss_m": self.last_pred_miss_m,
            "t": t,
            "h": h,
        }

</file>

<file src="pgk_gnc_doc.py">
"""
GNC architecture, navigation methodology, flight control, and gap closures.

Print with:  python -c "from pgk_gnc_doc import print_full_gnc_doc; print_full_gnc_doc()"
"""

NAVIGATION_METHODOLOGY = """
NAVIGATION METHODOLOGY (baseline PGK-class)
==========================================
1. Primary: GNSS (GPS) position and velocity
   - Rate: 1–10 Hz (sim default 10 Hz)
   - Latency: ~50 ms modeled
   - Dropouts: free-spin if nav stale > 2 s (mode manager)

2. Filter: 6-state constant-velocity Kalman
   State: [rx, ry, rz, vx, vy, vz]
   Predict: CV coast between fixes
   Update: GPS position (and velocity when available)
   Guidance uses filter outputs, never truth, in GPS mode

3. Roll / nose attitude (REQUIRED for hold)
   - Sensor on de-spun nose: magnetometer and/or relative encoder
   - Sim: NoseRollSensor noise model on phi_nose
   - Hold law tracks measured phi → theta_c (not truth state in FSW path)

4. Optional: barometric altitude aiding
5. IMU: NOT required for baseline ZEM dual-spin PGK
   - Problem statement lists IMU as example; classic PGK is GPS-dominant
   - Decision: baseline = GPS + nose roll; IMU reserved for future upgrade
"""

FLIGHT_CONTROL_ALGORITHMS = """
FLIGHT CONTROL (dual-spin fixed canard — NO fin servo)
======================================================
Outer loop (guidance, ~ control_dt 20–50 ms):
  - Predict impact with cheap Cd predictor + optional control force estimate
  - ZEM = target_xy - impact_xy
  - If ZEM < deadband → FREE_SPIN
  - Else theta_c = atan2(ZEM_y, ZEM_x) + phase
  - First-order slew on theta_c with rate limit

Inner loop (dual-spin hold, physics at dt):
  - FREE_SPIN: aero de-spin + alternator; canards rotate → mean force ~ 0
  - HOLD: brake torque drives omega_nose → 0 and phi → theta_c
  - Canard force direction from physical/measured nose attitude
  - No motorized canard deflection servo (matches real PGK concept)

Phase design rule:
  - Fixed-canard ID at 0/90/180/270° → impact shifts
  - Guided phase sweep → freeze phase (baseline 0°)

Fail-safe:
  - Early gross ZEM → inhibit guidance, free-spin, ballistic fuze still valid
  - GPS watchdog → free-spin
"""

FUZE_INTEGRATION = """
MULTI-MODE ELECTRONIC FUZE (functional)
=======================================
Modes: IMPACT | TIME | PROXIMITY
Safe–arm: setback + safe separation delay
Programming: pre-fire mode + time/prox parameters
Independence: GNC fail-safe free-spin does not inhibit fuze detonation logic
"""

SENSOR_SELECTION = """
SENSORS (real selection for electronics)
========================================
REQUIRED
  - GNSS receiver + antenna
  - Nose roll sensor (mag / encoder on de-spun section)
  - Setback / launch detect
  - Fuze: impact switch; timer; proximity channel if mode offered

OPTIONAL
  - Barometer
  - Body spin tach / alternator sense
  - Low-cost IMU (upgrade path only)

NOT USED IN BASELINE CONTROL
  - Continuous INS as primary nav
  - Motorized canard position servo
"""

FSW_ARCHITECTURE = """
FLIGHT SOFTWARE ARCHITECTURE
============================
Rates:
  GPS task           1–10 Hz
  Nav filter         on GPS + every control tick predict
  Guidance / modes   20–50 Hz (control_dt)
  Roll hold          continuous via physics / 50–100 Hz sense
  Fuze               event + 100 Hz poll

Mode FSM:
  SAFE → LAUNCH → COAST → ARM_CHECK → GUIDE | FREE_SPIN | FAILSAFE → TERMINAL

Watchdog:
  GPS stale → FREE_SPIN
  Fail-safe flag → FREE_SPIN + ballistic

HIL path (PoC):
  Playback GPS R/V stream → nav → ZEM → theta_c / free_spin log
  Compare to closed-loop sim truth
"""

CEP_METHOD = """
FORMAL CEP STUDY METHOD
=======================
Dispersions: MV, elevation/azimuth, wind, GPS noise, aero scale, canard authority
Metric: CEP50 = median radial miss; also CEP90, RMS
Claim: threshold < 50 m; objective < 30 m under stated conditions only
N ≥ 30 recommended (50–100 preferred)
Run: python -c "from pgk_montecarlo import run_monte_carlo, print_cep_report; ..."
"""


def print_full_gnc_doc():
    for block in (
        NAVIGATION_METHODOLOGY,
        FLIGHT_CONTROL_ALGORITHMS,
        FUZE_INTEGRATION,
        SENSOR_SELECTION,
        FSW_ARCHITECTURE,
        CEP_METHOD,
    ):
        print(block)
        print()

</file>

<file src="pgk_guidance.py">
"""
ZEM guidance + dual-spin mode logic (PGK-class).

Navigation: uses r_nav, v_nav only (GPS/Kalman estimates — never truth in GPS mode).
Flight control: FREE_SPIN vs HOLD; no motorized canard servo.
"""

import numpy as np
from pgk_params import Params
from pgk_aero import density, gravity_vector
from pgk_modes import ModeManager, FlightMode
from pgk_control import DualSpinFlightControl


class Guidance:
    def __init__(self, p: Params):
        self.p = p
        self.theta_c = 0.0
        self.free_spin = True
        self.fail_safe = False
        self.fail_safe_checked = False
        self.target = np.array([p.target_x, p.target_y, 0.0])
        self.last_F_can_est = np.zeros(3)
        self.zem = 0.0
        self.mode = FlightMode.SAFE
        self.modes = ModeManager(p)
        self.ctrl = DualSpinFlightControl(p)
        # Nose roll measurement (set by simulator each control tick)
        self.phi_meas = 0.0
        self.gps_ok = True
        self.last_hold_err = 0.0

    def predict_impact(self, r, v, include_control=False):
        p = self.p
        dt_pred = p.predictor_dt
        rr, vv = r.copy(), v.copy()
        t = 0.0
        F_ctrl = self.last_F_can_est if include_control else np.zeros(3)
        while rr[2] > 0.0 and t < 70.0:
            speed = np.linalg.norm(vv)
            rho_h = density(rr[2], p)
            qdyn = 0.5 * rho_h * speed * speed
            if speed > 1.0:
                # Onboard predictor: cheap constant Cd (not full Mach truth aero)
                a_drag = -0.27 * qdyn * p.ref_area / p.mass * (vv / speed)
            else:
                a_drag = np.zeros(3)
            a = a_drag + gravity_vector(rr, p) + F_ctrl / p.mass
            vv = vv + a * dt_pred
            rr = rr + vv * dt_pred
            t += dt_pred
        impact = rr.copy()
        impact[2] = 0.0
        return impact, max(t, 0.3)

    def update(self, t, r_nav, v_nav, dt, F_can_current=None,
               phi_meas=None, gps_ok=True, impact_imminent=False, fuze_terminal=False):
        """
        r_nav, v_nav = navigation estimates (not truth).
        phi_meas = nose roll from roll sensor (if None, hold uses dynamics truth externally).
        """
        if F_can_current is not None:
            self.last_F_can_est = 0.65 * self.last_F_can_est + 0.35 * F_can_current
        if phi_meas is not None:
            self.phi_meas = float(phi_meas)
        self.gps_ok = bool(gps_ok)

        # Pre-arm: skip expensive impact predictor (was hanging generate_figures)
        if t < self.p.arm_time and not self.fail_safe:
            self.zem = 0.0
            self.free_spin = True
            self.mode = self.modes.update(
                t, 1e6, self.gps_ok,
                impact_imminent=impact_imminent,
                fuze_terminal=fuze_terminal,
            )[0]
            self.ctrl.set_mode(True)
            self.theta_c = self.ctrl.theta_c
            return

        # Ballistic ZEM for mode logic (only after arm)
        impact, _ = self.predict_impact(r_nav, v_nav, include_control=True)
        zem_vec = self.target[:2] - impact[:2]
        self.zem = float(np.linalg.norm(zem_vec))

        # Legacy early fail-safe flag (also handled in ModeManager)
        if (not self.fail_safe_checked) and t >= self.p.fail_safe_check_time:
            self.fail_safe_checked = True
            impact0, _ = self.predict_impact(r_nav, v_nav, include_control=False)
            zem0 = float(np.linalg.norm(self.target[:2] - impact0[:2]))
            if zem0 > self.p.fail_safe_miss * 8.0:
                self.fail_safe = True
                self.modes.fail_safe = True

        self.modes.fail_safe = self.modes.fail_safe or self.fail_safe
        mode, free = self.modes.update(
            t, self.zem, self.gps_ok,
            impact_imminent=impact_imminent,
            fuze_terminal=fuze_terminal,
        )
        self.mode = mode
        self.free_spin = free
        self.fail_safe = self.modes.fail_safe
        self.ctrl.set_mode(self.free_spin)

        if self.free_spin or t < self.p.arm_time:
            self.theta_c = self.ctrl.theta_c
            self.last_hold_err = self.ctrl.hold_error(self.phi_meas)
            return

        # Outer loop: ZEM → theta_c
        self.ctrl.command_from_zem(zem_vec, dt)
        self.theta_c = self.ctrl.theta_c
        self.last_hold_err = self.ctrl.hold_error(self.phi_meas)

</file>

<file src="pgk_hold_metrics.py">
"""Hold-quality metrics from a guided log."""

from __future__ import annotations

import numpy as np


def hold_quality_report(log, arm_time: float = 7.0) -> dict:
    t = np.asarray(log["t"], float)
    free = np.asarray(log["free_spin"], float)
    nose = np.asarray(log["omega_nose_rps"], float)
    phi = np.asarray(log.get("phi_nose_deg", np.zeros_like(t)), float)
    tc = np.asarray(log.get("theta_c_deg", np.zeros_like(t)), float)
    herr = np.asarray(log.get("hold_err_deg", np.zeros_like(t)), float)

    post = t >= arm_time
    free_frac = float(free[post].mean()) if post.any() else 1.0

    # time to |nose| < 1 rps after arm
    t_despin = None
    for ti, n, f in zip(t, nose, free):
        if ti < arm_time:
            continue
        if f < 0.5 and abs(n) < 1.0:
            t_despin = float(ti)
            break

    hold_mask = post & (free < 0.5)
    if hold_mask.any() and np.any(np.abs(herr[hold_mask]) > 0):
        mean_err = float(np.mean(np.abs(herr[hold_mask])))
        max_err = float(np.max(np.abs(herr[hold_mask])))
    else:
        # fallback geometric error
        err = (phi - tc + 180) % 360 - 180
        if hold_mask.any():
            mean_err = float(np.mean(np.abs(err[hold_mask])))
            max_err = float(np.max(np.abs(err[hold_mask])))
        else:
            mean_err = max_err = float("nan")

    rep = {
        "free_spin_frac_after_arm": free_frac,
        "t_despin_s": t_despin,
        "mean_hold_err_deg": mean_err,
        "max_hold_err_deg": max_err,
    }
    return rep


def print_hold_report(log, arm_time: float = 7.0):
    r = hold_quality_report(log, arm_time)
    print("=== Hold quality ===")
    print(f"  Free-spin fraction after arm : {r['free_spin_frac_after_arm']:.3f}")
    print(f"  Time to de-spin (|n|<1 rps)  : {r['t_despin_s']}")
    print(f"  Mean |φ−θ_c| while holding   : {r['mean_hold_err_deg']:.1f} deg")
    print(f"  Max  |φ−θ_c| while holding   : {r['max_hold_err_deg']:.1f} deg")
    return r

</file>

<file src="pgk_imu.py">
"""
Lightweight IMU model for PGK-class kit (optional / supplementary).

Real M1156 is GPS-dominant for guidance; public descriptions emphasize GPS +
canard control, not a full tactical INS. Some kits and evolutions use limited
inertial sensing (rate / acceleration) for:
  - short GPS-outage coast
  - health / launch detect
  - optional roll-rate aiding of the de-spun nose

This module provides a simple 6-axis IMU with bias + noise, and a crude
complementary use path (velocity coast assist when GPS is bad).
"""

from __future__ import annotations

import numpy as np
from pgk_params import Params
from pgk_math import q_to_dcm


class SimpleIMU:
    """
    Body-frame specific force and angular rate measurements.
    Truth a_inertial and omega_body mapped to body with bias/noise.
    """

    def __init__(self, p: Params):
        self.p = p
        self.enabled = bool(getattr(p, "imu_enable", True))
        self.rate_hz = float(getattr(p, "imu_rate_hz", 100.0))
        self.period = 1.0 / max(self.rate_hz, 1.0)
        self.next_t = 0.0
        # Biases (random walk simplified as constant per flight)
        self.gyro_bias = np.random.normal(0.0, np.radians(0.05), size=3)  # rad/s
        self.accel_bias = np.random.normal(0.0, 0.05, size=3)  # m/s^2
        self.gyro_noise = np.radians(0.02)  # rad/s 1σ
        self.accel_noise = 0.08  # m/s^2 1σ
        self.last_omega = np.zeros(3)
        self.last_accel = np.zeros(3)

    def step(self, t: float, q, omega_body, a_inertial, g_inertial):
        if not self.enabled:
            return None
        if t + 1e-12 < self.next_t:
            return self.last_accel, self.last_omega
        self.next_t = t + self.period
        R = q_to_dcm(q)  # body → inertial
        # Specific force in body: R^T (a - g) approximately; use a_inertial as measured accel
        f_body = R.T @ (a_inertial - g_inertial)
        omega_m = (
            np.asarray(omega_body, float)
            + self.gyro_bias
            + np.random.normal(0.0, self.gyro_noise, size=3)
        )
        accel_m = (
            f_body
            + self.accel_bias
            + np.random.normal(0.0, self.accel_noise, size=3)
        )
        self.last_omega = omega_m
        self.last_accel = accel_m
        return accel_m, omega_m


class ImuGpsCoast:
    """
    When GPS is bad, integrate last velocity with IMU accel for short coast.
    Not a full AHRS — enough to show IMU value under dropout.
    """

    def __init__(self):
        self.v = None
        self.r = None

    def reset(self, r, v):
        self.r = np.asarray(r, float).copy()
        self.v = np.asarray(v, float).copy()

    def coast(self, dt, accel_inertial_est):
        if self.v is None:
            return None, None
        self.v = self.v + np.asarray(accel_inertial_est, float) * dt
        self.r = self.r + self.v * dt
        return self.r.copy(), self.v.copy()

</file>

<file src="pgk_main.py">
#!/usr/bin/env python3
"""PGK sim: restore ~10-20 m guided accuracy while keeping GPS/Kalman stack."""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_validation import open_loop_summary, compare_to_ballpark


def run_ol():
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.arm_time = 9999.0
    p.azimuth_bias_deg = 0.0
    p.gps_enable = False
    p.use_kalman = False
    p.use_met_layers = False
    p.wind = np.zeros(3)
    log = Simulator(p).run(log_every=100)
    return p, log, open_loop_summary(log, p)


def run_guided(s, gps_noise=0.0, use_gps=False):
    """
    Guided shot. Default use_gps=False = truth nav (best accuracy check).
    use_gps=True enables GPS+Kalman with optional noise.
    Do NOT override canard_authority down — pgk_params already has tuned CN≈0.029.
    """
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.target_x = s["impact_x"] + 100.0
    p.target_y = s["impact_y"] - 70.0
    p.azimuth_bias_deg = 0.35
    # Keep tuned defaults from Params (arm=7, deadband=12, CN≈0.029, a_max=1.15)
    p.guidance_phase_deg = 0.0
    p.fail_safe_miss = 5000.0
    p.use_met_layers = False
    p.wind = np.zeros(3)
    p.gps_enable = use_gps
    p.use_kalman = use_gps
    p.gps_noise_sigma = gps_noise
    p.gps_dropout_prob = 0.0
    p.gps_latency_s = 0.05 if use_gps else 0.0
    p.gps_rate_hz = 10.0
    log = Simulator(p).run(log_every=80)
    impact = np.array([log["x"][-1], log["y"][-1]])
    tgt = np.array([p.target_x, p.target_y])
    miss = float(np.linalg.norm(impact - tgt))
    ol_miss = float(np.linalg.norm(np.array([s["impact_x"], s["impact_y"]]) - tgt))
    return p, log, impact, tgt, miss, ol_miss


def main():
    print("=== Open-loop ===")
    _, log_ol, s = run_ol()
    compare_to_ballpark(s, Params())

    cn = p.canard_Cl_max * p.canard_authority * p.canard_area / p.ref_area if False else (
        Params().canard_Cl_max * Params().canard_authority * Params().canard_area / Params().ref_area
    )
    print(f"\nParams CN≈{cn:.4f} a_max={Params().canard_a_max} arm={Params().arm_time}s")

    print("\n=== Guided (truth nav) — target accuracy mode ===")
    p, log, impact, tgt, miss, ol_miss = run_guided(s, gps_noise=0.0, use_gps=False)
    print(f"Target     : ({tgt[0]:.0f}, {tgt[1]:.0f}) m")
    print(f"Impact     : ({impact[0]:.1f}, {impact[1]:.1f}) m")
    print(f"Guided miss: {miss:.1f} m")
    print(f"OL miss    : {ol_miss:.1f} m")
    print(f"Improvement: {ol_miss - miss:+.1f} m")
    print(f"Apogee {max(log['z']):.0f} m  ToF {log['t'][-1]:.1f} s  MaxAoA {max(np.abs(log['aoa'])):.1f}°")
    print(f"Body spin {log['p_rps'][-1]:.1f} rps  Nose {log['omega_nose_rps'][-1]:.1f} rps")
    print(f"Fail-safe {any(log['fail_safe'])}")
    t = np.array(log["t"]); free = np.array(log["free_spin"])
    print(f"Free-spin fraction after arm: {float(free[t >= p.arm_time].mean()):.3f}")

    print("\n=== Guided (GPS + Kalman, noise σ=2 m) ===")
    _, log_n, impact_n, tgt_n, miss_n, ol_n = run_guided(s, gps_noise=2.0, use_gps=True)
    print(f"Guided miss: {miss_n:.1f} m  (OL {ol_n:.1f} m)  gain {ol_n-miss_n:+.1f} m")

    # Plots
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(2, 3, (1, 4), projection="3d")
    ax.plot(log["x"], log["y"], log["z"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], log_ol["z"], lw=1.0, alpha=0.55, label="Open-loop")
    ax.scatter(0, 0, p.launch_height, c="g", s=40, label="Gun")
    ax.scatter(tgt[0], tgt[1], 0, c="m", marker="*", s=90, label="Target")
    ax.scatter(impact[0], impact[1], 0, c="r", s=40, label="Impact")
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("3-D Trajectory (accuracy restored)"); ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(log["t"], log["z"], label="Guided")
    ax.plot(log_ol["t"], log_ol["z"], alpha=0.55, label="OL")
    ax.set_title("Altitude"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 3)
    ax.plot(log["t"], log["speed"]); ax.set_title("Speed"); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 3, 5)
    ax.plot(log["t"], log["theta_c_deg"], label="theta_c")
    ax.plot(log["t"], log["phi_nose_deg"], label="phi_nose", alpha=0.8)
    ax.plot(log["t"], np.array(log["free_spin"]) * 30, alpha=0.5, label="free-spin x30")
    ax2 = ax.twinx()
    ax2.plot(log["t"], log["zem"], color="C3", alpha=0.7, label="ZEM")
    ax.legend(loc="upper left", fontsize=7); ax2.legend(loc="upper right", fontsize=7)
    ax.grid(alpha=0.3); ax.set_title("Guidance")

    ax = fig.add_subplot(2, 3, 6)
    ax.plot(log["x"], log["y"], lw=1.6, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], alpha=0.55, label="OL")
    ax.scatter(tgt[0], tgt[1], c="m", marker="*", s=100, label="Target")
    ax.scatter(impact[0], impact[1], c="r", marker="x", s=80, label="Guided")
    ax.scatter(s["impact_x"], s["impact_y"], c="orange", marker="x", s=60, label="OL impact")
    ax.set_title("Ground track"); ax.axis("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("m1156_pgk_accuracy.png", dpi=130)
    plt.close()
    print("\nSaved m1156_pgk_accuracy.png")
    print(f"FINAL: guided miss {miss:.1f} m (was ~11 m earlier; OL {ol_miss:.1f} m)")


if __name__ == "__main__":
    main()

</file>

<file src="pgk_math.py">
"""
Quaternion and vector helpers (scalar-first, body +X = nose).
"""

import numpy as np


def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else q


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body → inertial rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def q_dot(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    p, qr, r = omega
    Om = np.array([
        [0, -p, -qr, -r],
        [p,  0,  r, -qr],
        [qr, -r, 0,  p],
        [r,  qr, -p, 0],
    ])
    return 0.5 * Om @ q


def euler_to_q(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Pitch sign chosen so positive elevation → nose up."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(-pitch/2), np.sin(-pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return q_normalize(np.array([w, x, y, z]))


def q_to_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

</file>

<file src="pgk_modes.py">
"""
Flight software mode state machine (arming FSM + watchdog hooks).

Modes align with dual-spin PGK (no motorized canard servo):
  SAFE → LAUNCH → COAST → ARM_CHECK → GUIDE | FREE_SPIN | FAILSAFE → TERMINAL
"""

from __future__ import annotations

from enum import IntEnum
import numpy as np
from pgk_params import Params


class FlightMode(IntEnum):
    SAFE = 0
    LAUNCH = 1
    COAST = 2
    ARM_CHECK = 3
    GUIDE = 4
    FREE_SPIN = 5
    FAILSAFE = 6
    TERMINAL = 7


class ModeManager:
    """
    Discrete flight modes at control_dt.

    free_spin flag exposed to dynamics:
      True  → canards rotate (net force cancels)
      False → nose held at theta_c (steady force)
    """

    def __init__(self, p: Params):
        self.p = p
        self.mode = FlightMode.SAFE
        self.free_spin = True
        self.fail_safe = False
        self.armed = False
        self.launch_detected = False
        self.t_launch = None
        self.watchdog_count = 0
        self.last_gps_ok_t = 0.0

    def on_setback(self, t: float):
        """Gun setback / muzzle exit detect."""
        if self.mode == FlightMode.SAFE:
            self.mode = FlightMode.LAUNCH
            self.launch_detected = True
            self.t_launch = t

    def update(
        self,
        t: float,
        zem: float,
        gps_ok: bool,
        impact_imminent: bool = False,
        fuze_terminal: bool = False,
    ):
        p = self.p
        if gps_ok:
            self.last_gps_ok_t = t
            self.watchdog_count = 0
        else:
            self.watchdog_count += 1

        # Auto-launch if motion started (sim: t>0 after first step)
        if self.mode == FlightMode.SAFE and t > 0.05:
            self.on_setback(t)

        if self.mode == FlightMode.LAUNCH and t > (self.t_launch or 0) + 0.5:
            self.mode = FlightMode.COAST

        if self.mode == FlightMode.COAST and t >= p.arm_time:
            self.mode = FlightMode.ARM_CHECK

        if self.mode == FlightMode.ARM_CHECK:
            # Early gross-miss inhibit
            if zem > p.fail_safe_miss * 8.0:
                self.fail_safe = True
                self.mode = FlightMode.FAILSAFE
                self.free_spin = True
                self.armed = False
            else:
                self.armed = True
                self.mode = FlightMode.GUIDE

        if self.mode == FlightMode.GUIDE:
            if self.fail_safe:
                self.mode = FlightMode.FAILSAFE
                self.free_spin = True
            elif zem < p.zem_deadband:
                self.mode = FlightMode.FREE_SPIN
                self.free_spin = True
            elif t > p.arm_time + 8.0 and zem > p.fail_safe_miss * 6.0:
                self.fail_safe = True
                self.mode = FlightMode.FAILSAFE
                self.free_spin = True
            else:
                self.free_spin = False  # HOLD canards

        if self.mode == FlightMode.FREE_SPIN:
            if zem >= p.zem_deadband * 1.5 and not self.fail_safe and self.armed:
                # Re-engage if residual grows
                self.mode = FlightMode.GUIDE
                self.free_spin = False
            else:
                self.free_spin = True

        if self.mode == FlightMode.FAILSAFE:
            self.free_spin = True
            self.armed = False

        if fuze_terminal or impact_imminent:
            self.mode = FlightMode.TERMINAL
            self.free_spin = True

        # GPS watchdog: long dropout → free-spin (do not steer on dead nav)
        if self.armed and (t - self.last_gps_ok_t) > 2.0:
            self.free_spin = True
            if self.mode == FlightMode.GUIDE:
                self.mode = FlightMode.FREE_SPIN

        return self.mode, self.free_spin

</file>

<file src="pgk_montecarlo.py">
"""
Monte-Carlo CEP wrapper for the dual-spin PGK simulation.

Perturbs:
  - muzzle velocity
  - elevation / azimuth bias
  - constant wind
  - GPS position noise (guidance)
  - small aero scale factors

Reports circular error probable (CEP) style statistics and compares
against published PGK thresholds (~30–50 m CEP objective/threshold).
"""

import copy
import numpy as np
from pgk_params import Params
from pgk_simulator import Simulator


def _sample_params(base: Params, rng: np.random.Generator) -> Params:
    """Return a deep-ish copy of Params with realistic dispersions."""
    p = copy.deepcopy(base)

    # Muzzle velocity dispersion (~0.3–0.5% 1σ is optimistic; use ~0.4%)
    p.muzzle_velocity = base.muzzle_velocity * (1.0 + rng.normal(0.0, 0.004))

    # Elevation / azimuth pointing error (mils-scale → degrees)
    p.elevation_deg = base.elevation_deg + rng.normal(0.0, 0.05)   # ~0.9 mil
    p.azimuth_bias_deg = base.azimuth_bias_deg + rng.normal(0.0, 0.08)

    # Constant wind (m/s) — range + cross components
    p.wind = np.array([
        rng.normal(0.0, 3.0),   # range wind
        rng.normal(0.0, 3.0),   # cross wind
        0.0,
    ])

    # GPS horizontal noise (m, 1σ). Typical civil is worse; tactical SAASM better.
    p.gps_noise_sigma = float(rng.uniform(2.0, 8.0))

    # Mild aero uncertainty
    p.Cd0_ref = base.Cd0_ref * (1.0 + rng.normal(0.0, 0.03))
    p.Cl_alpha_ref = base.Cl_alpha_ref * (1.0 + rng.normal(0.0, 0.04))

    # Canard authority scatter
    p.canard_authority = float(np.clip(
        base.canard_authority * (1.0 + rng.normal(0.0, 0.08)), 0.15, 0.85
    ))

    return p


def run_monte_carlo(base: Params, n_runs: int = 40, seed: int = 42,
                    guided: bool = True) -> dict:
    """
    Run n_runs trajectories. Returns miss vectors and CEP-like metrics.

    CEP approximation used here: radius of circle containing 50% of impacts
    (median radial miss from target). Also report 90% circle and RMS.
    """
    rng = np.random.default_rng(seed)
    misses = []
    impacts = []
    fail_count = 0

    for i in range(n_runs):
        p = _sample_params(base, rng)
        if not guided:
            p.arm_time = 9999.0
        sim = Simulator(p)
        log = sim.run(log_every=50)
        impact = np.array([log["x"][-1], log["y"][-1]])
        target = np.array([p.target_x, p.target_y])
        miss_vec = impact - target
        miss_r = float(np.linalg.norm(miss_vec))
        misses.append(miss_r)
        impacts.append(impact)
        if any(log["fail_safe"]):
            fail_count += 1
        if (i + 1) % max(1, n_runs // 5) == 0:
            print(f"  MC {i+1}/{n_runs}  last miss={miss_r:.1f} m")

    misses = np.array(misses)
    impacts = np.array(impacts)
    # CEP ≈ median radial miss (simple, common engineering approximation)
    cep = float(np.median(misses))
    r90 = float(np.percentile(misses, 90))
    rms = float(np.sqrt(np.mean(misses ** 2)))
    mean_miss = float(np.mean(misses))

    return {
        "n_runs": n_runs,
        "guided": guided,
        "misses": misses,
        "impacts": impacts,
        "cep_m": cep,
        "r90_m": r90,
        "rms_m": rms,
        "mean_miss_m": mean_miss,
        "fail_safe_count": fail_count,
        "target": np.array([base.target_x, base.target_y]),
    }


def print_cep_report(result: dict) -> None:
    label = "Guided" if result["guided"] else "Open-loop"
    print(f"\n=== Monte-Carlo CEP — {label} (N={result['n_runs']}) ===")
    print(f"  Median radial miss (CEP approx) : {result['cep_m']:.1f} m")
    print(f"  90% circle radius               : {result['r90_m']:.1f} m")
    print(f"  RMS miss                        : {result['rms_m']:.1f} m")
    print(f"  Mean radial miss                : {result['mean_miss_m']:.1f} m")
    print(f"  Fail-safe triggers              : {result['fail_safe_count']}")
    print("  Published PGK reference         : threshold <50 m CEP, "
          "objective <30 m CEP")
    if result["guided"]:
        if result["cep_m"] < 30.0:
            print("  → CEP is within objective band (<30 m).")
        elif result["cep_m"] < 50.0:
            print("  → CEP is within threshold band (<50 m).")
        else:
            print("  → CEP is larger than published PGK thresholds — "
                  "expect with simplified aero / authority / phase; "
                  "tune phase offset, authority, and predictor.")

</file>

<file src="pgk_nav.py">
"""GPS measurement model + constant-velocity Kalman filter for PGK nav."""

import numpy as np
from pgk_params import Params
from pgk_math import q_to_euler


class GPSReceiver:
    def __init__(self, p: Params):
        self.p = p
        self.period = 1.0 / max(p.gps_rate_hz, 0.1)
        self.next_fix_time = p.gps_latency_s
        self._buf = []
        self.last_fix = None

    def step(self, t, r_true, v_true, q_true):
        p = self.p
        if not p.gps_enable:
            eul = q_to_euler(q_true)
            self.last_fix = (t, r_true.copy(), v_true.copy(), float(eul[0]), True)
            return self.last_fix

        self._buf.append((t, r_true.copy(), v_true.copy(), q_true.copy()))
        while self._buf and self._buf[0][0] < t - p.gps_latency_s - 1.0:
            self._buf.pop(0)

        if t + 1e-12 < self.next_fix_time:
            return self.last_fix

        self.next_fix_time = t + self.period
        t_target = t - p.gps_latency_s
        sample = None
        for item in reversed(self._buf):
            if item[0] <= t_target:
                sample = item
                break
        if sample is None:
            sample = self._buf[0] if self._buf else None
        if sample is None:
            return self.last_fix

        t_s, r_s, v_s, q_s = sample
        if np.random.random() < p.gps_dropout_prob:
            self.last_fix = (t, None, None, None, False)
            return self.last_fix

        r_m = r_s.copy()
        v_m = v_s.copy()
        r_m[:2] += np.random.normal(0.0, p.gps_noise_sigma, size=2)
        r_m[2] += np.random.normal(0.0, p.gps_noise_sigma * 1.5)
        v_m += np.random.normal(0.0, p.gps_noise_sigma_v, size=3)
        eul = q_to_euler(q_s)
        roll_m = float(eul[0] + np.random.normal(0.0, p.gps_roll_sigma))
        self.last_fix = (t, r_m, v_m, roll_m, True)
        return self.last_fix


class NavKalman:
    def __init__(self, p: Params, r0, v0):
        self.p = p
        self.x = np.hstack([np.asarray(r0, float), np.asarray(v0, float)])
        self.P = np.diag([25.0, 25.0, 40.0, 4.0, 4.0, 4.0])
        self.q_pos = 0.5
        self.q_vel = 2.0

    def predict(self, dt: float):
        if dt <= 0.0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        q = np.diag([self.q_pos*dt]*3 + [self.q_vel*dt]*3)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + q

    def update_gps(self, r_meas, v_meas=None):
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.diag([self.p.gps_noise_sigma**2]*2 + [(self.p.gps_noise_sigma*1.5)**2])
        y = r_meas - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        if v_meas is not None:
            Hv = np.zeros((3, 6))
            Hv[0, 3] = Hv[1, 4] = Hv[2, 5] = 1.0
            Rv = np.eye(3) * (self.p.gps_noise_sigma_v**2)
            yv = v_meas - Hv @ self.x
            Sv = Hv @ self.P @ Hv.T + Rv
            Kv = self.P @ Hv.T @ np.linalg.inv(Sv)
            self.x = self.x + Kv @ yv
            self.P = (np.eye(6) - Kv @ Hv) @ self.P

    @property
    def r(self):
        return self.x[0:3].copy()

    @property
    def v(self):
        return self.x[3:6].copy()

</file>

<file src="pgk_params.py">
"""
Parameters for high-fidelity M1156-style dual-spin fixed-canard PGK simulation.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Params:
    # ----- Projectile + kit mass properties (approx 155 mm HE + PGK) -----
    mass: float = 43.5
    mass_nose: float = 1.4
    Ixx: float = 0.148
    Iyy: float = 1.82
    Izz: float = 1.82
    Ixx_nose: float = 0.012
    diameter: float = 0.155
    ref_area: float = 0.01887
    cg_to_nose: float = 0.48
    length: float = 0.84

    # ----- Atmosphere / Earth -----
    rho0: float = 1.225
    T0: float = 288.15
    lapse: float = 0.0065
    g0: float = 9.80665
    latitude_deg: float = 32.0
    earth_radius: float = 6371000.0
    omega_earth: float = 7.292115e-5
    wind: np.ndarray = field(default_factory=lambda: np.zeros(3))
    use_met_layers: bool = False

    # ----- Body aero reference (scaled by tables) -----
    Cd0_ref: float = 0.215
    Cl_alpha_ref: float = 1.65
    max_Cl: float = 1.05
    Cmq_ref: float = -4.2
    Clp_ref: float = -0.048
    Cm_alpha_ref: float = -2.35
    Cn_pa_ref: float = 0.055
    Cm_pa_ref: float = -0.004

    # ----- Canards (physical authority) -----
    canard_area: float = 0.00075
    canard_Cl_max: float = 1.0
    canard_stall_alpha: float = np.radians(16.0)
    canard_authority: float = 0.68
    canard_min_speed: float = 70.0
    canard_a_max: float = 1.05
    canard_interference: float = 0.15

    # ----- Dual-spin / brake / alternator -----
    brake_torque_max: float = 8.0
    bearing_visc: float = 0.0015
    free_spin_rel_rate: float = 6.0
    K_brake: float = 1.0
    K_phi_hold: float = 30.0
    alternator_torque_coeff: float = 0.00025
    aero_despin_coeff: float = 0.00040

    # ----- Launch -----
    muzzle_velocity: float = 605.0
    elevation_deg: float = 42.0
    azimuth_bias_deg: float = 0.35
    initial_spin_rps: float = 180.0
    launch_height: float = 4.5
    gun_jump_mrad: float = 0.0
    inbore_yaw_mrad: float = 0.0
    mv_sigma_frac: float = 0.0

    # ----- Target -----
    target_x: float = 15500.0
    target_y: float = 400.0

    # ----- Guidance -----
    arm_time: float = 7.0
    zem_deadband: float = 12.0
    fail_safe_miss: float = 2500.0
    fail_safe_check_time: float = 8.0
    # PGK-style fuze: withhold detonation if early predicted miss is huge
    fuze_inhibit_miss_m: float = 150.0
    fuze_inhibit_check_time_s: float = 5.0
    # documentation: reduced-order authority calibrated for ~15-25 m nominal miss
    orientation_slew_tau: float = 0.25
    max_slew_rate_deg: float = 90.0
    K_orient: float = 6.0
    predictor_dt: float = 0.10
    guidance_phase_deg: float = 0.0  # empirical best from guided phase sweep

    # ----- GPS / navigation -----
    gps_rate_hz: float = 10.0
    gps_latency_s: float = 0.05
    gps_noise_sigma: float = 0.0
    gps_noise_sigma_v: float = 0.4
    gps_dropout_prob: float = 0.0
    gps_enable: bool = True
    use_kalman: bool = True
    gps_roll_sigma: float = np.radians(2.0)

    # ----- IMU (optional, GPS-dominant baseline) -----
    imu_enable: bool = True
    imu_rate_hz: float = 100.0
    imu_coast_on_gps_loss: bool = True


    # ----- Integration -----
    dt: float = 0.002
    control_dt: float = 0.02
    tmax: float = 60.0


def g0_at_latitude(lat_deg: float) -> float:
    lat = np.radians(lat_deg)
    g_fts2 = 32.174 * (1.0 - 0.0026 * np.cos(2.0 * lat))
    return g_fts2 * 0.3048
</file>

<file src="pgk_phase_id.py">
#!/usr/bin/env python3
"""
Fixed-canard phase identification (0/90/180/270°) then freeze guidance_phase_deg.

Method:
  1) Open-loop impact baseline
  2) Hold canards fixed at 0, 90, 180, 270° after t0
  3) Measure impact shift relative to OL
  4) Choose phase that best aligns force direction with a desired correction
     (or report all four shifts for manual freeze)

Usage:
  python pgk_phase_id.py
  python pgk_phase_id.py --t0 25 --apply
"""

from __future__ import annotations

import argparse
import numpy as np

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_guidance import Guidance
from pgk_validation import open_loop_summary


class FixedCanardGuid(Guidance):
    def __init__(self, p, deg, t0=25.0):
        super().__init__(p)
        self.fixed = np.radians(deg)
        self.t0 = t0
        self.theta_c = self.fixed
        self.free_spin = True

    def update(self, t, r_nav, v_nav, dt, F_can_current=None,
               phi_meas=None, gps_ok=True, impact_imminent=False, fuze_terminal=False):
        if F_can_current is not None:
            self.last_F_can_est = 0.65 * self.last_F_can_est + 0.35 * F_can_current
        if t < self.t0:
            self.free_spin = True
            self.zem = 0.0
            return
        self.free_spin = False
        self.theta_c = self.fixed
        self.zem = 0.0


def _quiet(**kw):
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.gps_enable = False
    p.use_kalman = False
    p.use_met_layers = False
    p.wind = np.zeros(3)
    p.fail_safe_miss = 5000.0
    for k, v in kw.items():
        if hasattr(p, k):
            setattr(p, k, v)
    return p


def run_phase_id(t0: float = 25.0, apply: bool = False):
    print("=== Fixed-canard phase identification ===")
    p_ol = _quiet(arm_time=9999.0, azimuth_bias_deg=0.0)
    log_ol = Simulator(p_ol).run(log_every=100)
    s = open_loop_summary(log_ol, p_ol)
    ol_xy = np.array([s["impact_x"], s["impact_y"]])
    print(f"OL impact: ({ol_xy[0]:.0f}, {ol_xy[1]:.0f}) m")

    shifts = {}
    for deg in (0, 90, 180, 270):
        p = _quiet(azimuth_bias_deg=0.0, arm_time=0.0)
        sim = Simulator(p)
        sim.guid = FixedCanardGuid(p, deg, t0=t0)
        log = sim.run(log_every=100)
        xy = np.array([log["x"][-1], log["y"][-1]])
        d = xy - ol_xy
        shifts[deg] = d
        print(f"  Fixed {deg:3d}°  Δimpact = ({d[0]:+7.1f}, {d[1]:+7.1f}) m  |Δ|={np.linalg.norm(d):.1f} m")

    # Desired correction example: reduce cross-range drift and add downrange
    # (same as typical guided job +100 x, -70 y on target relative to biased aim)
    desired = np.array([100.0, -70.0])
    best_deg = None
    best_score = -1e18
    for deg, d in shifts.items():
        # Score: projection of shift onto desired direction
        if np.linalg.norm(d) < 1.0:
            score = -1e9
        else:
            score = float(np.dot(d / np.linalg.norm(d), desired / np.linalg.norm(desired)))
            score *= float(np.linalg.norm(d))
        print(f"  score[{deg}°] = {score:.1f}")
        if score > best_score:
            best_score = score
            best_deg = deg

    # Guidance phase often ≈ canard hold angle that produces desired force direction
    # For fixed-canard ID, recommended phase is the best fixed angle (degrees).
    print(f"\nRecommended guidance_phase_deg ≈ {best_deg}° (from fixed-canard scores)")
    print("Freeze this in Params.guidance_phase_deg after confirming with a guided shot.")

    if apply and best_deg is not None:
        # Write into running recommendation only (print); user can set in params
        print(f"\nTo apply: set Params.guidance_phase_deg = {float(best_deg)}")

    return best_deg, shifts


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--t0", type=float, default=25.0)
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()
    run_phase_id(t0=args.t0, apply=args.apply)

</file>

<file src="pgk_run_nextlevel.py">
#!/usr/bin/env python3
"""
Next-level runner:
  1. Open-loop validation (range / drift / AoA / spin)
  2. Single guided shot with diagnostics (incl. yaw-of-repose estimate)
  3. Monte-Carlo CEP (guided + optional open-loop)

Atmosphere, Coriolis, wind, GPS noise, arming and fail-safe are already
in the core model; this script exercises them and reports CEP.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_validation import open_loop_summary, compare_to_ballpark
from pgk_montecarlo import run_monte_carlo, print_cep_report


def main():
    # ------------------------------------------------------------------
    # 1) Open-loop validation
    # ------------------------------------------------------------------
    p_ol = Params()
    p_ol.arm_time = 9999.0
    p_ol.azimuth_bias_deg = 0.0          # clean open-loop azimuth
    p_ol.wind = np.zeros(3)
    p_ol.gps_noise_sigma = 0.0

    print("Running open-loop trajectory for validation...")
    log_ol = Simulator(p_ol).run(log_every=8)
    summary = open_loop_summary(log_ol, p_ol)
    compare_to_ballpark(summary, p_ol)

    # Place target near open-loop impact so guided has a realistic job
    # (tens–hundreds of metres of intentional bias, not kilometres)
    p = Params()
    p.target_x = summary["impact_x"] + 80.0
    p.target_y = summary["impact_y"] - 60.0
    p.azimuth_bias_deg = 0.6
    p.fail_safe_miss = 400.0
    p.zem_deadband = 15.0
    p.arm_time = 10.0
    # Modest GPS noise on the single guided shot
    p.gps_noise_sigma = 3.0
    # Slight cross wind
    p.wind = np.array([0.0, 2.0, 0.0])

    # ------------------------------------------------------------------
    # 2) Single guided shot
    # ------------------------------------------------------------------
    print("\nRunning single guided shot...")
    log = Simulator(p).run(log_every=6)
    impact = np.array([log["x"][-1], log["y"][-1]])
    target = np.array([p.target_x, p.target_y])
    miss = float(np.linalg.norm(impact - target))

    print("\n=== Single guided shot ===")
    print(f"  Target     : ({p.target_x:.0f}, {p.target_y:.0f}) m")
    print(f"  Impact     : ({impact[0]:.1f}, {impact[1]:.1f}) m")
    print(f"  Miss       : {miss:.1f} m")
    print(f"  Apogee     : {max(log['z']):.0f} m")
    print(f"  ToF        : {log['t'][-1]:.1f} s")
    print(f"  Max |AoA|  : {max(np.abs(log['aoa'])):.1f}°")
    print(f"  Body spin  : {log['p_rps'][-1]:.1f} rps")
    print(f"  Nose rate  : {log['omega_nose_rps'][-1]:.1f} rps")
    if log.get("yaw_repose_deg"):
        yr = np.array(log["yaw_repose_deg"])
        print(f"  |α_R| est  : mean {np.mean(np.abs(yr)):.2f}°, "
              f"max {np.max(np.abs(yr)):.2f}°  (yaw-of-repose estimate)")
    if any(log["fail_safe"]):
        print("  NOTE: Fail-safe was triggered.")

    # ------------------------------------------------------------------
    # 3) Monte-Carlo CEP (guided)
    # ------------------------------------------------------------------
    print("\nRunning Monte-Carlo (guided)...")
    # Use same target / base settings; MC adds its own dispersions
    p_mc = Params()
    p_mc.target_x = p.target_x
    p_mc.target_y = p.target_y
    p_mc.fail_safe_miss = 500.0
    p_mc.arm_time = 10.0
    # Keep canard authority moderate so misses stay in tens of metres
    # when phase/authority are not perfectly tuned
    result_g = run_monte_carlo(p_mc, n_runs=25, seed=7, guided=True)
    print_cep_report(result_g)

    print("\nRunning Monte-Carlo (open-loop) for comparison...")
    result_ol = run_monte_carlo(p_mc, n_runs=15, seed=11, guided=False)
    print_cep_report(result_ol)

    # ------------------------------------------------------------------
    # Plots
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(14, 9))

    ax = fig.add_subplot(2, 3, (1, 4), projection="3d")
    ax.plot(log["x"], log["y"], log["z"], lw=1.5, label="Guided")
    ax.plot(log_ol["x"], log_ol["y"], log_ol["z"], lw=1.0, alpha=0.55, label="Open-loop")
    ax.scatter(0, 0, p.launch_height, c="g", s=35, label="Gun")
    ax.scatter(p.target_x, p.target_y, 0, c="m", marker="*", s=80, label="Target")
    ax.scatter(impact[0], impact[1], 0, c="r", s=35, label="Impact")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("3-D Trajectory"); ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(log["t"], log["z"], label="Guided")
    ax.plot(log_ol["t"], log_ol["z"], alpha=0.55, label="Open-loop")
    ax.set_title("Altitude"); ax.set_xlabel("t (s)"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 3)
    ax.plot(log["t"], log["aoa"], label="AoA")
    if log.get("yaw_repose_deg"):
        ax.plot(log["t"], log["yaw_repose_deg"], label="α_R est", alpha=0.8)
    ax.set_title("AoA / Yaw of repose est."); ax.set_xlabel("t (s)")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 5)
    ax.plot(log["t"], log["theta_c_deg"], label="θ_c")
    ax.plot(log["t"], log["phi_nose_deg"], label="φ_nose", alpha=0.8)
    ax.plot(log["t"], np.array(log["free_spin"]) * 30, label="free-spin×30", alpha=0.5)
    ax2 = ax.twinx()
    ax2.plot(log["t"], log["zem"], color="C3", alpha=0.7, label="ZEM")
    ax.set_xlabel("t (s)"); ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=7); ax2.legend(loc="upper right", fontsize=7)

    ax = fig.add_subplot(2, 3, 6)
    # Ground impacts from MC
    ax.scatter(result_g["impacts"][:, 0], result_g["impacts"][:, 1],
               c="C0", s=18, alpha=0.75, label="Guided MC")
    ax.scatter(result_ol["impacts"][:, 0], result_ol["impacts"][:, 1],
               c="C1", s=18, alpha=0.6, label="Open MC")
    ax.scatter(p.target_x, p.target_y, c="m", marker="*", s=120, label="Target")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Monte-Carlo impacts"); ax.axis("equal")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("m1156_pgk_nextlevel.png", dpi=130)
    plt.close()
    print("\nSaved: m1156_pgk_nextlevel.png")


if __name__ == "__main__":
    main()

</file>

<file src="pgk_sensors.py">
"""
Sensor selection and measurement models for PGK-class dual-spin kit.

Design decision (classic M1156-style PGK, not full IMU missile):
  - Primary nav: GNSS (GPS)
  - Nose/canard roll: dedicated roll sensing on de-spun section
    (magnetometer and/or optical/encoder relative to body + GPS up-finding)
  - No continuous tactical IMU required for baseline ZEM guidance
  - Optional baro for altitude aiding
  - Fuze sensors: impact switch, timer, proximity channel

Rates (flight software):
  GPS          1–10 Hz   (model default 10 Hz)
  Roll / hold  50–100 Hz control tick
  Fuze         continuous + discrete events
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from pgk_params import Params
from pgk_math import q_to_euler


@dataclass
class SensorSuiteSpec:
    """Documented sensor list for architecture / electronics handoff."""

    # Navigation
    gnss: str = "GPS L1 (SAASM or civil) position + velocity"
    gnss_rate_hz: float = 10.0
    gnss_latency_s: float = 0.05
    baro: str = "Optional MEMS barometer for altitude aiding"
    imu: str = (
        "NOT required for baseline ZEM PGK; optional low-cost IMU for "
        "future GIF-like upgrade only"
    )

    # Dual-spin / control
    nose_roll: str = (
        "Roll sensor on de-spun nose (magnetometer and/or relative encoder "
        "to spinning body) — REQUIRED for hold mode"
    )
    body_spin_proxy: str = "Optional: alternator frequency / tach for health"

    # Fuze
    impact: str = "Crush switch / accelerometer threshold"
    timer: str = "Crystal / MCU timer from gun-exit or setback"
    proximity: str = "RF or optical proximity channel (mode-dependent)"

    # Environment / power
    setback_switch: str = "Launch detect / safe-arm input"
    alternator: str = "Relative-spin generator for in-flight power"


# --- Measurement models used in simulation ---

class NoseRollSensor:
    """
    Models nose attitude measurement available to flight software.
    Truth phi_nose is corrupted by noise; rate limited by sample period.
    """

    def __init__(self, p: Params, sigma_deg: float = 2.0, rate_hz: float = 100.0):
        self.sigma = np.radians(sigma_deg)
        self.period = 1.0 / max(rate_hz, 1.0)
        self.next_t = 0.0
        self.phi_meas = 0.0
        self.omega_meas = 0.0

    def step(self, t: float, phi_true: float, omega_true: float):
        if t + 1e-12 < self.next_t:
            return self.phi_meas, self.omega_meas
        self.next_t = t + self.period
        self.phi_meas = (phi_true + np.random.normal(0.0, self.sigma)) % (2 * np.pi)
        self.omega_meas = omega_true + np.random.normal(0.0, 0.5)  # rad/s
        return self.phi_meas, self.omega_meas


class BaroSensor:
    def __init__(self, sigma_m: float = 5.0, rate_hz: float = 20.0):
        self.sigma = sigma_m
        self.period = 1.0 / max(rate_hz, 1.0)
        self.next_t = 0.0
        self.h_meas = 0.0

    def step(self, t: float, h_true: float):
        if t + 1e-12 < self.next_t:
            return self.h_meas
        self.next_t = t + self.period
        self.h_meas = h_true + np.random.normal(0.0, self.sigma)
        return self.h_meas


def print_sensor_selection():
    s = SensorSuiteSpec()
    print("=== PGK sensor selection (baseline) ===")
    for k, v in s.__dict__.items():
        print(f"  {k}: {v}")

</file>

<file src="pgk_simulator.py">
"""6-DOF + dual-spin + GPS/Kalman navigation simulator."""

import numpy as np
from pgk_params import Params, g0_at_latitude
from pgk_math import q_normalize, q_dot, euler_to_q, q_to_euler, q_to_dcm
from pgk_aero import gravity_vector, coriolis_acceleration
from pgk_forces import forces_moments
from pgk_guidance import Guidance
from pgk_nav import GPSReceiver, NavKalman
from pgk_sensors import NoseRollSensor
from pgk_fuze import MultiModeFuze
from pgk_imu import SimpleIMU, ImuGpsCoast
from pgk_aero import gravity_vector


class Simulator:
    def __init__(self, p: Params):
        self.p = p
        p.g0 = g0_at_latitude(p.latitude_deg)
        self.guid = Guidance(p)
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])
        self.Iinv = np.linalg.inv(self.I)
        self._last_F_can = np.zeros(3)
        self.gps = None
        self.nav = None
        self.roll_sensor = NoseRollSensor(p, sigma_deg=2.0, rate_hz=100.0)
        self.fuze = MultiModeFuze(p)
        self.imu = SimpleIMU(p)
        self.imu_coast = ImuGpsCoast()
        self._t_despin = None
        self._hold_err_acc = []

    def deriv(self, t, s, theta_c, free_spin):
        r, v = s[0:3], s[3:6]
        q = q_normalize(s[6:10])
        omega = s[10:13]
        phi_nose, omega_nose = s[13], s[14]
        F, M_body, tau_brake, F_can, _ = forces_moments(
            r, v, q, omega, phi_nose, omega_nose, free_spin, theta_c, self.p)
        self._last_F_can = F_can
        a = F / self.p.mass + gravity_vector(r, self.p) + coriolis_acceleration(v, self.p)
        wdot = self.Iinv @ (M_body - np.cross(omega, self.I @ omega))
        omega_nose_dot = tau_brake / self.p.Ixx_nose
        ds = np.zeros(15)
        ds[0:3] = v
        ds[3:6] = a
        ds[6:10] = q_dot(q, omega)
        ds[10:13] = wdot
        ds[13] = omega_nose
        ds[14] = omega_nose_dot
        return ds

    def step(self, t, s, dt, theta_c, free_spin):
        k1 = self.deriv(t, s, theta_c, free_spin)
        k2 = self.deriv(t + dt/2, s + dt/2*k1, theta_c, free_spin)
        k3 = self.deriv(t + dt/2, s + dt/2*k2, theta_c, free_spin)
        k4 = self.deriv(t + dt, s + dt*k3, theta_c, free_spin)
        ns = s + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        ns[6:10] = q_normalize(ns[6:10])
        ns[13] = ns[13] % (2 * np.pi)
        return ns

    def run(self, log_every: int = 8):
        p = self.p
        elev = np.radians(p.elevation_deg) + p.gun_jump_mrad * 1e-3
        az = (np.arctan2(p.target_y, p.target_x)
              + np.radians(p.azimuth_bias_deg)
              + p.inbore_yaw_mrad * 1e-3)
        vm = p.muzzle_velocity
        if p.mv_sigma_frac > 0:
            vm *= (1.0 + np.random.normal(0, p.mv_sigma_frac))

        s = np.zeros(15)
        s[0:3] = [0.0, 0.0, p.launch_height]
        s[3:6] = [vm*np.cos(elev)*np.cos(az),
                  vm*np.cos(elev)*np.sin(az),
                  vm*np.sin(elev)]
        s[6:10] = euler_to_q(0.0, elev, az)
        s[10] = 2*np.pi*p.initial_spin_rps
        s[13] = 0.0
        s[14] = 2*np.pi*p.initial_spin_rps

        self.gps = GPSReceiver(p)
        self.nav = NavKalman(p, s[0:3], s[3:6]) if p.use_kalman else None

        log = {k: [] for k in [
            "t", "x", "y", "z", "speed", "pitch", "yaw",
            "p_rps", "theta_c_deg", "phi_nose_deg", "omega_nose_rps",
            "free_spin", "aoa", "zem", "fail_safe", "yaw_repose_deg",
            "nav_x", "nav_y", "can_a", "hold_err_deg", "imu_wx", "imu_ax"
        ]}

        t = 0.0
        sub = max(int(round(p.control_dt / p.dt)), 1)
        n = int(np.ceil(p.tmax / p.dt))
        last_nav_t = 0.0

        for i in range(n + 1):
            # GPS + Kalman every physics step
            fix = self.gps.step(t, s[0:3], s[3:6], s[6:10])
            if self.nav is not None:
                self.nav.predict(p.dt)
                if fix is not None and fix[4] and fix[1] is not None:
                    self.nav.update_gps(fix[1], fix[2])
                r_nav, v_nav = self.nav.r, self.nav.v
            else:
                if fix is not None and fix[4] and fix[1] is not None:
                    r_nav, v_nav = fix[1], fix[2]
                else:
                    r_nav, v_nav = s[0:3], s[3:6]

            # IMU measurement + optional coast when GPS bad
            a_true = (s[3:6] - getattr(self, "_v_prev", s[3:6])) / max(p.dt, 1e-6) if i > 0 else np.zeros(3)
            self._v_prev = s[3:6].copy()
            g_vec = gravity_vector(s[0:3], p)
            imu_out = self.imu.step(t, s[6:10], s[10:13], a_true, g_vec)
            gps_ok_now = True
            if p.gps_enable:
                gps_ok_now = (fix is not None and fix[4] and fix[1] is not None)
            if gps_ok_now:
                self.imu_coast.reset(r_nav, v_nav)
            elif getattr(p, "imu_coast_on_gps_loss", True) and imu_out is not None and self.imu_coast.v is not None:
                # crude: map body accel to inertial via DCM for coast
                from pgk_math import q_to_dcm
                R = q_to_dcm(s[6:10])
                a_est = R @ imu_out[0] + g_vec
                rc, vc = self.imu_coast.coast(p.dt, a_est)
                if rc is not None:
                    r_nav, v_nav = rc, vc

            if i % sub == 0:
                phi_m, _ = self.roll_sensor.step(t, float(s[13]), float(s[14]))
                gps_ok = True
                if p.gps_enable and fix is not None:
                    gps_ok = bool(fix[4])
                elif p.gps_enable:
                    gps_ok = False
                pred_miss = None
                if t >= getattr(p, "fuze_inhibit_check_time_s", 5.0):
                    try:
                        imp_p, _ = self.guid.predict_impact(r_nav, v_nav, include_control=False)
                        pred_miss = float(np.linalg.norm(self.guid.target[:2] - imp_p[:2]))
                    except Exception:
                        pred_miss = float(getattr(self.guid, "zem", 0.0)) or None
                self.fuze.step(
                    t, float(s[2]),
                    impact_contact=(s[2] < 0.0 and t > 1.5),
                    predicted_miss_m=pred_miss,
                )
                self.guid.update(
                    t, r_nav, v_nav, sub * p.dt,
                    F_can_current=self._last_F_can,
                    phi_meas=phi_m,
                    gps_ok=gps_ok,
                    impact_imminent=(s[2] < 50.0 and t > 5.0),
                    fuze_terminal=self.fuze.fired,
                )

            if i % log_every == 0:
                self._log(log, t, s, r_nav)

            if s[2] < 0.0 and t > 1.5:
                break

            s = self.step(t, s, p.dt, self.guid.theta_c, self.guid.free_spin)
            t += p.dt

        return log

    def _log(self, log, t, s, r_nav):
        r, v = s[0:3], s[3:6]
        q = q_normalize(s[6:10])
        omega = s[10:13]
        eul = q_to_euler(q)
        speed = np.linalg.norm(v)
        R = q_to_dcm(q)
        bx = R[:, 0]
        aoa = 0.0
        if speed > 1.0:
            ca = np.clip(np.dot(bx, v/speed), -1.0, 1.0)
            aoa = np.degrees(np.arccos(ca))
        # Cheap log ZEM: reuse guidance value if available, else skip full predict
        zem = float(getattr(self.guid, "zem", 0.0))
        if (not self.guid.free_spin) and t >= self.p.arm_time and zem <= 0.0:
            impact, _ = self.guid.predict_impact(r, v, include_control=True)
            zem = np.linalg.norm(self.guid.target[:2] - impact[:2])
        log["t"].append(t)
        log["x"].append(r[0]); log["y"].append(r[1]); log["z"].append(r[2])
        log["speed"].append(speed)
        log["pitch"].append(np.degrees(eul[1]))
        log["yaw"].append(np.degrees(eul[2]))
        log["p_rps"].append(omega[0]/(2*np.pi))
        log["theta_c_deg"].append(np.degrees(self.guid.theta_c))
        log["phi_nose_deg"].append(np.degrees(s[13] % (2*np.pi)))
        log["omega_nose_rps"].append(s[14]/(2*np.pi))
        log["free_spin"].append(1.0 if self.guid.free_spin else 0.0)
        log["aoa"].append(aoa)
        log["zem"].append(zem)
        log["fail_safe"].append(1.0 if self.guid.fail_safe else 0.0)
        log["nav_x"].append(float(r_nav[0])); log["nav_y"].append(float(r_nav[1]))
        Fa = float(np.linalg.norm(getattr(self, "_last_F_can", np.zeros(3))))
        log.setdefault("can_a", []).append(Fa / max(self.p.mass, 1e-6))

        herr = float(getattr(self.guid, "last_hold_err", 0.0))
        log["hold_err_deg"].append(float(np.degrees(herr)))
        log["imu_wx"].append(float(self.imu.last_omega[0]) if self.imu.enabled else 0.0)
        log["imu_ax"].append(float(self.imu.last_accel[0]) if self.imu.enabled else 0.0)
        if (not self.guid.free_spin) and t >= self.p.arm_time:
            self._hold_err_acc.append(abs(herr))
            if self._t_despin is None and abs(s[14]) < 5.0:
                self._t_despin = t

        try:
            from pgk_validation import yaw_of_repose_estimate
            gamma = float(np.arcsin(np.clip(v[2]/speed, -1, 1))) if speed > 1 else 0.0
            aR = yaw_of_repose_estimate(speed, float(r[2]), float(omega[0]), gamma, self.p)
            log["yaw_repose_deg"].append(float(np.degrees(aR)))
        except Exception:
            log["yaw_repose_deg"].append(0.0)
</file>

<file src="pgk_stress_cep.py">
#!/usr/bin/env python3
"""
GPS realism stress + wind-layer CEP sets + hold metrics on a reference guided shot.

Usage:
  python pgk_stress_cep.py
  python pgk_stress_cep.py --mc 15
"""

from __future__ import annotations

import argparse
import numpy as np

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_validation import open_loop_summary
from pgk_montecarlo import run_monte_carlo, print_cep_report, _sample_params
from pgk_hold_metrics import print_hold_report
from pgk_phase_id import run_phase_id


def base_target():
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.arm_time = 9999.0
    p.azimuth_bias_deg = 0.0
    p.gps_enable = False
    p.use_kalman = False
    p.use_met_layers = False
    p.wind = np.zeros(3)
    s = open_loop_summary(Simulator(p).run(log_every=100), p)
    return s


def guided_ref(s, **kw):
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.target_x = s["impact_x"] + 100.0
    p.target_y = s["impact_y"] - 70.0
    p.azimuth_bias_deg = 0.35
    p.fail_safe_miss = 5000.0
    p.use_met_layers = False
    p.wind = np.zeros(3)
    for k, v in kw.items():
        setattr(p, k, v)
    log = Simulator(p).run(log_every=80)
    miss = float(np.hypot(log["x"][-1] - p.target_x, log["y"][-1] - p.target_y))
    return p, log, miss


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=12)
    ap.add_argument("--skip-phase", action="store_true")
    args = ap.parse_args()

    print("========== 1) Phase ID ==========")
    if not args.skip_phase:
        try:
            run_phase_id(t0=25.0)
        except Exception as e:
            print("Phase ID skipped:", e)

    print("\n========== 2) Reference guided + hold metrics ==========")
    s = base_target()
    p, log, miss = guided_ref(s, gps_enable=False, use_kalman=False)
    print(f"Truth nav miss: {miss:.1f} m")
    print_hold_report(log, arm_time=p.arm_time)

    print("\n========== 3) GPS stress (1 Hz, dropouts) ==========")
    p2, log2, miss2 = guided_ref(
        s,
        gps_enable=True,
        use_kalman=True,
        gps_rate_hz=1.0,
        gps_noise_sigma=5.0,
        gps_dropout_prob=0.15,
        gps_latency_s=0.1,
        imu_enable=True,
        imu_coast_on_gps_loss=True,
    )
    print(f"GPS stress miss: {miss2:.1f} m  (1 Hz, dropout 15%, σ=5 m, IMU coast on)")
    print_hold_report(log2, arm_time=p2.arm_time)

    print("\n========== 4) CEP calm air ==========")
    base = Params()
    base.dt = 0.0025
    base.control_dt = 0.03
    base.target_x = s["impact_x"] + 100.0
    base.target_y = s["impact_y"] - 70.0
    base.azimuth_bias_deg = 0.35
    base.gps_enable = True
    base.use_kalman = True
    base.use_met_layers = False
    base.fail_safe_miss = 5000.0
    res = run_monte_carlo(base, n_runs=args.mc, seed=42, guided=True)
    print_cep_report(res)

    print("\n========== 5) CEP with MET wind layers ==========")
    base_w = Params()
    base_w.dt = 0.0025
    base_w.control_dt = 0.03
    base_w.target_x = base.target_x
    base_w.target_y = base.target_y
    base_w.azimuth_bias_deg = 0.35
    base_w.gps_enable = True
    base_w.use_kalman = True
    base_w.use_met_layers = True  # wind layers ON
    base_w.fail_safe_miss = 5000.0
    res_w = run_monte_carlo(base_w, n_runs=args.mc, seed=7, guided=True)
    print_cep_report(res_w)

    print("\nDone.")


if __name__ == "__main__":
    main()

</file>

<file src="pgk_validation.py">
"""Open-loop validation + yaw-of-repose estimate."""

import numpy as np
from pgk_params import Params
from pgk_aero import density, body_aero_coeffs, mach_from_speed


def yaw_of_repose_estimate(speed, h, p_spin, flight_path_angle, p: Params) -> float:
    if speed < 50.0:
        return 0.0
    rho = density(h, p)
    M = mach_from_speed(speed, h, p)
    _, _, Cm_alpha, _, _, _, _ = body_aero_coeffs(M, p)
    Cm_a = abs(Cm_alpha) + 1e-6
    numer = 2.0 * p.Ixx * p_spin * p.g0 * np.cos(flight_path_angle)
    denom = rho * (speed ** 3) * p.ref_area * p.diameter * Cm_a
    return float(np.clip(numer / (denom + 1e-12), -np.radians(25), np.radians(25)))


def open_loop_summary(log, p: Params) -> dict:
    x, y, z, t = map(np.array, (log["x"], log["y"], log["z"], log["t"]))
    return {
        "range_m": float(np.hypot(x[-1], y[-1])),
        "drift_m": float(y[-1]),
        "impact_x": float(x[-1]),
        "impact_y": float(y[-1]),
        "tof_s": float(t[-1]),
        "apogee_m": float(np.max(z)),
        "max_aoa_deg": float(np.max(np.abs(log["aoa"]))),
        "final_spin_rps": float(log["p_rps"][-1]),
    }


def compare_to_ballpark(summary, p: Params) -> None:
    print("\n--- Open-loop validation (ballpark 155 mm) ---")
    print(f"  Range     : {summary['range_m']:.0f} m   (expect ~12-22 km)")
    print(f"  Drift (Y) : {summary['drift_m']:.0f} m")
    print(f"  ToF       : {summary['tof_s']:.1f} s")
    print(f"  Apogee    : {summary['apogee_m']:.0f} m")
    print(f"  Max |AoA| : {summary['max_aoa_deg']:.1f} deg")
    print(f"  Final spin: {summary['final_spin_rps']:.0f} rps")
    if summary["range_m"] < 8000 or summary["range_m"] > 28000:
        print("  WARNING: range outside typical band")
    if summary["max_aoa_deg"] > 20:
        print("  WARNING: high AoA")

</file>

<file src="run_all.py">
#!/usr/bin/env python3
"""
One-shot: open-loop + guided + Monte Carlo + figures.

Usage (from the code/ directory):
  python run_all.py
  python run_all.py --mc 10          # fewer MC runs (faster)
  python run_all.py --mc 50          # formal CEP sample
  python run_all.py --no-gif         # skip GIF generation
  python run_all.py --no-mc          # skip Monte Carlo
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_validation import open_loop_summary, compare_to_ballpark
from pgk_montecarlo import run_monte_carlo, print_cep_report
from pgk_control_budget import print_control_budget


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=10, help="Monte Carlo runs (default 10; use 50 for formal CEP)")
    ap.add_argument("--no-mc", action="store_true")
    ap.add_argument("--no-gif", action="store_true")
    ap.add_argument("--out", type=str, default=".")
    args = ap.parse_args()

    t0 = time.time()

    # --- Open-loop ---
    print("=== 1) Open-loop ===")
    p_ol = Params()
    p_ol.dt = 0.0025
    p_ol.control_dt = 0.03
    p_ol.arm_time = 9999.0
    p_ol.azimuth_bias_deg = 0.0
    p_ol.gps_enable = False
    p_ol.use_kalman = False
    p_ol.use_met_layers = False
    p_ol.wind = np.zeros(3)
    log_ol = Simulator(p_ol).run(log_every=100)
    s = open_loop_summary(log_ol, p_ol)
    compare_to_ballpark(s, p_ol)

    # --- Guided truth ---
    print("\n=== 2) Guided (truth nav) ===")
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.target_x = s["impact_x"] + 100.0
    p.target_y = s["impact_y"] - 70.0
    p.azimuth_bias_deg = 0.35
    p.gps_enable = False
    p.use_kalman = False
    p.use_met_layers = False
    p.wind = np.zeros(3)
    p.fail_safe_miss = 5000.0
    log = Simulator(p).run(log_every=80)
    tgt = np.array([p.target_x, p.target_y])
    imp = np.array([log["x"][-1], log["y"][-1]])
    miss = float(np.linalg.norm(imp - tgt))
    ol_miss = float(np.hypot(100.0, 70.0))
    cn = p.canard_Cl_max * p.canard_authority * p.canard_area / p.ref_area
    print(f"  CN≈{cn:.4f}  a_max={p.canard_a_max}")
    print(f"  Guided miss: {miss:.1f} m   OL miss: {ol_miss:.1f} m   gain: {ol_miss-miss:+.1f} m")
    print_control_budget(log, arm_time=p.arm_time)

    # --- Guided GPS ---
    print("\n=== 3) Guided (GPS σ=2 m) ===")
    p2 = Params()
    p2.dt = 0.0025
    p2.control_dt = 0.03
    p2.target_x, p2.target_y = p.target_x, p.target_y
    p2.azimuth_bias_deg = 0.35
    p2.gps_enable = True
    p2.use_kalman = True
    p2.gps_noise_sigma = 2.0
    p2.gps_dropout_prob = 0.0
    p2.use_met_layers = False
    p2.wind = np.zeros(3)
    p2.fail_safe_miss = 5000.0
    log2 = Simulator(p2).run(log_every=80)
    miss2 = float(np.hypot(log2["x"][-1] - tgt[0], log2["y"][-1] - tgt[1]))
    print(f"  GPS guided miss: {miss2:.1f} m")

    # --- Monte Carlo ---
    if not args.no_mc and args.mc > 0:
        print(f"\n=== 4) Monte Carlo N={args.mc} ===")
        base = Params()
        base.dt = 0.0025
        base.control_dt = 0.03
        base.target_x, base.target_y = p.target_x, p.target_y
        base.azimuth_bias_deg = 0.35
        base.gps_enable = True
        base.use_kalman = True
        base.use_met_layers = False
        base.fail_safe_miss = 5000.0
        res = run_monte_carlo(base, n_runs=args.mc, seed=42, guided=True)
        print_cep_report(res)

    # --- Figures ---
    print("\n=== 5) Figures ===")
    from generate_figures import fig_suite, fig_workflow, fig_real_motion_full, fig_ji_style, gif_canards_local
    from pathlib import Path

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fig_suite(out)
    fig_workflow(out)
    try:
        fig_real_motion_full(out)
    except Exception as e:
        print("  real_motion_full skipped:", e)
    try:
        fig_ji_style(out)
    except Exception as e:
        print("  ji_style skipped:", e)
    if not args.no_gif:
        try:
            gif_canards_local(out)
        except Exception as e:
            print("  GIF skipped:", e)

    print(f"\nDone in {time.time()-t0:.1f} s")
    print(f"Truth miss {miss:.1f} m | GPS miss {miss2:.1f} m")


if __name__ == "__main__":
    main()
</file>

<file src="test_fuze.py">
#!/usr/bin/env python3
"""
Multi-mode electronic fuze test simulation.

Modes (problem statement):
  IMPACT    — fire on ground contact
  TIME      — fire at programmed time from setback
  PROXIMITY — fire when height AGL <= prox_height (fallback: impact)

Usage:
  python test_fuze.py
  python test_fuze.py --mode time --time 25
  python test_fuze.py --mode prox --prox-h 50
  python test_fuze.py --all
"""

from __future__ import annotations

import argparse
import numpy as np

from pgk_params import Params
from pgk_simulator import Simulator
from pgk_fuze import MultiModeFuze, FuzeMode
from pgk_validation import open_loop_summary


def _base_ol_params():
    p = Params()
    p.dt = 0.0025
    p.control_dt = 0.03
    p.arm_time = 9999.0          # ballistic (no guidance needed for fuze test)
    p.azimuth_bias_deg = 0.0
    p.gps_enable = False
    p.use_kalman = False
    p.use_met_layers = False
    p.wind = np.zeros(3)
    p.tmax = 90.0  # allow full descent for impact/prox tests
    return p


def run_fuze_case(mode: FuzeMode, time_s: float = 30.0, prox_h: float = 50.0,
                  label: str = ""):
    p = _base_ol_params()
    sim = Simulator(p)
    # Program fuze BEFORE flight (pre-fire interface)
    ok = sim.fuze.program(mode, time_s=time_s, prox_height_m=prox_h)
    assert ok, "fuze.program failed (already launched?)"

    # Instrument: log fuze state each physics step via thin wrapper
    log = {k: [] for k in ["t", "z", "armed", "fired", "mode"]}
    # Run with denser logging by monkey-patching step loop is heavy;
    # instead run normal and poll fuze at log points via custom loop.

    elev = np.radians(p.elevation_deg)
    az = 0.0
    vm = p.muzzle_velocity
    s = np.zeros(15)
    s[0:3] = [0.0, 0.0, p.launch_height]
    s[3:6] = [vm * np.cos(elev), 0.0, vm * np.sin(elev)]
    from pgk_math import euler_to_q, q_normalize
    s[6:10] = euler_to_q(0.0, elev, az)
    s[10] = 2 * np.pi * p.initial_spin_rps
    s[14] = 2 * np.pi * p.initial_spin_rps

    sim.gps = __import__("pgk_nav", fromlist=["GPSReceiver"]).GPSReceiver(p)
    t = 0.0
    n = int(np.ceil(p.tmax / p.dt))
    fired_t = None
    fired_h = None

    for i in range(n + 1):
        st = sim.fuze.step(t, float(s[2]), impact_contact=(s[2] < 0.0 and t > 1.5))
        if i % 40 == 0:
            log["t"].append(t)
            log["z"].append(float(s[2]))
            log["armed"].append(1.0 if st["armed"] else 0.0)
            log["fired"].append(1.0 if st["fired"] else 0.0)
            log["mode"].append(st["mode"])
        if st["fired"] and fired_t is None:
            fired_t = t
            fired_h = float(s[2])
            break
        if s[2] < 0.0 and t > 1.5:
            break
        s = sim.step(t, s, p.dt, theta_c=0.0, free_spin=True)
        t += p.dt

    name = label or mode.name
    print(f"\n--- Fuze mode: {name} ---")
    print(f"  Programmed : mode={mode.name}, time={time_s}s, prox_h={prox_h}m")
    print(f"  Safe delay : {sim.fuze.safe_delay_s}s after setback")
    if fired_t is not None:
        print(f"  FIRED at t = {fired_t:.2f} s,  h = {fired_h:.1f} m")
    else:
        print("  Did NOT fire before end of trajectory")
        print(f"  Last t={t:.1f}s  h={s[2]:.1f}m  armed={sim.fuze.armed}")
    return {
        "mode": mode.name,
        "fired_t": fired_t,
        "fired_h": fired_h,
        "log": log,
        "fuze": sim.fuze,
    }


def plot_fuze(results, out="fuze_test.png"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("Plot skipped:", e)
        return

    fig, axes = plt.subplots(len(results), 1, figsize=(10, 3.2 * len(results)), sharex=True)
    if len(results) == 1:
        axes = [axes]
    for ax, res in zip(axes, results):
        lg = res["log"]
        t = np.array(lg["t"])
        z = np.array(lg["z"])
        ax.plot(t, z, "C0-", lw=1.5, label="altitude")
        ax.plot(t, np.array(lg["armed"]) * (np.max(z) * 0.15), "g--", alpha=0.7, label="armed (scaled)")
        if res["fired_t"] is not None:
            ax.axvline(res["fired_t"], color="r", lw=2, label=f"FIRE t={res['fired_t']:.1f}s")
            ax.scatter([res["fired_t"]], [res["fired_h"]], c="r", s=60, zorder=5)
        ax.set_ylabel("h (m)")
        ax.set_title(f"Fuze: {res['mode']}")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t (s)")
    plt.tight_layout()
    plt.savefig(out, dpi=130)
    plt.close()
    print(f"\nSaved {out}")




def run_inhibit_demo():
    """PGK-style: huge predicted miss → no detonation even on impact."""
    print("\n--- Fuze: PGK detonation INHIBIT demo ---")
    p = _base_ol_params()
    fuze = MultiModeFuze(p)
    fuze.program(FuzeMode.IMPACT)
    fuze.inhibit_miss_m = 150.0
    fuze.inhibit_check_time_s = 5.0
    for tt in np.linspace(0.0, 8.0, 40):
        fuze.step(tt, h=2000.0, impact_contact=False, predicted_miss_m=500.0)
    print(f"  After t>5s with pred_miss=500m: inhibit={fuze.detonate_inhibit} armed={fuze.armed}")
    st = fuze.step(60.0, h=-0.1, impact_contact=True, predicted_miss_m=500.0)
    print(f"  At impact with inhibit: fired={st['fired']} (expect False)")
    return {
        "mode": "INHIBIT",
        "fired_t": None,
        "fired_h": None,
        "log": {"t": [0, 5, 60], "z": [0, 2000, 0], "armed": [0, 0, 0], "fired": [0, 0, 0], "mode": [0, 0, 0]},
        "fuze": fuze,
        "inhibit": fuze.detonate_inhibit,
    }


def main():
    ap = argparse.ArgumentParser(description="Test multi-mode electronic fuze")
    ap.add_argument("--mode", choices=["impact", "time", "prox", "inhibit", "all"], default="all")
    ap.add_argument("--time", type=float, default=25.0, help="TIME mode function time (s)")
    ap.add_argument("--prox-h", type=float, default=80.0, help="PROXIMITY burst height (m)")
    ap.add_argument("--out", type=str, default="fuze_test.png")
    args = ap.parse_args()

    print("=== Multi-mode electronic fuze simulation ===")
    print("Safe–arm + PGK inhibit if early pred miss > ~150 m.")

    results = []
    if args.mode in ("impact", "all"):
        results.append(run_fuze_case(FuzeMode.IMPACT, label="IMPACT"))
    if args.mode in ("time", "all"):
        results.append(run_fuze_case(FuzeMode.TIME, time_s=args.time, label=f"TIME ({args.time}s)"))
    if args.mode in ("prox", "all"):
        results.append(run_fuze_case(FuzeMode.PROXIMITY, prox_h=args.prox_h,
                                     label=f"PROXIMITY (h≤{args.prox_h}m)"))
    if args.mode in ("inhibit", "all"):
        results.append(run_inhibit_demo())

    plot_results = [r for r in results if r.get("mode") != "INHIBIT"]
    if plot_results:
        plot_fuze(plot_results, out=args.out)

    print("\n=== Summary ===")
    for r in results:
        if r.get("mode") == "INHIBIT":
            print(f"  INHIBIT       detonate_inhibit={r.get('inhibit')}  fired={r['fuze'].fired}")
            continue
        ft = f"{r['fired_t']:.2f}s" if r["fired_t"] is not None else "NO FIRE"
        fh = f"{r['fired_h']:.1f}m" if r["fired_h"] is not None else "-"
        print(f"  {r['mode']:12s}  fire @ {ft:>10s}  h={fh}")


if __name__ == "__main__":
    main()

</file>

</codebase>
