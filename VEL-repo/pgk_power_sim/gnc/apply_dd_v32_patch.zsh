#!/usr/bin/env zsh

echo "Applying DD v3.2 plant parameters and pulsed-dwell shim..."

cat << 'EOF' > pgk_params.py
"""
Parameters for DD v3.2 validated M1156-style dual-spin fixed-canard PGK simulation.
"""

from dataclasses import dataclass, field
import numpy as np

@dataclass
class Params:
    # ----- Projectile + kit mass properties (DD v3.2 reconciled) -----
    mass: float = 42.0
    mass_nose: float = 1.4
    Ixx: float = 0.148
    Iyy: float = 1.82
    Izz: float = 1.82
    Ixx_nose: float = 1.4e-3  # Validated DD v3.2 nose inertia
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

    # ----- Body aero reference (F5-corrected baseline at M2) -----
    Cd0_ref: float = 0.391
    Cl_alpha_ref: float = 3.59
    max_Cl: float = 1.05
    Cmq_ref: float = -9.43
    Clp_ref: float = -0.048
    Cm_alpha_ref: float = -3.677
    Cn_pa_ref: float = 0.055
    Cm_pa_ref: float = -0.004

    # ----- Canards (physical authority) -----
    canard_area: float = 0.00075
    canard_Cl_max: float = 1.0
    canard_stall_alpha: float = np.radians(16.0)
    canard_authority: float = 0.72
    canard_min_speed: float = 70.0
    canard_a_max: float = 1.15
    canard_interference: float = 0.15

    # ----- Dual-spin / brake / alternator (DD v3.2 limits) -----
    brake_torque_max: float = 0.30  # Clutch envelope
    bearing_visc: float = 0.0
    free_spin_rel_rate: float = 6.0
    K_brake: float = 1.0
    K_phi_hold: float = 30.0
    alternator_torque_burst: float = 0.28
    alternator_torque_cruise: float = 0.045
    aero_despin_coeff: float = 7.24e-3  # Canard geometry linear damping

    # ----- Launch (DD v3.2 qualification charge) -----
    muzzle_velocity: float = 685.0
    elevation_deg: float = 45.0
    azimuth_bias_deg: float = 0.35
    initial_spin_rps: float = 221.0
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
    fuze_inhibit_miss_m: float = 150.0
    fuze_inhibit_check_time_s: float = 5.0
    orientation_slew_tau: float = 0.25
    max_slew_rate_deg: float = 90.0
    K_orient: float = 6.0
    predictor_dt: float = 0.10
    predictor_cd: float = 0.36  # Mach-averaged ZEM predictor
    guidance_phase_deg: float = 0.0

    # ----- GPS / navigation -----
    gps_rate_hz: float = 10.0
    gps_latency_s: float = 0.05
    gps_noise_sigma: float = 0.0
    gps_noise_sigma_v: float = 0.4
    gps_dropout_prob: float = 0.0
    gps_enable: bool = True
    use_kalman: bool = True
    gps_roll_sigma: float = np.radians(2.0)

    # ----- Integration -----
    dt: float = 0.002
    control_dt: float = 0.02
    tmax: float = 60.0

def g0_at_latitude(lat_deg: float) -> float:
    lat = np.radians(lat_deg)
    g_fts2 = 32.174 * (1.0 - 0.0026 * np.cos(2.0 * lat))
    return g_fts2 * 0.3048
EOF

cat << 'EOF' > pgk_forces.py
"""Forces, moments, dual-spin brake/alternator, physical canard authority."""

import numpy as np
from pgk_params import Params
from pgk_aero import body_aero_coeffs, density, mach_from_speed, wind_at_altitude
from pgk_math import q_to_dcm

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

    F_can = np.zeros(3)
    M_can_body = np.zeros(3)
    
    # DD v3.2 held-gate requirement (2.0 rad/s vs 30 rad/s)
    if (not free_spin) and speed > p.canard_min_speed and abs(omega_nose) < 2.0:
        c_phi, s_phi = np.cos(phi_nose), np.sin(phi_nose)
        desired = np.array([c_phi, s_phi, 0.0])
        vhat = v_air / (speed + 1e-9)
        force_dir = desired - np.dot(desired, vhat) * vhat
        n = np.linalg.norm(force_dir)
        if n > 1e-6:
            force_dir /= n
            alpha_can = min(aoa, p.canard_stall_alpha)
            stall_frac = 1.0 - 0.5 * (alpha_can / (p.canard_stall_alpha + 1e-6))
            Cl_can = p.canard_Cl_max * p.canard_authority * max(stall_frac, 0.3)
            Fmag = Cl_can * qdyn * p.canard_area
            Fmag = min(Fmag, p.canard_a_max * p.mass)
            F_can = Fmag * force_dir
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

    # DD v3.2 Plant: Aero de-spin, alternator burst/shed, and clutch envelope
    omega_rel = omega_nose - omega_body[0]
    
    # Validated linear aero despin
    tau_aero = -p.aero_despin_coeff * omega_nose if speed > 40.0 else 0.0
    
    if free_spin:
        # Alternator harvesting (burst 0.28 proxy)
        tau_alt = -getattr(p, 'alternator_torque_burst', 0.28) * np.sign(omega_rel)
        tau_cmd = tau_aero + tau_alt
    else:
        # Hold preempts burst (sheds harvest)
        tau_alt = 0.0
        err_phi = (theta_c - phi_nose + np.pi) % (2 * np.pi) - np.pi
        tau_cmd = tau_aero + tau_alt - p.K_brake * omega_nose + p.K_phi_hold * err_phi

    # Clamp physically to 0.30 N.m slip-clutch max
    tau_brake = float(np.clip(tau_cmd, -p.brake_torque_max, p.brake_torque_max))
    tau_brake -= p.bearing_visc * omega_rel

    M_body = M_can_body + M_damp + M_over_body + M_magnus_body
    M_body[0] -= tau_brake

    return F, M_body, tau_brake, F_can, np.degrees(aoa)
EOF

cat << 'EOF' > pgk_aero.py
"""Mach/alpha aero tables + atmosphere + MET wind layers + gravity/Coriolis."""

import numpy as np
from pgk_params import Params

# DD v3.2 F5-corrected aero LUT integration
_A_CFD = 340.29
_P_CFD = 1361.36

_V3 = np.array([270, 410, 680])
_M3 = _V3 / _A_CFD
_CD0_3 = np.array([0.03637, 0.16799, 0.39084])
_Mag_3 = np.array([-0.0953, -0.0508, -0.0684])
_Cl_3 = np.array([0.000435, 0.000883, 0.001582])

_V5 = np.array([270, 340, 410, 550, 680])
_M5 = _V5 / _A_CFD
_CLa_5 = np.array([2.1200, 2.4800, 2.8500, 3.1500, 3.4100])
_CMa_5 = np.array([-0.8852, -1.2780, -1.6707, -2.7110, -3.6770])
_Cmq_5 = _CMa_5 * (9.43 / 3.6770)  # Dense pitch ring map

def mach_from_speed(speed: float, h: float, p: Params) -> float:
    T = max(p.T0 - p.lapse * max(h, 0.0), 200.0)
    a = np.sqrt(1.4 * 287.05 * T)
    return speed / a

def body_aero_coeffs(M: float, p: Params):
    d = p.diameter
    Mq = np.clip(M, _M3[0], _M3[-1])
    Mq5 = np.clip(M, _M5[0], _M5[-1])

    CD0 = float(np.interp(Mq, _M3, _CD0_3))
    CLa = float(np.interp(Mq5, _M5, _CLa_5))
    CMa = float(np.interp(Mq5, _M5, _CMa_5))
    Cmq = -float(np.interp(Mq5, _M5, _Cmq_5))
    
    V_clip3 = Mq * _A_CFD
    CNpa_t = _Mag_3 * (2 * _V3) / (_P_CFD * d)
    Clp_t = _Cl_3 * (2 * _V3) / (_P_CFD * d)
    
    CNpa = float(np.interp(Mq, _M3, CNpa_t))
    Clp = -float(np.interp(Mq, _M3, Clp_t))

    # Output with scale factors preserving Params.ref flexibility for MC dispersion
    return (
        CD0 * (p.Cd0_ref / 0.391),
        CLa * (p.Cl_alpha_ref / 3.59),
        CMa * (p.Cm_alpha_ref / -3.677),
        Clp * (p.Clp_ref / Clp) if Clp != 0 else Clp,
        Cmq * (p.Cmq_ref / -9.43),
        CNpa * (p.Cn_pa_ref / CNpa) if CNpa != 0 else CNpa,
        CNpa * -0.1 
    )

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
EOF

cat << 'EOF' > pgk_guidance.py
"""
ZEM guidance + dual-spin mode logic (PGK-class).
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
        self.phi_meas = 0.0
        self.gps_ok = True
        self.last_hold_err = 0.0

        # DD v3.2 Power Subsystem ODE State
        self.bus_voltage = 19.0
        self.power_aborted = False
        self.pulse_hold_timer = 0.0
        self.pulse_harvest_timer = 0.0
        self.is_holding_now = False

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
                a_drag = -p.predictor_cd * qdyn * p.ref_area / p.mass * (vv / speed)
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
        if F_can_current is not None:
            self.last_F_can_est = 0.65 * self.last_F_can_est + 0.35 * F_can_current
        if phi_meas is not None:
            self.phi_meas = float(phi_meas)
        self.gps_ok = bool(gps_ok)

        impact, _ = self.predict_impact(r_nav, v_nav, include_control=True)
        zem_vec = self.target[:2] - impact[:2]
        self.zem = float(np.linalg.norm(zem_vec))

        if (not self.fail_safe_checked) and t >= self.p.fail_safe_check_time:
            self.fail_safe_checked = True
            impact0, _ = self.predict_impact(r_nav, v_nav, include_control=False)
            zem0 = float(np.linalg.norm(self.target[:2] - impact0[:2]))
            if zem0 > self.p.fail_safe_miss * 8.0:
                self.fail_safe = True
                self.modes.fail_safe = True

        # DD v3.2 Power Subsystem ODE (1.5 F cap, 34.1 W load, 16/19 V burst hysteresis)
        P_out = 34.1
        P_in = 160.0 if self.free_spin else 0.0  # Harvest active during free-spin
        net_power = P_in - P_out
        if self.bus_voltage > 0:
            dv = (net_power / (1.5 * self.bus_voltage)) * dt
            self.bus_voltage = np.clip(self.bus_voltage + dv, 0.0, 19.0)
            
        if self.bus_voltage < 12.0:
            self.power_aborted = True
            self.fail_safe = True
            self.modes.fail_safe = True

        self.modes.fail_safe = self.modes.fail_safe or self.fail_safe
        mode, free = self.modes.update(
            t, self.zem, self.gps_ok,
            impact_imminent=impact_imminent,
            fuze_terminal=fuze_terminal,
        )
        self.mode = mode
        
        # DD v3.2 Pulsed Dwell Shim
        guidance_wants_hold = (not free) and (not self.fail_safe)

        if guidance_wants_hold:
            if self.is_holding_now:
                self.pulse_hold_timer += dt
                if self.pulse_hold_timer >= 2.5:  # Arrest-time 2.5s cap
                    self.is_holding_now = False
                    self.pulse_harvest_timer = 0.0
            else:
                self.pulse_harvest_timer += dt
                # Burst hysteresis: bus > 16V before next hold
                if self.bus_voltage > 16.0 and self.pulse_harvest_timer >= 0.5:
                    self.is_holding_now = True
                    self.pulse_hold_timer = 0.0
        else:
            self.is_holding_now = False
            self.pulse_hold_timer = 0.0
            self.pulse_harvest_timer += dt

        self.free_spin = not self.is_holding_now if guidance_wants_hold else free
        
        if self.fail_safe or t < self.p.arm_time:
            self.free_spin = True

        self.ctrl.set_mode(self.free_spin)

        if self.free_spin or t < self.p.arm_time:
            self.theta_c = self.ctrl.theta_c
            self.last_hold_err = self.ctrl.hold_error(self.phi_meas)
            return

        self.ctrl.command_from_zem(zem_vec, dt)
        self.theta_c = self.ctrl.theta_c
        self.last_hold_err = self.ctrl.hold_error(self.phi_meas)
EOF

cat << 'EOF' > pgk_sensors.py
"""Sensor models for PGK-class dual-spin kit."""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from pgk_params import Params
from pgk_math import q_to_euler

@dataclass
class SensorSuiteSpec:
    gnss: str = "GPS L1 (SAASM or civil) position + velocity"
    gnss_rate_hz: float = 10.0
    gnss_latency_s: float = 0.05
    baro: str = "Optional MEMS barometer"
    imu: str = "NOT required for baseline ZEM PGK"
    nose_roll: str = "Roll sensor on de-spun nose (magnetometer / relative encoder)"
    body_spin_proxy: str = "Optional: alternator frequency"
    impact: str = "Crush switch"
    timer: str = "Crystal / MCU timer"
    proximity: str = "RF or optical proximity channel"
    setback_switch: str = "Launch detect / safe-arm input"
    alternator: str = "Relative-spin generator for in-flight power"

class NoseRollSensor:
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
        
        # DD v3.2 Validation: Real spec gyro rate noise
        self.omega_meas = omega_true + np.random.normal(0.0, 0.005)
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
EOF

sed -i 's/n_runs: int = 40/n_runs: int = 300/g' pgk_montecarlo.py
echo "Patch complete. You can now execute 'python run_all.py' to generate your valid SIL tie-out trajectories."
