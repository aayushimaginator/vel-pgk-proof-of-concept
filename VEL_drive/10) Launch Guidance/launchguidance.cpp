/**
 * artillery_fcs_4dof.hpp
 * 4DOF ENU fire control: x,y,z + axial spin p (rad/s)
 * Forces: drag, gravity, Coriolis, yaw-of-repose lateral lift
 * Spin: muzzle from rifling, then roll-damping decay (Clp)
 *
 * Azimuth: 0° = North, 90° = East (ENU)
 */
#pragma once

#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct FcsConfig {
  float mass_kg   = 43.0f;
  float diam_m    = 0.155f;
  float area_m2   = 0.0f;       // 0 => auto
  float Ix        = 0.15f;      // axial inertia
  float Cd        = 0.22f;
  float Cl_alpha  = 2.4f;       // 1/rad
  float Cm_alpha  = 1.8f;       // 1/rad (use |Cmα|)
  float Clp       = -0.045f;    // roll damping (neg)
  float twist_cal = 20.0f;      // calibers / turn
  float g         = 9.80665f;
  float rho       = 1.225f;
  float V0        = 680.0f;
  float lat_deg   = 15.0f;
  float dt        = 0.025f;
  float yaw_repose_sign = 1.0f; // flip if drift side wrong
  uint8_t use_coriolis   = 1;
  uint8_t use_yaw_repose = 1;
};

struct Vec3 { float x, y, z; };

struct FireSolution {
  float elev_deg, az_deg, tof_s;
  float impact_E, impact_N;
  float impact_spin_rps;  // p at impact
  float miss_m;
  uint8_t ok;
};

inline float vlen(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3{
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

class ArtilleryFCS4 {
public:
  FcsConfig cfg;
  Vec3 Omega{};

  void init(const FcsConfig& c) {
    cfg = c;
    if (cfg.area_m2 <= 0.f)
      cfg.area_m2 = (float)M_PI * 0.25f * cfg.diam_m * cfg.diam_m;
    if (cfg.dt < 0.005f) cfg.dt = 0.025f;

    const float om  = 7.292115e-5f;
    const float phi = cfg.lat_deg * (float)M_PI / 180.f;
    Omega = Vec3{0.f, om * cosf(phi), om * sinf(phi)};
  }

  /** Muzzle axial spin from rifling [rad/s] */
  float muzzle_spin(float V) const {
    const float pitch = cfg.twist_cal * cfg.diam_m;
    return (2.f * (float)M_PI * V) / (pitch + 1e-9f);
  }

  /**
   * 4DOF integrate: r(3), v(3), p_spin(1)
   * p_dot from roll damping; yaw-of-repose uses current p.
   */
  void trajectory(float elev_deg, float az_deg,
                  float& iE, float& iN, float& tof, float& p_imp) const {
    const float el = elev_deg * (float)M_PI / 180.f;
    const float az = az_deg   * (float)M_PI / 180.f;

    Vec3 v{
      cfg.V0 * cosf(el) * sinf(az),
      cfg.V0 * cosf(el) * cosf(az),
      cfg.V0 * sinf(el)
    };
    Vec3  r{0.f, 0.f, 0.f};
    float p = muzzle_spin(cfg.V0);  // 4th DOF state
    float t = 0.f;

    const float S = cfg.area_m2;
    const float d = cfg.diam_m;
    const float tmax = 160.f;

    Vec3  r0 = r;
    float t0 = 0.f;
    float p0 = p;

    while (t < tmax) {
      r0 = r; t0 = t; p0 = p;

      const float V = vlen(v);
      if (V < 1.f) break;

      const float q    = 0.5f * cfg.rho * V * V;
      const float invV = 1.f / V;
      const Vec3  uv{v.x * invV, v.y * invV, v.z * invV};

      // --- Translational acceleration ---
      const float kdrag = (q * S * cfg.Cd) / cfg.mass_kg;
      Vec3 a{-kdrag * uv.x, -kdrag * uv.y, -kdrag * uv.z - cfg.g};

      if (cfg.use_coriolis) {
        const Vec3 oc = cross(Omega, v);
        a.x -= 2.f * oc.x;
        a.y -= 2.f * oc.y;
        a.z -= 2.f * oc.z;
      }

      // Yaw of repose from CURRENT spin p (4th DOF)
      if (cfg.use_yaw_repose) {
        const float vh = sqrtf(v.x * v.x + v.y * v.y);
        const float th = atan2f(v.z, vh + 1e-6f);
        const float Ma = q * S * d * fabsf(cfg.Cm_alpha) + 1e-9f;
        float aR = (cfg.Ix * p * cfg.g * cosf(th)) / (Ma * V);
        aR *= cfg.yaw_repose_sign;

        float rx = 1.f, ry = 0.f;
        if (vh > 1.f) {
          rx =  v.y / vh;
          ry = -v.x / vh;
        }
        const float am = (q * S * cfg.Cl_alpha * aR) / cfg.mass_kg;
        a.x += am * rx;
        a.y += am * ry;
      }

      // --- Axial spin dynamics (4th DOF) ---
      // Mx = q S d * Clp * (p d / (2 V))
      // p_dot = Mx / Ix
      const float pbar = p * d / (2.f * V);
      const float Mx   = q * S * d * cfg.Clp * pbar;
      const float pdot = Mx / (cfg.Ix + 1e-12f);

      // Euler–Cromer
      v.x += a.x * cfg.dt;
      v.y += a.y * cfg.dt;
      v.z += a.z * cfg.dt;
      r.x += v.x * cfg.dt;
      r.y += v.y * cfg.dt;
      r.z += v.z * cfg.dt;
      p   += pdot * cfg.dt;
      t   += cfg.dt;

      if (r.z < 0.f && t > 0.4f) {
        const float dz = r.z - r0.z;
        const float f  = (fabsf(dz) < 1e-8f) ? 1.f : (r0.z / (r0.z - r.z));
        iE   = r0.x + f * (r.x - r0.x);
        iN   = r0.y + f * (r.y - r0.y);
        tof  = t0   + f * (t  - t0);
        p_imp = p0  + f * (p  - p0);
        return;
      }
    }
    iE = r.x; iN = r.y; tof = t; p_imp = p;
  }

  FireSolution solve(float tgt_E, float tgt_N) const {
    FireSolution s{};
    s.ok = 0;

    float az = atan2f(tgt_E, tgt_N) * 180.f / (float)M_PI;
    float el = 35.f;
    float iE = 0.f, iN = 0.f, tof = 0.f, pimp = 0.f;

    for (int it = 0; it < 14; ++it) {
      trajectory(el, az, iE, iN, tof, pimp);
      const float eE = tgt_E - iE;
      const float eN = tgt_N - iN;
      const float miss = sqrtf(eE * eE + eN * eN);

      s.elev_deg = el; s.az_deg = az; s.tof_s = tof;
      s.impact_E = iE; s.impact_N = iN;
      s.impact_spin_rps = pimp / (2.f * (float)M_PI);
      s.miss_m = miss;

      if (miss < 3.f) { s.ok = 1; return s; }

      const float he = 0.12f, ha = 0.12f;
      float iE1, iN1, iE2, iN2, t1, pp;
      trajectory(el + he, az, iE1, iN1, t1, pp);
      trajectory(el, az + ha, iE2, iN2, t1, pp);

      const float J00 = (iE1 - iE) / he, J10 = (iN1 - iN) / he;
      const float J01 = (iE2 - iE) / ha, J11 = (iN2 - iN) / ha;
      const float det = J00 * J11 - J01 * J10;

      float del = 0.f, daz = 0.f;
      if (fabsf(det) > 1e-5f) {
        del = ( J11 * eE - J01 * eN) / det;
        daz = (-J10 * eE + J00 * eN) / det;
      } else {
        const float rt = sqrtf(tgt_E * tgt_E + tgt_N * tgt_N);
        const float ri = sqrtf(iE * iE + iN * iN);
        del = (rt > ri) ? 0.3f : -0.3f;
        daz = atan2f(eE, rt + 1.f) * 180.f / (float)M_PI;
      }

      del *= 0.65f; daz *= 0.65f;
      if (del > 4.f) del = 4.f; if (del < -4.f) del = -4.f;
      if (daz > 4.f) daz = 4.f; if (daz < -4.f) daz = -4.f;

      el += del; az += daz;
      if (el < 5.f) el = 5.f;
      if (el > 72.f) el = 72.f;
    }
    s.ok = (s.miss_m < 40.f) ? 1 : 0;
    return s;
  }
};

/*
  FcsConfig c;
  c.V0 = 680.f; c.lat_deg = 15.f;
  c.use_coriolis = 1; c.use_yaw_repose = 1;
  ArtilleryFCS4 fcs; fcs.init(c);
  FireSolution s = fcs.solve(1000.f, 15000.f);
*/