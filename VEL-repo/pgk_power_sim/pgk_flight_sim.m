% ============================================================
% PGK FLIGHT SIM v7.6.1 - deterministic + parallel Monte Carlo
% Usage (deterministic):  octave --no-gui pgk_flight_sim.m
% Usage (MC worker):      octave --no-gui pgk_flight_sim.m <slice> <n_runs> <seed> <out.mat>
% v7.6: 2-DoF lofted trajectory + ISA density (closes flat-atmo range gap);
%       impact detection; authority scaled by true time-to-go; vsc window
%       ends at impact. Carries v7.5 3-table aero + F5 and v7.3 power/control.
% ============================================================
clear all; close all; clc;
args = argv();
WORKER = (length(args) >= 4);

if WORKER
  slice_id = str2double(args{1});
  N_MC     = str2double(args{2});
  seed     = str2double(args{3});
  outfile  = args{4};
  rand('state', seed); randn('state', seed);
  RUN_MC = true;
else
  RUN_MC = false;
  N_MC   = 200;
end

SCENARIO = 'NOMINAL';   % 'NOMINAL' | 'WEAK_FINS' | 'AGGRESSIVE_FINS'

% ---------- A. FLIGHT / MECH ----------
V0_base = 685; twist_len = 3.1; m_shell = 42;
J_shell = 0.126; J_nose = 1.4e-3; T_clutch = 0.300;
g = 9.81;
launch_elev_deg = 45.0;              % M107 max-range elevation (brochure 18 km)
rho_isa = @(h) 1.225*((288.15 - 0.0065*min(max(h,0),11000))/288.15)^4.256;

fin_shell_A = 357e-6; fin_shell_R = 0.0325; fin_shell_N = 2; fin_shell_delta = 7.5*pi/180;
fin_nose_A  = 0.002;  fin_nose_R  = 0.030;  fin_nose_N  = 4; fin_nose_trim   = 0.05*pi/180;
fin_CLa = 2.4;
if strcmp(SCENARIO, 'WEAK_FINS'),       fin_CLa = fin_CLa*0.6; end
if strcmp(SCENARIO, 'AGGRESSIVE_FINS'), fin_CLa = fin_CLa*1.4; end

% ---------- B. POWER (PWM-connect buck harvest) ----------
poles = 8;
V_oc_ref = 26.8;  om_ref = 1397;
R_int = 1.33;  R_ntc = 0.30;  t_ntc = 1.0;
P_load = 30;  eta_buck = 0.88;  P_charge_margin = 8;
V_boot = 9.5;  V_gate = 16.0;  V_hyst = 3.0;  V_hold_min = 12.0;
C_sc = 1.5;  V_max = 28;  R_brk = 2.2;  T_hv_max = 0.280;

% ---------- C. AERO / ACTUATOR ----------
canard_max_deg  = 15.0;
steer_delta_deg = 1.0;

% ---------- D. CONTROL ----------
hold_cmd = 0.8;
K_dwell = 0.014;
W_HOLD_GATE = 2.0;
gyro_noise = 0.005;
WAKE_OFFSETS = [4 16 31];
STEER_FALLBACK = [8 20 35];

% ---------- E. CONFIG ----------
t_end = 70;                            % extended to capture impact at 45 deg
if RUN_MC, dt = 2e-3; else, dt = 1e-3; end
t = 0:dt:t_end; N = length(t);
Kp_pll = 888; Ki_pll = 3.95e5; Ts_pll = 1e-4; n_sub = 10;
USE_PLL = ~RUN_MC;

if RUN_MC
  tot_corr = zeros(N_MC,1); resid = zeros(N_MC,1); vsc_min = zeros(N_MC,1);
  runs = N_MC;
else
  runs = 1;
end

rho0 = 1.225; aoa_trim = 1.0;
ao = struct('ref','pgk','use270',true,'spin_scale',true);
Vg = (100:20:700)'; dV = 20; ng = numel(Vg);
Cd_g = zeros(ng,1); Cr_g = zeros(ng,1); Cf_g = zeros(ng,1);
for ig = 1:ng
  Ri = pgk_aero_model(aoa_trim, Vg(ig), 1361.36, ao);
  Cd_g(ig) = Ri.Cd; Cr_g(ig) = Ri.Croll; Cf_g(ig) = Ri.CmagF;
end
S_body = Ri.S_body; S_pk = Ri.S_pk; d_pk = Ri.d_pk;
steer_delta_rad = min(steer_delta_deg, canard_max_deg)*pi/180;

for mc = 1:runs
  if RUN_MC
    V0   = V0_base*(1+0.02*randn());
    wind = 3*randn();
    mc_rand_cd   = 1 + 0.1*randn();
    mc_rand_roll = 1 + 0.2*randn();
  else
    V0 = V0_base; wind = 0;
    mc_rand_cd = 1.0; mc_rand_roll = 1.0;
  end
  om_sh0 = (V0/twist_len)*2*pi;

  om_sh = zeros(1,N); om_ns = zeros(1,N); om_rel = zeros(1,N); th_ns = zeros(1,N);
  T_hv = zeros(1,N); T_br = zeros(1,N); T_drv = zeros(1,N); T_fin = zeros(1,N);
  qv = zeros(1,N); F_st = zeros(1,N); F_mag = zeros(1,N); delta_cmd = zeros(1,N);
  P_hv = zeros(1,N); P_br = zeros(1,N); duty_a = zeros(1,N);
  V_sc = zeros(1,N); Qsc = zeros(1,N); w_est = zeros(1,N);
  corr_dv = zeros(1,3); t_bank = zeros(1,3); corr = zeros(1,3);
  V = zeros(1,N); gam = zeros(1,N); hh = zeros(1,N); xx = zeros(1,N);
  V(1) = V0; gam(1) = launch_elev_deg*pi/180;
  om_sh(1) = om_sh0; om_ns(1) = om_sh0; Qsc(1) = 0; V_sc(1) = 0; dv = 0;
  th_pll = 0; pll_int = 0; w_pll = 0; th_rel = 0;
  steer_t = STEER_FALLBACK; dwell_dur = repmat(hold_cmd,1,3);
  despun = false; closed = false; booted = false; elec_on = false; burst_on = false;
  impacted = false; t_impact = t_end; x_impact = 0; h_apo = 0; k_stop = N-1;
  t_wake = NaN; t_conn = NaN; t_boot = NaN; t_bus_ok = NaN; schedule_set = false;
  gn = gyro_noise*randn(1,N);

  for k = 1:N-1
    tk = t(k);
    om_rel(k) = om_sh(k) - om_ns(k);

    % --- freestream with ISA density ---
    if impacted
      qk = 0;
    else
      qk = 0.5*rho_isa(hh(k))*V(k)^2;
    end
    qv(k) = qk;
    iG = min(max(floor((V(k)-Vg(1))/dV)+1, 1), ng-1);
    fG = (V(k)-Vg(iG))/dV;
    Cdk    = (Cd_g(iG) + fG*(Cd_g(iG+1)-Cd_g(iG))) * mc_rand_cd;
    Crollk = (Cr_g(iG) + fG*(Cr_g(iG+1)-Cr_g(iG))) * mc_rand_roll;
    CmagFk =  Cf_g(iG) + fG*(Cf_g(iG+1)-Cf_g(iG));
    rs = min(max(om_sh(k)/1361.36,0),1.5);
    T_drv(k) = qk*S_pk*d_pk*Crollk*rs;
    F_mag(k) = qk*S_pk*CmagFk*rs;

    % --- 2-DoF point-mass trajectory (L=0 ballistic arc) ---
    if ~impacted
      Ddrag = qk*S_body*Cdk;
      V(k+1)   = max(V(k) - (Ddrag/m_shell + g*sin(gam(k)))*dt, 30);
      gam(k+1) = gam(k) - (g*cos(gam(k))/max(V(k),50))*dt;
      hh(k+1)  = hh(k) + V(k)*sin(gam(k))*dt;
      xx(k+1)  = xx(k) + V(k)*cos(gam(k))*dt;
      if (k > 100) && (hh(k+1) < 0)
        impacted = true; t_impact = tk + dt; x_impact = xx(k+1);
        h_apo = max(hh(1:k+1)); k_stop = k;
      end
    else
      V(k+1) = V(k); gam(k+1) = gam(k); hh(k+1) = hh(k); xx(k+1) = xx(k);
    end

    % --- shell spin ODE (2 canted fins, 7.5 deg) ---
    T_shell_cant = fin_shell_N*qk*fin_shell_A*fin_CLa*fin_shell_delta*fin_shell_R;
    c_shell_damp = fin_shell_N*qk*fin_shell_A*fin_CLa*(fin_shell_R^2)/max(V(k),100);
    om_sh(k+1) = om_sh(k) - ((T_shell_cant + c_shell_damp*om_sh(k))/J_shell)*dt;

    % --- nose aero torque (canard damping + trim cant) ---
    c_nose_damp = fin_nose_N*qk*fin_nose_A*fin_CLa*(fin_nose_R^2)/max(V(k),100);
    T_nose_cant = fin_nose_N*qk*fin_nose_A*fin_CLa*fin_nose_trim*fin_nose_R;
    T_fin(k) = T_nose_cant + c_nose_damp*om_ns(k);

    % --- latches ---
    V_oc  = V_oc_ref*abs(om_rel(k))/om_ref;
    V_bus = V_sc(k);
    if (~despun) && (om_ns(k) < -2.0)
      despun = true; t_wake = tk;
    end
    if (~closed) && despun && (V_oc >= 18.0)
      closed = true; t_conn = tk;
    end
    if (~booted) && (V_bus >= V_boot)
      booted = true; t_boot = tk;
    end
    if (~elec_on) && despun && booted
      elec_on = true;
    end
    if elec_on && (~schedule_set)
      steer_t = t_wake + WAKE_OFFSETS;
      schedule_set = true;
    end
    if elec_on && schedule_set && isnan(t_bus_ok)
      if V_bus >= V_gate
        t_bus_ok = tk;
        steer_t(1) = max(steer_t(1), tk);
        steer_t(2) = steer_t(1) + 12;
        steer_t(3) = steer_t(1) + 27;
      elseif tk > t_wake + 6
        t_bus_ok = tk;
      end
    end

    % --- dwell state (arrest-time-aware) ---
    w_sens = om_ns(k) + gn(k);
    in_dwell = false;
    for s = 1:3
      if elec_on && (~impacted) && (tk >= steer_t(s))
        if abs(tk - steer_t(s)) < dt/2
          c_arr = fin_nose_N*qk*fin_nose_A*fin_CLa*(fin_nose_R^2)/max(V(k),100);
          t_arr = (J_nose/max(c_arr,1e-4))*log(max(abs(w_sens),W_HOLD_GATE)/W_HOLD_GATE);
          dwell_dur(s) = min(max(t_arr + 0.3, 0.5), 2.5);
        end
        if tk < steer_t(s) + dwell_dur(s)
          in_dwell = true;
        end
      end
    end

    % --- harvest torque command: burst / hold-shed / cruise ---
    R_path = R_int;
    if closed && (tk < t_conn + t_ntc), R_path = R_int + R_ntc; end
    I_line = max(0, (V_oc - V_bus)/R_path);
    T_line = min(V_oc*I_line/max(abs(om_rel(k)),1), T_hv_max);
    if V_bus < V_gate,          burst_on = true;  end
    if V_bus > V_gate + V_hyst, burst_on = false; end
    T_cmd = 0;
    if closed && despun && (~impacted)
      if in_dwell
        T_cmd = max(min(T_nose_cant - K_dwell*w_sens, T_hv_max), 0);
      elseif burst_on
        T_cmd = T_hv_max;
      else
        T_cmd = min(max((P_load/eta_buck + P_charge_margin)/max(abs(om_rel(k)),1), 0.020), T_hv_max);
      end
    end
    T_cmd = min(T_cmd, T_line);
    duty = T_cmd/max(T_line, 1e-6);
    I_alt = duty*I_line;
    T_hv(k) = duty*T_line;
    P_hv(k) = V_oc*I_alt;
    duty_a(k) = duty;

    % --- brake fills what harvest sheds during hold ---
    Tbr = 0;
    if in_dwell
      T_need = max(T_nose_cant - K_dwell*w_sens, 0);
      bcap = min((V_bus^2/R_brk)/max(abs(om_rel(k)),1), max(T_clutch - T_hv(k), 0));
      Tbr = min(max(T_need - T_hv(k), 0), bcap);
    end
    T_br(k) = Tbr;
    P_br(k) = T_br(k)*abs(om_rel(k));

    % --- PLL ---
    if USE_PLL && elec_on
      for s = 1:n_sub
        th_el = (poles/2)*(th_rel + om_rel(k)*Ts_pll*s);
        va = cos(th_el); vb = cos(th_el-2*pi/3); vc = cos(th_el+2*pi/3);
        val = (2/3)*(va - 0.5*vb - 0.5*vc);
        vbe = (vb - vc)/sqrt(3);
        amp = sqrt(val*val + vbe*vbe) + 1e-9;
        vq = (-val*sin(th_pll) + vbe*cos(th_pll))/amp;
        pll_int = pll_int + Ki_pll*vq*Ts_pll;
        w_pll = Kp_pll*vq + pll_int;
        th_pll = mod(th_pll + w_pll*Ts_pll, 2*pi);
      end
      w_est(k) = w_pll*(2/poles);
    elseif elec_on
      w_est(k) = om_rel(k) + 0.1*randn();
    end
    th_rel = th_rel + om_rel(k)*dt;

    % --- directed lift only while arrested ---
    held_gate = double(in_dwell && (abs(w_sens) < W_HOLD_GATE));
    F_st_max = fin_nose_N*qk*fin_nose_A*fin_CLa*steer_delta_rad;
    F_st(k) = F_st_max*held_gate;
    delta_cmd(k) = steer_delta_deg*in_dwell;
    dv = dv + ((F_st(k) + F_mag(k)*held_gate)/m_shell)*dt;

    for s = 1:3
      if abs(tk - steer_t(s) - dwell_dur(s)) < dt/2
        corr_dv(s) = dv; t_bank(s) = tk; dv = 0;
      end
    end

    % --- nose roll dynamics ---
    Ttot = -T_fin(k) + T_hv(k) + T_br(k);
    om_ns(k+1) = om_ns(k) + (Ttot/J_nose)*dt;
    th_ns(k+1) = th_ns(k) + om_ns(k)*dt;

    % --- cap charge balance ---
    I_load_bus = 0;
    if booted, I_load_bus = (P_load/eta_buck)/max(V_bus, 6.0); end
    I_brk_bus  = T_br(k)*abs(om_rel(k))/max(V_bus, 6.0);
    I_cap = I_alt - I_load_bus - I_brk_bus;
    Qsc(k+1) = max(Qsc(k) + I_cap*dt, 0);
    V_sc(k+1) = min(Qsc(k+1)/C_sc, V_max);
  end

  if ~impacted, h_apo = max(hh); x_impact = xx(N); end
  for s = 1:3
    corr(s) = corr_dv(s)*max(t_impact - t_bank(s), 0);
  end
  tot_corr(mc) = sum(corr);
  resid(mc) = wind + 0.05*randn();
  idx_v = (t >= 30) & (t <= t_impact);
  vsc_min(mc) = min(V_sc(idx_v));

  if WORKER
    % silent
  elseif RUN_MC
    if mod(mc,20) == 0, fprintf('  MC %d/%d complete\n', mc, runs); end
  else
    fprintf('\n===== PGK FLIGHT SIM v7.6.1 [%s] =====\n', SCENARIO);
    fprintf('Despin t=%.2f | connect t=%.2f | boot t=%.2f | gate t=%.2f s\n', ...
            t_wake, t_conn, t_boot, t_bus_ok);
    fprintf('Trajectory: elev %.0f deg | apogee %.2f km | impact t=%.1f s | downrange %.1f km (brochure 18)\n', ...
            launch_elev_deg, h_apo/1000, t_impact, x_impact/1000);
    if schedule_set
      fprintf('Steer schedule: %.1f / %.1f / %.1f s\n', steer_t);
    else
      fprintf('** no wake: schedule never set **\n');
    end
    fprintf('Peak fin torque = %.0f mNm | Peak harvest = %.0f mNm | Peak duty = %.2f\n', ...
            max(T_fin)*1000, max(T_hv)*1000, max(duty_a));
    fprintf('Harvest mech energy %.0f J | Brake energy %.0f J\n', sum(P_hv)*dt, sum(P_br)*dt);
    fprintf('Supercap: min %.2f V (30s..impact), final %.2f V\n', min(V_sc(idx_v)), V_sc(k_stop));
    for s = 1:3
      is = find(t >= steer_t(s), 1);
      if ~isempty(is)
        fprintf('  event %d t=%.1f s: h=%.2f km V=%.0f m/s q=%.0f kPa Vsc=%.2f V dwell=%.2f s\n', ...
                s, t(is), hh(is)/1000, V(is), qv(is)/1000, V_sc(is), dwell_dur(s));
      end
    end
    fprintf('Correction per event: %.1f / %.1f / %.1f m (total %.1f m)\n', corr, sum(corr));
    tt = t(1:k_stop);
    figure('Position',[80 80 1400 900]);
    subplot(3,2,1); plot(tt, om_sh(1:k_stop)*60/(2*pi), 'b', tt, om_ns(1:k_stop)*60/(2*pi), 'r', ...
                         tt, V(1:k_stop)/10, 'g', tt, hh(1:k_stop)/100, 'm');
      legend('shell RPM','nose RPM','V/10','h/100'); ylabel('RPM / m/s/10 / m/100'); grid on; title('Speeds & altitude');
    subplot(3,2,2); plot(tt, T_fin(1:k_stop)*1000, 'm', tt, T_br(1:k_stop)*1000, 'r', tt, T_hv(1:k_stop)*1000, 'b');
      legend('nose aero','brake','harvest'); ylabel('mNm'); grid on; title('Torques');
    subplot(3,2,3); plot(tt, P_hv(1:k_stop), 'b', tt, P_br(1:k_stop), 'r', tt, duty_a(1:k_stop), 'g');
      legend('harvest mech','brake','duty'); ylabel('W / -'); grid on; title('Power & PWM duty');
    subplot(3,2,4); plot(tt, V_sc(1:k_stop), 'b'); yline(V_gate,'g--'); yline(V_boot,'r--');
      ylabel('V'); grid on; title('Supercap = bus');
    subplot(3,2,5); plot(tt, mod(th_ns(1:k_stop)*180/pi, 360), 'k');
      ylabel('deg'); grid on; title('Nose azimuth (flat = arrested hold)');
    subplot(3,2,6); plot(tt, delta_cmd(1:k_stop), 'r', tt, F_st(1:k_stop)/10, 'b', tt, F_mag(1:k_stop)/10, 'c');
      legend('cmd (deg)','F_st/10','Magnus/10'); ylabel('deg / N'); grid on; title('Actuator & disturbance');
    print(sprintf('v76_%s_6panel.png', lower(SCENARIO)), '-dpng');
    fprintf('figure saved: v76_%s_6panel.png\n', lower(SCENARIO));
  end
end

if WORKER
  save(outfile, 'tot_corr', 'resid', 'vsc_min');
  fprintf('worker %d done: %d runs -> %s\n', slice_id, N_MC, outfile);
elseif RUN_MC
  fprintf('\n=== MONTE CARLO N=%d [%s] ===\n', N_MC, SCENARIO);
  fprintf('Correction: mean %.1f m, std %.1f m\n', mean(tot_corr), std(tot_corr));
  sres = sort(abs(resid));
  fprintf('Residual 95%%: %.1f m | Supercap min (worst, 30s..impact): %.2f V\n', ...
          sres(ceil(0.95*N_MC)), min(vsc_min));
  figure; hist(tot_corr, 20);
  xlabel('Total correction (m)'); grid on;
  print('v76_mc_corr_hist.png','-dpng');
  fprintf('figure saved: v76_mc_corr_hist.png\n');
end
