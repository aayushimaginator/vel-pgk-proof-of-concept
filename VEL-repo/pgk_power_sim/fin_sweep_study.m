% ============================================================
% FIN SWEEP STUDY v2.1 - deck-ready
% Fixes: x to 55 s, never-wake on 0 m shelf, baked annotations,
%        stairs legend, no colorbar, req line, never-wake shade,
%        fine-sweep crossings + J-at-73mNm computed
% ============================================================
clear all; close all; clc;

J=1.4e-3; c_fin=0.0152; T_d0=0.181; q0=287000; m_shell=42;
t_q=[0 25 50]; q_kPa=[287 101 43];
t_end=55; dt=1e-3; t=0:dt:t_end; N=length(t);
q = max(1000*interp1(t_q,q_kPa,t,'linear','extrap'), 43000);
A_fin=0.002; net_fin=2; C_L=2.4*(1*pi/180); max_dwell=1.5;
om_sh0=(685/3.1)*2*pi;

% ---------- SWEEP 1 (coarse, for plot) ----------
T_stall_vec=[0.05 0.073 0.10 0.15 0.20 0.25 0.30 0.40 0.45 0.50 0.60];
t_wake=nan(size(T_stall_vec));
for i=1:length(T_stall_vec)
  Ts=T_stall_vec(i); om_n=om_sh0; tw=NaN;
  for k=1:N-1
    Tu=T_d0*(q(k)/q0)+c_fin*om_n;
    Tf=max(min(Tu, Ts*(q(k)/q0)), -Ts*(q(k)/q0));
    om_n=om_n+(-Tf/J)*dt;
    if isnan(tw) && om_n < -2.0, tw=t(k); end
  end
  t_wake(i)=tw;
end

figure('Position',[100 100 900 600]);
patch([0 90 90 0],[0 0 60 60],[0.85 0.85 0.85],'EdgeColor','none'); hold on;
text(6,52,'no wake within flight','Color',[0.35 0.35 0.35],'FontSize',10);
plot(T_stall_vec*1000, t_wake, 'bo-','LineWidth',1.5);
yline(55,'r--'); text(455,56.5,'t_end (flight ends)','Color','r','FontSize',9);
yline(5,'g--');  text(455,6.5,'req <= 5 s','Color','g','FontSize',9);
xline(73,'m--'); text(76,25,'mech current 73 mNm','Color','m','Rotation',90,'FontSize',9);
xline(450,'k--'); text(455,25,'spec 450 mNm','Rotation',90,'FontSize',9);
xlabel('Fin stall torque [mNm]'); ylabel('Electrical wake-up time [s]');
title('Despin: wake-up vs fin stall torque (J=1.4e-3, q(t) decay)');
grid on; ylim([0 60]);
print('fin_wake_vs_stall.png','-dpng');

% ---------- SWEEP 1b (fine, for crossing numbers) ----------
stall_fine=100:5:600; tw_fine=nan(size(stall_fine));
for i=1:length(stall_fine)
  Ts=stall_fine(i)/1000; om_n=om_sh0; tw=NaN;
  for k=1:N-1
    Tu=T_d0*(q(k)/q0)+c_fin*om_n;
    Tf=max(min(Tu, Ts*(q(k)/q0)), -Ts*(q(k)/q0));
    om_n=om_n+(-Tf/J)*dt;
    if isnan(tw) && om_n < -2.0, tw=t(k); end
  end
  tw_fine(i)=tw;
end
s_cliff = interp1(tw_fine, stall_fine, 8, 'linear');   % bare first-event cliff
s_5s    = interp1(tw_fine, stall_fine, 5, 'linear');   % 5 s requirement crossing
idx5 = t<=5;
J_73 = trapz(t(idx5), 0.073*(q(idx5)/q0)) / (om_sh0 + 2.0);

% ---------- SWEEP 2 ----------
t_wake_sweep=0:1:55;
corr_tot=zeros(size(t_wake_sweep)); ev_held=zeros(size(t_wake_sweep));
for i=1:length(t_wake_sweep)
  tw=t_wake_sweep(i); ev=[8 20 35]; c=0; nh=0;
  for s=1:3
    te=ev(s);
    if te>tw
      nh=nh+1;
      qe=max(1000*interp1(t_q,q_kPa,te,'linear','extrap'),43000);
      if te<30, Thv=0.100; else, Thv=0.020; end
      om_eq=(Thv - T_d0*(qe/q0))/c_fin;
      dw=min(max(pi/max(abs(om_eq),0.5),0.2),max_dwell);
      F=net_fin*qe*A_fin*C_L;
      c=c+F*dw/m_shell*(t_end-te);
    end
  end
  corr_tot(i)=c; ev_held(i)=nh;
end

figure('Position',[100 100 900 600]);
h1=plot(t_wake_sweep, corr_tot, 'k-','LineWidth',1.8); hold on;
text(12,41,'events 2+3 only = 38.3 m','FontSize',9);
text(22,11.5,'event 3 only = 9.3 m','FontSize',9);
text(36,5,'current fins: never wake -> 0 m','Color','m','FontSize',10);
xline(4.0,'b--'); text(0.8,86,'nominal wake 4.0 s','Color','b','FontSize',9);
xline(8,'g--');  text(8.6,86,'first steer event (wake left of this)','Color','g','FontSize',9);
xline(55,'m--');
ylabel('Total impact correction [m]'); xlabel('Electrical wake-up time [s]');
ylim([0 95]); grid on;
yyaxis right;
h2=stairs(t_wake_sweep, ev_held, 'b-','LineWidth',1.2);
ylabel('Events held (of 3)'); ylim([0 3.2]);
legend([h1 h2],{'total correction [m]','events held (right axis)'},'Location','southwest');
title('Steering correction vs wake-up time (absolute schedule 8/20/35 s)');
print('fin_corr_vs_wake.png','-dpng');

% ---------- console: deck numbers from code ----------
fprintf('\nfirst-event cliff (wake=8 s):  %.0f mNm\n', s_cliff);
fprintf('5 s requirement crossing:      %.0f mNm\n', s_5s);
fprintf('J needed for 5 s at 73 mNm:    %.1e kg m^2 (budget 1.4e-3)\n', J_73);
fprintf('\n stall[mNm]  wake[s]\n');
for i=1:length(T_stall_vec)
  if isnan(t_wake(i))
    fprintf('   %5.0f     >55 (NEVER WAKES)\n', T_stall_vec(i)*1000);
  else
    fprintf('   %5.0f      %5.1f\n', T_stall_vec(i)*1000, t_wake(i));
  end
end
