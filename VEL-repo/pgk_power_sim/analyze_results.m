% PGK Power Stage post-processing v2 (handles wrdata time,value pairs)
clear all; close all; clc;
d = load('pgk_power.dat');
if size(d,2) >= 8
  t = d(:,1); v = d(:,2); vg = d(:,4); il = d(:,6); ib = d(:,8);
else
  t = d(:,1); v = d(:,2); vg = d(:,3); il = d(:,4); ib = d(:,5);
end
idx = t > 1e-3;
i_load_calc = v(idx)/19.2;
fprintf('\n===== POWER STAGE ANALYSIS =====\n');
fprintf('Bus: avg %.2f V | max %.2f V | min %.2f V | ripple %.2f V\n', ...
        mean(v(idx)), max(v(idx)), min(v(idx)), max(v(idx))-min(v(idx)));
fprintf('Load: sense %.2f A | computed %.2f A | power %.1f W\n', ...
        mean(il(idx)), mean(i_load_calc), mean(v(idx).*i_load_calc));
if mean(il(idx)) < 0.5*mean(i_load_calc)
  fprintf('  ** sense column suspicious; trust computed **\n');
end
fprintf('Brake: avg %.2f A | peak %.2f A | duty %.1f%%\n', ...
        mean(ib(idx)), max(ib(idx)), 100*mean(vg(idx) > 2.5));
figure('Position',[150 150 1100 750]);
subplot(3,1,1); plot(t*1e3, v, 'b'); ylabel('V bus'); grid on;
subplot(3,1,2); plot(t*1e3, il, 'b', t*1e3, ib, 'r');
  legend('load','brake'); ylabel('A'); grid on;
subplot(3,1,3); plot(t*1e3, vg, 'k'); ylim([-0.5 5.5]); ylabel('gate V'); grid on;
