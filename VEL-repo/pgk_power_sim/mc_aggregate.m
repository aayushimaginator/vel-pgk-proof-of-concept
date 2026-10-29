clear all; close all; clc;
files = glob('mc_slice_*.mat');
tc = []; rs = []; vm = [];
for i = 1:length(files)
  S = load(files{i});
  tc = [tc; S.tot_corr(:)];
  rs = [rs; S.resid(:)];
  vm = [vm; S.vsc_min(:)];
end
N = length(tc);
fprintf('\n=== MONTE CARLO AGGREGATE (N=%d across %d workers) ===\n', N, length(files));
fprintf('Correction: mean %.1f m | std %.1f m | min %.1f | max %.1f\n', ...
        mean(tc), std(tc), min(tc), max(tc));
sres = sort(abs(rs));
fprintf('Residual impact error 95%%: %.2f m\n', sres(ceil(0.95*N)));
fprintf('Worst-case supercap min: %.2f V\n', min(vm));
fprintf('Brownout runs (Vmin < 14 V): %d / %d\n', sum(vm < 14.0), N);
figure; hist(tc, 25); xlabel('Total correction (m)'); grid on;
print('mc_correction_hist.png','-dpng');
figure; hist(vm, 25); xlabel('Worst-case supercap min (V)'); grid on;
print('mc_vsc_hist.png','-dpng');
