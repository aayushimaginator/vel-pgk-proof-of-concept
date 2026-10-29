clear all; close all; clc;
d = dir('mc_slice_*.mat');
if isempty(d), error('no slices found - workers crashed or still running?'); end
files = {d.name};
tot_corr = []; resid = []; vsc_min = [];
for i = 1:numel(files)
  S = load(files{i});
  tot_corr = [tot_corr; S.tot_corr];
  resid    = [resid;    S.resid];
  vsc_min  = [vsc_min;  S.vsc_min];
end
fprintf('slices merged: %d (N=%d)\n', numel(files), numel(tot_corr));
fprintf('N=%d | corr mean %.1f std %.1f m | resid95 %.1f m | Vsc worst %.2f V\n', ...
        numel(tot_corr), mean(tot_corr), std(tot_corr), ...
        prctile(abs(resid),95), min(vsc_min));
figure; hist(tot_corr,30); grid on;
xlabel('Total correction (m)'); ylabel('count');
title(sprintf('MC correction authority (N=%d)', numel(tot_corr)));
print('mc_corr_hist.png','-dpng');
figure; hist(vsc_min,30); grid on;
xlabel('Worst-case supercap min (V)');
print('mc_vsc_hist.png','-dpng');
fprintf('histograms saved\n');
