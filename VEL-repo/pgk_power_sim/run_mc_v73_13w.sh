#!/bin/bash

# 1. Ensure combine_mc.m is the auto-globbing version
cat > combine_mc.m <<'MEOF'
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
MEOF

# 2. Clean up old slices
echo "Removing old slices to force regeneration..."
rm -f mc_slice_*.mat

# 3. Launch 13 workers
NW=13
PER=100
echo "Launching $NW workers x $PER runs (Total N=$((NW*PER)))..."

pids=()
for i in $(seq 1 $NW); do
  seed=$((1000+i*77))
  octave --no-gui pgk_flight_sim.m $i $PER $seed mc_slice_$i.mat > worker_$i.log 2>&1 &
  pids+=($!)
done

# 4. Wait and verify
echo "Waiting for workers (PIDs: ${pids[*]})..."
wait ${pids[@]}

echo "All workers done! Verifying outputs:"
for i in $(seq 1 $NW); do 
  echo "  slice $i: $(tail -n 1 worker_$i.log)"
done

# 5. Aggregate
echo "Aggregating N=$((NW*PER)) runs..."
octave --no-gui combine_mc.m
