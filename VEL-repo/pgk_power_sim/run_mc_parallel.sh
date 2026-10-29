#!/bin/bash
# Parallel Monte Carlo: splits N runs across K workers, then aggregates
N=${1:-200}
K=${2:-$(nproc)}
PER=$(( (N + K - 1) / K ))
rm -f mc_slice_*.mat
pids=()
for ((i=0;i<K;i++)); do
  start=$((i*PER)); cnt=$PER
  if (( start + cnt > N )); then cnt=$((N - start)); fi
  (( cnt <= 0 )) && continue
  seed=$((1000 + i*7919))
  octave --no-gui --quiet pgk_flight_sim.m $i $cnt $seed mc_slice_$i.mat &
  pids+=($!)
  echo "launched worker $i: $cnt runs (seed $seed)"
done
wait "${pids[@]}"
echo "== all workers done; aggregating =="
octave --no-gui --quiet mc_aggregate.m
