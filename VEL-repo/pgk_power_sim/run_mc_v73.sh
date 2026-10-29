#!/bin/bash
echo "Removing old slices to force regeneration..."
rm -f mc_slice1.mat mc_slice2.mat

echo "Launching worker 1..."
octave --no-gui pgk_flight_sim.m 1 100 1001 mc_slice1.mat > worker1.log 2>&1 &
PID1=$!

echo "Launching worker 2..."
octave --no-gui pgk_flight_sim.m 2 100 2002 mc_slice2.mat > worker2.log 2>&1 &
PID2=$!

echo "Waiting for workers (PIDs $PID1, $PID2) to finish..."
wait $PID1 $PID2

echo "Verifying worker outputs:"
echo "Worker 1: $(tail -n 1 worker1.log)"
echo "Worker 2: $(tail -n 1 worker2.log)"

echo "Aggregating..."
octave --no-gui combine_mc.m
