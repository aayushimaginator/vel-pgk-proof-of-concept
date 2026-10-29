#!/bin/bash
echo "== PGK Sim Pipeline V6.1 =="
octave --no-gui --quiet pgk_flight_sim.m || exit 1
for PT in "muzzle:13340" "mid:11285" "terminal:9541"; do
  NAME=${PT%%:*}; SPD=${PT##*:}
  sed "s/SPEED=13340/SPEED=$SPD/" power_stage.cir > ps_$NAME.cir
  sed -i "s/pgk_power.dat/pgk_power_$NAME.dat/" ps_$NAME.cir
  ngspice -b ps_$NAME.cir -r pgk_$NAME.raw > ng_$NAME.log 2>&1 \
    && echo "$NAME OK" || echo "$NAME FAILED (see ng_$NAME.log)"
  if [ -f pgk_power_$NAME.dat ]; then
    cp pgk_power_$NAME.dat pgk_power.dat
    octave --no-gui --quiet analyze_results.m
  fi
done
echo "== Done =="
