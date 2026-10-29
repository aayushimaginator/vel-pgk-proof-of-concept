#!/usr/bin/env bash
set -euo pipefail

echo "========================================================"
echo " PGK Electromechanical and Power Simulation v7.6.1"
echo " Nominal trajectory execution"
echo "========================================================"

echo ""
echo "[1/2] Checking dependencies"

if ! command -v octave >/dev/null 2>&1; then
    echo "Error: GNU Octave was not found in PATH."
    echo "Install Octave 6.0 or newer and retry."
    exit 1
fi

echo "GNU Octave found: $(command -v octave)"

echo ""
echo "[2/2] Running nominal trajectory"

octave --no-gui --eval "pgk_flight_sim"

echo ""
echo "========================================================"
echo " Nominal trajectory execution complete"
echo "========================================================"
