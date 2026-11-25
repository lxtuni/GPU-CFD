#!/usr/bin/env bash
# Single oscillating fin, pitch law 28 deg, inflow 0.2229 m/s, 0.6 mm lattice, 1.5 s.
# About 4-5 minutes on an RTX 4090 (about 7 GB VRAM). Forces go to out_L1a28/force.dat.
# Build first:  cd ../FluidX3D && make Linux
set -e
cd "$(dirname "$0")"
E=$(pwd)
mkdir -p "$E/out_L1a28"
LBM1_STL="$E/A2_blade_cfd.stl" LBM1_KIN="$E/kin_L1a28.csv" LBM1_OUT="$E/out_L1a28" \
LBM1_DX=0.0006 LBM1_TEND=1.5 ../FluidX3D/bin/FluidX3D
