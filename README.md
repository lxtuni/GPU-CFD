# GPU-CFD: oscillating fins with a modified FluidX3D

GPU lattice Boltzmann simulation of rigid fins with prescribed, non-uniform motion
(pitching and heaving), single fin or two fins in tandem, with time-resolved fin forces.
Used to check the quasi-steady fin force model of a robot simulator against CFD.

> **This repository contains an altered version of FluidX3D.**
> FluidX3D is written by Dr. Moritz Lehmann: https://github.com/ProjectPhysX/FluidX3D.
> The first commit is the unmodified FluidX3D v3.9 (commit cc40a38); all later changes
> are listed in [`FluidX3D/MODIFIED.md`](FluidX3D/MODIFIED.md).
> The FluidX3D license ([`FluidX3D/LICENSE.md`](FluidX3D/LICENSE.md)) applies:
> non-commercial and non-military use only, no training of AI models on the source code,
> and publications must cite the [FluidX3D references](https://github.com/ProjectPhysX/FluidX3D#references).

## What was changed and why

Stock FluidX3D handles moving solids well for uniform rotation. A fin that pitches and
heaves with varying speed showed two problems, fixed by `lbm1/lbm1_patch.py`:

1. **Cells left behind.** When the fin moves on, solid cells were released only if their
   stored velocity matched the new mesh velocity. With varying motion they never match.
   Now each fin has its own marker (X or Y) and every cell with that marker that the mesh
   has left goes back to fluid.
2. **Force on a moving wall.** The force summed all populations of a solid cell times 2.
   It now uses link-wise momentum exchange over fluid-to-solid links only, with the
   moving-wall term `c_i (2 f_i - 6 w_i (c_i . u_w))`.

Build settings: D3Q19, SRT, FP32, Smagorinsky subgrid model, equilibrium boundaries on all
six faces.

## Layout

| Path | Content |
|---|---|
| `FluidX3D/` | FluidX3D v3.9 with the patch applied, ready to build |
| `lbm1/lbm1_patch.py` | applies the patch to a clean FluidX3D v3.9 tree (idempotent) |
| `lbm1/lbm1_setup.cpp` | fin case: STL fin(s), motion table(s), inflow, force output |
| `lbm1/lbm1_setup_viz.cpp` | same case plus rendered frames and mid-plane velocity slices |
| `lbm1/lbm1_kin.py` | builds a motion table from one period of motion (periodic extension, smooth start, phase shift) |
| `lbm1/*.sh` | run a list of cases one after another; build the rendering variant |
| `lbm1/viz_*.py` | turn frames and slices into GIFs, videos and comparison plots |
| `lbm1/lists/` | case lists used for our runs (one case per line: name and environment variables) |
| `example/` | one fin blade (`A2_blade_cfd.stl`) and one motion table to try the code |

The scripts in `lbm1/` assume the layout `~/lbm1/{FluidX3D,code,data,out}` of our server;
adjust the paths at the top of each script.

## Build and run the example

Linux with an OpenCL GPU driver:

```bash
cd FluidX3D && make Linux && cd ..
bash example/run_example.sh
```

On an RTX 4090 the example (66 M cells at 0.6 mm, 1.5 s physical time) takes about
4-5 minutes. Output in `example/out_L1a28/`:

- `force.dat`: `t_s Fx1_N Fy1_N Fz1_N [Fx2_N Fy2_N Fz2_N]`, one line per update;
  +x is thrust
- `DONE`: number of steps and wall time

## Case parameters

All settings are environment variables (command-line arguments are used by FluidX3D to
pick the GPU):

| Variable | Meaning | Default |
|---|---|---|
| `LBM1_STL` | fin geometry, binary STL in metres, origin on the pivot axis | required |
| `LBM1_KIN` | motion table of fin 1 | required |
| `LBM1_KIN2` | motion table of fin 2 (enables tandem) | none |
| `LBM1_DY` | lateral offset of fin 2 [m] | 0.016 |
| `LBM1_OUT` | output directory | `FluidX3D/bin/lbm1_out` |
| `LBM1_DX` | lattice spacing [m] | 0.0006 |
| `LBM1_U` | forward speed of the body [m/s]; water comes from +x | 0.2229 |
| `LBM1_V`, `LBM1_W` | lateral (left +) and vertical body speed [m/s] | 0 |
| `LBM1_TEND` | simulated time [s] | 1.5 |
| `LBM1_ULBM` | lattice velocity of the reference speed | 0.025 |
| `LBM1_UREF` | reference speed that sets the time step [m/s] | 0.2229 |
| `LBM1_REVOX` | revoxelize the fin every n steps | 2 |

Coordinates: x forward, y left, z up. Motion table columns
`t,Hx,Hz,fin_deg,ry_deg`: time [s], pivot position x and z [m], chord angle [deg],
rotation relative to the start pose [deg]. The rendering variant adds
`LBM1_FRAME_DT`, `LBM1_FRAME_T0`, `LBM1_VIEWS` and `LBM1_SLICE_DT`, `LBM1_SLICE_T0`,
`LBM1_SLICE_Y` (see the header of `lbm1_setup_viz.cpp`).

## Known limits

- Against OpenFOAM (finite volume, body-fitted overset mesh) on the same single-fin cases,
  thrust is consistently about 25 % lower (ratio 0.73-0.76; 0.80 at 0.42 mm) while
  waveforms and flow structures agree. Use it for relative comparisons, or calibrate.
- Near the moving fin the flow field shows lattice-scale noise from revoxelization;
  integrated forces are little affected.
- Not validated against experiments.
