# This is an altered version of FluidX3D

This directory is **not** the original FluidX3D. It is FluidX3D v3.9
(https://github.com/ProjectPhysX/FluidX3D, commit cc40a38) by Dr. Moritz Lehmann,
modified for prescribed-motion oscillating fins. The unmodified source is the first
commit of this repository. All changes are made by `../lbm1/lbm1_patch.py`:

| File | Change |
|---|---|
| `src/defines.hpp` | `BENCHMARK` and `FP16S` off (FP32 storage); `FORCE_FIELD`, `EQUILIBRIUM_BOUNDARIES`, `MOVING_BOUNDARIES`, `SUBGRID` on |
| `src/kernel.cpp`, `voxelize_mesh` | A solid cell is released back to fluid based on its per-mesh X/Y marker only. The original test (stored velocity equals the new mesh velocity) only holds for uniform rotation and leaves cells behind for non-uniformly moving fins. |
| `src/kernel.cpp`, `update_force_field` | Link-wise momentum exchange summed only over fluid-to-solid links, with the moving-wall term: `c_i (2 f_i - 6 w_i (c_i . u_w))`. The original sum over all directions times 2 lacks the moving-wall correction and uses stale populations from solid neighbours. |
| `src/lbm.cpp` | `update_force_field` kernel takes the velocity field `u` as an extra argument. |
| `src/setup.cpp` | `../lbm1/lbm1_setup.cpp` appended (fin case setup). |

The license in `LICENSE.md` applies unchanged to this altered version.
