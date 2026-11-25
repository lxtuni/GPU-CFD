# GPU-CFD: oscillating fins with a modified FluidX3D

GPU lattice Boltzmann simulation of rigid fins with prescribed, non-uniform motion
(pitching and heaving), single fin or two fins in tandem, with time-resolved fin forces.
Used to check the quasi-steady fin force model of a robot simulator against CFD.

[中文说明见后半部分](#中文说明)

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
| `twin_hydro/` | fin hydrodynamic parameters of the MuJoCo simulator, before and after the GPU-CFD correction |

The scripts in `lbm1/` assume the layout `~/lbm1/{FluidX3D,code,data,out}` of our server;
adjust the paths at the top of each script.

## Simulator fin parameters corrected with this CFD

The MuJoCo simulator ("twin") applies fin forces with a quasi-steady plate model
(normal force with attached-flow term `Kp` and cross-flow term `Kv`, chordwise force `CS`,
friction `CF`, 3 chordwise stations, rotational circulation, added mass). Its coefficients
were fitted to OpenFOAM cases of a flapping fin. GPU-CFD showed that they are fine while the
fin flaps but too large when the fin is held still: broadside to the flow the simulator gave
2.0-2.3 times the CFD drag, and no spanwise force at all.

What changed (files in `twin_hydro/`):

| Item | Before (flapping, OpenFOAM fit) | After, fin not driven (GPU-CFD fit) |
|---|---|---|
| `Kp` attached-flow normal force | 1.3431 | 0.7166 |
| `Kv` cross-flow normal force | 2.4011 | 1.1919 |
| `CS` chordwise force | -1.094 | -0.042 |
| `CF` friction | 0.0829 | 0.188 |
| spanwise drag `CB` | none (spanwise flow ignored) | 0.13 |
| RMS error vs. GPU static Cd/Cl curve | 0.784 | 0.039 (3.1 % of the peak coefficient 1.27) |

- `fin_params_flapping_openfoam.json`: the original coefficients, unchanged, with their fit
  statistics; still used whenever the fin flaps.
- `fin_params_static_gpu.json`: the new coefficients, the switching rule, and the GPU static
  drag and lift curve (one fixed fin, inflow 0.10 m/s, chord angle -105 to 45 deg) next to
  the curve of the old coefficients.
- `fin_static_patch.py`: adds the switch `FIN_STATIC=1` (off by default) to the simulator's
  fin force code. Per fin, the angular speed of the driving servo link relative to the hull
  is peak-held with a 0.1 s decay; at or below 0.15 rad/s the static coefficients plus
  spanwise drag are used, at or above 0.4 rad/s the original ones, linear in between.
  With the switch off the simulator is bit-identical to before. The same rule is applied to
  the foot-brace plates (not checked against CFD). The simulator code itself is not part of
  this repository; the patch is included to document the change. Override the values with
  `FIN_STATIC_P="Kp,Kv,CS,CF,CB,w1,w2,tau"`.

Limits: one fin and one inflow speed; slow strokes (half frequency, 1.5-2.6 times too large)
are not corrected; GPU-CFD absolute forces are about 25 % below OpenFOAM in flapping cases
and this offset is not removed from the static fit.

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

---

# 中文说明

用显卡上的格子玻尔兹曼法，算给定运动（俯仰加升沉、速度不均匀）的刚性尾鳍，单片鳍或前后两片鳍，
输出随时间变化的鳍上受力。用来拿 CFD 核对机器人仿真器（孪生）里的准定常鳍力模型。

> **本仓库包含修改过的 FluidX3D。**
> FluidX3D 的作者是 Dr. Moritz Lehmann：https://github.com/ProjectPhysX/FluidX3D 。
> 第一个提交是未改动的 FluidX3D v3.9（commit cc40a38），之后的全部改动列在
> [`FluidX3D/MODIFIED.md`](FluidX3D/MODIFIED.md)。
> 适用 FluidX3D 的许可证（[`FluidX3D/LICENSE.md`](FluidX3D/LICENSE.md)）：只许非商业、非军事用途，
> 不许用源码训练 AI 模型，发表论文须引用 [FluidX3D 的参考文献](https://github.com/ProjectPhysX/FluidX3D#references)。

## 改了什么、为什么改

原版 FluidX3D 处理匀速转动的固体没有问题。尾鳍做变速的俯仰和升沉时出现两个问题，由
`lbm1/lbm1_patch.py` 修正：

1. **鳍移开后留下固体格。** 原来只有当格子里存的速度等于网格新的速度时，才把固体格还给流体；
   运动不匀速时两者永远不相等。现在每片鳍有自己的标记（X 或 Y），网格离开后，带这个标记的格子
   一律还给流体。
2. **运动壁面上的受力。** 原来把固体格上所有方向的分布函数加起来再乘 2。现在只对「流体 → 固体」
   的连线做动量交换，并加上动壁项：`c_i (2 f_i - 6 w_i (c_i . u_w))`。

编译设置：D3Q19、单松弛（SRT）、FP32、Smagorinsky 亚格子模型，六个面都是平衡态边界。

## 目录

| 路径 | 内容 |
|---|---|
| `FluidX3D/` | 打好补丁的 FluidX3D v3.9，可以直接编译 |
| `lbm1/lbm1_patch.py` | 给干净的 FluidX3D v3.9 打补丁（可重复执行） |
| `lbm1/lbm1_setup.cpp` | 尾鳍算例：STL 鳍、运动表、来流、受力输出 |
| `lbm1/lbm1_setup_viz.cpp` | 同一算例，另外输出渲染帧和鳍中间竖直切面的速度 |
| `lbm1/lbm1_kin.py` | 由一个周期的运动生成运动表（周期延拓、平滑起步、相位平移） |
| `lbm1/*.sh` | 按清单逐个运行算例；编译渲染版 |
| `lbm1/viz_*.py` | 把帧和切面做成 GIF、视频和对比图 |
| `lbm1/lists/` | 我们用过的算例清单（每行一个算例：名字和环境变量） |
| `example/` | 一片叶片（`A2_blade_cfd.stl`）和一个运动表，用来试跑 |
| `twin_hydro/` | MuJoCo 仿真器的鳍水动力参数，显卡 CFD 修正前后两套 |

`lbm1/` 里的脚本按我们服务器上的目录 `~/lbm1/{FluidX3D,code,data,out}` 写的，
使用前改一下各脚本开头的路径。

## 用这套 CFD 修正的仿真器鳍参数

MuJoCo 仿真器（孪生）的鳍力用准定常平板模型：法向力含附着流项 `Kp` 和横流项 `Kv`，
沿弦线的力 `CS`，摩擦 `CF`，沿弦线分 3 个站点，另有转动环量和附加质量。原来的系数是在
来回划水的 OpenFOAM 算例上拟合的。显卡 CFD 显示：鳍划水时这套系数没问题，鳍不动时偏大；
迎面挡水时仿真器的阻力是 CFD 的 2.0–2.3 倍，而且完全没有展向力。

改动如下（文件在 `twin_hydro/`）：

| 项 | 原来（划水，按 OpenFOAM 拟合） | 鳍不动时（按显卡 CFD 拟合） |
|---|---|---|
| `Kp` 附着流法向力 | 1.3431 | 0.7166 |
| `Kv` 横流法向力 | 2.4011 | 1.1919 |
| `CS` 沿弦线的力 | -1.094 | -0.042 |
| `CF` 摩擦 | 0.0829 | 0.188 |
| 展向阻力 `CB` | 无（忽略展向来流） | 0.13 |
| 对显卡鳍不动 Cd/Cl 曲线的均方根误差 | 0.784 | 0.039（最大力系数 1.27 的 3.1 %） |

- `fin_params_flapping_openfoam.json`：原来的系数，未改动，附拟合统计；鳍划水时仍用这套。
- `fin_params_static_gpu.json`：新系数、切换规则，以及显卡算的鳍不动阻力和升力曲线
  （一片固定的鳍，来流 0.10 m/s，弦线角 -105° 到 45°），旁边是原系数给出的曲线。
- `fin_static_patch.py`：给仿真器的鳍力代码加开关 `FIN_STATIC=1`（默认关）。每片鳍取带动它的
  舵机连杆相对船体的角速度，做衰减的峰值保持（时间常数 0.1 s）；不超过 0.15 rad/s 用鳍不动的
  系数并加展向阻力，不低于 0.4 rad/s 用原来的系数，中间线性过渡。开关关着时仿真器和原来逐位一致。
  脚撑板也用同样的规则（没有用 CFD 核对过）。仿真器本身的代码不在本仓库里，补丁只用来说明改了
  什么。可以用 `FIN_STATIC_P="Kp,Kv,CS,CF,CB,w1,w2,tau"` 改这些值。

局限：只用了一片鳍、一个来流速度；低频划水（频率减半时偏大 1.5–2.6 倍）没有修正；显卡 CFD
在划水算例上的绝对力比 OpenFOAM 低约 25 %，拟合鳍不动系数时没有扣除这个偏差。

## 编译并运行示例

需要 Linux 和支持 OpenCL 的显卡驱动：

```bash
cd FluidX3D && make Linux && cd ..
bash example/run_example.sh
```

RTX 4090 上示例（0.6 mm 格子、6 600 万格、算 1.5 s 物理时间）约 4–5 分钟。输出在
`example/out_L1a28/`：

- `force.dat`：`t_s Fx1_N Fy1_N Fz1_N [Fx2_N Fy2_N Fz2_N]`，每次更新一行；+x 为推力
- `DONE`：总步数和用时

## 算例参数

全部用环境变量设置（命令行参数被 FluidX3D 用来选显卡）：

| 变量 | 含义 | 默认值 |
|---|---|---|
| `LBM1_STL` | 鳍的几何，二进制 STL，单位米，原点在销轴上 | 必填 |
| `LBM1_KIN` | 第 1 片鳍的运动表 | 必填 |
| `LBM1_KIN2` | 第 2 片鳍的运动表（给了就是两片鳍） | 无 |
| `LBM1_DY` | 第 2 片鳍的横向偏移 [m] | 0.016 |
| `LBM1_OUT` | 输出目录 | `FluidX3D/bin/lbm1_out` |
| `LBM1_DX` | 格子大小 [m] | 0.0006 |
| `LBM1_U` | 船体前进速度 [m/s]，水从 +x 方向来 | 0.2229 |
| `LBM1_V`、`LBM1_W` | 船体侧向（左为正）和竖向速度 [m/s] | 0 |
| `LBM1_TEND` | 算多长的物理时间 [s] | 1.5 |
| `LBM1_ULBM` | 参考速度对应的格子速度 | 0.025 |
| `LBM1_UREF` | 决定时间步长的参考速度 [m/s] | 0.2229 |
| `LBM1_REVOX` | 每几步重新体素化一次鳍 | 2 |

坐标：x 向前、y 向左、z 向上。运动表的列 `t,Hx,Hz,fin_deg,ry_deg`：时间 [s]、销轴位置 x 和 z [m]、
弦线角 [°]、相对初始姿态的转角 [°]。渲染版另有 `LBM1_FRAME_DT`、`LBM1_FRAME_T0`、`LBM1_VIEWS`
和 `LBM1_SLICE_DT`、`LBM1_SLICE_T0`、`LBM1_SLICE_Y`（见 `lbm1_setup_viz.cpp` 开头的说明）。

## 已知局限

- 和 OpenFOAM（有限体积法，贴体重叠网格）在相同的单鳍算例上比，推力一直低约 25 %
  （比值 0.73–0.76，格子加密到 0.42 mm 时 0.80），波形和流动结构一致。只用来比相对变化，
  或者先定标。
- 运动的鳍附近流场有格子尺度的噪声，来自重新体素化；积分出来的力受影响不大。
- 没有和实验对比过。
