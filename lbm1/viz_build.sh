#!/usr/bin/env bash
# 报告用的出图版 FluidX3D：从 FluidX3D_b（已打 LBM1 补丁）复制一份到 FluidX3D_viz，开 GRAPHICS，换成 lbm1_setup_viz.cpp，编译。
#   bash ~/lbm1/code/viz_build.sh <Q 阈值，默认 5e-6> <U_MAX，默认 0.10>
set -e
Q=${1:-5e-6}; UM=${2:-0.10}; S=$HOME/lbm1/FluidX3D_b; V=$HOME/lbm1/FluidX3D_viz
[ -d $V ] || { mkdir -p $V; cp -r $S/src $S/makefile $S/make.sh $V/; mkdir -p $V/bin; }
python3 $HOME/lbm1/code/lbm1_patch.py $V $HOME/lbm1/code/lbm1_setup_viz.cpp > $V/patch.log
D=$V/src/defines.hpp
sed -i 's|^//#define GRAPHICS //|#define GRAPHICS //|' $D
sed -i -E 's|^#define GRAPHICS_FRAME_WIDTH [0-9]+|#define GRAPHICS_FRAME_WIDTH 1280|; s|^#define GRAPHICS_FRAME_HEIGHT [0-9]+|#define GRAPHICS_FRAME_HEIGHT 720|; s|^#define GRAPHICS_BACKGROUND_COLOR 0x[0-9A-Fa-f]+|#define GRAPHICS_BACKGROUND_COLOR 0xFFFFFF|' $D
sed -i -E "s|^#define GRAPHICS_U_MAX [0-9.]+f|#define GRAPHICS_U_MAX ${UM}f|; s|^#define GRAPHICS_Q_CRITERION [0-9.eE+-]+f|#define GRAPHICS_Q_CRITERION ${Q}f|" $D
grep -E "^#define (GRAPHICS|GRAPHICS_FRAME|GRAPHICS_BACKGROUND|GRAPHICS_U_MAX|GRAPHICS_Q_CRITERION|FORCE_FIELD|MOVING_BOUNDARIES|SUBGRID|EQUILIBRIUM_BOUNDARIES)\b" $D
cd $V && make Linux -j$(nproc) > build.log 2>&1 && echo BUILD_OK || { tail -20 build.log; echo BUILD_FAIL; }
