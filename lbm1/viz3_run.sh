#!/usr/bin/env bash
# 报告用：显卡 T1 的切面数据（和 OpenFOAM TND1 比）。单独编一份 FluidX3D_vs（不碰正在用的 FluidX3D_viz），等前面的渲染让出显卡再跑。
set -e
S=$HOME/lbm1/FluidX3D_viz; V=$HOME/lbm1/FluidX3D_vs; O=$HOME/lbm1/viz/RTS; D=$HOME/lbm1/data
mkdir -p $V/bin; cp -r $S/src $S/makefile $S/make.sh $V/
python3 $HOME/lbm1/code/lbm1_patch.py $V $HOME/lbm1/code/lbm1_setup_viz.cpp > $V/patch.log
cd $V && nice -n 10 make Linux -j$(nproc) > build.log 2>&1 && echo BUILD_OK
mkdir -p $O/slices
while pgrep -u "$USER" -x FluidX3D > /dev/null; do sleep 15; done
sleep 20; while pgrep -u "$USER" -x FluidX3D > /dev/null; do sleep 15; done
echo "$(date '+%H:%M:%S') RTS 开始"
L=$(grep '^RTS ' $HOME/lbm1/lists/viz3.txt | cut -d' ' -f2-)
( cd $D && env LBM1_STL=$D/A2_blade_cfd.stl LBM1_OUT=$O $L $V/bin/FluidX3D > $O/log.txt 2>&1 )
echo "$(date '+%H:%M:%S') RTS 结束：$(cat $O/DONE 2>/dev/null || echo 没有DONE)；切面 $(ls $O/slices | wc -l)"
