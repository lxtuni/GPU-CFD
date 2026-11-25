#!/usr/bin/env bash
# 报告用的流场渲染：按清单顺序跑出图版 FluidX3D（一次一个），每个算例跑完把帧合成 GIF。清单每行：名字 环境变量...
#   nohup setsid bash ~/lbm1/code/viz_run.sh ~/lbm1/lists/viz.txt > ~/lbm1/viz/run.log 2>&1 < /dev/null &
D=$HOME/lbm1/data; O=$HOME/lbm1/viz; FX=$HOME/lbm1/FluidX3D_viz; mkdir -p $O
while pgrep -u "$USER" -x FluidX3D > /dev/null; do sleep 20; done
while read -r name env; do
  case "$name" in ""|\#*) continue ;; esac
  [ -f "$O/$name/DONE" ] && { echo "$(date '+%H:%M:%S') $name 已有"; continue; }
  mkdir -p "$O/$name/frames"; rm -f "$O/$name"/frames/* "$O/$name"/*.* ; echo "$(date '+%H:%M:%S') $name 开始"
  ( cd "$D" && env LBM1_STL=$D/A2_blade_cfd.stl LBM1_OUT=$O/$name $env "$FX/bin/FluidX3D" > "$O/$name/log.txt" 2>&1 )
  echo "$(date '+%H:%M:%S') $name 结束：$(cat "$O/$name/DONE" 2>/dev/null || echo 没有DONE)；帧 $(ls $O/$name/frames | wc -l)"
  nice -n 10 python3 $HOME/lbm1/code/viz_gif.py "$O/$name" >> $O/run.log 2>&1
done < "$1"
date '+%H:%M:%S' > "$1.DONE"
