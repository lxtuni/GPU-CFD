#!/usr/bin/env bash
# 卡 LBM1 / TND2：按清单文件顺序跑显卡算例（一次一个）。清单每行：名字 环境变量...（# 开头的行跳过）
#   nohup setsid bash ~/lbm1/code/lbm1_runlist.sh ~/lbm1/lists/tnd2.txt > ~/lbm1/out/tnd2.log 2>&1 < /dev/null &
# 用 ~/lbm1/FluidX3D_b 里的程序（和正在跑的队列分开编译）；输出 ~/lbm1/out/<名字>/；全部跑完写 <清单>.DONE。已有 DONE 的跳过。
D=$HOME/lbm1/data; O=$HOME/lbm1/out; FX="${FX:-$HOME/lbm1/FluidX3D_b}"; cd "$FX" || exit 1
rm -f "$1.DONE"
while pgrep -u "$USER" -x FluidX3D > /dev/null; do sleep 20; done      # 等前一个队列让出显卡
while read -r name env; do
  case "$name" in ""|\#*) continue ;; esac
  [ -f "$O/$name/DONE" ] && { echo "$(date '+%H:%M:%S') $name 已有"; continue; }
  mkdir -p "$O/$name"; rm -f "$O/$name"/*; echo "$(date '+%H:%M:%S') $name 开始：$env"
  ( cd "$D" && env LBM1_STL=$D/A2_blade_cfd.stl LBM1_OUT=$O/$name $env "$FX/bin/FluidX3D" > "$O/$name/log.txt" 2>&1 )
  echo "$(date '+%H:%M:%S') $name 结束：$(cat "$O/$name/DONE" 2>/dev/null || echo 没有DONE)"
done < "$1"
date '+%H:%M:%S' > "$1.DONE"
