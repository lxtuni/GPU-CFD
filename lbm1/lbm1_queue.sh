#!/usr/bin/env bash
# 卡 LBM1：在 4090 服务器上按顺序跑一串显卡算例（一次只跑一个，显卡是多人共用的）。
#   nohup setsid bash ~/lbm1/code/lbm1_queue.sh "V2 T1 V3a V3b V2b" > ~/lbm1/out/queue.log 2>&1 < /dev/null &
# 每个算例的输出在 ~/lbm1/out/<名字>/（force.dat、log.txt、DONE）；全部跑完写 ~/lbm1/out/QUEUE_DONE。已有 DONE 的跳过。
D=$HOME/lbm1/data; O=$HOME/lbm1/out; cd $HOME/lbm1/FluidX3D || exit 1
rm -f $O/QUEUE_DONE
for c in $1; do
  case $c in
    V1)  E="LBM1_KIN=$D/kin_L1a28.csv LBM1_DX=0.0006 LBM1_TEND=1.5" ;;
    V2)  E="LBM1_KIN=$D/kin_L1a28.csv LBM1_DX=0.0005 LBM1_TEND=1.5" ;;
    V2b) E="LBM1_KIN=$D/kin_L1a28.csv LBM1_DX=0.00042 LBM1_TEND=1.5" ;;
    V3a) E="LBM1_KIN=$D/kin_L0.csv LBM1_DX=0.0006 LBM1_TEND=1.5" ;;
    V3b) E="LBM1_KIN=$D/kin_L2U000.csv LBM1_DX=0.0006 LBM1_TEND=1.5 LBM1_U=0" ;;
    T1)  E="LBM1_KIN=$D/kin_tand_fore.csv LBM1_KIN2=$D/kin_tand_hind.csv LBM1_DY=0.016 LBM1_DX=0.0006 LBM1_TEND=2.0" ;;
    T2)  E="LBM1_KIN=$D/kin_tand_fore.csv LBM1_KIN2=$D/kin_tand_hind.csv LBM1_DY=0.016 LBM1_DX=0.0005 LBM1_TEND=2.0" ;;
    *) echo "未定义 $c"; continue ;;
  esac
  [ -f $O/$c/DONE ] && { echo "$(date '+%H:%M:%S') $c 已有"; continue; }
  mkdir -p $O/$c; rm -f $O/$c/*; echo "$(date '+%H:%M:%S') $c 开始：$E"
  env LBM1_STL=$D/A2_blade_cfd.stl LBM1_OUT=$O/$c $E bin/FluidX3D > $O/$c/log.txt 2>&1
  echo "$(date '+%H:%M:%S') $c 结束：$(cat $O/$c/DONE 2>/dev/null || echo 没有DONE)"
done
date '+%H:%M:%S' > $O/QUEUE_DONE
