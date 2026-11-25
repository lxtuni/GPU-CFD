#!/usr/bin/env python3
"""卡 LBM1 / TND2：不经过 OpenFOAM 算例生成器，直接从一个周期的运动表做出显卡算例要的运动表（t,Hx,Hz,fin_deg,ry_deg）。
和 make_fluke_cfd.py 的做法逐行一致（周期延拓、起步 0.5 个周期平滑、原点 = 髋轴）。
    python3 lbm1_kin.py --motion motion_a28.csv --out kin.csv [--phase 0.5] [--dx 0.14] [--cycles 4]
--phase：运动比原表晚几个周期；--dx：整片鳍往下游（−x）挪多少米。"""
import argparse
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("--motion", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--phase", type=float, default=0.0); ap.add_argument("--dx", type=float, default=0.0)
ap.add_argument("--cycles", type=float, default=4.0); ap.add_argument("--ramp", type=float, default=0.5)
ap.add_argument("--origin", default="0.06345,0.004"); ap.add_argument("--nsub", type=int, default=800)
a = ap.parse_args()
OX, OZ = [float(v) for v in a.origin.split(",")]; OX -= a.dx
D = np.genfromtxt(a.motion, delimiter=",", skip_header=1); n = len(D); sh = int(round(a.phase * n)) % n; D[:, 1:] = np.roll(D[:, 1:], sh, axis=0)
tc = D[:, 0]; T = float(tc[-1] + (tc[1] - tc[0])); nT = int(a.nsub * a.cycles * 1.02) + 1; t = np.linspace(0, a.cycles * T * 1.02, nT); tm = t % T
ip = lambda y: np.interp(tm, tc, y, period=T)
fin = np.degrees(np.unwrap(np.radians(D[:, 3]))); Hx = OX - ip(D[:, 1]); Hz = OZ + ip(D[:, 2]); FIN = ip(fin)
if a.ramp > 0:
    x = np.clip(t / (a.ramp * T), 0, 1); s = x * x * (3 - 2 * x); Hx = Hx[0] + s * (Hx - Hx[0]); Hz = Hz[0] + s * (Hz - Hz[0]); FIN = FIN[0] + s * (FIN - FIN[0])
np.savetxt(a.out, np.column_stack([t, Hx, Hz, FIN, FIN - FIN[0]]), delimiter=",", header="t,Hx,Hz,fin_deg,ry_deg", comments="", fmt="%.7f")
