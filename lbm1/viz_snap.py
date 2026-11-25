"""报告用：OpenFOAM（TND1）和显卡（T1，同一工况）在同一个竖直切面 y = 0、同一时刻的涡量 ω_y 并排出帧。
在服务器上跑：python3 viz_cmp.py  →  ~/lbm1/viz/cmp/f_<序号>.png（上 OpenFOAM、下显卡，同一窗口、同一色标；文字只用英文数字，中文标注回本机加）
OpenFOAM：postProcessing/midPlane/<t>/mid_c{0,1,2}.vtp（重叠网格三个区，按单元原样画多边形，先背景区、再两片鳍的区盖上去），
鳍的轮廓 = finSurf/<t>/fin*.vtp 和 y = 0 平面的交线。显卡：slices/s_<k>.bin（ux、uz，m/s；固体标记），ω_y = ∂ux/∂z − ∂uz/∂x（中心差分）。"""
import glob, os, sys
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy as v2n
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.colors import Normalize

OF = os.path.expanduser("~/tnd1/tand_L1a28_p50/postProcessing"); GP = os.path.expanduser("~/lbm1/viz/RTS/slices"); OUT = os.path.expanduser("~/lbm1/viz/cmp")
os.makedirs(OUT, exist_ok=True)
WMAX = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0; STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 2
meta = open(GP + "/meta.txt").read().split(); m = {meta[i]: float(meta[i + 1]) for i in range(0, len(meta) - 1, 2)}
Nx, Nz, dx, lox, loz = int(m["Nx"]), int(m["Nz"]), m["dx"], m["lo_x"], m["lo_z"]
X0, X1, Z0, Z1 = lox + 0.01, lox + (Nx - 1) * dx - 0.01, loz + 0.01, loz + (Nz - 1) * dx - 0.01
norm = Normalize(-WMAX, WMAX); cmap = plt.get_cmap("RdBu_r").copy(); cmap.set_bad("#7d858c")


def read(path):
    r = vtk.vtkXMLPolyDataReader(); r.SetFileName(path); r.Update(); return r.GetOutput()


def polys(d):
    P = v2n(d.GetPoints().GetData()); ca = v2n(d.GetPolys().GetData()); out, i = [], 0
    while i < len(ca):
        n = ca[i]; out.append(P[ca[i + 1:i + 1 + n]][:, [0, 2]]); i += n + 1
    return out


CASE = os.path.dirname(OF)
KZ = {"c1": np.loadtxt(CASE + "/kinematics_check.csv", delimiter=",", skiprows=1), "c2": np.loadtxt(CASE + "/kinematics_check_hind.csv", delimiter=",", skiprows=1)}


def zone_move(pp, z, t):
    """OpenFOAM 的切面（cuttingPlane, zones）在重叠网格的鳍区里几何停在 0 时刻（11-25 查到：鳍区网格和鳍形空洞在初始位姿，鳍面 finSurf 在当前位姿）。
    值是按单元取的、是当前时刻的；鳍区整体刚性运动（在切面内平移 + 绕 y 转），所以把多边形从初始位姿搬到 t 时刻的位姿：
    q = R(ry)·(p − H0) + H(t)，R 和 make_fluke_cfd 的 Ry 一致（x' = c x + s z，z' = −s x + c z），ry = 运动表里相对初始的转角。"""
    if z not in KZ:
        return pp
    k = KZ[z]; t = float(t); H0 = k[0, [1, 2]]; H = np.array([np.interp(t, k[:, 0], k[:, 1]), np.interp(t, k[:, 0], k[:, 2])]); a = np.radians(np.interp(t, k[:, 0], k[:, 4]))
    c, s = np.cos(a), np.sin(a); R = np.array([[c, s], [-s, c]])
    return [(q - H0) @ R.T + H for q in pp]


def fin_lines(t):
    segs = []
    for f in ("fin.vtp", "fin2.vtp"):
        p = f"{OF}/finSurf/{t}/{f}"
        if not os.path.exists(p):
            continue
        pl = vtk.vtkPlane(); pl.SetOrigin(0, 0, 0); pl.SetNormal(0, 1, 0); c = vtk.vtkCutter(); c.SetCutFunction(pl); c.SetInputData(read(p)); c.Update(); o = c.GetOutput()
        if o.GetNumberOfPoints() == 0:
            continue
        P = v2n(o.GetPoints().GetData()); L = v2n(o.GetLines().GetData()).reshape(-1, 3)[:, 1:]; segs += [P[l][:, [0, 2]] for l in L]
    return segs



# ---- 紧凑快照：报告 PDF 用；窗口裁到两片鳍附近，不画坐标文字
import sys as _s
TS = [float(x) for x in _s.argv[3].split(",")] if len(_s.argv) > 3 else [1.5, 1.75]
WX0, WX1, WZ0, WZ1 = -0.215, 0.055, -0.095, 0.03
tdirs = sorted(os.listdir(OF + "/midPlane"), key=float); gfs = sorted(glob.glob(GP + "/s_*.bin"))
fg, axs = plt.subplots(2, len(TS), figsize=(3.3 * len(TS), 2 * 3.3 * (WZ1 - WZ0) / (WX1 - WX0) + 0.1), dpi=220)
fg.subplots_adjust(left=0.005, right=0.995, top=0.995, bottom=0.005, wspace=0.02, hspace=0.03)
for j, tt in enumerate(TS):
    t = min(tdirs, key=lambda q: abs(float(q) - tt)); k = int(round((tt - 1.0) / 0.0125))
    ax = axs[0, j]
    for z in ("c0", "c1", "c2"):
        d = read(f"{OF}/midPlane/{t}/mid_{z}.vtp"); ax.add_collection(PolyCollection(zone_move(polys(d), z, t), array=v2n(d.GetCellData().GetArray("vorticity"))[:, 1], cmap=cmap, norm=norm, edgecolors="face", linewidths=0.2))
    ax.add_collection(LineCollection(fin_lines(t), colors="#20262b", linewidths=1.2))
    raw = np.fromfile(gfs[k], dtype=np.uint8)
    ux = np.frombuffer(raw[4:4 + 4 * Nx * Nz].tobytes(), np.float32).reshape(Nz, Nx); uz = np.frombuffer(raw[4 + 4 * Nx * Nz:4 + 8 * Nx * Nz].tobytes(), np.float32).reshape(Nz, Nx)
    sm = raw[4 + 8 * Nx * Nz:].reshape(Nz, Nx).astype(bool); wy = np.gradient(ux, dx, axis=0) - np.gradient(uz, dx, axis=1); wy[sm] = np.nan
    axs[1, j].imshow(wy, origin="lower", extent=(lox, lox + (Nx - 1) * dx, loz, loz + (Nz - 1) * dx), cmap=cmap, norm=norm, interpolation="nearest")
    for ax in axs[:, j]:
        ax.set_xlim(WX0, WX1); ax.set_ylim(WZ0, WZ1); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values(): sp.set_color("#b9c1c7"); sp.set_linewidth(0.6)
fg.savefig(OUT + "/snap.png", facecolor="white"); print("SNAP_OK", TS)
