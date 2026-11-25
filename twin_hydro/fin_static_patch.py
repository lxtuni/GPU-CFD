"""卡 CAL4e（sim/docs/v18.md，11-24）：给孪生加一个默认关闭的开关 FIN_STATIC：舵机不带着鳍转时，尾鳍和脚撑板的准定常系数换成 CAL4d 对显卡鳍不动曲线拟合的那套，
另加沿展向的摩擦（显卡 Cd 0.13）。原来的函数一行不改（开关关着时逐位不变），两条路径（hydro2.py、hydro2_nb.py）都另加一份。
用法：python patch_fin_static.py <swimopt_ma 目录>"""
import io, sys, os
D = sys.argv[1]


def patch(fn, reps):
    p = os.path.join(D, fn); s = io.open(p, encoding="utf-8").read()
    if "FIN_STATIC" in s:
        print(fn, "已经打过补丁，跳过"); return
    for a, b in reps:
        assert s.count(a) == 1, (fn, a[:70])
        s = s.replace(a, b)
    io.open(p, "w", encoding="utf-8", newline="\n").write(s); print(fn, "ok")


# ---------------- hydro2.py ----------------
H_INIT_OLD = "        self.gain = np.ones(len(self.fb))                       # per-fin force gain (damage / wake); 1 = unchanged\n"
H_INIT_NEW = H_INIT_OLD + "        self.static = None; self.beta_bid = self.fb; self.hull_id = None; self.last_beta = None; self._wf = None; self._tl = 0.0   # 卡 CAL4e 的开关 FIN_STATIC（默认关）\n"
H_FORCES_OLD = """    def forces(self, d, point_vel):
        \"\"\"returns (bodies, points, forces) for all station + rotational-circulation forces, and the per-fin total\"\"\"
        fb = self.fb; Rf = d.xmat[fb].reshape(-1, 3, 3)
"""
H_FORCES_NEW = """    def forces(self, d, point_vel):
        \"\"\"returns (bodies, points, forces) for all station + rotational-circulation forces, and the per-fin total\"\"\"
        if self.static is not None:
            return self._forces_static(d, point_vel)
        fb = self.fb; Rf = d.xmat[fb].reshape(-1, 3, 3)
"""
H_STATIC_FN = '''
    def beta(self, d):
        """卡 CAL4e：每片的「在动」程度 0–1。舵机带的那根杆（尾鳍取父体 = 脚撑，脚撑板取自己）相对船体的角速度 |ω|，
        取衰减的峰值保持 ω_f = max(|ω|, ω_f·exp(−Δt/τ))，划水换向时的瞬间过零不算停；ω_f ≤ w1 → 0（按鳍不动的系数），≥ w2 → 1（按原来的系数），中间线性"""
        c = self.static; om = d.cvel[self.beta_bid][:, :3] - d.cvel[self.hull_id][:3]; wr = np.linalg.norm(om, axis=1); t = float(d.time)
        if self._wf is None or t < self._tl or t - self._tl > 0.05:
            self._wf = wr.copy()
        else:
            self._wf = np.maximum(wr, self._wf * np.exp(-(t - self._tl) / c["tau"]))
        self._tl = t
        return np.clip((self._wf - c["w1"]) / (c["w2"] - c["w1"]), 0.0, 1.0)
    def _forces_static(self, d, point_vel):
        """同 forces，系数按 beta 在原来的（来回划水标定）和鳍不动的（CAL4d）之间插值，另加沿展向的摩擦 −½ρA·CB·|w_b|w_b（乘 1 − beta）"""
        c = self.static; fb = self.fb; Rf = d.xmat[fb].reshape(-1, 3, 3)
        e = np.einsum("nij,nj->ni", Rf, self.fe); n = np.einsum("nij,nj->ni", Rf, self.fn); bsp = np.einsum("nij,nj->ni", Rf, self.fs)
        Ps = np.einsum("nij,nsj->nsi", Rf, self.loc) + d.xpos[fb][:, None, :]; nf, ns = Ps.shape[:2]
        bs = np.repeat(fb, ns); ws = point_vel(d, bs, Ps.reshape(-1, 3)).reshape(nf, ns, 3)
        kb = np.sum(ws*bsp[:, None, :], -1); ws = ws - kb[..., None]*bsp[:, None, :]
        ims = smooth_imm(Ps[..., 2], self.h_imm); bt = self.beta(d); self.last_beta = bt
        Ff = self.fm.station_forces(ws, e, n, ims)
        ps = c["ps"]; Kp, Kv, CS, CF = ps[0], ps[1], ps[2], ps[3]
        e_ = e[:, None, :]; n_ = n[:, None, :]
        wn = np.sum(ws*n_, -1); we = np.sum(ws*e_, -1); sp2 = wn*wn + we*we; sp = np.sqrt(sp2) + 1e-12
        sa = wn/sp; ca = np.abs(we)/sp; sg = np.where(we >= 0, 1.0, -1.0); q = 0.5*1000.0*self.fm.A*sp2*ims
        Fst = (-q*(Kp*sa*ca + Kv*sa*np.abs(sa)))[..., None]*n_ + (-q*(CS*sa*sa + CF*ca)*sg)[..., None]*e_
        Fst = Fst + (-0.5*1000.0*self.fm.A*ims*c["CB"]*np.abs(kb)*kb)[..., None]*bsp[:, None, :]
        Fs = bt[:, None, None]*Ff + (1.0 - bt)[:, None, None]*Fst
        Pc = np.einsum("nij,nj->ni", Rf, self.cloc) + d.xpos[fb]; wc = point_vel(d, fb, Pc)
        om_b = np.sum(d.cvel[fb][:, :3]*bsp, 1); Fr = self.fm.rot_force(wc, e, n, om_b, smooth_imm(Pc[:, 2], self.h_imm))
        Fs = Fs*self.gain[:, None, None]; Fr = Fr*self.gain[:, None]
        return (np.r_[bs, fb], np.r_[Ps.reshape(-1, 3), Pc], np.r_[Fs.reshape(-1, 3), Fr]), Fs.sum(1) + Fr, ims

def point_vel(m, d, bids, P):'''
H_PV_OLD = "\ndef point_vel(m, d, bids, P):"
H_WATER_OLD = "        lb, lp, la, lA, lh = [], [], [], [], []; pv = 0.0015\n"
H_WATER_NEW = """        # ---- 卡 CAL4e：开关 FIN_STATIC=1（默认关）。舵机不带着转时尾鳍和脚撑板换成鳍不动的系数（CAL4d 对显卡拟合）+ 展向摩擦；
        #      FIN_STATIC_P = "Kp,Kv,CS,CF,CB,w1,w2,tau" 可改默认值 ----
        self.fin_static = None
        if os.environ.get("FIN_STATIC", "0") == "1":
            v = [float(x) for x in os.environ.get("FIN_STATIC_P", "0.7166,1.1919,-0.042,0.188,0.13,0.15,0.4,0.1").split(",")]
            self.fin_static = dict(ps=np.array(v[:4]), CB=v[4], w1=v[5], w2=v[6], tau=v[7])
            self.fins.static = self.fin_static; self.fins.beta_bid = m.body_parentid[self.fins.fb].copy(); self.fins.hull_id = self.hull
            for sp in self.struts:
                sp.static = self.fin_static; sp.beta_bid = sp.fb.copy(); sp.hull_id = self.hull
""" + H_WATER_OLD
patch("hydro2.py", [(H_INIT_OLD, H_INIT_NEW), (H_FORCES_OLD, H_FORCES_NEW), (H_PV_OLD, H_STATIC_FN), (H_WATER_OLD, H_WATER_NEW)])

# ---------------- hydro2_nb.py ----------------
NB_FN = '''

@numba.njit(cache=True)
def _finset_static(F, Tq, tF, tT, xm, xpos, xipos, cvel, scom, root, cur, fb, fe, fn, fs, loc, cloc, gain, p, A, S, cref, h, Ffin, ims, ps, CB, beta):
    """卡 CAL4e（FIN_STATIC=1）：同 _finset，系数按每片的 beta 在 p（原来的）和 ps（鳍不动的）之间插值，另加展向摩擦（乘 1 − beta）"""
    nf = fb.shape[0]; ns = loc.shape[1]; CR = p[6]
    E = np.empty((nf, 3)); N = np.empty((nf, 3)); B = np.empty((nf, 3))
    for i in range(nf):
        b = fb[i]
        E[i, 0], E[i, 1], E[i, 2] = _rv(xm, b, fe[i, 0], fe[i, 1], fe[i, 2])
        N[i, 0], N[i, 1], N[i, 2] = _rv(xm, b, fn[i, 0], fn[i, 1], fn[i, 2])
        B[i, 0], B[i, 1], B[i, 2] = _rv(xm, b, fs[i, 0], fs[i, 1], fs[i, 2])
        Ffin[i, 0] = 0.0; Ffin[i, 1] = 0.0; Ffin[i, 2] = 0.0
    for i in range(nf):
        b = fb[i]; bt = beta[i]
        Kp = ps[0] + bt * (p[0] - ps[0]); Kv = ps[1] + bt * (p[1] - ps[1]); CS = ps[2] + bt * (p[2] - ps[2]); CF = ps[3] + bt * (p[3] - ps[3]); Cb = (1.0 - bt) * CB
        for s in range(ns):
            lx, ly, lz = _rv(xm, b, loc[i, s, 0], loc[i, s, 1], loc[i, s, 2])
            px = lx + xpos[b, 0]; py = ly + xpos[b, 1]; pz = lz + xpos[b, 2]
            wx, wy, wz = _pv(cvel, scom, root, b, px, py, pz, cur)
            k = wx * B[i, 0] + wy * B[i, 1] + wz * B[i, 2]
            wx = wx - k * B[i, 0]; wy = wy - k * B[i, 1]; wz = wz - k * B[i, 2]
            im = _imm(pz, h); ims[i, s] = im
            wn = wx * N[i, 0] + wy * N[i, 1] + wz * N[i, 2]; we = wx * E[i, 0] + wy * E[i, 1] + wz * E[i, 2]
            sp2 = wn * wn + we * we; sp = np.sqrt(sp2) + 1e-12
            sa = wn / sp; ca = abs(we) / sp; sg = 1.0 if we >= 0 else -1.0
            q = 0.5 * RHO_ * A[s] * sp2; q = q * im
            fnn = -q * (Kp * sa * ca + Kv * sa * abs(sa)); ft = -q * (CS * sa * sa + CF * ca) * sg
            fb_ = -0.5 * RHO_ * A[s] * im * Cb * abs(k) * k
            fx = (fnn * N[i, 0] + ft * E[i, 0] + fb_ * B[i, 0]) * gain[i]; fy = (fnn * N[i, 1] + ft * E[i, 1] + fb_ * B[i, 1]) * gain[i]; fz = (fnn * N[i, 2] + ft * E[i, 2] + fb_ * B[i, 2]) * gain[i]
            Ffin[i, 0] += fx; Ffin[i, 1] += fy; Ffin[i, 2] += fz
            _acc(tF, tT, xipos, b, px, py, pz, fx, fy, fz)
    for i in range(nf):
        b = fb[i]
        lx, ly, lz = _rv(xm, b, cloc[i, 0], cloc[i, 1], cloc[i, 2])
        px = lx + xpos[b, 0]; py = ly + xpos[b, 1]; pz = lz + xpos[b, 2]
        wx, wy, wz = _pv(cvel, scom, root, b, px, py, pz, cur)
        om = cvel[b, 0] * B[i, 0] + cvel[b, 1] * B[i, 1] + cvel[b, 2] * B[i, 2]
        f = -CR * RHO_ * S * cref * om * abs(wx * E[i, 0] + wy * E[i, 1] + wz * E[i, 2]); f = f * _imm(pz, h)
        fx = f * N[i, 0] * gain[i]; fy = f * N[i, 1] * gain[i]; fz = f * N[i, 2] * gain[i]
        Ffin[i, 0] += fx; Ffin[i, 1] += fy; Ffin[i, 2] += fz
        _acc(tF, tT, xipos, b, px, py, pz, fx, fy, fz)
    _flush(F, Tq, tF, tT)


@numba.njit(cache=True)
def _buoy('''
NB_BUOY_OLD = "\n\n@numba.njit(cache=True)\ndef _buoy("
NB_CALL_OLD = """    fa = Z["fins"]; w.fins.gain = np.asarray(w.fins.gain, float)
    _finset(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, fa["fb"], fa["fe"], fa["fn"], fa["fs"], fa["loc"], fa["cloc"],
            np.ascontiguousarray(w.fins.gain, float), fa["p"], fa["A"], fa["S"], fa["cref"], fa["h"], fa["Ffin"], fa["ims"])
    Fst = []
    for sp, sa in zip(w.struts, Z["struts"]):
        _finset(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, sa["fb"], sa["fe"], sa["fn"], sa["fs"], sa["loc"], sa["cloc"],
                np.ascontiguousarray(sp.gain, float), sa["p"], sa["A"], sa["S"], sa["cref"], sa["h"], sa["Ffin"], sa["ims"])
        Fst.append(sa["Ffin"].copy())
"""
NB_CALL_NEW = """    fa = Z["fins"]; w.fins.gain = np.asarray(w.fins.gain, float); stc = getattr(w, "fin_static", None)
    if stc is None:
        _finset(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, fa["fb"], fa["fe"], fa["fn"], fa["fs"], fa["loc"], fa["cloc"],
                np.ascontiguousarray(w.fins.gain, float), fa["p"], fa["A"], fa["S"], fa["cref"], fa["h"], fa["Ffin"], fa["ims"])
    else:                                                        # 卡 CAL4e（FIN_STATIC=1）
        bt = np.ascontiguousarray(w.fins.beta(d), float); w.fins.last_beta = bt; ps = np.ascontiguousarray(stc["ps"], float)
        _finset_static(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, fa["fb"], fa["fe"], fa["fn"], fa["fs"], fa["loc"], fa["cloc"],
                       np.ascontiguousarray(w.fins.gain, float), fa["p"], fa["A"], fa["S"], fa["cref"], fa["h"], fa["Ffin"], fa["ims"], ps, float(stc["CB"]), bt)
    Fst = []
    for sp, sa in zip(w.struts, Z["struts"]):
        if stc is None:
            _finset(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, sa["fb"], sa["fe"], sa["fn"], sa["fs"], sa["loc"], sa["cloc"],
                    np.ascontiguousarray(sp.gain, float), sa["p"], sa["A"], sa["S"], sa["cref"], sa["h"], sa["Ffin"], sa["ims"])
        else:
            bs_ = np.ascontiguousarray(sp.beta(d), float); sp.last_beta = bs_
            _finset_static(F, Tq, tF, tT, xm9, xpos, xipos, cvel, scom, Z["root"], cur, sa["fb"], sa["fe"], sa["fn"], sa["fs"], sa["loc"], sa["cloc"],
                           np.ascontiguousarray(sp.gain, float), sa["p"], sa["A"], sa["S"], sa["cref"], sa["h"], sa["Ffin"], sa["ims"], ps, float(stc["CB"]), bs_)
        Fst.append(sa["Ffin"].copy())
"""
patch("hydro2_nb.py", [(NB_BUOY_OLD, NB_FN), (NB_CALL_OLD, NB_CALL_NEW)])
