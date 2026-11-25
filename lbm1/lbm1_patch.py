#!/usr/bin/env python3
"""卡 LBM1：给 FluidX3D（v3.9，commit cc40a38）打补丁，让它能算「变速摆动的尾鳍」并给出可信的受力。
    python3 lbm1_patch.py <FluidX3D 目录> <lbm1_setup.cpp>
改四处（都是对原软件的改动，按它的许可证第 5 条，发表结果时要连同这份补丁一起公开，并引用作者的论文）：
 1. defines.hpp：关 BENCHMARK、FP16S（用 FP32）；开 FORCE_FIELD、EQUILIBRIUM_BOUNDARIES、MOVING_BOUNDARIES、SUBGRID。
 2. kernel.cpp voxelize_mesh：原来靠「格子里存的速度 == 这次算出的速度」判断一个固体格是不是属于这个网格，
    只对匀速转动成立；改成只看 X / Y 标记（每片鳍一个标记），网格让出来的格子一律放回流体。
 3. kernel.cpp update_force_field：原来把固体格上所有方向的分布函数加起来乘 2。对运动的固体这有两个问题：
    (a) 少了动壁修正项 −6 w_i ρ (c_i·u_w) c_i（不满足伽利略不变性，会多出一个和壁面速度成正比的假力）；
    (b) 来自固体邻居方向的分布函数是旧值（格子被盖住以后不再更新）。
    改成只对「流体 → 固体」的连线求和，每条连线的动量交换 = c_i (2 f_i − 6 w_i (c_i·u_w))。
 4. lbm.cpp：update_force_field 内核多传一个 u。
 5. setup.cpp：末尾追加 lbm1_setup.cpp。
可以重复执行（已经改过的地方跳过）。"""
import io, re, sys

root, setup = sys.argv[1].rstrip("/"), sys.argv[2]
rd = lambda p: io.open(p, encoding="utf-8").read()


def wr(p, s):
    io.open(p, "w", encoding="utf-8", newline="\n").write(s)


def once(s, a, b, what):
    if b in s and a not in s:
        print("  已改过：" + what); return s
    assert s.count(a) == 1, f"{what}：原文出现 {s.count(a)} 次"
    print("  改：" + what); return s.replace(a, b)


p = root + "/src/defines.hpp"; s = rd(p)
for k in ("BENCHMARK", "FP16S"):
    s = re.sub(r"(?m)^#define " + k + r"\b", "//#define " + k, s)
for k in ("FORCE_FIELD", "EQUILIBRIUM_BOUNDARIES", "MOVING_BOUNDARIES", "SUBGRID"):
    s = re.sub(r"(?m)^//#define " + k + r"\b", "#define " + k, s)
wr(p, s); print("defines.hpp：", [m for m in re.findall(r"(?m)^#define (\w+)", s) if not m.startswith(("GRAPHICS", "TYPE", "VIS", "D3", "D2")) and len(m) > 2][:12])

p = root + "/src/kernel.cpp"; s = rd(p)
s = once(s, "if(un.x==u_set.x&&un.y==u_set.y&&un.z==u_set.z) {", "if(true) { /* LBM1 patch: release every cell with this X/Y marker that the mesh has left */", "voxelize_mesh 的归属判断")
s = once(s, "const ulong t, global float* F) { // calculate force from the fluid on solid boundaries from fi directly",
         "const ulong t, global float* F, const global float* u) { // LBM1 patch: link-wise momentum exchange with moving-wall term", "update_force_field 的参数")
a = re.search(r"\tfloat Fb=1\.0f, fx=0\.0f, fy=0\.0f, fz=0\.0f;\n\tcalculate_rho_u\(fhn, &Fb, &fx, &fy, &fz\);[^\n]*\n\tstore3\(F, n, 2\.0f\*Fb\*\(float3\)\(fx, fy, fz\)\);[^\n]*\n", s)
new = """\tconst float3 uw = load3(u, n);
\tfloat3 Fn = (float3)(0.0f, 0.0f, 0.0f);
\tfor(uint i=1u; i<def_velocity_set; i+=2u) {
\t\tconst float3 ci = (float3)((float)c(i), (float)c(def_velocity_set+i), (float)c(2u*def_velocity_set+i));
\t\tconst float wu = 6.0f*w(i)*dot(ci, uw);
\t\tif((flags[j[i+1u]]&TYPE_BO)!=TYPE_S) Fn += (2.0f*fhn[i]-wu)*ci;
\t\tif((flags[j[i]]&TYPE_BO)!=TYPE_S) Fn -= (2.0f*fhn[i+1u]+wu)*ci;
\t}
\tstore3(F, n, Fn);
"""
if a:
    s = s[:a.start()] + new + s[a.end():]; print("  改：update_force_field 的求和")
else:
    assert "const float wu = 6.0f*w(i)*dot(ci, uw);" in s, "update_force_field 的原文没找到"; print("  已改过：update_force_field 的求和")
wr(p, s)

p = root + "/src/lbm.cpp"; s = rd(p)
s = once(s, 'Kernel(device, N, "update_force_field", fi, flags, t, F);', 'Kernel(device, N, "update_force_field", fi, flags, t, F, u);', "lbm.cpp 的内核参数")
wr(p, s)

p = root + "/src/setup.cpp"; s = rd(p); mark = "// ==== LBM1 setup (appended by lbm1_patch.py) ====\n"
s = s.split(mark)[0].rstrip("\n") + "\n\n\n\n" + mark + rd(setup); wr(p, s); print("setup.cpp：已追加 lbm1_setup.cpp")
