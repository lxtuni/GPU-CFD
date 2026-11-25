// 卡 LBM1：尾鳍给定运动的格子玻尔兹曼算例（FluidX3D，显卡）。由 lbm1_patch.py 追加到 FluidX3D 的 src/setup.cpp 末尾。
// 需要 defines.hpp：FORCE_FIELD、EQUILIBRIUM_BOUNDARIES、MOVING_BOUNDARIES、SUBGRID；不要 BENCHMARK、FP16S。
// 坐标和 OpenFOAM 算例一致：x 前进方向、y 左、z 上；来流从 +x 吹向 −x；力 +x = 推力。
// 配置全部走环境变量（命令行参数被 FluidX3D 用来选显卡）：
//   LBM1_STL   尾鳍 STL（二进制，原点 = 销轴，米）            LBM1_KIN   前鳍运动表（OpenFOAM 算例的 kinematics_check.csv：t,Hx,Hz,fin_deg,ry）
//   LBM1_KIN2  后鳍运动表（给了就是两片鳍）                    LBM1_DY    后鳍的横向偏移 [m]
//   LBM1_OUT   输出目录                                        LBM1_DX    格子大小 [m]（默认 0.0006）
//   LBM1_U     来流 [m/s]（默认 0.2229）                       LBM1_ULBM  来流对应的格子速度（默认 0.025）
//   LBM1_TEND  算到多少秒（默认 1.5）                          LBM1_REVOX 每几步重新体素化一次（默认 2）
//   LBM1_V / LBM1_W  船体的侧向（左为正）/ 竖向速度 [m/s]（默认 0）：水相对船的速度 = −(U, V, W)；卡 CAL1 的横移要用
//   LBM1_UREF  和 LBM1_ULBM 对应的参考速度 [m/s]（默认 0.2229）：时间步长由它定，和来流无关（来流小的算例尾鳍照样快）
#ifndef BENCHMARK
#include <fstream>
#include <sstream>
#include <cstdlib>
struct Lbm1Kin { // 运动表：线性插值
	vector<double> t, hx, hz, fin;
	void load(const string& path) {
		std::ifstream f(path); string line; std::getline(f, line);
		while(std::getline(f, line)) {
			std::stringstream ss(line); string c; vector<double> v;
			while(std::getline(ss, c, ',')) v.push_back(atof(c.c_str()));
			if(v.size()>=4u) { t.push_back(v[0]); hx.push_back(v[1]); hz.push_back(v[2]); fin.push_back(v[3]); }
		}
		if(t.size()<2u) { print_error("运动表读不出来："+path); }
	}
	double at(const vector<double>& y, const double tt) const {
		if(tt<=t.front()) return y.front();
		if(tt>=t.back()) return y.back();
		const double s = (tt-t.front())/(t[1]-t[0]); const size_t i = (size_t)s; const double a = s-(double)i;
		return i+1u<y.size() ? y[i]*(1.0-a)+y[i+1u]*a : y.back();
	}
};
struct Lbm1Fin { // 一片鳍：参考网格（弦系，格子单位，销轴在原点）+ 当前网格
	Mesh* mesh = nullptr; vector<float3> r0, r1, r2; Lbm1Kin kin; double dy = 0.0; uchar flag = TYPE_S;
};
static string lbm1_env(const char* name, const string& def) { const char* v = getenv(name); return v ? string(v) : def; }
static void lbm1_pose(Lbm1Fin& f, const double t, const double dx, const float3& lo, float3& H, float& ang) { // 把网格摆到 t 时刻的位姿
	H = float3((float)((f.kin.at(f.kin.hx, t)-(double)lo.x)/dx), (float)((f.dy-(double)lo.y)/dx), (float)((f.kin.at(f.kin.hz, t)-(double)lo.z)/dx));
	ang = (float)radians(f.kin.at(f.kin.fin, t)); const float c = cosf(ang), s = sinf(ang); // 和 make_fluke_cfd.py 的 Ry 一样：x' = c x + s z，z' = −s x + c z
	for(uint i=0u; i<f.mesh->triangle_number; i++) {
		const float3 a = f.r0[i], b = f.r1[i], d = f.r2[i];
		f.mesh->p0[i] = H+float3(c*a.x+s*a.z, a.y, -s*a.x+c*a.z);
		f.mesh->p1[i] = H+float3(c*b.x+s*b.z, b.y, -s*b.x+c*b.z);
		f.mesh->p2[i] = H+float3(c*d.x+s*d.z, d.y, -s*d.x+c*d.z);
	}
	f.mesh->find_bounds(); f.mesh->set_center(H);
}
void main_setup() { // 尾鳍给定运动；required extensions in defines.hpp: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
	const string stl = lbm1_env("LBM1_STL", ""), out = lbm1_env("LBM1_OUT", get_exe_path()+"lbm1_out")+"/";
	const double dx = atof(lbm1_env("LBM1_DX", "0.0006").c_str()), si_u = atof(lbm1_env("LBM1_U", "0.2229").c_str()), si_T = atof(lbm1_env("LBM1_TEND", "1.5").c_str()), si_v = atof(lbm1_env("LBM1_V", "0").c_str()), si_w = atof(lbm1_env("LBM1_W", "0").c_str());
	const float lbm_u = (float)atof(lbm1_env("LBM1_ULBM", "0.025").c_str()); const uint revox = (uint)atoi(lbm1_env("LBM1_REVOX", "2").c_str());
	const double si_nu = 1.0E-6, si_rho = 1000.0, u_ref = atof(lbm1_env("LBM1_UREF", "0.2229").c_str()); // 时间步长由参考速度定，和来流无关
	vector<Lbm1Fin> fins(1u); fins[0].kin.load(lbm1_env("LBM1_KIN", "")); fins[0].flag = TYPE_S|TYPE_X;
	const string kin2 = lbm1_env("LBM1_KIN2", "");
	if(kin2!="") { fins.resize(2u); fins[1].kin.load(kin2); fins[1].dy = atof(lbm1_env("LBM1_DY", "0.016").c_str()); fins[1].flag = TYPE_S|TYPE_Y; }
	// ---- 盒子：销轴轨迹的包络 + 弦长余量；下游（−x）多留
	double xmin=1E9, xmax=-1E9, zmin=1E9, zmax=-1E9, ymax=0.0;
	for(const Lbm1Fin& f : fins) { for(size_t i=0u; i<f.kin.t.size(); i++) { xmin=fmin(xmin, f.kin.hx[i]); xmax=fmax(xmax, f.kin.hx[i]); zmin=fmin(zmin, f.kin.hz[i]); zmax=fmax(zmax, f.kin.hz[i]); } ymax=fmax(ymax, f.dy); }
	const double chord = 0.04, span2 = 0.03;
	const double umag = fmax(sqrt(si_u*si_u+si_v*si_v+si_w*si_w), 0.05), ex = 0.06*fabs(si_u)/umag, ey = 0.08*fabs(si_v)/umag, ez = 0.05*fabs(si_w)/umag; // 下游一侧多留：水往 −(U, V, W) 的方向走
	double ymin = 0.0; for(const Lbm1Fin& f : fins) ymin = fmin(ymin, f.dy);
	const float3 lo((float)(xmin-chord-0.08-(si_u>=0.0 ? fmax(ex, 0.04) : 0.0)), (float)(ymin-span2-0.05-(si_v>0.0 ? ey : 0.0)), (float)(zmin-chord-0.07-(si_w>0.0 ? ez : 0.0)));
	const float3 hi((float)(xmax+chord+0.08+(si_u<0.0 ? ex : 0.0)), (float)(ymax+span2+0.05+(si_v<0.0 ? ey : 0.0)), (float)(zmax+chord+0.07+(si_w<0.0 ? ez : 0.0)));
	const uint Nx = (uint)((hi.x-lo.x)/dx+0.5), Ny = (uint)((hi.y-lo.y)/dx+0.5), Nz = (uint)((hi.z-lo.z)/dx+0.5);
	units.set_m_kg_s(1.0f, lbm_u, 1.0f, (float)dx, (float)u_ref, (float)si_rho);
	const double dt = dx*(double)lbm_u/u_ref; const ulong lbm_T = (ulong)(si_T/dt);
	print_info("LBM1: "+to_string((uint)fins.size())+" fin(s), box "+to_string(Nx)+"x"+to_string(Ny)+"x"+to_string(Nz)+" = "+to_string((uint)((ulong)Nx*(ulong)Ny*(ulong)Nz/1000000ull))+" M cells, dx = "+to_string((float)(dx*1E3), 3u)+" mm, dt = "+to_string((float)(dt*1E6), 2u)+" us, steps = "+to_string(lbm_T)+", nu_lbm = "+to_string(units.nu((float)si_nu), 6u));
	LBM lbm(Nx, Ny, Nz, units.nu((float)si_nu));
	// ---- 几何
	const float eps = 1.0E-9f; // 让速度永远不等于 0：体素化内核只在速度非零时才给固体格写速度、把让出来的格子重置成平衡态
	for(Lbm1Fin& f : fins) {
		f.mesh = read_stl(stl, (float)(1.0/dx)); // 米 → 格子单位，不重新定位
		for(uint i=0u; i<f.mesh->triangle_number; i++) { f.r0.push_back(f.mesh->p0[i]); f.r1.push_back(f.mesh->p1[i]); f.r2.push_back(f.mesh->p2[i]); }
		float3 H; float ang; lbm1_pose(f, 0.0, dx, lo, H, ang);
		lbm.voxelize_mesh_on_device(f.mesh, f.flag, H, float3(eps, 0.0f, 0.0f), float3(0.0f));
	}
	const float uin = -lbm_u*(float)(si_u/u_ref), vin = -lbm_u*(float)(si_v/u_ref), win = -lbm_u*(float)(si_w/u_ref);
	parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(!(lbm.flags[n]&TYPE_S)) { lbm.u.x[n] = uin; lbm.u.y[n] = vin; lbm.u.z[n] = win; }
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // 六个面都是平衡态边界（来流速度、密度 1）
	});
	lbm.run(0u, lbm_T);
	string head = "# t_s";
	for(size_t k=0u; k<fins.size(); k++) head += " Fx"+to_string((uint)(k+1u))+"_N Fy"+to_string((uint)(k+1u))+"_N Fz"+to_string((uint)(k+1u))+"_N";
	write_file(out+"force.dat", head+" (lbm steps "+to_string(lbm_T)+", dt "+to_string((float)dt, 9u)+")\n");
	string buf = ""; Clock clock; ulong k = 0ull;
	while(lbm.get_t()<lbm_T) {
		const double tm = ((double)lbm.get_t()+0.5*(double)revox)*dt, h = 0.000625; // 这一小段的中点时刻；速度用 ±h 的中心差分（运动表的步长）
		for(Lbm1Fin& f : fins) {
			float3 H; float ang; lbm1_pose(f, tm, dx, lo, H, ang);
			const float3 v((float)((f.kin.at(f.kin.hx, tm+h)-f.kin.at(f.kin.hx, tm-h))/(2.0*h)*dt/dx)+eps, 0.0f, (float)((f.kin.at(f.kin.hz, tm+h)-f.kin.at(f.kin.hz, tm-h))/(2.0*h)*dt/dx));
			const float om = (float)(radians(f.kin.at(f.kin.fin, tm+h)-f.kin.at(f.kin.fin, tm-h))/(2.0*h)*dt); // 绕 +y，弧度每步
			lbm.voxelize_mesh_on_device(f.mesh, f.flag, H, v, float3(0.0f, om, 0.0f));
		}
		lbm.run(revox, lbm_T);
		buf += to_string((float)((double)lbm.get_t()*dt), 7u);
		for(Lbm1Fin& f : fins) { const float3 F = lbm.object_force(f.flag); buf += " "+to_string(units.si_F(F.x), 9u)+" "+to_string(units.si_F(F.y), 9u)+" "+to_string(units.si_F(F.z), 9u); }
		buf += "\n";
		if(++k%500ull==0ull) {
			std::ofstream fo(out+"force.dat", std::ios::app); fo<<buf; fo.close(); buf = "";
			print_info("t = "+to_string((float)((double)lbm.get_t()*dt), 4u)+" s of "+to_string((float)si_T, 2u)+", step "+to_string(lbm.get_t())+"/"+to_string(lbm_T)+", elapsed "+to_string((float)clock.stop(), 0u)+" s");
		}
	}
	std::ofstream fo(out+"force.dat", std::ios::app); fo<<buf; fo.close();
	write_file(out+"DONE", "steps "+to_string(lbm_T)+" elapsed_s "+to_string((float)clock.stop(), 1u)+"\n");
	print_info("LBM1 done, elapsed "+to_string((float)clock.stop(), 1u)+" s");
}
#endif // BENCHMARK
