# -*- coding: utf-8 -*-
"""Run Q2 optimization, sensitivity sweeps, and numerical audit exports.

Outputs numerical tables for observation-point selection and visualization."""
import sys, os, csv, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from math import cos, sin, radians, degrees, sqrt, atan2, hypot
from q2_observation_optimization import (support_boundary, max_dist_to_support, in_frec,
                         rbound, frec_margin,
                         theta2_range, worst_diam, region_diam, region_diam_bruteforce,
                         region_polygon, build_P1, minimum_enclosing_circle,
                         diameter_by_enumeration, poly_area, N_ARC, theta2_candidates,
                         project_to_frec_boundary, frec_boundary_point)

DELTA = 1.0; R0 = 1800.0; R_MIN = 1000.0; D_CLR = 20.0

# ---------------- 候选区域统计窗口（唯一来源，图与表共用）----------------
# 窗口必须**完整覆盖 F_rec**，否则 A_2 的格点数就不是可行域的忠实测度。
# 耦合口径（方案B）下 F_rec* 的精确极值已实测：
#   x_min = 0（由「远弧端点 (1500u, R*=1500)」给出；常数口径下该端为 500.1143 m）
#   x_max = 1004.999234668（由「近弧端点 (5u, R=1000)」给出）
# 故取窗口 x ∈ [-50, 1050]、y ∈ [-1000, 1000]，四边各留 ≥ 60 m 余量。
WIN_X0, WIN_X1 = -50.0, 1050.0
WIN_Y0, WIN_Y1 = -1000.0, 1000.0
WIN_GRID = 10.0

# ==================================================================
#  并行批量求值
# ==================================================================
#  技术瓶颈不在算法复杂度而在常数：一次 worst_diam 要在 2m∪11=415 个 θ2 上各做一次
#  "半平面裁剪 + 旋转卡壳"，中心构型单点实测约 57 ms（见 code/q2_solver_audit.py
#  的 cost_per_point_screen_11 行）。分层搜索的
#  每一层——粗网格、深度抛光、绘图网格——都是对成百上千个互不依赖的 S2 各算一次，
#  故把这些网格型循环交给 multiprocessing。**点数与每点的计算函数都不变，只是并行执行，
#  故结果与串行逐位一致**（排序键相同、同值取先出现者）。
#
#  ★ 内存是这里唯一的硬约束，两次踩坑都出在进程池的复用策略上：
#    1) δ 灵敏度要对 5 个不同的 δ 各搜一遍，若 δ 不进池的键就会同时存在 5 个池 × 14 个子进程
#       = 70 个解释器，每个都要 import numpy（约 40 MB 起步），提交内存撞穿页面文件——实测报
#       "OSError: [WinError 1455] 页面文件太小" 并伴随大量 "OpenBLAS error: Memory
#       allocation still failed"。
#    2) ★ 但 δ 也**不能**只随任务传递：池的全局量 samples 与 p1 都是 δ 的函数
#       （supp_1 的方向锥张角、F_rec 的支撑多边形都随 δ 变）。若键里没有 δ，第一次建的池
#       会把它的 δ 固化下来，之后的 δ 档位就在"δ=第一个值的支撑集 + 本档的锥宽"这种
#       混合口径下求值——实测 δ 灵敏度五档全部被 δ=0.8 的支撑集污染，
#       grid_J 的 J 场也随之整体偏低（同一格点 (840,-550)：污染值 97.44，正确值 110.99），
#       连带 A_η 的格点数从 8 虚增到 496。
#       故正确做法是：δ 进键，并且**键一变就关旧池**——这样同时只有一个池驻留（内存可控），
#       且全局量永远与当前 δ 匹配。代价只是每换一个 δ 重建一次池（约 2 s）。
_POOL_KEY_NOTE = 'delta 参与键，保证 _W_G 的 samples/p1 与当前 delta 一致'
_W_G = {}
_POOL_CACHE = {}
_POOL_KEY = [None]


def _w_init(samples, p1, S1, theta1, delta, rb=None):
    _W_G['samples'] = samples
    _W_G['p1'] = p1
    _W_G['S1'] = S1
    _W_G['theta1'] = theta1
    _W_G['delta'] = delta
    _W_G['rb'] = rb


def _w_eval(task):
    S2, n_theta, bp = task
    return worst_diam(S2, _W_G['samples'], _W_G['S1'], _W_G['theta1'],
                      n_theta=n_theta, delta=_W_G['delta'], p1=_W_G['p1'],
                      breakpoints=bp)[0]


def _w_seed(task):
    """一个种子点的完整罗盘局部搜索（各盆地相互独立，故可并行）。"""
    S2, n_theta, steps = task
    return _local_search(S2, _W_G['S1'], _W_G['theta1'], _W_G['samples'],
                         _W_G['p1'], _W_G['delta'], n_theta, steps=tuple(steps),
                         rb=_W_G.get('rb'))


def _rb_key(rb):
    """接收半径数组 rb 的指纹，用于进程池的键。

    ★ 为什么必须有：耦合口径下 rb = [max(R_recv, ‖S1-g‖)]_g 是 R_recv 的函数，而
      R 灵敏度要对 5 个不同的 R_recv 各搜一遍；`_w_seed`（种子罗盘）用 rb 做"是否越出
      F_rec"的过滤。若键里不含 rb，第一次建的池会把它的 rb 固化，之后的档位就在
      "上一档的 rb + 本档的样本"这种混合口径下求值——与当年 δ 不进键导致的污染同类。
    rb 由 (samples, S1, r_min) 完全决定，而 samples/S1 已在键里，故 (min,max,len)
    足以区分本文的 r_min ∈ [1000,1500]（该区间内 min(rb) ≡ r_min）。
    """
    if rb is None:
        return None
    return (round(float(min(rb)), 9), round(float(max(rb)), 9), len(rb))


def _pool_for(samples, p1, S1, theta1, delta, rb=None):
    """取（或重建）与当前 (S1,θ1,δ) 匹配的进程池；建不起来则返回 None（调用方退化串行）。

    ★ 健壮性：Windows spawn 在内存紧张或被外部清理进程干扰时，会在
      Popool._repopulate_pool_static → DuplicateHandle 处抛 PermissionError
      [WinError 5]（子进程刚起就死，句柄已失效）。该异常发生在池的守护线程里，
      会让 pool.map 永久阻塞（实测 run7 因此卡死 15 min 才由线程异常收场）。
      故建池失败一律捕获并退化为串行，宁可慢也不整段报废。
    """
    import multiprocessing as mp
    n = max(1, min(6, (mp.cpu_count() or 2) - 2))   # 6 个 worker 已足够，且省内存
    # ★ R_recv（接收半径假设）必须进键：耦合口径下 rb = R*(g) 是 r_min 的函数，
    #   而 R 灵敏度要对 5 个不同的 R_recv 各搜一遍。若键里没有它，第一次建的池会把
    #   它的 rb 固化，之后的档位就在"上一档的 rb + 本档的 samples"这种混合口径下求值
    #   ——与当年 δ 不进键导致的污染是同一类错误。此处以 rb 的指纹进键。
    key = (S1, theta1, round(float(delta), 9), _rb_key(rb), n)
    if _POOL_KEY[0] == key and _POOL_CACHE.get(key) is not None:
        return _POOL_CACHE[key]
    try:
        _close_pools()                  # ★ 换键即淘汰，保证同时只有一个池驻留
        p = mp.Pool(processes=n, initializer=_w_init,
                    initargs=(samples, p1, S1, theta1, delta, rb),
                    maxtasksperchild=400)   # 限制子进程内存增长
        _POOL_CACHE[key] = p
        _POOL_KEY[0] = key
        return p
    except Exception as e:
        print(f'    [warn] 进程池创建失败（{type(e).__name__}: {e}），本段退化为串行')
        _POOL_CACHE.clear(); _POOL_KEY[0] = None
        return None


def _close_pools():
    for p in list(_POOL_CACHE.values()):
        try:
            p.terminate(); p.join()
        except Exception:
            pass
    _POOL_CACHE.clear()
    _POOL_KEY[0] = None


def _eval_batch(pts, samples, p1, S1, theta1, delta, n_theta, bp=True, min_par=40, rb=None):
    """批量求 J(S2)。点数 < min_par 时串行（进程间通信的固定开销不值得）。

    rb 不参与 J 的计算，只需传给 _pool_for 用作池的键，保证池里 worker 的 rb 与
    当前接收半径口径一致（_w_seed 会用它做 F_rec 过滤）。"""
    pts = list(pts)
    if not pts:
        return []
    if len(pts) < min_par:
        return [worst_diam(S2, samples, S1, theta1, n_theta=n_theta, delta=delta,
                           p1=p1, breakpoints=bp)[0] for S2 in pts]
    pool = _pool_for(samples, p1, S1, theta1, delta, rb)
    if pool is not None:
        try:
            return pool.map(_w_eval, [(S2, n_theta, bp) for S2 in pts], chunksize=4)
        except Exception as e:
            print(f'    [warn] 并行求值失败（{type(e).__name__}: {e}），退化为串行')
            _close_pools()
    return [worst_diam(S2, samples, S1, theta1, n_theta=n_theta, delta=delta,
                       p1=p1, breakpoints=bp)[0] for S2 in pts]


def _rederive(samples, p1, S1, theta1, S2b, delta, n_uniform=201):
    """在给定 S2 上用完整候选集（断点 ∪ n_uniform 点均匀兜底）精算：
    最坏直径 J、最坏 θ2、最坏分支最小覆盖圆半径 R_min、maxdist、R2 多边形。
    ★ 不再用"4001 点均匀采样"：J(θ2) 有窄尖峰，均匀采样会漏
    （实测见 data/q2_solver_audit.csv 的 coarse_underestimate 行）。"""
    Jw, thw = -1.0, None
    for th in theta2_candidates(S2b, samples, delta=delta, n_uniform=n_uniform):
        st, D = region_diam(S1, theta1, S2b, th, delta=delta, p1=p1)
        if D is not None and D > Jw:
            Jw, thw = D, th
    poly = region_polygon(S1, theta1, S2b, thw, delta=delta, p1=p1)
    if len(poly) >= 3:
        _, rm = minimum_enclosing_circle(poly)
    else:
        rm = 0.0
    dmax = max_dist_to_support(S2b, samples)
    return Jw, thw, rm, dmax, poly


def _local_search(S2_0, S1, theta1, samples, p1, delta, n_theta, steps=(10.0, 3.0, 1.0),
                  max_iter=40, rb=None):
    """从 S2_0 出发的罗盘搜索（8 邻域取最优者，步长逐级缩小）；只在 F_rec 内移动。

    J 在 F_rec 内的下降方向指向可行域边界，故逐级缩小步长可收敛到该盆地内
    ∂F_rec 上的局部最小。
    ★ 廉价准则只用于给邻点**排序**，不能用于**筛掉**邻点：均匀 θ2 会系统性低估 J
      （81 点最多低估 6.52 m，见正文 §4.4），在 J≈30 m 的东缘切向构型上这种偏置
      足以颠倒邻点次序。早期版本只精算"均匀准则前 2 名"，实测因此停在一个 J 高
      更高的邻点上（10 m 邻域内 Ĵ 最小的邻点为 28.866949 m，真值 28.793213 m，
      见 data/q2_solver_audit.csv 的 roam_neighbor 行）。
      现改为按廉价次序**逐个精算、遇改进即停**（first-improvement 罗盘下降）：它用于
      降低单次局部搜索成本（多数迭代只精算 1~2 个邻点），但**不保证**看到本轮全部
      8 个邻点——若第 1 个邻点只改善 0.1 m 而第 5 个能改善 2 m，本轮会在第 1 个处
      提前停。该路径依赖由多盆地多种子搜索、逐级缩小步长、四级深度抛光与最终统一
      精算共同抑制，而非由单轮罗盘保证全局最优。
    max_iter 限制每级步长的最大迭代数，防止沿边界长时间平移导致耗时失控。
    """
    S2 = S2_0
    if rb is None:
        rb = rbound(samples, S1)
    J, th, _ = worst_diam(S2, samples, S1, theta1, n_theta=n_theta, delta=delta, p1=p1)
    for g in steps:
        for _ in range(max_iter):
            nb = []
            for dx in (-g, 0.0, g):
                for dy in (-g, 0.0, g):
                    if dx == 0.0 and dy == 0.0:
                        continue
                    cand = (S2[0] + dx, S2[1] + dy)
                    if frec_margin(cand, samples, rb) > 2e-3:
                        continue                       # 越出 F_rec
                    Jq, _, _ = worst_diam(cand, samples, S1, theta1, n_theta=81,
                                          delta=delta, p1=p1, breakpoints=False)
                    nb.append((Jq, cand))
            if not nb:
                break
            nb.sort(key=lambda z: z[0])
            improved = False
            for _Jq, cand in nb:
                Jc, thc, _ = worst_diam(cand, samples, S1, theta1, n_theta=n_theta,
                                        delta=delta, p1=p1)
                if Jc < J - 1e-12:
                    J, th, S2, improved = Jc, thc, cand, True
                    break
            if not improved:
                break
    return J, S2, th


def solve_precise(S1, theta1, delta=1.0, R_recv=1000.0, grid=25.0,
                  topk=8, n_theta_screen=11, n_theta_fine=11, n_theta_final=201,
                  min_sep=60.0, deep=True):
    """分层搜索 min J(S2)，返回 (S2*, J*, θ2_worst, R_min_worst, maxdist, R2 顶点表)。

    R_recv 为接收半径假设（默认最坏 1000 m）。四段结构：
      ① 以 grid 为步长的粗网格扫遍 F_rec，用**完整断点集**准则挑出 J 最小的 topk 个
         格点作为初始盆地（按 min_sep 去重，避免同一盆地重复占位）；
      ② 每个盆地做罗盘搜索（步长 10→3→1 m），准则同为完整断点集；
      ③ 对最优收敛点做 ±6 m/1 m 补扫；
      ③b deep=True 时再加两级多分辨率深度抛光（±100 m/10 m → ±10 m/1 m）＋罗盘收敛，
         用于跨出罗盘搜索的单盆地上限；
      ④ 把最优解沿 ∇d_max 投到 ∂F_rec，再沿 ∂F_rec 做四级方位角扫描并精算比较。

    ★ 粗筛为什么必须也用断点集，而不能用"纯均匀 θ2 的廉价准则"：
      J(θ2) 在锥窗内有窄尖峰，纯均匀 θ2 会**系统性低估** J（81 点最多低估 6.52 m，
      见 data/q2_solver_audit.csv 的 coarse_underestimate 行），且低估量随位置变化。在 J 只有 30 m 量级的东缘切向构型上，
      1.14 m 的排序误差就足以把真最优所在的盆地挤出种子名单：早期版本用 81 点均匀粗筛、
      并且只精算"前 2 名"邻点，实测返回 (2072,435) 处 J=29.9342 m，而同一可行域内
      (1339,419) 处 J=28.7932 m（复算见 data/q2_solver_audit.csv，差 1.1410 m，且 29.93>28.79
      出现在更大的可行域上，逻辑上自相矛盾，据此判定为求解器缺陷）。
      改用断点集后，准则在 0.1 m 以内已是精确的（断点集给出的最大值比 4001 点均匀密扫还高
      0.0258~0.0938 m），排序可靠；单点成本约 57 ms（2m ∪ 11 点，共 415 个候选），
      仍在全网格可承受范围内。
    """
    if deep:
        # 深度搜索时适度扩大种子池：粗筛准则（断点集）已是精确的，排序可靠，
        # 故种子只需覆盖「网格分辨率带来的格点差」这一层不确定性，10 个足够；
        # 真正的跨盆地能力由第 (4) 步的两级深度抛光提供。
        topk = max(int(topk), 10)
    samples = support_boundary(S1, theta1, delta=delta, n_ang=N_ARC, n_rad=801)
    p1 = build_P1(S1, theta1, delta=delta)
    # ★ 接收半径口径：R*(g) 与 S2 的可行域判定
    #   耦合口径（本文采用） R*(g) = max(R_recv, ||S1-g||)
    #   —— 源 g 已在 S1 被收到过，故其实际接收半径必 ≥ ||S1-g||；R_recv 是题面下界。
    #   R_recv 灵敏度就是对这个常数下界取不同值重搜，耦合项照旧生效。
    rb = rbound(samples, S1, R_recv)
    ux, uy = cos(radians(theta1)), sin(radians(theta1))
    vx, vy = -uy, ux

    def in_rec(S2, tol=2e-3):
        return frec_margin(S2, samples, rb) <= tol

    def uv2xy(u, v):
        return (S1[0] + u * ux + v * vx, S1[1] + u * uy + v * vy)

    u_lo = u_hi = None
    for u in np.arange(0.0, 1800.0, 5.0):
        if in_rec(uv2xy(u, 0.0)):
            if u_lo is None:
                u_lo = u
            u_hi = u
    if u_lo is None:
        return None, float('inf'), None, None, None, None
    u_mid = (u_lo + u_hi) / 2
    # ★ v 的搜索区间必须取 F_rec 在 u=u_mid 上的**完整**可行区间 [v_lo, v_hi]，
    #   不能只取 [v_lo, 0]。"半区间 + 镜像"只在 supp_1 关于 θ1 轴对称时才成立
    #   （中心向东、东缘向内属此列）；东缘切向构型下 supp_1 被 Ω 裁剪后已不关于
    #   θ1 轴对称（Ω 的圆心在原点而不在 S1），半区间会系统性漏掉真最优——
    #   实测该构型半区间给 J=29.9342 m，全区间给 J=28.7989 m（差 3.8%）。
    v_lo = v_hi = None
    for v in np.arange(-1800.0, 1800.0, 5.0):
        if in_rec(uv2xy(u_mid, v)):
            if v_lo is None:
                v_lo = v
            v_hi = v
    if v_lo is None:
        return None, float('inf'), None, None, None, None

    uvs, SS = [], []
    for u in np.arange(u_lo, u_hi + grid, grid):
        for v in np.arange(v_lo, v_hi + grid, grid):
            S2 = uv2xy(u, v)
            if not in_rec(S2):
                continue
            uvs.append((u, v))
            SS.append(S2)
    if not SS:
        return None, float('inf'), None, None, None, None
    JS = _eval_batch(SS, samples, p1, S1, theta1, delta, n_theta_screen, rb=rb)
    cands = [(J, u, v) for J, (u, v) in zip(JS, uvs)]
    cands.sort(key=lambda z: z[0])
    seeds, used = [], []
    for _J, u, v in cands:
        if any(hypot(u - a, v - b) < min_sep for a, b in used):
            continue
        used.append((u, v))
        seeds.append((u, v))
        if len(seeds) >= topk:
            break

    conv = []
    seed_pts = [uv2xy(su, sv) for su, sv in seeds]
    pool = _pool_for(samples, p1, S1, theta1, delta, rb) if len(seed_pts) >= 3 else None
    if pool is not None:
        # 各盆地相互独立 ⟹ 并行；结果与串行逐位一致（只是把 for 换成 map）。
        try:
            for r in pool.map(_w_seed, [(p0, n_theta_fine, (10.0, 3.0, 1.0))
                                        for p0 in seed_pts], chunksize=1):
                conv.append(r)
        except Exception as e:
            print(f'    [warn] 种子并行失败（{type(e).__name__}），退化为串行')
            _close_pools(); pool = None; conv = []
    if not conv:
        for p0 in seed_pts:
            conv.append(_local_search(p0, S1, theta1, samples, p1,
                                      delta, n_theta_fine, rb=rb))
    conv.sort(key=lambda z: z[0])

    # ③ 稠密抛光：最优点附近 J 极为平坦（相邻格点差异常在 0.01 m 量级），
    #    罗盘搜索可能停在稍差的邻点上；对最优收敛点再做一次 ±6 m 的 1 m 重扫补齐。
    pol = None
    c3 = [(conv[0][1][0] + dx, conv[0][1][1] + dy)
          for dx in np.arange(-6.0, 6.0 + 1e-9, 1.0)
          for dy in np.arange(-6.0, 6.0 + 1e-9, 1.0)]
    c3 = [p0 for p0 in c3 if frec_margin(p0, samples, rb) <= 2e-3]
    if c3:
        J3 = _eval_batch(c3, samples, p1, S1, theta1, delta, n_theta_fine, rb=rb)
        k3 = int(np.argmin(J3))
        pol = (J3[k3], c3[k3], None)
    if pol is not None and pol[0] < conv[0][0]:
        conv.append(pol)
        conv.sort(key=lambda z: z[0])

    # ③b 深度抛光（deep=True，四级、先宽后窄）。必要性：罗盘搜索只沿单点邻域逐格下降，
    #     而粗网格的种子点可能落在"另一个盆地"上（中心构型的粗筛种子在 (840,550)、
    #     真最优在 (843.1,−545.4)，相距 1095 m），罗盘无法跨越这种距离，必须由网格接力。
    #     A  级 ±100 m/10 m：在种子附近粗定位；
    #     A2 级 ±60 m/20 m ：中尺度环带，把"相邻盆地"纳入视野；
    #     B  级 ±20 m/2 m  ：**关键一级**——J = max(两支光滑函数)，最优点是一道斜向窄谷
    #       的两支交点，谷宽仅数米，10 m 网格会整条跳过，必须用 2 m 分辨率把它找回；
    #     C  级 ±8 m/1 m   ：去掉 B 级的 2 m 离散化（窗口取 ±8 m，因谷底斜向、需覆盖对角邻格）；
    #     末点再由 2→1 m 罗盘收敛。空间搜索一律用 n_theta_fine；J 的精确值统一留到
    #     finals 用 n_theta_final 重算，避免把最贵的准则用在纯空间搜索上。
    #     ★ 这一级只把候选点（网格最优点）追加进 conv，不假设它一定更好——最终仍由
    #     finals 用统一准则排序决定谁胜出，故"广域网格"不会带来偏差，只带来覆盖。
    if deep:
        def _grid_best(center, half, stp, cur):
            """以 center 为心、±half 范围、stp 步长的完整断点集网格；cur 为已有最优候选。
            np.argmin 取首个最小值，与原串行写法"仅在严格更小时替换"逐位等价。"""
            pts = [(center[0] + dx, center[1] + dy)
                   for dx in np.arange(-half, half + 1e-9, stp)
                   for dy in np.arange(-half, half + 1e-9, stp)]
            pts = [p0 for p0 in pts if frec_margin(p0, samples, rb) <= 2e-3]
            if not pts:
                return cur
            JS = _eval_batch(pts, samples, p1, S1, theta1, delta, n_theta_fine, rb=rb)
            k = int(np.argmin(JS))
            if cur is None or JS[k] < cur[0]:
                cur = (JS[k], pts[k], None)
            return cur

        bd = _grid_best(conv[0][1], 100.0, 10.0, None)      # A 级：宽域粗扫（±100 m/10 m）
        if bd is not None:
            # A2 级（±60 m/20 m）：A 级若一路走到可行域的某个角落，B 级的 ±20 m 窗口
            #   就够不着真正的最优盆地；以 A 级最优点为心再做一圈 20 m 步长的中尺度网格，
            #   把"相邻盆地"纳入视野。中心构型的种子点落在 (840,550)、真最优在 (843.1,−545.4)，
            #   两者相距约 1095 m——这类跨半域的跳转只能靠 A2/A 的广域网格发现，不能靠罗盘。
            bd = _grid_best(bd[1], 60.0, 20.0, bd)
            bd = _grid_best(bd[1], 20.0, 2.0, bd)           # B 级：把窄谷找回来（关键一级）
            bd = _grid_best(bd[1], 8.0, 1.0, bd)            # C 级：1 m 去离散化
            Jd, S2d, thd = _local_search(bd[1], S1, theta1, samples, p1, delta,
                                        n_theta_fine, steps=(2.0, 1.0), rb=rb)
            conv.append((Jd, S2d, thd))
            conv.sort(key=lambda z: z[0])

    # ④ 边界精化：把"∂F_rec 也是搜索空间的一部分"落到实处。
    #    ★ 先澄清一个容易写过头的事实：f(S2)=max_g(‖S2−g‖−R*(g)) 是凸函数且在最远处 ‖∇f‖≡1，
    #      F_rec={f≤0} 为凸集，但这**只推出"若最优点不在 F_rec 内部，则它在 ∂F_rec 上"**，
    #      并不推出"全局最优一定落在 ∂F_rec 上"：本文三类构型里，中心构型与东缘向内构型
    #      的最坏分支都恰好贴到 R*(g)（最优在边界），而东缘切向构型的 margin 只有负几百米
    #      （最优严格落在 F_rec 内部，精确值见 data/q2_checks.csv 的 k5_east_tangent 行）。
    #      故④不承担"求出最优"的职责，只承担"别漏掉边界这一带"的职责。
    #    做法：4a 沿 ∇f 把③③b 的收敛点精确投到 ∂F_rec（f = 0）；
    #      4b 取 C0 = P1 的最小覆盖圆圆心（必落在 F_rec 内部；F_rec 凸 ⟹ 自 C0 出发的
    #         射线与 ∂F_rec 恰交一次），用方位角 φ 把 ∂F_rec 单参数化，在投影点邻域
    #         做四级一维扫描 ±30°/1° → ±1°/0.05° → ±1°/0.1° → ±0.15°/0.02°。
    #    ★ 该性质与 R* 取常数还是取 max(r_min,‖S1-g‖) 无关，两种口径共用同一套投影/参数化。
    #    ★ 边界扫描的步长按角度给，圆弧上的弧长步长随之变化：在半径约 1000 m 处
    #      1° ≈ 17.5 m、0.05° ≈ 0.87 m，故前两级只作"粗定位边界上的大致方位"，
    #      后两级的 0.1°/0.02° 才进入米级。四级都**只用完整断点集准则**
    #      （_cheap 与 _fullJ 的差别仅是均匀兜底点数 11 与 201，断点集完全相同），
    #      故不存在"廉价准则把最优方位角带偏"的问题。边界扫描只是**候选点来源之一**：
    #      它的末点与②③③b 的全部收敛点一起进 finals 统一精算、按同一准则排序，
    #      由 J 最小者胜出，因此边界带与内部带之间不需要人为定权重。
    def _cheap(S2):
        return worst_diam(S2, samples, S1, theta1, n_theta=n_theta_screen,
                          delta=delta, p1=p1, breakpoints=True)[0]

    def _fullJ(S2):
        return worst_diam(S2, samples, S1, theta1, n_theta=n_theta_final,
                          delta=delta, p1=p1)[0]

    C0 = minimum_enclosing_circle(p1)[0]
    Pb, dPb = project_to_frec_boundary(conv[0][1], samples, rb=rb)
    if dPb > 1e-6:
        Pb = conv[0][1]
    phi0 = degrees(atan2(Pb[1] - C0[1], Pb[0] - C0[0]))
    bJ, bphi, bPt = _fullJ(Pb), phi0, Pb
    # 四级扫描的跨度与步长：±30°/1° → ±1°/0.05° → ±1°/0.1° → ±0.15°/0.02°；
    # 每级开始时按本级准则重置基准 bJ（_cheap 与 _fullJ 的均匀兜底点数不同，数值不可直接比较）。
    bound_pts = [Pb]
    for span, step, crit in ((30.0, 1.0, _cheap), (1.0, 0.05, _cheap),
                             (1.0, 0.10, _fullJ), (0.15, 0.02, _fullJ)):
        bJ, bphi = crit(bPt), phi0
        for ph in np.linspace(phi0 - span, phi0 + span, int(round(2 * span / step)) + 1):
            Q, dq = frec_boundary_point(C0, ph, samples, rb=rb)
            if abs(dq) > 1e-5:
                continue
            bound_pts.append(Q)
            Jq = crit(Q)
            if Jq < bJ:
                bJ, bphi, bPt = Jq, ph, Q
        phi0 = bphi

    # ★ finals 只保留"候选点里 J 最小的一小批"再精算，而不是把所有 150 多个扫描点全部
    #   重算一遍：扫描点里有大量 J 明显偏大的方位（远离最优），逐个用 n_theta_final=201
    #   重算既慢又无意义。这里先用已有的廉价值排序取前 n_final 名，再统一用 _rederive
    #   精算——由于 _rederive 与廉价准则的断点集完全相同、只差均匀兜底点数，廉价排序
    #   不会把真正的最优点挤出入围名单（保留 12 个已远超需要）。
    cheap_all = [(_cheap(S2b), S2b) for (_J, S2b, _th) in conv]
    for Q in bound_pts:
        cheap_all.append((_cheap(Q), Q))
    cheap_all.sort(key=lambda z: z[0])
    seen, finals = [], []
    for _Jc, S2b in cheap_all:
        if any(hypot(S2b[0] - a, S2b[1] - b) < 1e-6 for a, b in seen):
            continue
        seen.append(S2b); finals.append(S2b)
        if len(finals) >= 12:
            break
    results = []
    for S2b in finals:
        Jw, thw, rm, dmax, wpoly = _rederive(samples, p1, S1, theta1, S2b, delta,
                                             n_uniform=n_theta_final)
        results.append((Jw, S2b, thw, rm, dmax, wpoly))
    results.sort(key=lambda z: z[0])
    # ★ 返回顺序必须与调用方解包一致：(S2*, J*, θ2_worst, R_min, maxdist, R2 顶点表)
    #   本函数内部的排序键是 J*（results 元素为 (Jw, S2b, thw, rm, dmax, wpoly)），
    #   故此处必须换序返回，否则调用方会把 J* 当成坐标用（此前的 TypeError 即源于此）。
    Jw, S2b, thw, rm, dmax, wpoly = results[0]
    return S2b, Jw, thw, rm, dmax, wpoly


def sensitivity_delta(S1, theta1):
    """δ ∈ [0.8,1.2] 灵敏度（逐档重搜最优 S2、重算 J）。

    ★ 粗网格取 50 m（主答案用 25 m）、种子池取 4：本表是**趋势**量，只需各档位在同一套
      可靠准则（完整断点集）下互相可比。级末仍保留局部罗盘搜索与 ∂F_rec 边界精化，
      故每档给出的最优 S2 与被搜索到的 J 都是真实搜到的值（不是廉价筛选值），
      只是盆地枚举密度略低于主答案。
    """
    rows = []
    for d in (0.8, 0.9, 1.0, 1.1, 1.2):
        S2b, Jw, thw, rm, dm, _ = solve_precise(S1, theta1, delta=d, grid=50.0,
                                                topk=4, deep=False)
        rows.append((d, Jw, S2b, thw, rm))
    return rows


def sensitivity_R(S1, theta1):
    """接收半径假设 R ∈ [1000,1500] 灵敏度。
    ★ F_rec = ∩_{g∈P1}B(g, R*(g)) 本身随 R 变，故每个档位必须重新搜索最优 S2。
    ★ 本表只给趋势，故 deep=False 省去约 40 s/档的深度抛光、粗网格放宽到 50 m；
      主答案（R=1000 档）另由 §5.1 的 deep=True 搜索单独给出。
    ★ 耦合口径下 R*(g) = max(R, ‖S1-g‖)：R 取 1500 时耦合项被吸收、退化为常数口径；
      R 取 1000 时耦合项作用最强（这正是本文正式取值所在的那一档）。
    """
    rows = []
    for R in (1000.0, 1125.0, 1250.0, 1375.0, 1500.0):
        S2b, Jw, thw, rm, dm, _ = solve_precise(S1, theta1, R_recv=R, grid=50.0,
                                                topk=4, deep=False)
        rows.append((R, Jw, S2b, thw, rm))
    return rows


def layer_areas(S1, theta1, Jstar, tau_mult=1.2, eta=0.6, grid=10.0, n_theta=11,
                write_grid=True):
    """候选区域三层结构 + 绘图用的 J 网格（一次算完，避免重复计算）。

    A2 = F_rec；Q2(τ) = {J ≤ τ = tau_mult·J*}；A_η = {J ≤ J*+η}。
    统计窗口由 WIN_X0/WIN_X1/WIN_Y0/WIN_Y1 给出（x∈[-50,1050]、y∈[-1000,1000]），
    该窗口**完整覆盖耦合口径下的 F_rec**（x∈[0,1004.9992]、|y|≤938.1470），
    故 A2 的格点数是可行域面积的忠实测度；且与 grid_maxdist.csv 同格，
    MATLAB 侧 grid_J 与 grid_maxdist 天然同格，A2 的格点数可与 nnz(inF) 断言。

    返回 (nA2, nQ, nEta, rows)：
      nA2/nQ/nEta 为格点数（面积 = 格点数 × grid²）；
      rows 为 (x, y, J 或 '') 的完整网格，供写出 data/grid_J.csv。
    ★ 这里与主答案用同一套 θ2 候选集（断点集），保证图上/分层表里的 J 与 J* 同口径。
    ★ F_rec 的判定用耦合口径（in_frec 默认走 CALIBER='coupled'，rb 由 rbound 给出）。
    """
    samples = support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801)
    p1 = build_P1(S1, theta1)
    rb = rbound(samples, S1)
    tau = tau_mult * Jstar
    # ★ 先按原嵌套序（x 外、y 内）铺满整个窗口并记住每个格点在 F_rec 内的判定，
    #   再只把 F_rec 内的点批量并行求 J；最后按同一顺序回填，故 grid_J.csv 的行序
    #   与 grid_maxdist.csv 完全一致（MATLAB 侧 paper_grid3 依赖这个行序）。
    cells = []
    for x in np.arange(WIN_X0, WIN_X1 + 1e-9, grid):
        for y in np.arange(WIN_Y0, WIN_Y1 + 1e-9, grid):
            S2 = (float(x), float(y))
            cells.append((float(x), float(y),
                          S2 if in_frec(S2, samples, S1, rb=rb) else None))
    nA2 = sum(1 for c in cells if c[2] is not None)
    idx = [i for i, c in enumerate(cells) if c[2] is not None]
    JS = _eval_batch([cells[i][2] for i in idx], samples, p1, S1, theta1,
                     DELTA, n_theta, rb=rb)
    Jmap = dict(zip(idx, JS))
    nQ = nEta = 0
    rows = []
    for i, (x, y, S2) in enumerate(cells):
        if S2 is None:
            if write_grid:
                rows.append((x, y, ''))
            continue
        J = Jmap[i]
        if write_grid:
            rows.append((x, y, round(J, 3)))
        if J <= tau:
            nQ += 1
        if J <= Jstar + eta:
            nEta += 1
    return nA2, nQ, nEta, rows


def out_dir():
    """结果目录：优先本工程根下的 data/（交付包结构），否则退化为脚本同目录。"""
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.join(os.path.dirname(here), 'results', 'generated', 'q2')
    os.makedirs(d, exist_ok=True)
    return d


def main():
    out_dir_ = out_dir()
    t0 = time.time()

    S1 = (0.0, 0.0); theta1 = 0.0
    print('=' * 78)
    print('问题2 独立重做（口径B）· 中心构型 S1=(0,0) θ1=0°')
    print('=' * 78)
    c_S2b, c_Jw, c_thw, c_rm, c_dm, c_poly = solve_precise(S1, theta1, grid=25.0)
    # ★ 镜像规范化（与"多构型"一节同一约定，此处是论文主答案，规范化尤其重要）：
    #   中心构型的几何关于 y=0 严格对称，J(x,+y) 与 J(x,−y) 只差浮点噪声（实测
    #   110.969320839 对 110.969320843），返回哪一支取决于舍入 ⟹ 不规范化则数值不可复现。
    #   约定取 y<0 的"南侧解"，另一支由镜像对称给出。
    samples = support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801)
    p1 = build_P1(S1, theta1)
    if abs(c_S2b[1]) > 1e-9:
        S2m = (c_S2b[0], -c_S2b[1])
        Jm, thm, rmm, dmm, polym = _rederive(samples, p1, S1, theta1, S2m, DELTA)
        if Jm < c_Jw - 1e-9 or (abs(Jm - c_Jw) <= 1e-9 and S2m[1] < c_S2b[1]):
            c_S2b, c_Jw, c_thw, c_rm, c_dm, c_poly = S2m, Jm, thm, rmm, dmm, polym
    print(f'最优 S2* = ({c_S2b[0]:.2f}, {c_S2b[1]:.2f})（及关于 θ1 轴的镜像）')
    print(f'最坏后验直径 J* = {c_Jw:.4f} m')
    print(f'最坏 θ2 = {c_thw:.4f}°')
    print(f'maxdist(P1) = {c_dm:.4f} m（贴 F_rec 边界）')
    print(f'最坏分支最小覆盖圆半径 R_min = {c_rm:.4f} m → '
          f'{"可一步清除" if c_rm <= D_CLR else "不可一步清除（需继续观测）"}')

    # 基线对比：名义中点垂线策略。
    # ★ 基线的最坏值必须用与主答案**同一套**估计量（断点集 ∪ 均匀兜底，n_uniform=201），
    #   否则两者不在同一口径下，改进率不可比（早期版本此处误用 4001 点纯均匀网格，
    #   而纯均匀恰恰会漏掉 J(θ2) 的窄尖峰）。
    base = (750.0, 500.0)
    Jb, thb, _, _db, _ = _rederive(samples, p1, S1, theta1, base, DELTA, n_uniform=201)
    imp = 100 * (Jb - c_Jw) / Jb
    print(f'\n基线 (750,500) 最坏 D2 = {Jb:.4f} m（最坏 θ2 = {thb:.4f}°），改进 {imp:.2f}%')

    # 候选区域面积（三层结构一次算完，见下方 layer_areas）
    print(f'\n候选区域与三层结构面积见下方（10m 网格）')

    # ============ 多构型 ============
    print('\n' + '=' * 78)
    print('多构型（说明性）')
    print('=' * 78)
    configs = [
        ('中心向东', (0.0, 0.0), 0.0),
        ('东缘向内', (1700.0, 0.0), 180.0),
        ('东缘切向', (1700.0, 0.0), 90.0),
    ]
    config_rows = []
    for name, s1, t1 in configs:
        S2b, Jw, thw, rm, dm, _ = solve_precise(s1, t1, grid=25.0)
        _close_pools()          # ★ 每组构型的 (S1,θ1) 不同，立刻释放上一组的池
        # ★ 关于对称轴的两个等价最优解先规范化再入表。中心构型与东缘向内构型（θ1=0°/180°）
        #   的整个几何关于 y=0 严格对称，故 J(x,+y) 与 J(x,−y) 在数值上只差 1e-9 量级的
        #   浮点噪声，返回哪一支完全取决于舍入——同一份代码换台机器就可能报出相反符号，
        #   论文数值便不可复现。约定：在两支等价的解里取 y<0 的那一支（"南侧解"），
        #   另一支由镜像对称直接给出，正文只陈述一次。东缘切向构型（θ1=90°）不具该对称性，
        #   其镜像点 J 明显更大，下面的比较会自然把它排除，故本规范化对它无副作用。
        ss_c = support_boundary(s1, t1, n_ang=N_ARC, n_rad=801)
        p1_c = build_P1(s1, t1)
        if abs(S2b[1]) > 1e-9:
            S2m = (S2b[0], -S2b[1])
            Jm, thm, rmm, dmm, _ = _rederive(ss_c, p1_c, s1, t1, S2m, DELTA)
            if Jm < Jw - 1e-9:
                S2b, Jw, thw, rm, dm = S2m, Jm, thm, rmm, dmm
            elif abs(Jm - Jw) <= 1e-9 and S2m[1] < S2b[1]:
                S2b, Jw, thw, rm, dm = S2m, Jm, thm, rmm, dmm
        clr = '是' if (rm is not None and rm <= D_CLR) else '否'
        print(f'{name}: S1={s1} θ1={t1}° → S2*=({S2b[0]:.2f},{S2b[1]:.2f}), '
              f'J*={Jw:.4f} m, θ2w={thw:.4f}°, R_min={rm:.4f} m, 清除={clr}')
        config_rows.append((name, s1, t1, S2b, Jw, rm, clr, thw))

    # 东缘切向的位置口径核对（要求 S2 ∈ Ω，R0=1800）。
    # ★ 这里刻意**不**重跑一遍独立粗网格。早期版本的做法是"在 Ω 内从头再搜一次"，但它用
    #   5 m 粗网格起步、精化层也只有 5→1 m 的罗盘，收敛程度低于无约束的 solve_precise，
    #   于是报出一个比无约束解更差的 J（28.7989 对 28.7932），看上去像"口径收紧导致劣化"，
    #   实际上只是求解器没收敛——约束是否起作用是完全可以直接判断的事实：
    #   **若无约束最优本身落在 Ω 内，则它在约束问题里严格可行，两口径必然同解。**
    #   故这里改为：先给出 |S2*| 与 R0 的大小关系判定约束是否作用，再以该点为起点做一次
    #   "限定在 Ω 内"的罗盘收敛作独立核对（约束不起作用时应收敛回同一点）。
    print('\n东缘切向 · 位置口径核对（S2 ∈ Ω，R0=1800）：')
    s1e, t1e = (1700.0, 0.0), 90.0
    samples_e = support_boundary(s1e, t1e, n_ang=N_ARC, n_rad=801)
    p1e = build_P1(s1e, t1e)
    cons_rows = []
    S2e_unc = next(r[3] for r in config_rows if r[0] == '东缘切向')
    r_unc = hypot(S2e_unc[0], S2e_unc[1])
    in_omega = r_unc <= R0
    if in_omega:
        S2e_c = S2e_unc
    else:                                   # 越界则沿径向拉回 Ω 边界再收敛
        k = R0 / r_unc
        S2e_c = (S2e_unc[0] * k, S2e_unc[1] * k)
    for g in (2.0, 1.0):
        for _ in range(40):
            J0 = worst_diam(S2e_c, samples_e, s1e, t1e, n_theta=11, delta=DELTA,
                            p1=p1e)[0]
            best = None
            for dx in (-g, 0.0, g):
                for dy in (-g, 0.0, g):
                    if dx == 0.0 and dy == 0.0:
                        continue
                    cand = (S2e_c[0] + dx, S2e_c[1] + dy)
                    if cand[0] ** 2 + cand[1] ** 2 > R0 ** 2:
                        continue                      # 越出 Ω
                    if not in_frec(cand, samples_e, s1e):
                        continue
                    Jq = worst_diam(cand, samples_e, s1e, t1e, n_theta=11,
                                    delta=DELTA, p1=p1e)[0]
                    if Jq < J0 - 1e-12 and (best is None or Jq < best[0]):
                        best = (Jq, cand)
            if best is None:
                break
            S2e_c = best[1]
    Jcw, thcw, rmc, dmc, _ = _rederive(samples_e, p1e, s1e, t1e, S2e_c, DELTA)
    print(f'    无约束最优 S2* = ({S2e_unc[0]:.2f}, {S2e_unc[1]:.2f})，'
          f'|S2*| = {r_unc:.1f} m {"<=" if in_omega else ">"} {R0:.0f} m '
          f'⟹ Ω 约束{"不作用（同解）" if in_omega else "起作用"}')
    print(f'    Ω 内收敛   S2  = ({S2e_c[0]:.2f}, {S2e_c[1]:.2f}), J = {Jcw:.4f} m, '
          f'θ2w = {thcw:.4f}°, R_min = {rmc:.4f} m, 清除 = {"是" if rmc <= D_CLR else "否"}')
    cons_rows.append(('东缘切向_保守口径_S2inOmega', s1e, t1e, S2e_c, Jcw, rmc, thcw,
                      '是' if rmc <= D_CLR else '否'))

    # ============ 灵敏度 ============
    print('\n' + '=' * 78)
    print('灵敏度分析（中心构型）')
    print('=' * 78)
    sd = sensitivity_delta(S1, theta1)
    print('  δ(°) : 最坏 D2(m) @ 最优 S2')
    for d, J, S2b, thw, rm in sd:
        print(f'   {d:.1f}  :  {J:.4f}  @ ({S2b[0]:.2f},{S2b[1]:.2f}) θ2w={thw:.4f} R_min={rm:.4f}')
    sr = sensitivity_R(S1, theta1)
    print('  接收半径假设 R(m) : 最坏 D2(m) @ 最优 S2')
    for R, J, S2b, thw, rm in sr:
        print(f'   {R:6.0f}          :  {J:.4f}  @ ({S2b[0]:.2f},{S2b[1]:.2f}) θ2w={thw:.4f} R_min={rm:.4f}')

    # ============ 候选区域三层 + 绘图用 J 网格（一次算完）============
    GR = 10.0
    nA2, nQ, nEta, grid_rows = layer_areas(S1, theta1, c_Jw, grid=GR)
    A2, Q2, AE = nA2 * GR * GR, nQ * GR * GR, nEta * GR * GR
    print(f'\n候选区域三层（{GR:.0f} m 网格）：A2 = {A2:.0f} m² ({nA2} 格点)；'
          f'Q2(1.2J*) = {Q2:.0f} m² ({nQ})；A_η = {AE:.0f} m² ({nEta})')

    # ★ 网格自检（out-of-band 复核）：从刚算出的网格里抽 12 个点，用**串行、独立构造**
    #   的 samples/p1 重算 J 并比对。这一步是为了杜绝"批量求值路径与模型直算不一致"这类
    #   静默错误——早期版本因进程池的全局量 samples/p1 与请求的 δ 不匹配，grid_J 整片
    #   偏低（同一格点 (840,−550)：97.44 对正确值 110.99），而 A_η 的格点数从 8 虚增到 496，
    #   表面上看不出任何异常。凡 grid_J 与直算不符，此处即报错中止，不再产出带病的图。
    _ss_chk = support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801)
    _p1_chk = build_P1(S1, theta1)
    _rb_chk = rbound(_ss_chk, S1)
    # 取样点同时覆盖三类：(i) 旧口径也在 F_rec 内的点；(ii) **只有耦合口径才落入的新增区**
    #   （x<500 一带，用来验证新口径的网格与直算一致）；(iii) F_rec 之外的点（对照）。
    _chk_pts = []
    for _fx, _fy in ((840.0, -550.0), (970.0, 20.0), (1000.0, -10.0), (700.0, -400.0),
                     (600.0, 200.0), (900.0, 100.0), (1100.0, 0.0), (843.0, -545.0),
                     (500.0, -100.0), (950.0, -300.0), (1200.0, 0.0), (650.0, 50.0),
                     (200.0, 400.0), (0.0, 0.0), (300.0, 800.0), (100.0, 300.0)):
        _j = worst_diam((_fx, _fy), _ss_chk, S1, theta1, n_theta=11, delta=DELTA,
                        p1=_p1_chk, breakpoints=True)[0] \
            if in_frec((_fx, _fy), _ss_chk, S1, rb=_rb_chk) else None
        _chk_pts.append((_fx, _fy, _j))
    _gmap = {(float(r[0]), float(r[1])): r[2] for r in grid_rows}
    _chk_rows, _maxdiff = [], 0.0
    for _fx, _fy, _j in _chk_pts:
        _gv = _gmap.get((_fx, _fy))
        # ★ 文件里的 J 保留 3 位小数（与 MATLAB 侧读取口径一致），故比对时先做同样的舍入，
        #   否则会把"写盘舍入"误判成"路径不一致"。
        _d = abs(_gv - round(_j, 3)) if (_gv not in (None, '') and _j is not None) else None
        if _d is not None and _d > _maxdiff:
            _maxdiff = _d
        _chk_rows.append((_fx, _fy, _gv, _j, _d, 'F_rec 内' if _j is not None else 'F_rec 外'))
    with open(os.path.join(out_dir_, 'q2_grid_audit.csv'), 'w', newline='',
              encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'grid_J_file', 'direct_J', 'abs_diff_m', 'frec'])
        for row in _chk_rows:
            w.writerow([row[0], row[1], row[2], '' if row[3] is None else round(row[3], 6),
                        '' if row[4] is None else '%.3e' % row[4], row[5]])
    print(f'  网格自检：{len(_chk_rows)} 点直算比对，最大差 = {_maxdiff:.3e} m'
          f'（阈值 1e-6 m，已扣除写盘 3 位小数舍入）→ data/q2_grid_audit.csv')
    assert _maxdiff < 1e-6, f'grid_J 与模型直算不符（最大差 {_maxdiff:.3e} m）'
    with open(os.path.join(out_dir_, 'grid_J.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'J'])
        for (gx, gy, gv) in grid_rows:
            w.writerow([gx, gy, gv])
    print(f'grid_J.csv 已写出（{len(grid_rows)} 行，其中 F_rec 内 {nA2} 点有 J）')

    # ============ 写 CSV ============
    with open(os.path.join(out_dir_, 'q2_center_result.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['S1_x', 'S1_y', 'theta1_deg', 'S2_x', 'S2_y', 'J_worst_m',
                    'theta2_worst_deg', 'R_min_m', 'maxdist_m', 'baseline_x', 'baseline_y',
                    'baseline_J_m', 'improvement_pct'])
        w.writerow([S1[0], S1[1], theta1, c_S2b[0], c_S2b[1], c_Jw, c_thw, c_rm, c_dm,
                    base[0], base[1], Jb, imp])
    with open(os.path.join(out_dir_, 'q2_sensitivity_delta.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f); w.writerow(['delta_deg', 'J_worst_m', 'S2_x', 'S2_y', 'theta2_worst_deg', 'R_min_m'])
        for d, J, S2b, thw, rm in sd:
            w.writerow([d, J, S2b[0], S2b[1], thw, rm])
    with open(os.path.join(out_dir_, 'q2_sensitivity_R.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f); w.writerow(['R_recv_m', 'J_worst_m', 'S2_x', 'S2_y', 'theta2_worst_deg', 'R_min_m'])
        for R, J, S2b, thw, rm in sr:
            w.writerow([R, J, S2b[0], S2b[1], thw, rm])
    with open(os.path.join(out_dir_, 'q2_configs.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['name', 'S1_x', 'S1_y', 'theta1_deg', 'S2_x', 'S2_y', 'J_worst_m',
                    'R_min_m', 'clear', 'theta2_worst_deg'])
        for name, s1, t1, S2b, Jw, rm, clr, thw in config_rows:
            w.writerow([name, s1[0], s1[1], t1, S2b[0], S2b[1], Jw, rm, clr, thw])
        for name, s1, t1, S2b, Jw, rm, thw, clr in cons_rows:
            w.writerow([name, s1[0], s1[1], t1, S2b[0], S2b[1], Jw, rm, clr, thw])
    with open(os.path.join(out_dir_, 'q2_candidate_layers.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['layer', 'grid_m', 'window', 'cells', 'area_m2', 'threshold'])
        _win = 'x[%g,%g] y[%g,%g]' % (WIN_X0, WIN_X1, WIN_Y0, WIN_Y1)
        w.writerow(['A2_hard_feasible', GR, _win, nA2, A2,
                    'max_g(||S2-g||-R*(g))<=0, R*(g)=max(1000,||S1-g||)'])
        w.writerow(['Q2_quality', GR, _win, nQ, Q2,
                    f'J<=1.2*Jstar={1.2 * c_Jw:.6f}'])
        w.writerow(['A_eta_near_optimal', GR, _win, nEta, AE,
                    'J<=Jstar+0.6'])
    # 最坏分支几何（§5.7 用）
    with open(os.path.join(out_dir_, 'q2_worst_branch.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['item', 'value'])
        w.writerow(['S2_x', f'{c_S2b[0]:.6f}']); w.writerow(['S2_y', f'{c_S2b[1]:.6f}'])
        w.writerow(['theta2_worst_deg', f'{c_thw:.6f}'])
        w.writerow(['J_worst_m', f'{c_Jw:.6f}'])
        w.writerow(['R_min_m', f'{c_rm:.6f}'])
        w.writerow(['vertices_n', len(c_poly)])
        if c_poly is not None and len(c_poly) >= 2:
            bi, bj, D = diameter_by_enumeration(c_poly)
            a, b = c_poly[bi], c_poly[bj]
            w.writerow(['diam_end_1', f'({a[0]:.6f},{a[1]:.6f})'])
            w.writerow(['diam_end_2', f'({b[0]:.6f},{b[1]:.6f})'])
            w.writerow(['|diam_end_1|', f'{sqrt(a[0]**2 + a[1]**2):.6f}'])
            w.writerow(['|diam_end_2|', f'{sqrt(b[0]**2 + b[1]**2):.6f}'])
            for k, p in enumerate(c_poly):
                w.writerow([f'vertex_{k}', f'({p[0]:.6f},{p[1]:.6f})'])

    print('\nCSV 已写出：q2_center_result / q2_sensitivity_delta / q2_sensitivity_R / '
          'q2_configs / q2_candidate_layers / q2_worst_branch')
    print(f'总耗时 {time.time()-t0:.1f}s')
    _close_pools()          # 释放多进程池，避免子进程悬挂


if __name__ == '__main__':
    main()
