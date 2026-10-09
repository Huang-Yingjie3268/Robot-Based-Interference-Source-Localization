# -*- coding: utf-8 -*-
"""Observation-point optimization under bounded bearing error.
Uses a clipped physical support region and position-dependent reception bounds.
The posterior diameter is evaluated over candidate second-bearing angles."""
from math import cos, sin, radians, degrees, atan2, sqrt, hypot, pi, tan, asin, acos
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry_halfplanes import (sector_to_halfplanes, halfplane_intersection,
                      diameter_by_enumeration, rotating_calipers)

# ---------------- 题面常量 ----------------
DELTA = 1.0      # 示向度误差上界 (度)
R0 = 1800.0      # 全局目标圆域半径 (m)
R_MIN = 1000.0   # 有效接收半径下确界（最坏情形）
R_MAX = 1500.0   # 有效接收半径上界
D_NEAR = 5.0     # 近距阈值 (m)：≤5 m 信号过强无法获得示向度
D_CLR = 20.0     # 清除阈值 (m)
N_ARC = 201      # 远弧内接折线的分段数（弦高误差 5.7e-6 m；实测对 diam 无影响，见 docstring）

# ---------------- 基础工具 ----------------

def bearing(S, G):
    """从 S 指向 G 的方位角(度, [0,360))。"""
    return degrees(atan2(G[1] - S[1], G[0] - S[0])) % 360.0


def circumcenter(a, b, c):
    """三点外心；退化（近共线）返回 None。"""
    d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return None
    ux = ((a[0]**2 + a[1]**2) * (b[1] - c[1]) + (b[0]**2 + b[1]**2) * (c[1] - a[1])
          + (c[0]**2 + c[1]**2) * (a[1] - b[1])) / d
    uy = ((a[0]**2 + a[1]**2) * (c[0] - b[0]) + (b[0]**2 + b[1]**2) * (a[0] - c[0])
          + (c[0]**2 + c[1]**2) * (b[0] - a[0])) / d
    return (ux, uy)


_MEC_CACHE = {}


def minimum_enclosing_circle(pts):
    """最小覆盖圆（暴力枚举候选圆心，numpy 向量化版）。
    候选：顶点本身 / 任意两点中点 / 任意三点外心；取能覆盖全部顶点的最小半径。
    返回 (center, radius)。

    ★ 与初版的**唯一**差别是性能：候选集合、判定容差（sqrt(d^2) <= r + 1e-6）、
      以及"取最小半径、同半径取先枚举者"的选点规则逐条不变，只把 O(n) 的覆盖判定
      由纯 Python 的 all(...) 换成 numpy 分块判定，并利用"候选按半径升序扫描、
      遇到第一个覆盖全部顶点的候选即可停"这一等价事实提前退出。
      原因：n=203 时候选数约 1.4e6、覆盖判定共约 2.9e8 次算术，初版单次调用耗时
      46 s（本机实测，n=203 单次调用），而本函数在每个 solve_precise 的第④步
      （∂F_rec 单参数化的极心 C0 = P1 的最小覆盖圆圆心）都要调一次，成为整体耗时的
      主要来源。向量化后降到约 4 s，返回的圆心逐位不变。
    ★ 结果按顶点表缓存：同一 P1 在一个进程内会被多次调用（中心构型、其基线点、
      R 灵敏度同档），命中直接复用。仅在 n>=30（昂贵情形）时缓存，避免小多边形占内存。
    """
    n = len(pts)
    if n == 0:
        return (0.0, 0.0), 0.0
    if n == 1:
        return pts[0], 0.0
    if n == 2:
        c = ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
        r = sqrt((pts[0][0] - pts[1][0])**2 + (pts[0][1] - pts[1][1])**2) / 2
        return c, r
    key = tuple(pts) if n >= 30 else None
    if key is not None and key in _MEC_CACHE:
        return _MEC_CACHE[key]

    P = np.asarray(pts, dtype=float)
    x, y = P[:, 0], P[:, 1]
    # --- 候选 1：顶点本身（r=0）---
    Cvert = np.column_stack([x, y, np.zeros(n)])
    # --- 候选 2：任意两点中点 ---
    ii, jj = np.triu_indices(n, 1)
    Cmid = np.column_stack([(x[ii] + x[jj]) / 2.0, (y[ii] + y[jj]) / 2.0,
                            np.hypot(x[ii] - x[jj], y[ii] - y[jj]) / 2.0])
    # --- 候选 3：任意三点外心（逐 i 向量化生成，避免 1.4e6 个 Python 元组）---
    parts = []
    for i in range(n - 2):
        a, b = np.triu_indices(n - i - 1, 1)
        j = a + i + 1
        k = b + i + 1
        ax, ay = x[i], y[i]
        bx, by = x[j], y[j]
        cx, cy = x[k], y[k]
        d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        good = np.abs(d) >= 1e-12
        if not good.any():
            continue
        d = d[good]
        bx, by = bx[good], by[good]
        cx, cy = cx[good], cy[good]
        sa = ax * ax + ay * ay
        sb = bx * bx + by * by
        sc = cx * cx + cy * cy
        ux = (sa * (by - cy) + sb * (cy - ay) + sc * (ay - by)) / d
        uy = (sa * (cx - bx) + sb * (ax - cx) + sc * (bx - ax)) / d
        parts.append(np.column_stack([ux, uy, np.hypot(bx - ux, by - uy)]))
    C = np.vstack([Cvert, Cmid] + parts) if parts else np.vstack([Cvert, Cmid])

    # 按半径升序扫描（stable 保全枚举次序）⟹ 遇到的第一个"覆盖全部顶点"的候选
    # 就是最小覆盖圆，可立即结束（初版是扫完全部候选再取最小，两者等价）。
    order = np.argsort(C[:, 2], kind='stable')
    Cs, Rs = C[order, :2], C[order, 2]
    best_c, best_r = pts[0], float('inf')
    B = 4096
    for s in range(0, len(order), B):
        if Rs[s] >= best_r:          # 已升序 ⟹ 后续候选半径只会更大
            break
        cc = Cs[s:s + B]
        rr = Rs[s:s + B]
        d2 = ((cc[:, None, 0] - P[None, :, 0]) ** 2
              + (cc[:, None, 1] - P[None, :, 1]) ** 2)
        ok = d2.max(axis=1) <= (rr + 1e-6) ** 2
        if ok.any():
            idx = int(np.argmax(ok))
            best_c = (float(cc[idx, 0]), float(cc[idx, 1]))
            best_r = float(rr[idx])
            break
    if key is not None:
        _MEC_CACHE[key] = (best_c, best_r)
    return best_c, best_r


# ==================================================================
#  P1 支撑集：凸多边形实现（式(9)）
# ==================================================================

def _angular_span(vs):
    """顶点集相对原点的方位角区间 (lo, hi)，弧度；用最大间隙法处理 ±π 环绕。"""
    angs = sorted(atan2(p[1], p[0]) for p in vs)
    n = len(angs)
    mg, gi = -1.0, 0
    for i in range(n):
        g = (angs[(i + 1) % n] - angs[i]) % (2 * pi)
        if g > mg:
            mg, gi = g, i
    lo = angs[(gi + 1) % n]
    hi = angs[gi]
    if hi < lo:
        hi += 2 * pi
    return lo, hi


def _clip_poly_ge(poly, nx, ny, c, eps=1e-12):
    """Sutherland–Hodgman 单次裁剪：保留 {p : n·p >= c}。poly 为简单凸多边形（顶点有序）。"""
    out = []
    m = len(poly)
    if m == 0:
        return out
    for i in range(m):
        a = poly[i]
        b = poly[(i + 1) % m]
        da = nx * a[0] + ny * a[1] - c
        db = nx * b[0] + ny * b[1] - c
        ina = da >= -eps
        inb = db >= -eps
        if ina:
            out.append(a)
        if ina != inb:
            t = da / (da - db)
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


_P1_CACHE = {}


def build_P1(S1, theta1, delta=DELTA, n_arc=N_ARC):
    """式(9) 的凸多边形顶点表（逆时针），供半平面交与暴力枚举共用（唯一数据源）。

    构造：顶点 S1 局部系下 a∈[theta1-delta, theta1+delta]、r∈[5,1500] 的环形扇区之凸包:
        (5, a_lo) --(径向边)--> 远弧(a_lo→a_hi, n_arc 点) --(径向边)--> (5, a_hi) --(弦)--> 闭合
    再与 Ω(R0) 求交（用切向半平面裁剪，弦高误差 <1e-4 m）；原点构型下 1500<1800 自动跳过。
    近距缺口（r<5）按凸包补平，见模块 docstring 与正文 §3.1 的声明。
    """
    key = (round(S1[0], 12), round(S1[1], 12), round(theta1, 12), delta, n_arc)
    if key in _P1_CACHE:
        return _P1_CACHE[key]
    a_lo, a_hi = radians(theta1 - delta), radians(theta1 + delta)
    verts = [(S1[0] + D_NEAR * cos(a_lo), S1[1] + D_NEAR * sin(a_lo))]
    for a in np.linspace(a_lo, a_hi, n_arc):
        verts.append((S1[0] + R_MAX * cos(a), S1[1] + R_MAX * sin(a)))
    verts.append((S1[0] + D_NEAR * cos(a_hi), S1[1] + D_NEAR * sin(a_hi)))

    # --- Ω 裁剪：先判是否需要（远弧上是否有 ||g||>R0 的点）---
    fmax = max(p[0] * p[0] + p[1] * p[1] for p in verts)
    if fmax > R0 * R0:
        lo, hi = _angular_span(verts)
        if hi - lo > pi:
            # 顶点相对原点的最大角隙 < 180° ⟹ 原点可能落在顶点凸包内部，
            # 此时 [lo,hi] 不能覆盖全部顶点，改用整圆裁剪（保守且安全）。
            lo, hi = 0.0, 2.0 * pi
        span = hi - lo
        n_hp = max(600, int(degrees(span) / 0.02) + 1)
        step = span / (n_hp - 1) if n_hp > 1 else span
        cc = R0 * cos(step / 2.0)          # 取内切多边形（P1_poly ⊆ P1_true，保守）
        # 圆盘 Ω = ⋂_φ{n(φ)·p ≤ R0}，取内接多边形即把 R0 换成 R0·cos(step/2)；
        # 约束方向必须保留 n·p ≤ cc（等价于 _clip_poly_ge 的 n'·p ≥ c'，n'=-n、c'=-cc）。
        # ★ 早期版本误写成保留 n·p ≥ cc，等于取"圆盘之外"的区域，会在 Ω 真正生效的
        #   构型（如东缘切向 S1=(1700,0)、θ1=90°）上把 P1 裁成空集。
        for phi in np.linspace(lo, hi, n_hp):
            verts = _clip_poly_ge(verts, -cos(phi), -sin(phi), -cc)
            if len(verts) < 3:
                break
        assert len(verts) >= 3, f'build_P1: Ω 裁剪后为空（S1={S1}, θ1={theta1}）'
        bad = max(p[0] * p[0] + p[1] * p[1] for p in verts)
        assert bad <= R0 * R0 + 1e-3, f'build_P1: Ω 裁剪越界 {sqrt(bad):.6f} > {R0}'
    _P1_CACHE[key] = verts
    return verts


def poly_area(poly):
    """凸多边形面积（鞋带公式）。"""
    s = 0.0
    m = len(poly)
    for i in range(m):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % m]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def poly_edges_halfplanes(poly):
    """逆时针凸多边形的每条边 -> 内侧半平面 (nx, ny, c)，约束 n·p >= c。"""
    hp = []
    m = len(poly)
    for i in range(m):
        a, b = poly[i], poly[(i + 1) % m]
        dx, dy = b[0] - a[0], b[1] - a[1]
        nx, ny = -dy, dx                    # (dx,dy) 逆时针转 90° = 内法向
        hp.append((nx, ny, nx * a[0] + ny * a[1]))
    return hp


# ======================= F_rec 判定的最大值候选集 Γ =======================
# ★★ 耦合口径下的关键更正（2026-09-12 复核，反例见下）★★
#   f(S2) = max_{g∈P1} h(g)，其中 h(g) = ‖S2−g‖ − max(R_min, ‖S1−g‖)。
#   · f 关于 **S2** 是凸函数（有限个"到定点距离减常数"之最大值）⟹ F_rec = {f ≤ 0} 仍是凸集，
#     "沿 ∇f 二分投影到 ∂F_rec""以 F_rec 内点为极心按方位角参数化 ∂F_rec"等算法**依然成立**。
#   · 但 h 关于 **g** 是**两个凸函数之差**（‖S2−g‖ 凸、max(R_min,‖S1−g‖) 也凸），h 不是凸函数
#     ⟹ "凸函数在凸多边形上的最大值必在顶点取到"这一步**在耦合口径下失效**。
#     只用 build_P1 的顶点（近弧 r=5 两点 + 远弧 r=1500 共 201 点）求 max，得到的只是 f 的
#     **下界**，于是旧版会把一批本该排除的 S2 判成可行，F_rec 被系统性放大。
#     【反例】S1=(0,0)、θ1=0°：取 S2=(200,600)。顶点表给 f=−57.2576（判为可行）；
#     而取完全合法的 g=1000·u(−1°)=(999.8477,−17.4524)∈P1，有 ‖S1−g‖=1000 ⟹ R*(g)=1000，
#     ‖S2−g‖=1010.4473 ⟹ h=+10.4473>0 ⟹ S2∉F_rec。实测 A2 因此高估 1178 格点（8.8%）。
#   · 正确候选集（本函数实现）：按 r 方向的结构把 max 化为有限集 Γ。
#     P1 沿 S1 看是星形，径向是区间 [r_in(α), r_out(α)]，r_out = min(R_MAX, r_Ω(α))：
#       ① r ≤ R_min 段：h = ‖S2−g‖ − R_min 关于 r 凸 ⟹ max 在 r = r_in 或 r = R_min；
#       ② r ≥ R_min 段：∂h/∂r = (r − (S2−S1)·u)/‖S2−g‖ − 1 ≤ 0 恒成立 ⟹ h 对 r 单调不增 ⟹ max 在 r = R_min。
#     故每根射线只需 r = r_in(α) 与 r = min(r_out(α), R_min) 两点；再对 α 取 max：
#       · r = min(r_out,R_min) 的轨迹是"以 S1 为心、半径 R_min"或"以原点为心、半径 R0"（出 Ω 时）
#         的圆弧 —— 圆弧上 ‖S2−g‖ 的极值只可能在弧端点或"离 S2 最远的方位"
#         （圆盘上即 arg(S1−S2) 方向、Ω 圆上即反极点 −R0·Ŝ2）；
#       · 近弧 r = r_in(α) 同理（两端点 + arg(S1−S2)）。
#     ⟹ Γ = {ρ(α) 弧按 α 密采样} ∪ {近弧两端点} ∪ {至多 3 个解析临界方位点}。
#     **远弧 r=1500 上的点恒被 ①② 支配**，故不再进入候选集 —— 这就是"控制耦合接收约束的
#     是 r=5 与 r=1000，而不是 r=5 与 r=1500"的结论。
CAND_ARC = 201      # Γ 中弧的方位采样段数（角误差扇区仅 2δ 宽，201 段对应步长 0.01°，远密于需要）


def r_exit(S1, a):
    """从 S1 沿方位角 a（弧度）的射线离开 Ω（半径 R0、圆心在原点）的半径；射线不出 Ω 则返回 inf。"""
    ux, uy = cos(a), sin(a)
    b = S1[0] * ux + S1[1] * uy
    c = S1[0] * S1[0] + S1[1] * S1[1] - R0 * R0
    disc = b * b - c
    if disc <= 0.0:
        return float('inf')
    t = -b + sqrt(disc)
    return t if t > 0.0 else float('inf')


def _switch_angles(S1, r_min=R_MIN):
    """r_Ω(α)=r_min 的切换方位（解析闭式，单位为度、升序、[0,360)）。

    推导：r_Ω(α) 是方程 |S1 + t·u(α)|²=R0² 的正根 t=-b+sqrt(b²-c)，其中
    b=S1·u、c=|S1|²-R0²。令 t=r_min 平方整理得 2·r_min·b = -c - r_min²，
    即 b = (-c - r_min²)/(2 r_min)；又 b = |S1|·cos(α-φ)，φ=atan2(S1y,S1x)。
    故 α* = φ ± acos(b/|S1|)。b/|S1|∈[-1,1] 时有两支，否则无切换（射线全程
    或始终在 r_min 内、或始终在 r_min 外，ρ(α) 恒取 1000 或恒取 r_Ω）。
    """
    n = hypot(S1[0], S1[1])
    if n < 1e-12:
        return []
    c = n * n - R0 * R0
    b = (-c - r_min * r_min) / (2.0 * r_min)
    ratio = b / n
    if ratio < -1.0 or ratio > 1.0:
        return []
    phi = degrees(atan2(S1[1], S1[0]))
    d = degrees(acos(max(-1.0, min(1.0, ratio))))
    out = sorted({(phi + d) % 360.0, (phi - d) % 360.0})
    return out




class SupportCandidates(list):
    """F_rec 判定的最大值候选集 Γ，同时携带几何元数据。

    ★ 为什么做成 list 子类：samples 这个名字原来承担了两种语义——
      (a) **F_rec 判定的取值集**（frec_margin / in_frec / 投影 / 边界参数化）：必须是 Γ；
      (b) **P1 的几何顶点表**（theta2_range / theta2_candidates 的 2m 断点、max_dist_to_support
          的几何最远距离）：必须是 build_P1 的多边形顶点。
      把 (b) 所需的顶点表挂在 .poly 上，让那几个几何量函数内部自取，调用方就无需区分两套点集，
      既避免了下游 7 个脚本的成批改动，也避免"samples 与 rb 配错对"这类静默错误。
    """

    __slots__ = ('poly', 'S1', 'theta1', 'delta', 'r_min', 'caliber')

    def __init__(self, pts, poly, S1, theta1, delta, r_min, caliber):
        super().__init__(pts)
        self.poly = list(poly)
        self.S1 = S1
        self.theta1 = theta1
        self.delta = delta
        self.r_min = r_min
        self.caliber = caliber


def _poly_of(samples):
    """几何量函数专用：取回 P1 的多边形顶点表（普通 list 直接原样返回）。"""
    return getattr(samples, 'poly', samples)


def support_candidates(S1, theta1, delta=DELTA, n_ang=CAND_ARC, r_min=R_MIN):
    """Γ：耦合口径下 f(S2)=max_{g∈P1}(‖S2−g‖−R*(g)) 的最大值候选集（与 S2 无关的静态部分）。

    ★ 完备性（2026-09-12 修正）：ρ(α)=min(r_min, r_Ω(α), R_MAX) 在 r_Ω(α)=r_min 处
      有折角。若该切换角 α* 落在角误差扇区 [θ₁−δ,θ₁+δ] 内部，而 Γ 又只在扇区上等分
      n_ang 个采样角 + 两个光滑弧临界方位，就会漏掉折角处的最大值（一般构型上出现
      "程序判可行、真值不可行"的翻转，见 probe_switch_angle.py）。故把切换角 α*
      对应的点（沿 α* 取 ρ(α*)=r_min）解析加入候选集。
    """
    a_lo, a_hi = radians(theta1 - delta), radians(theta1 + delta)
    pts = []
    for a in np.linspace(a_lo, a_hi, int(n_ang)):
        a = float(a)
        r = min(float(r_min), r_exit(S1, a), R_MAX)
        if r >= D_NEAR:
            pts.append((S1[0] + r * cos(a), S1[1] + r * sin(a)))
    # ★ 分段切换角：r_Ω(α*)=r_min 的方位，若落在扇区内则沿 α* 补 ρ(α*)=r_min 的点。
    for sw in _switch_angles(S1, r_min=r_min):
        if theta1 - delta - 1e-12 <= sw <= theta1 + delta + 1e-12:
            a = radians(sw)
            pts.append((S1[0] + r_min * cos(a), S1[1] + r_min * sin(a)))
    for a in (a_lo, a_hi):                      # 近弧两端点（①：左端 r = r_in）
        pts.append((S1[0] + D_NEAR * cos(a), S1[1] + D_NEAR * sin(a)))
    return pts


def _crit_points(S1, theta1, delta, S2, r_min=R_MIN):
    """与 S2 有关的解析临界方位点：使弧上 ‖S2−g‖ 的极值被精确取到（见 Γ 的构造说明）。"""
    a_lo, a_hi = theta1 - delta, theta1 + delta
    out = []
    bears = []
    if abs(S1[0] - S2[0]) + abs(S1[1] - S2[1]) > 0.0:
        # (i) 以 S1 为心的圆弧上离 S2 最远的方位
        bears.append(degrees(atan2(S1[1] - S2[1], S1[0] - S2[0])))
    n2 = sqrt(S2[0] * S2[0] + S2[1] * S2[1])
    if n2 > 1e-12:
        # (ii) Ω 圆上离 S2 最远的点（反极点 −R0·Ŝ2）相对 S1 的方位
        gs = (-R0 * S2[0] / n2, -R0 * S2[1] / n2)
        bears.append(degrees(atan2(gs[1] - S1[1], gs[0] - S1[0])))
    for b in bears:
        for k in (-360.0, 0.0, 360.0):
            bb = b + k
            if a_lo - 1e-12 <= bb <= a_hi + 1e-12:
                a = radians(bb)
                r = min(float(r_min), r_exit(S1, a), R_MAX)
                if r >= D_NEAR:                     # ρ(α) 弧上的临界点
                    out.append((S1[0] + r * cos(a), S1[1] + r * sin(a)))
                if D_NEAR <= r:                     # 近弧上的同名临界点
                    out.append((S1[0] + D_NEAR * cos(a), S1[1] + D_NEAR * sin(a)))
                break
    return out


def support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801, delta=DELTA, caliber=None,
                     r_min=R_MIN):
    """返回 F_rec 判定的最大值候选集 Γ（**不是** P1 的几何边界，几何边界请用 build_P1）。

    保留 n_ang/n_rad 形参只是为了兼容旧调用签名（n_ang 决定多边形顶点数，n_rad 已弃用）。
    口径决定候选集：
      'coupled'      → Γ = support_candidates(...)（见上文的构造与反例）；
      'conservative' → R*(g) ≡ R_min 使 h 关于 g 重新变凸，故仍是多边形顶点表（此时二者等价）。
    """
    cal = CALIBER if caliber is None else caliber
    poly = build_P1(S1, theta1, delta=delta, n_arc=n_ang)
    pts = poly if cal == 'conservative' else support_candidates(S1, theta1, delta=delta)
    return SupportCandidates(pts, poly, S1, theta1, delta, r_min, cal)


def max_dist_to_support(S2, samples):
    """max_{g∈P1} ‖S2−g‖（**几何量**，不是 f）。

    对固定的 S2，‖S2−·‖ 关于 g 凸 ⟹ 在凸多边形 P1 上的最大值必在顶点取到，
    故本函数按 P1 顶点表（.poly）计算；仅用于 maxdist 等几何报表与作图。
    F_rec 的判定请用 frec_margin（耦合口径下 h 非凸，顶点采样只是下界）。
    """
    mx = 0.0
    for (gx, gy) in _poly_of(samples):
        d = (S2[0] - gx) ** 2 + (S2[1] - gy) ** 2
        if d > mx:
            mx = d
    return sqrt(mx)


# ---------------- 接收半径口径：R*(g) 与 F_rec 的判定 ----------------
# 物理依据：第二次观测要在 S2 收到源 g，须 ‖S2-g‖ ≤ R_g。对 R_g 有两条已知信息：
#   ① 题面给出接收半径下界 R_g ≥ R_recv^min = 1000 m；
#   ② 更强的一条：源 g 已在第一次观测中被收到过（否则不会有 θ1、也不会有 supp_1），
#      故 R_g ≥ ‖S1-g‖ 必然成立——"能在 S1 听到它"本身就说明它的嗓门至少够到 S1。
# 两条合起来：R_g ≥ max(R_recv^min, ‖S1-g‖) =: R*(g)。据此定义两种口径：
#   'conservative'：一律取常数 R*(g) ≡ r_min（对每个可能位置都按最小嗓门设防，更保守）；
#   'coupled'     ：逐源取 R*(g) = max(r_min, ‖S1-g‖)，把①与②都用上（本文采用）。
# 数学性质：F_rec = ∩_g B(g, R*(g)) 两种口径下都是凸集；且
#   f(S2) = max_g (‖S2-g‖ - R*(g)) 是凸函数、在最远处 |∇f| = 1，
#   故"沿梯度二分投影到 ∂F_rec""以 F_rec 内点为极心按方位角参数化边界"等算法
#   对两种口径同样成立（见下方 project_to_frec_boundary / frec_boundary_point）。
# ★ 关键差别：常数口径下 ∂F_rec 的左端由"远弧端点 (1500u, R=1000)"这条配对给出，
#   而该配对物理上不可实现——R=1000 m 的源在 1500 m 外根本收不到。
#   耦合口径把这条配对改成 (1500u, R=1500)，左端随之由 x≥500.11 m 放宽到 x≥0。
# ★★ 但必须同时记住：f 的凸性**是针对 S2**的（极值族中每一项都关于 S2 凸）；
#   对 **g** 而言 h(g)=‖S2−g‖−max(r_min,‖S1−g‖) 是两个凸函数之差，**不是凸函数**，
#   因此"最大值必在 P1 顶点取到"在耦合口径下**不成立**，F_rec 的取值集必须换成 Γ
#   （见上方 support_boundary / support_candidates 的构造与反例）。
S1_ORIGIN = (0.0, 0.0)      # 第一次检测点（题面取为原点）
CALIBER = 'coupled'         # 'conservative' | 'coupled'


def rbound(samples, S1=S1_ORIGIN, r_min=R_MIN, caliber=None):
    """每个顶点 g 的有效接收半径下界 R*(g)，与 samples 逐项对齐。"""
    cal = CALIBER if caliber is None else caliber
    if cal == 'conservative':
        return [float(r_min)] * len(samples)
    if cal != 'coupled':
        raise ValueError('未知接收半径口径：%r' % (cal,))
    return [max(float(r_min), hypot(gx - S1[0], gy - S1[1])) for (gx, gy) in samples]


def frec_margin(S2, samples, rb):
    """f(S2) = max_g ( ‖S2-g‖ - R*(g) )。f ≤ 0 ⟺ S2 ∈ F_rec（对 P1 内任一源都保证接收）。

    samples 必须是 Γ（support_boundary 的返回值）：耦合口径下 h 关于 g 非凸，
    只在顶点采样得到的只是 f 的下界（见文首反例）。若 Γ 带几何元数据，这里再补上
    至多 3 个与 S2 有关的解析临界方位点，使弧上极值被精确取到。
    """
    if len(samples) != len(rb):
        raise ValueError('samples 与 rb 长度不一致（%d vs %d）：二者必须来自同一口径'
                         % (len(samples), len(rb)))
    mx = -1e30
    for (gx, gy), r in zip(samples, rb):
        v = hypot(S2[0] - gx, S2[1] - gy) - r
        if v > mx:
            mx = v
    if isinstance(samples, SupportCandidates) and samples.caliber == 'coupled':
        for (gx, gy) in _crit_points(samples.S1, samples.theta1, samples.delta,
                                     S2, r_min=samples.r_min):
            r = max(float(samples.r_min), hypot(gx - samples.S1[0], gy - samples.S1[1]))
            v = hypot(S2[0] - gx, S2[1] - gy) - r
            if v > mx:
                mx = v
    return mx


def in_frec(S2, samples, S1=S1_ORIGIN, r_min=R_MIN, tol=1e-6, rb=None):
    """S2 ∈ F_rec（保证第二次观测能收到 P1 内任一源）。按模块口径 CALIBER 判定。"""
    if rb is None:
        rb = rbound(samples, S1, r_min)
    return frec_margin(S2, samples, rb) <= tol


# ---------- F_rec 边界的解析投影（"最优解落在 ∂F_rec 上"的最后一步精化） ----------
# 依据：f(S2)=max_{g∈P1}(‖S2-g‖-R*(g)) 关于 **S2** 是凸函数（有限个"到定点距离"之最大值减常数），
#   故 F_rec={f≤0} 是凸集；且在取到最大值的 g 上 ∇f 恰为"由 g 指向 S2 的单位向量"，
#   |∇f| ≡ 1，于是"沿 ∇f 移动 t 米 ⇒ f 改变约 t 米"，可直接二分把任一点投到 ∂F_rec。
#   ★ 该性质与 R* 取常数还是取 max(r_min,‖S1-g‖) 无关，两种口径共用同一套投影/参数化。
#   ★ 注意：凸性说的是**对 S2**；对 g 的凸性在耦合口径下已失效（见 support_candidates）。

def dmax_grad(S2, samples, h=1e-2, rb=None, S1=S1_ORIGIN, r_min=R_MIN):
    """f(S2)=frec_margin 的数值梯度（中心差分）。"""
    if rb is None:
        rb = rbound(samples, S1, r_min)

    def k(p):
        return frec_margin(p, samples, rb)
    gx = (k((S2[0] + h, S2[1])) - k((S2[0] - h, S2[1]))) / (2.0 * h)
    gy = (k((S2[0], S2[1] + h)) - k((S2[0], S2[1] - h))) / (2.0 * h)
    return gx, gy


def project_to_frec_boundary(S2, samples, rb=None, S1=S1_ORIGIN, r_min=R_MIN, iters=80):
    """沿 ±∇f 二分，把 S2 推到 ∂F_rec（f = 0）上。返回 (边界点, f)。"""
    if rb is None:
        rb = rbound(samples, S1, r_min)
    d = frec_margin(S2, samples, rb)
    gx, gy = dmax_grad(S2, samples, rb=rb)
    n = sqrt(gx * gx + gy * gy)
    if n < 1e-12:
        return S2, d
    sgn = 1.0 if d < 0.0 else -1.0             # 朝"增大 f"的方向推（f>0 即出界）
    u = (sgn * gx / n, sgn * gy / n)
    lo, hi = 0.0, max(4.0 * abs(d), 1e-6)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        dm = frec_margin((S2[0] + mid * u[0], S2[1] + mid * u[1]), samples, rb)
        if dm * sgn > 0:
            hi = mid
        else:
            lo = mid
    t = 0.5 * (lo + hi)
    Q = (S2[0] + t * u[0], S2[1] + t * u[1])
    return Q, frec_margin(Q, samples, rb)


def frec_boundary_point(C0, phi_deg, samples, rb=None, S1=S1_ORIGIN, r_min=R_MIN,
                        r_hi=None, iters=70):
    """从 F_rec 的内点 C0 沿方位 phi_deg 的射线与 ∂F_rec 的交点（二分）。

    F_rec 凸 ⟹ 以任一内点为极心时，该射线与 ∂F_rec 恰交一次，
    于是 ∂F_rec 可用方位角 φ 单参数化（本文取 C0 = P1 的最小覆盖圆圆心）。
    """
    if rb is None:
        rb = rbound(samples, S1, r_min)
    u = (cos(radians(phi_deg)), sin(radians(phi_deg)))
    hi = r_hi if r_hi else 3.0 * R0
    lo = 0.0
    Qh = (C0[0] + hi * u[0], C0[1] + hi * u[1])
    if frec_margin(Qh, samples, rb) <= 0.0:
        return Qh, frec_margin(Qh, samples, rb)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        dm = frec_margin((C0[0] + mid * u[0], C0[1] + mid * u[1]), samples, rb)
        if dm > 0.0:
            hi = mid
        else:
            lo = mid
    r = 0.5 * (lo + hi)
    Q = (C0[0] + r * u[0], C0[1] + r * u[1])
    return Q, frec_margin(Q, samples, rb)


def theta2_range(S2, samples, delta=DELTA):
    """Θ(S2) 的角度范围 [lo, hi]（度）。lo=min arg(g-S2)-δ, hi=max arg(g-S2)+δ。

    ★ 判据分两支（不可省任一支）：
    (a) S2 ∉ int(conv P1)：S2 与 P1 可被一条过 S2 的直线分开，方位角集合是圆周上的
        一段弧，其补集恰为各顶点方位角序列在圆周上的**最大间隙** ⟹ 取该间隙的补集
        再向两侧各放宽 δ。这样即使方位角跨越 ±180° 也能正确处理（atan2 在 ±180°
        处跳变，直接取逐点 min/max 会把区间取反）。
    (b) S2 ∈ int(conv P1)：S2 的一个邻域整体落在 P1 内，任意方位上都能取到 P1 的点
        ⟹ 真实方位角集合是**整个圆周**，Θ(S2) = [0°, 360°)。只走 (a) 的补集逻辑会把
        这种情形误缩成一段宽 < 360° 的弧。
    判据：顶点方位角的**最大间隙 < 180° ⟺ S2 ∈ int(conv P1)**（方向正张成）。
    等价地，最大间隙 = 180° 当且仅当 S2 恰在 ∂(conv P1) 上（此时闭半圆，走 (a) 正确）。

    ★ 本支不是防御性代码：F_rec ∩ int(P1) ≠ ∅（§3.2 的构造性见证
      C = S1 + 752.61·u(theta1) 就落在 P1 内部），故搜索必然踩到。实测把 (b) 补上后，
      int(P1) 内的点其 J 一字不变（补集支遗漏的那段弧上 diam 的最大值恒不超过保留弧的
      最大值），见 q2_checks.py 的 [k-10] 组。
    """
    angs = sorted(atan2(gy - S2[1], gx - S2[0]) for (gx, gy) in _poly_of(samples))
    n = len(angs)
    max_gap, gap_idx = -1.0, 0
    for i in range(n):
        gap = (angs[(i + 1) % n] - angs[i]) % (2 * pi)
        if gap > max_gap:
            max_gap, gap_idx = gap, i
    if max_gap < pi:
        # S2 ∈ int(conv P1)：Θ(S2) 为整圆
        return 0.0, 360.0
    lo = angs[(gap_idx + 1) % n]
    hi = angs[gap_idx]
    if hi < lo:
        hi += 2 * pi
    return degrees(lo) - delta, degrees(hi) + delta


def theta2_candidates(S2, samples, delta=DELTA, n_uniform=41, breakpoints=True):
    """Θ(S2) 上 θ2 的候选集（度，升序、去重）：断点集 ∪ 均匀兜底。

    断点的定义：Θ(S2) 的上下端，以及每个 P1 顶点进出方向锥的时刻
        bearing(S2→v) − δ 与 bearing(S2→v) + δ   （v 遍历 P1 的全部顶点，共 2m 个）
    依据：J(θ2) = diam(P1 ∩ C(S2,θ2)) 在相邻断点之间是"有限族光滑函数的上包络"，
    其切换只可能发生在这些断点上；实测（q2_solver_audit.py，在优质域 Q2(1.2J*) 内
    等距抽 5 个 S2 点，结果见 data/q2_solver_audit.csv）
    2m=406 个断点给出的最大值比 4001 点均匀密扫还高 0.0258~0.0938 m ——
    均匀采样恰恰会漏掉这些尖峰（81 点均匀采样最多低估 J 达 6.52 m）。
    因此本文以断点集为主、均匀网格作兜底。候选数 = 2m ∪ n_uniform（m=203）：
    空间搜索档 n_uniform=11 → 415 个，最终精算档 n_uniform=201 → 605 个，
    分别比 4001 点密扫快约 9.6 倍与 6.6 倍，且更准。
    breakpoints=False 时退化为纯均匀网格（仅供分层搜索的粗筛层使用）。
    """
    lo, hi = theta2_range(S2, samples, delta=delta)
    cands = [lo, hi]
    if breakpoints:
        for (gx, gy) in _poly_of(samples):
            a = degrees(atan2(gy - S2[1], gx - S2[0]))
            while a < lo - delta:
                a += 360.0
            while a > hi + delta:
                a -= 360.0
            for e in (-delta, delta):
                t = a + e
                if lo <= t <= hi:
                    cands.append(t)
    cands += list(np.linspace(lo, hi, n_uniform))
    return sorted({round(c, 10) for c in cands})


# ==================================================================
#  后验区域与直径（两套独立实现，共用同一 P1 多边形）
# ==================================================================

def _status_of(poly, tol=1e-9):
    """按顶点数给出退化状态。"""
    if len(poly) == 0:
        return 'empty', 0.0
    clean = [poly[0]]
    for p in poly[1:]:
        if sqrt((p[0] - clean[-1][0]) ** 2 + (p[1] - clean[-1][1]) ** 2) > tol:
            clean.append(p)
    if len(clean) > 1 and sqrt((clean[0][0] - clean[-1][0]) ** 2 +
                               (clean[0][1] - clean[-1][1]) ** 2) < tol:
        clean = clean[:-1]
    if len(clean) == 0:
        return 'empty', 0.0
    if len(clean) == 1:
        return 'point', 0.0
    if len(clean) == 2:
        return 'segment', sqrt((clean[0][0] - clean[1][0]) ** 2
                               + (clean[0][1] - clean[1][1]) ** 2)
    return 'polygon', None


def region_polygon(S1, theta1, S2, theta2, delta=DELTA, p1=None):
    """返回 R2 = P1 ∩ C(S2,θ2) 的顶点表（S-H 裁剪结果），供最小覆盖圆等后处理使用。"""
    poly = p1 if p1 is not None else build_P1(S1, theta1, delta=delta)
    for (nx, ny, c) in sector_to_halfplanes(S2, theta2, delta):
        poly = _clip_poly_ge(poly, nx, ny, c)
        if len(poly) < 3:
            break
    return poly


def region_diam(S1, theta1, S2, theta2, delta=DELTA, p1=None):
    """R2 = P1 ∩ C(S2,θ2) 的直径，主实现（Sutherland–Hodgman 增量裁剪 + 旋转卡壳）。

    P1 有界 ⟹ R2 = P1 ∩ C 恒有界，故本函数**不存在无界分支**；
    返回 (status, diameter)，status ∈ {'polygon','segment','point','empty'}。

    ★ 直径子算法用旋转卡壳 O(m) 而非顶点枚举 O(m²)：口径B 下剪影多边形常在锥窗内
      保留上百个远弧顶点（实测 m 最大 170 于 m_arc=201），枚举会把单次计算拖到 ~3 ms。
      两者在 240 个实际裁剪多边形上逐位相同（最大差 0.000e+00，本机实测），
      且独立复核 region_diam_bruteforce 仍用顶点枚举 + 暴力交点，
      于是"双算法"在裁剪方式与直径算法两处都相互独立。
    """
    poly = region_polygon(S1, theta1, S2, theta2, delta=delta, p1=p1)
    st, D = _status_of(poly)
    if st != 'polygon':
        return st, D
    _, _, D = rotating_calipers(poly)
    return 'polygon', D


def _line_intersect(h1, h2):
    """两条半平面边界 n·p=c 的交点；平行返回 None。"""
    nx1, ny1, c1 = h1
    nx2, ny2, c2 = h2
    det = nx1 * ny2 - nx2 * ny1
    if abs(det) < 1e-12:
        return None
    x = (c1 * ny2 - c2 * ny1) / det
    y = (nx1 * c2 - nx2 * c1) / det
    return (x, y)


def _feasible(p, hpl, tol=1e-7):
    for (nx, ny, c) in hpl:
        if nx * p[0] + ny * p[1] < c - tol:
            return False
    return True


def region_diam_bruteforce(S1, theta1, S2, theta2, delta=DELTA, p1=None):
    """独立复核：不变量是"R2 的顶点 ∈ P1 的顶点 ∪ (C 的边 ∩ P1 的边) ∪ {C 的锥顶 S2}"。
    候选点全部显式枚举，再逐条约束回代筛选，最后顶点枚举求直径。
    与 S-H 增量裁剪在算法路径上完全独立。

    ★ 锥顶 S2 必须进候选集：当 S2 ∈ P1 时（第二检测点位于第一次观测支撑集内部是常见情形），
      S2 是 R2 的一个真顶点（2δ=2° 的凸角尖点），但既不是 P1 的顶点、也不是"锥边 ∩ P1 边"，
      漏掉会让结果被误判为 segment（实测 50 组样本中有 2 组触发，直径被低估 8.8~15.5 倍）。
    """
    poly = p1 if p1 is not None else build_P1(S1, theta1, delta=delta)
    ph = poly_edges_halfplanes(poly)
    ch = sector_to_halfplanes(S2, theta2, delta)
    allhp = ph + ch
    cands = list(poly)                       # P1 自身的顶点
    cands.append((S2[0], S2[1]))             # ★ C 的锥顶
    for h1 in ch:                            # C 的边 × P1 的边
        for h2 in ph:
            p = _line_intersect(h1, h2)
            if p is not None:
                cands.append(p)
    verts = []
    for p in cands:
        if not _feasible(p, allhp):
            continue
        if all((p[0] - v[0]) ** 2 + (p[1] - v[1]) ** 2 > 1e-14 for v in verts):
            verts.append(p)
    st, D = _status_of(verts)
    if st != 'polygon':
        return st, D
    _, _, D = diameter_by_enumeration(verts)
    return 'polygon', D


def worst_diam(S2, samples, S1, theta1, n_theta=41, delta=DELTA, p1=None, breakpoints=True):
    """J(S2) = max_{θ2∈Θ(S2)} Φ(R2)。返回 (J, 最坏θ2, None)。

    θ2 的采样用 theta2_candidates（断点集 ∪ n_theta 点均匀兜底），见其说明；
    breakpoints=False 时退化为纯均匀网格，仅供分层搜索的粗筛层使用。
    P1 有界 ⟹ R2 = P1 ∩ C 恒有界 ⟹ J 恒为有限值（本版不存在 +∞ 分支）。
    """
    poly = p1 if p1 is not None else build_P1(S1, theta1, delta=delta)
    best_J, best_th = -1.0, None
    for th in theta2_candidates(S2, samples, delta=delta,
                                n_uniform=max(int(n_theta), 11), breakpoints=breakpoints):
        st, D = region_diam(S1, theta1, S2, th, delta=delta, p1=poly)
        if D is not None and D > best_J:
            best_J, best_th = D, th
    return best_J, best_th, None


def region_diam_safe(S1, theta1, S2, theta2, delta=DELTA, p1=None):
    """对 (S2,θ2) 做左右微扰取 max，用于抑制顶点枚举在退化点上的抖动。
    仅在特征点复核时使用，主搜索仍用 region_diam。"""
    best = 0.0
    st = 'empty'
    for eth in (0.0, -delta * 1e-3, delta * 1e-3):
        s, D = region_diam(S1, theta1, S2, theta2 + eth, delta=delta, p1=p1)
        if D is not None and D > best:
            best, st = D, s
    return st, best


# ================= 主流程 =================

def solve(S1, theta1, grid=50.0, n_theta=81, n_ang=N_ARC, n_rad=401):
    """在 F_rec 内网格搜索，求 min J(S2)。返回结果字典。"""
    samples = support_boundary(S1, theta1, n_ang=n_ang, n_rad=n_rad)
    p1 = build_P1(S1, theta1)

    ux, uy = cos(radians(theta1)), sin(radians(theta1))
    vx, vy = -uy, ux

    u_lo, u_hi = None, None
    for u in np.arange(0.0, 1800.0, 10.0):
        S2 = (S1[0] + u * ux, S1[1] + u * uy)
        if in_frec(S2, samples):
            if u_lo is None:
                u_lo = u
            u_hi = u
    if u_lo is None:
        return {'error': 'F_rec 为空', 'S1': S1, 'theta1': theta1}

    u_mid = (u_lo + u_hi) / 2
    v_lo, v_hi = None, None
    for v in np.arange(-1500.0, 1500.0, 10.0):
        S2 = (S1[0] + u_mid * ux + v * vx, S1[1] + u_mid * uy + v * vy)
        if in_frec(S2, samples):
            if v_lo is None:
                v_lo = v
            v_hi = v

    best = None
    for u in np.arange(u_lo, u_hi + grid, grid):
        for v in np.arange(v_lo, v_hi + grid, grid):
            S2 = (S1[0] + u * ux + v * vx, S1[1] + u * uy + v * vy)
            if not in_frec(S2, samples):
                continue
            J, th_w, _ = worst_diam(S2, samples, S1, theta1, n_theta=n_theta, p1=p1)
            if best is None or J < best[0]:
                best = (J, S2, th_w)
    return {
        'S1': S1, 'theta1': theta1, 'grid': grid,
        'u_lo': u_lo, 'u_hi': u_hi, 'v_lo': v_lo, 'v_hi': v_hi,
        'best_J': best[0], 'best_S2': best[1], 'best_th_worst': best[2],
        'samples': samples,
    }


if __name__ == '__main__':
    print('Geometry library. Run src/q2_geometry_verification.py for verification,')
    print('or src/q2_optimization_experiments.py for full optimization experiments.')
