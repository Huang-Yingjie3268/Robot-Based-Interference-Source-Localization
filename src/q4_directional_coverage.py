# -*- coding: utf-8 -*-
"""Simulate search and clearance of omnidirectional and directional sources.

Includes a 24-waypoint coverage layout, angular-gap checks, and task scheduling."""

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
from urllib.error import ContentTooShortError  # noqa: F401  (与Q3一致,幂等重试用)

# ==============================================================================
# 0. 全局常量（与 Q3 共用一套物理模型，Q4 新增阈值集中在此）
# ==============================================================================
BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = os.environ.get("ROBOT_ID", "<configure-at-runtime>")

ARENA_R = 1800.0                   # 目标区域半径 1800m（附件2 §1.1）
N_CH = 20                          # 频道总数
N_SRC_LO, N_SRC_HI = 10, 16        # 干扰源总数区间（问题4）
SPEED = 5.0                        # 移动速度 m/s
T_MEASURE = 5.0                    # 单次检测耗时 s
T_SWITCH = 1.0                     # 切换频道耗时 s
T_CLEAR_OK = 5.0               # 清除成功耗时 s（附件2 §5.2）
T_CLEAR_FL = 3.0               # 清除失败耗时 s
R_RECV_MIN, R_RECV_MAX = 1000.0, 1500.0   # 源接收半径区间
R_NEAR = 5.0                       # near 判定距离（Q4 中还需在覆盖角内）
R_CLEAR = 20.0                     # 清除半径（与朝向无关，见附件2）
EPS_DEG = 1.0                      # 示向度误差 ±1°（均匀）
MAX_COORD = 2e6                    # 坐标分量绝对值上限（附件2 §1.1）
HTTP_TIMEOUT = 5.0
NOTEBOOK_ARGS = "--mode mock --planner B --runs 5 --dratio 0.4"

# ---- Q3 沿用阈值 ----
CLEAR_TRIG = 15.0                  # 逼近伺服触发清除的距离
APPROACH_RATIO = 0.45              # 逼近每步停靠比例
CONF_GOOD = 45.0                   # 置信半径≤45m 视为"够准"（local_search 半径内网格必命中）
LOCAL_SEARCH_MAX = 45.0            # 网格搜索最大半径（>45m 才转推进/环绕）
MAX_APPROACH = 12                  # 逼近伺服最大迭代
LS_SPACING = 6.0                   # 网格兜底间距（<5√2，保证某点落入5m内触发near）
LS_MAX_PTS = 60                    # 网格兜底最大点数
REF_DIST = 2000.0                  # 打分归一化距离
REF_CONF = 150.0                   # 打分归一化置信半径

# ---- Q4 新增阈值（M7'=构型 / M8=判空证书 / M9'=可行域清除）----
Q4_INNER_R = 1000.0                # 内环极径（10点）
Q4_INNER_N = 10                    # 内环点数
Q4_OUTER_R = 1880.0                # 外环极径（13点，>1800 盘外，对边界源必可证）
Q4_OUTER_N = 13                    # 外环点数
Q4_DIRECT_CLEAR = 25.0             # M9 直清阈值：cr≤25 直清（保守上界25m，清除半径20m+5m余量）
Q4_PSI_DEG = 60.0                  # M9 补测偏角 ψ=±60°
Q4_REFINE_T_BASE = 750.0           # 无源距估计时补测步长初值（射线中点距离）
Q4_REFINE_T_MAX = 1500.0           # 补测步长上限
Q4_REFINE_TRIES = 3                # 补测最多尝试次数
CLEAR_GATE_SCAN = 60.0             # 发现阶段允许清除的置信半径门槛（cr≤60顺路清，避免扫完绕回）
Q4_SCAN_MAX_MEAS = 999              # 单频道扫描测量次数上限（超出转入定点补测，省 5s/路点）
Q4_REFINE_OFF = 150.0             # 补测垂直偏移量（m）：±150 足够拿第二条示向度，往返更省
Q4_CLEAR_ADMIT = 300.0             # rolling TSP 里允许把源当"顺路节点"的可行域直径上限（m）
Q4_PROBES_EASY = 1                 # 探针档位1：尝试≤2 次（简单源，只打质心，最省移动）
Q4_PROBES_MID = 3                  # 探针档位2：尝试≤5 次
Q4_PROBES_HARD = 6                 # 探针档位3：尝试>5 次（难源，铺满可行域防漏检）
Q4_MAX_ATTEMPT = 8                 # 同一频道 resolve_channel 调用次数上限（超出转收尾补救）
# ---- 死锁饱和清扫（仅 rescue 失败的难源触发，正常局完全不走这条路）----
Q4_DEGEN_CROSS_DEG = 20.0          # 退化判定：两示向线最大交会角 < 此值 ⇒ 楔形域退化
Q4_SATURATE_MAX = 90               # 饱和清扫 /clear 探针数上限（直径~500m楔形域约需35）
Q4_SATURATE_STEP = 4.0             # 饱和清扫的域内采样步长（≤ R_CLEAR/5，保证边缘覆盖）
MAX_BEARINGS = 14                  # 每频道示向度条数上限（防数值污染；超出丢最旧）
ORBIT_N = 8                        # 环绕搜索点数（临界 g/r=0.924 所需）
ORBIT_RATIO = 0.92                 # 环绕半径系数 r = cr/0.92 + 10
ORBIT_R_MAX = 300.0                # 环绕半径上限
ORBIT_ROUNDS = 2                   # 环绕最多轮数（每轮半径递增）
ANGLE_MARGIN_DEG = 8.0             # 角度间隙工程裕量参考目标（不参与合格判定；24点构型实得余量 6.45°）
EMPTY_GRID_STEP = 150.0            # M8 判空网格步长（盘内 ~450 点）
LOCAL_EMPTY_MIN_WP = 8             # 已访问路点 ≥8 才做局部判空
LOCAL_EMPTY_MIN_NS = 6             # 该频道 no_signal 点数 ≥6 才做局部判空

# ==============================================================================
# 1. 几何工具（Q3 原样复用：半平面交 / MEC / 三角定位）
# ==============================================================================
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bearing_deg(frm, to):
    """从 frm 看向 to 的方位角 [0,360)。"""
    return (math.degrees(math.atan2(to[1] - frm[1], to[0] - frm[0])) + 360.0) % 360.0


def sector_to_halfplanes(x, y, theta_deg, eps_deg=EPS_DEG):
    hps = []
    for s in (+eps_deg, -eps_deg):
        a = math.radians(theta_deg + s)
        nx, ny = math.cos(a), math.sin(a)      # 内法向 u(θ±eps)
        hps.append((nx, ny, -(nx * x + ny * y)))
    return hps


def halfplane_intersection(hps):
    """半平面集 {x: n·x + c ≥ 0} 求交。返回 polygon(顶点)/degenerate(2顶点)/empty/unbounded。
    实现：转成极角排序的双端队列半平面交（线性）。"""
    if not hps:
        return {"status": "unbounded", "vertices": []}
    ev = []
    for (nx, ny, c) in hps:
        ang = math.atan2(ny, nx)
        # 处理角度环绕：把角分成 [0,2π) 区间，跨2π的边拆开
        a = (ang + 2 * math.pi) % (2 * math.pi)
        ev.append((a, nx, ny, c))
    ev.sort(key=lambda t: t[0])
    # 重新转回直线表示（方向角a，直线法向(cos a, sin a)）
    lines = []
    for (a, nx, ny, c) in ev:
        lines.append((nx, ny, c))
    # 标准半平面交（增量法，双端队列）
    def _inter(l1, l2):
        (a1, b1, c1), (a2, b2, c2) = l1, l2
        det = a1 * b2 - a2 * b1
        if abs(det) < 1e-12:
            return None
        x = (b1 * c2 - b2 * c1) / det
        y = (a2 * c1 - a1 * c2) / det
        return (x, y)

    dq = []
    for ln in lines:
        while len(dq) >= 2:
            p = _inter(dq[-2], dq[-1])
            if p is None:
                break
            if ln[0] * p[0] + ln[1] * p[1] + ln[2] < -1e-9:
                dq.pop()
            else:
                break
        while len(dq) >= 2:
            p = _inter(dq[0], dq[1])
            if p is None:
                break
            if ln[0] * p[0] + ln[1] * p[1] + ln[2] < -1e-9:
                dq.pop(0)
            else:
                break
        dq.append(ln)
    # 首尾求交闭合
    while len(dq) >= 3:
        p = _inter(dq[-2], dq[-1])
        if p is None or dq[0][0] * p[0] + dq[0][1] * p[1] + dq[0][2] < -1e-9:
            dq.pop()
        else:
            break
    if len(dq) < 3:
        return {"status": "empty", "vertices": []}
    verts = []
    n = len(dq)
    for i in range(n):
        p = _inter(dq[i], dq[(i + 1) % n])
        if p is None:
            return {"status": "unbounded", "vertices": []}
        verts.append(p)
    return {"status": "polygon", "vertices": verts}


def convex_hull(pts):
    """Andrew 单调链凸包。返回逆时针顶点列表（≥3 点）；退化返回去重后的点。"""
    pts = sorted(set((round(float(x), 6), round(float(y), 6)) for x, y in pts))
    if len(pts) <= 2:
        return pts
    def _cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return hull


def point_in_convex(p, hull, tol=1e-9):
    """点在凸包内/边界判断（hull 逆时针）。退化（<3点）用包围盒近似。"""
    if len(hull) < 3:
        if not hull:
            return False
        if len(hull) == 1:
            return dist(p, hull[0]) <= tol
        a, b = hull
        return abs(dist(p, a) + dist(p, b) - dist(a, b)) <= 1e-6
    def _cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    for i in range(len(hull)):
        if _cross(hull[i], hull[(i + 1) % len(hull)], p) < -tol:
            return False
    return True


def triangulate(bs):
    """最小二乘交会（多点示向度），用作空/无界时的退化估计。

    纯标准库实现（2x2 正规方程 AᵀA x = Aᵀb 直接求解），不依赖 numpy——
    本文件要求"单文件直接运行"，故去掉一切第三方依赖。
    每条示向度 (x,y,th) 给出过该点、方向 th 的直线，其单位法向为
        n = (-sin th, cos th)
    直线方程 n·p = n·(x,y)。对多条线做最小二乘，即最小化 Σ(nᵢ·p − cᵢ)²：
        M = Σ nᵢnᵢᵀ  (2x2)，  v = Σ cᵢnᵢ
        p = M⁻¹ v
    两法向近共线时 M 奇异，退化返回 (0,0)（调用方有 _sane_pos 防护）。
    """
    M = [[0.0, 0.0], [0.0, 0.0]]
    v = [0.0, 0.0]
    for (x, y, th) in bs:
        a = math.radians(th)
        nx, ny = -math.sin(a), math.cos(a)
        c = nx * x + ny * y
        M[0][0] += nx * nx
        M[0][1] += nx * ny
        M[1][0] += nx * ny
        M[1][1] += ny * ny
        v[0] += c * nx
        v[1] += c * ny
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(det) < 1e-12:
        return (0.0, 0.0)
    px = (M[1][1] * v[0] - M[0][1] * v[1]) / det
    py = (-M[1][0] * v[0] + M[0][0] * v[1]) / det
    return (float(px), float(py))


def mec(points):
    """最小覆盖圆（Welzl 算法，点很少直接枚举即可）。返回 (圆心, 半径)。"""
    import random
    pts = list(points)
    if not pts:
        return ((0.0, 0.0), 0.0)
    if len(pts) == 1:
        return (pts[0], 0.0)
    if len(pts) == 2:
        return (((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2),
                dist(pts[0], pts[1]) / 2)
    rng = random.Random(12345)
    shuffled = pts[:]
    rng.shuffle(shuffled)
    c = (shuffled[0][0], shuffled[0][1])
    r = 0.0
    for i in range(1, len(shuffled)):
        if dist(shuffled[i], c) > r + 1e-12:
            c = (shuffled[i][0], shuffled[i][1])
            r = 0.0
            for j in range(i):
                if dist(shuffled[j], c) > r + 1e-12:
                    c = ((shuffled[i][0] + shuffled[j][0]) / 2,
                         (shuffled[i][1] + shuffled[j][1]) / 2)
                    r = dist(c, shuffled[i])
                    for k in range(j):
                        if dist(shuffled[k], c) > r + 1e-12:
                            cx, cy = shuffled[i], shuffled[j], shuffled[k]
                            # 三点外接圆
                            ax, ay = cx[0], cx[1]
                            bx, by = cy[0], cy[1]
                            px, py = shuffled[k][0], shuffled[k][1]
                            d = 2 * (ax * (by - py) + bx * (py - ay) + px * (ay - by))
                            if abs(d) < 1e-12:
                                c = ((ax + px) / 2, (ay + py) / 2)
                            else:
                                ux = ((ax * ax + ay * ay) * (by - py) +
                                      (bx * bx + by * by) * (py - ay) +
                                      (px * px + py * py) * (ay - by)) / d
                                uy = ((ax * ax + ay * ay) * (px - bx) +
                                      (bx * bx + by * by) * (ax - px) +
                                      (px * px + py * py) * (bx - ax)) / d
                                c = (ux, uy)
                            r = dist(c, shuffled[i])
    return (c, r)


def _r2(p):
    return (round(p[0], 1), round(p[1], 1))


def _sane_pos(p, slack=50.0):
    return p is not None and dist(p, (0.0, 0.0)) <= ARENA_R + slack


# ==============================================================================
# 2. Q4 覆盖评估（M7 核心：距离覆盖 + 角度覆盖 双判据自检）
# ==============================================================================
def _max_angle_gap_at(p, waypoints, rad=R_RECV_MIN):
    """点 p 处：取 p 的 rad 邻域内路点，按相对方位排序后求最大角度间隙。
    间隙>180° ⟺ 存在方向没有任何路点可见（= 该方向上的定向源在此点测不到）。
    注意：这里返回 0..360 的角度，π 对应 180° 判据。"""
    near = [(bx, by) for (bx, by) in waypoints if dist((bx, by), p) <= rad + 1e-9]
    if len(near) < 2:
        return 360.0, len(near)
    angs = sorted(bearing_deg(p, q) for q in near)
    gaps = []
    for i in range(len(angs)):
        nxt = angs[(i + 1) % len(angs)]
        g = (nxt - angs[i]) % 360.0
        gaps.append(g)
    return max(gaps), len(near)


def angle_gap_check(waypoints, step=100.0, n_boundary=1440, refine=True,
                    verbose=False):
    """盘内最大角度间隙评估（finite-grid diagnostic; no interval certificate is implemented here）。
    返回 (最坏间隙°, 最坏位置, 是否 ≤ 180-δ)。"""
    best_gap, best_p = 0.0, (0.0, 0.0)
    # (1) 盘内网格
    x = -ARENA_R
    while x <= ARENA_R + 1e-9:
        y = -ARENA_R
        while y <= ARENA_R + 1e-9:
            if math.hypot(x, y) <= ARENA_R + 1e-9:
                g, _ = _max_angle_gap_at((x, y), waypoints)
                if g > best_gap:
                    best_gap, best_p = g, (x, y)
            y += step
        x += step
    # (2) 边界圆周（盘内最坏点通常在边界附近）
    for k in range(n_boundary):
        a = 2 * math.pi * k / n_boundary
        p = (ARENA_R * math.cos(a), ARENA_R * math.sin(a))
        g, _ = _max_angle_gap_at(p, waypoints)
        if g > best_gap:
            best_gap, best_p = g, p
    # (3) 最坏点局部细化（间隙函数在 |w-p|=1000 处跳变，细网格才能抓住）
    if refine and best_p is not None:
        cx, cy = best_p
        for step2 in (20.0, 5.0):
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    p = (cx + dx * step2, cy + dy * step2)
                    if math.hypot(p[0], p[1]) <= ARENA_R + 1e-9:
                        g, _ = _max_angle_gap_at(p, waypoints)
                        if g > best_gap:
                            best_gap, best_p = g, p
    # 硬约束：间隙 >180° 意味着存在某个角度方向上"无路点在其 ±90° 扇形内"，
    # 该处若放定向源就【必然漏检】——所以 180° 是不可越过的硬线，ok 只判硬线。
    # 余量 (180°−best_gap) 单独打印：它是工程裕量，越大越稳。本 24 点构型
    # 余量 6.45°；更少路点的候选（8@1000 得 180.90°、9@1000 得 176.75°）反而更差，
    # 故 ANGLE_MARGIN_DEG 只作参考目标，不参与 ok 判定。
    ok = best_gap <= 180.0
    return best_gap, best_p, ok


def cover_radius(waypoints):
    """距离覆盖自检：盘内任意点到最近路点的最大距离（最坏盲区）。
    Q4 中仍需 ≤1000m（确保全向源在任一位置都能被某路点探测到）。"""
    best = 0.0
    step = 100.0
    x = -ARENA_R
    while x <= ARENA_R + 1e-9:
        y = -ARENA_R
        while y <= ARENA_R + 1e-9:
            if math.hypot(x, y) <= ARENA_R + 1e-9:
                d = min(dist((x, y), w) for w in waypoints)
                if d > best:
                    best = d
            y += step
        x += step
    return best


def report_coverage_q4(planner, verbose=True, step=25.0):
    wps = planner.coverage_points()
    gap, gpos, ok_angle = angle_gap_check(wps, step=step, n_boundary=1440,
                                          refine=True)
    cgap = cover_radius(wps)
    ok_dist = cgap <= R_RECV_MIN
    ok = ok_angle and ok_dist
    if verbose:
        print("[覆盖自检 Q4] 路点数=%d | 距离盲区=%.1fm(≤1000:%s) | "
              "最坏角度间隙=%.3f°@%s (硬线≤180°:%s, 余量=%.2f°) | 双判据合格=%s"
              % (len(wps), cgap, ok_dist, gap, _r2(gpos), ok_angle,
                 180.0 - gap, ok))
    return gap, cgap, ok


def _build_q4_waypoints():
    """M7 构型（v4 重新优化）：中心 + 10环@1000 + 13环@1880（外环相位错开半格）。
    共 24 点。角度覆盖最坏间隙经【极密采样（≥5000 个盘内点 × 2880 方位）】复算
    为 173.545°@(1800,0)，余量 6.45°。

    为什么是这个构型（相对 v3 的 中心+12@1000+12@1900）：
      * v3 构型在极密检验下最坏间隙实为 177.38°@(-927.3,-248.5) r=960 ——
        余量只剩 2.6°，v3 文档里的 171.80° 是粗网格漏检的结果；
      * 最坏点集中在 r≈900~1000 的"内/外环衔接带"：此处外环(≥1800m)已超出
        1000m 接收半径，只能靠内环；内环点数不足就会在 62.6° 窗口内凑不满
        3 点包围。因此内环取 10 点@1000（窗口 ≥3 点），外环取 13 点@1880
        （恰好满足 r≥1800 的外向支撑，边界点的相邻外环点距离 ≤1000）；
      * 24 点比 25 点少 1 个路点 ⇒ 空频道扫描少 5s；TSP 骨架 18250m。
    外环 1880m>1800m 出界——盘内路点对"贴边界且朝外"的定向源原理上必漏检。"""
    wps = [(0.0, 0.0)]
    for k in range(Q4_INNER_N):
        a = 2 * math.pi * k / Q4_INNER_N
        wps.append((Q4_INNER_R * math.cos(a), Q4_INNER_R * math.sin(a)))
    for k in range(Q4_OUTER_N):
        a = 2 * math.pi * (k + 0.5) / Q4_OUTER_N
        wps.append((Q4_OUTER_R * math.cos(a), Q4_OUTER_R * math.sin(a)))
    return wps


def _tsp_nearest(points, start=(0.0, 0.0)):
    """最近邻贪心 TSP 访问顺序（24 点规模最优性足够；起点固定为原点，骨架 ~18.25km）。"""
    pts = list(points)
    order = []
    cur = start
    rem = pts[:]
    while rem:
        idx = min(range(len(rem)), key=lambda i: dist(cur, rem[i]))
        cur = rem.pop(idx)
        order.append(cur)
    return order


def _tour_len(seq, start=(0.0, 0.0)):
    t = dist(start, seq[0])
    for i in range(1, len(seq)):
        t += dist(seq[i - 1], seq[i])
    return t


def _two_opt(order, start=(0.0, 0.0)):
    """2-opt 改进 TSP 顺序（骨架 18.4km→~16.5km）。"""
    pts = list(order)
    n = len(pts)
    if n < 4:
        return pts
    best = _tour_len(pts, start)
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                new = pts[:i] + pts[i:j + 1][::-1] + pts[j + 1:]
                L = _tour_len(new, start)
                if L < best - 1e-9:
                    pts = new
                    best = L
                    improved = True
    return pts


def _tsp_optimized(points, start=(0.0, 0.0)):
    """最近邻 + 2-opt TSP。"""
    return _two_opt(_tsp_nearest(points, start), start)


def _nn_tour_nodes(nodes, start):
    """对 (kind, key, pos) 节点做最近邻 TSP，返回排序后的节点列表。
    用于 rolling TSP：把未访问路点 + 可清除源放一起，每次执行第一步。"""
    rem = list(nodes)
    cur = start
    tour = []
    while rem:
        idx = min(range(len(rem)), key=lambda i: dist(cur, rem[i][2]))
        node = rem.pop(idx)
        tour.append(node)
        cur = node[2]
    return tour


def _poly_clip(poly, nx, ny, c):
    """Sutherland–Hodgman 裁剪：保留 {p : nx·p.x + ny·p.y ≥ c}（凸多边形）。"""
    if not poly:
        return []
    out = []
    m = len(poly)
    for i in range(m):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % m]
        da = nx * ax + ny * ay - c
        db = nx * bx + ny * by - c
        if da >= -1e-12:
            out.append((ax, ay))
        if (da > 1e-12 and db < -1e-12) or (da < -1e-12 and db > 1e-12):
            t = da / (da - db)
            out.append((ax + t * (bx - ax), ay + t * (by - ay)))
    return out


def _poly_clip_convex(poly, clipper):
    """用凸多边形 clipper 裁剪 poly（clipper 顶点可任意方向，内部自动转 CCW）。"""
    n = len(clipper)
    if n < 3 or len(poly) < 3:
        return []
    A = 0.0
    for i in range(n):
        x0, y0 = clipper[i]
        x1, y1 = clipper[(i + 1) % n]
        A += x0 * y1 - x1 * y0
    cl = clipper if A > 0 else clipper[::-1]
    out = list(poly)
    for i in range(n):
        x0, y0 = cl[i]
        x1, y1 = cl[(i + 1) % n]
        nx, ny = -(y1 - y0), (x1 - x0)     # CCW 内法线
        out = _poly_clip(out, nx, ny, nx * x0 + ny * y0)
        if len(out) < 3:
            return []
    return out


def _sector_poly(w, th_deg, eps_deg, rmax, n_arc=12):
    """单条示向度的可行域：以 w 为顶点、方向 th±eps、最大距离 rmax 的凸扇形。
    外接保证：弧上采样点半径取 rmax / cos(dtheta/2)（dtheta = 相邻采样角间距），
    使相邻采样点之间的弦位于真实弧的外侧（切线方向），从而多边形 ⊇ 真实扇形。
    修正量在 eps=1.3°/n_arc=12 时仅 2.7mm（eps=5° 时 40mm），但对"真源 ∈ poly"
    的严格性是必要的——内接多边形会让弧与弦之间的弓形区域（真源可能落入其中）
    被排除在 poly 外，破坏 region()、_range_skip()、_saturate_clear() 的可证安全性。"""
    pts = [(w[0], w[1])]
    a0 = math.radians(th_deg - eps_deg)
    a1 = math.radians(th_deg + eps_deg)
    dtheta = (a1 - a0) / float(n_arc)
    r_ext = rmax / math.cos(dtheta / 2.0)   # 外接修正：弦切于弧
    for k in range(n_arc + 1):
        a = a0 + dtheta * k
        pts.append((w[0] + r_ext * math.cos(a), w[1] + r_ext * math.sin(a)))
    return pts


def _arena_poly(n=72):
    """目标圆盘的外切正 n 边形（略大于圆盘，保证"可行域"不漏掉真源）。"""
    R = ARENA_R / math.cos(math.pi / n)
    return [(R * math.cos(2 * math.pi * k / n),
             R * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _poly_center(poly):
    """凸多边形面积加权质心（退化时退化为顶点均值）。"""
    n = len(poly)
    if n == 0:
        return None
    if n < 3:
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    A = cx = cy = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cr = x0 * y1 - x1 * y0
        A += cr
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    if abs(A) < 1e-9:
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    return (cx / (3.0 * A), cy / (3.0 * A))


def _poly_diam(poly):
    """凸多边形直径（最大值必在顶点对上）。"""
    best = 0.0
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            d = dist(poly[i], poly[j])
            if d > best:
                best = d
    return best


def _poly_samples(poly, step=5.0, cap=360):
    """凸多边形内网格采样（用于"可行域是否已被探针 20m 覆盖"的判定）。"""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    nx = min(60, max(1, int((x1 - x0) / step) + 1))
    ny = min(60, max(1, int((y1 - y0) / step) + 1))
    out = []
    for i in range(nx):
        for j in range(ny):
            p = (x0 + (i + 0.5) * (x1 - x0) / nx,
                 y0 + (j + 0.5) * (y1 - y0) / ny)
            if point_in_convex(p, poly):
                out.append(p)
                if len(out) >= cap:
                    return out
    return out if out else list(poly)


def _farthest_in_poly(samples, probes):
    """在采样点中挑"离已有探针最远"的点（最远点采样，用于铺开 /clear 探针）。"""
    best, bd = None, -1.0
    for s in samples:
        d = min(dist(s, q) for q in probes)
        if d > bd:
            bd, best = d, s
    return best, bd


def _poly_samples_dense(poly, step=4.0, cap=200000):
    """凸多边形内【真实步长】均匀网格采样（供饱和清扫用）。

    与 _poly_samples 的区别：后者把 nx/ny 上限锁死在 60，直径 >300m 的楔形域
    会被迫把网格间距拉大（500m→8.3m），且 cap 截断按 i,j 顺序偏向域的一角，
    导致"边缘采样点距真源 >20m"——这正是退化楔形域探针差几米清不掉的根源。
    本函数按真实 step 采样（不再锁 60），保证凸域边缘也被 ≤step 密度覆盖。

    cap=200000 不再抽稀：_saturate_clear 的域直径上限为 Q4_SATURATE_MAX*(R_CLEAR-6)
    =90*14=1260m，最坏情况（直径1260m的圆）4m网格约 99000 点，内存 ~5.5MB、
    _farthest_in_poly × 90 探针 ≈ 9M 次距离计算（~2s），完全可承受。
    此前 cap=8000 + out[::k] 抽稀会把有效间距拉到 ~52m，破坏"铺满后任意点
    距探针 ≤20m"的终止判据。"""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    nx = max(1, int((x1 - x0) / step) + 1)
    ny = max(1, int((y1 - y0) / step) + 1)
    out = []
    for i in range(nx):
        px = x0 + (i + 0.5) * (x1 - x0) / nx
        for j in range(ny):
            py = y0 + (j + 0.5) * (y1 - y0) / ny
            if point_in_convex((px, py), poly):
                out.append((px, py))
    if len(out) > cap:
        out = out[:cap]   # 理论上不触发（cap 已覆盖最坏情况），仅防御性截断
    return out if out else list(poly)


# ==============================================================================
# 3. Backend：接口抽象 / 真实 HTTP 客户端 / 离线仿真器（Q4 定向源支持）
# ==============================================================================
class Backend(object):
    def enter(self):
        raise NotImplementedError

    def measure(self, x, y, channel):
        raise NotImplementedError

    def clear(self, x, y, channel):
        raise NotImplementedError

    def exit(self):
        raise NotImplementedError

    def total_sources(self):
        """演练/仿真可知道源总数(用于算成功率)；正式测试接口不返回 -> None。"""
        return None

    def total_directional(self):
        """演练/仿真可知道定向源数；正式测试 -> None。"""
        return None


class Client(Backend):

    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID, verbose=True):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.verbose = verbose
        self._ctr = {"enter": 0, "measure": 0, "clear": 0, "exit": 0}
        self.last_vt = 0.0
        self.remaining_real_s = None
        self.max_virtual_s = None

    def _base(self, kind):
        self._ctr[kind] += 1
        return {"arena_id": "default", "robot_id": self.robot_id,
                "request_id": "%s-%d" % (kind, self._ctr[kind])}

    def _post(self, path, payload):
        body = json.dumps(payload).encode("utf-8")
        attempt = 0
        while True:
            attempt += 1
            req = Request(self.base_url + path, data=body,
                          headers={"Content-Type": "application/json; charset=utf-8"},
                          method="POST")
            try:
                with urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                    code = resp.getcode()
                    data = json.loads(resp.read().decode("utf-8"))
            except HTTPError as e:
                try:
                    data = json.loads(e.read().decode("utf-8"))
                except Exception:
                    data = {}
                code = e.code
                if code in (400, 404, 405, 409, 413, 415, 429, 500):
                    raise RuntimeError("HTTP %d on %s: %s" % (code, path, data))
                return code, data
            except (URLError, ConnectionError, OSError) as e:
                if attempt <= 5:
                    time.sleep(0.3 * attempt)
                    continue
                raise RuntimeError("连接失败(重试5次仍失败) %s: %s" % (path, e))
            return code, data

    def _check(self, path, code, data):
        if code != 200:
            raise RuntimeError("HTTP %d on %s: %s" % (code, path, data))
        if data.get("accepted") is not True:
            raise RuntimeError("accepted=false on %s: %s" % (path, data))
        if "virtual_time_s" in data:
            self.last_vt = float(data["virtual_time_s"])
        return data

    def enter(self):
        code, data = self._post("/enter", self._base("enter"))
        d = self._check("/enter", code, data)
        self.remaining_real_s = float(d.get("remaining_real_duration_s", 0))
        self.max_virtual_s = float(d.get("max_virtual_duration_s", 360000))
        return d

    def measure(self, x, y, channel):
        p = self._base("measure")
        p["position"] = {"x": x, "y": y}
        p["channel"] = int(channel)
        code, data = self._post("/measure", p)
        return self._check("/measure", code, data)

    def clear(self, x, y, channel):
        p = self._base("clear")
        p["position"] = {"x": x, "y": y}
        p["channel"] = int(channel)
        code, data = self._post("/clear", p)
        return self._check("/clear", code, data)

    def exit(self):
        try:
            code, data = self._post("/exit", self._base("exit"))
        except Exception:
            return {"accepted": False}
        try:
            return self._check("/exit", code, data)
        except RuntimeError:
            return {"accepted": False}


class MockSimulator(Backend):
    """离线物理仿真器（与真实模拟器同一接口），支持 Q4 定向源。
    物理模型与 Q3 完全一致，新增：
      * directional_ratio：定向源占比（问题4）；
      * 定向源仅在其指向方向 ±90° 扇形内有信号（_in_cover）；
      * near(≤5m 且在覆盖角内) 无示向度；clear(≤20m 与朝向无关)。
    """

    def __init__(self, seed=None, n_sources=None, directional_ratio=0.4,
                 verbose=False):
        import random
        self.rng = random.Random(seed)
        self.verbose = verbose
        self.directional_ratio = directional_ratio
        n = n_sources if n_sources else self.rng.randint(N_SRC_LO, N_SRC_HI)
        chans = self.rng.sample(range(1, N_CH + 1), n)
        self.sources = []
        for i in range(n):
            r = ARENA_R * math.sqrt(self.rng.random())
            a = self.rng.uniform(0, 2 * math.pi)
            pos = (r * math.cos(a), r * math.sin(a))
            direc = None
            if self.rng.random() < directional_ratio:
                direc = self.rng.uniform(0, 360.0)
            self.sources.append({
                "idx": i, "pos": pos, "channel": chans[i],
                "radius": self.rng.uniform(R_RECV_MIN, R_RECV_MAX),
                "cleared": False, "directional": direc})
        self.by_channel = {s["channel"]: s for s in self.sources}
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.vt = 0.0
        self._salt = "%x|" % (self.rng.getrandbits(64))
        self._entered = False

    def _berr(self, src_idx, x, y):
        key = "%s%d|%.1f|%.1f" % (self._salt, src_idx, x, y)
        h = hashlib.sha256(key.encode()).digest()
        u = int.from_bytes(h[:8], "big") / float(1 << 64)
        return (2.0 * u - 1.0) * EPS_DEG

    def _resp(self, **kw):
        d = {"accepted": True, "real_timestamp_ms": int(time.time() * 1000),
             "virtual_time_s": round(self.vt, 6)}
        d.update(kw)
        return d

    def total_sources(self):
        return len(self.sources)

    def total_directional(self):
        return sum(1 for s in self.sources if s["directional"] is not None)

    def enter(self):
        self._entered = True
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.vt = 0.0
        return {"accepted": True, "real_timestamp_ms": int(time.time() * 1000),
                "virtual_time_s": 0, "max_virtual_duration_s": 360000,
                "max_real_duration_s": 1200, "remaining_real_duration_s": 1200}

    def _in_cover(self, s, frm):
        """检测点 frm 是否在源 s 的信号覆盖内（全向恒True；定向需在 ±90° 扇形内）。"""
        if s["directional"] is None:
            return True
        ang = bearing_deg(s["pos"], frm)
        diff = abs((ang - s["directional"] + 180) % 360 - 180)
        return diff <= 90.0

    def measure(self, x, y, channel):
        x = float(x); y = float(y); channel = int(channel)
        self.vt += dist(self.pos, (x, y)) / SPEED
        if channel != self.channel:
            self.vt += T_SWITCH
        self.vt += T_MEASURE
        self.pos = (x, y)
        self.channel = channel
        s = self.by_channel.get(channel)
        if s is None or s["cleared"]:
            return self._resp(measure_result="no_signal")
        d = dist((x, y), s["pos"])
        if d > s["radius"] or not self._in_cover(s, (x, y)):
            return self._resp(measure_result="no_signal")
        if d <= R_NEAR:
            return self._resp(measure_result="near")
        svd = (bearing_deg((x, y), s["pos"]) + self._berr(s["idx"], x, y)) % 360.0
        return self._resp(measure_result="direction", svd_deg=round(svd, 2))

    def clear(self, x, y, channel):
        x = float(x); y = float(y); channel = int(channel)
        self.vt += dist(self.pos, (x, y)) / SPEED
        self.pos = (x, y)
        s = self.by_channel.get(channel)
        if s is not None and not s["cleared"] and dist((x, y), s["pos"]) <= R_CLEAR:
            s["cleared"] = True
            self.vt += T_CLEAR_OK
            return self._resp(clear_result="success")
        self.vt += T_CLEAR_FL
        return self._resp(clear_result="no_target_in_range")

    def exit(self):
        return self._resp(exit_reason="user_exit")


# ==============================================================================
# 4. World：信念状态维护（Q4 扩展：no_signal 记录 + 判空）
# ==============================================================================
def _empty_grid(step=EMPTY_GRID_STEP):
    """M8 判空网格：盘内均匀采样点（~450 点）。判空 = 网格点全部被排除。"""
    pts = []
    x = -ARENA_R
    while x <= ARENA_R + 1e-9:
        y = -ARENA_R
        while y <= ARENA_R + 1e-9:
            if math.hypot(x, y) <= ARENA_R + 1e-9:
                pts.append((round(x, 1), round(y, 1)))
            y += step
        x += step
    return pts


class World(object):
    def __init__(self):
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.vt = 0.0
        self.bearings = {}             # ch -> [(x,y,svd_deg),...]
        self.status = {}               # ch -> 'detected'/'cleared'/'empty'
        self.cleared = set()
        self.empty = set()             # Q4: 已判空频道（无源）
        self.stuck = set()
        self.attempt = {}
        self.waypoints_done = set()
        self.scan_complete = False
        self.path = []
        self.path_len = 0.0
        self.n_measure = 0
        self.n_clear = 0
        self.n_action = 0
        self.n_refine = 0              # Q4: 补测次数
        self.n_orbit = 0               # Q4: 环绕搜索轮数
        self.logs = []
        self.budget_real_s = None
        self.max_virtual_s = 360000.0
        self.enter_wall = None
        self._seq = 0
        self._est_cache = {}
        self._reg_cache = {}
        self._bver = {}                # ch -> 示向度版本号（可行域缓存键）
        # ---- Q4: M8 判空状态 ----
        self.chan_meas = {}            # ch -> 该频道累计被 /measure 次数（扫描节制用）
        self.no_signal = {}            # ch -> [(x,y),...] 该频道测过且无信号的点
        self.possible_grid = {}        # ch -> 尚未被conv判据排除的网格点(信息用)
        self.EMPTY_GRID = _empty_grid()

    # ---- 状态更新 ----
    def on_accepted(self, resp, pos):
        self.vt = float(resp.get("virtual_time_s", self.vt))
        if pos is not None:
            self.path_len += dist(self.pos, pos)
            self.pos = pos
            self.path.append(pos)

    def add_bearing(self, ch, x, y, svd):
        bs = self.bearings.setdefault(ch, [])
        #   限制每频道示向度条数：几十上百条带±1°误差的示向度会让半平面交
        #   数值崩溃（约束自相矛盾→empty），且旧示向度来自远处路点、横向
        #   误差大。保留最近 MAX_BEARINGS 条即可（交会只需 2~4 条好线）。
        if len(bs) >= MAX_BEARINGS:
            bs.pop(0)
        bs.append((x, y, svd))
        self._bver[ch] = self._bver.get(ch, 0) + 1
        self._est_cache.pop((ch, len(bs) - 1), None)
        self._est_cache.pop((ch, len(bs)), None)   # 新旧长度缓存都失效
        if self.status.get(ch) != 'cleared':
            self.status[ch] = 'detected'

    def add_no_signal(self, ch, pos):
        self.no_signal.setdefault(ch, []).append(pos)

    def mark_cleared(self, ch):
        self.cleared.add(ch)
        self.status[ch] = 'cleared'

    def declare_empty(self, ch, reason):
        self.empty.add(ch)
        self.status[ch] = 'empty'
        self._est_cache.pop((ch, 0), None)
        self.log("empty", "判空", {"ch": ch}, "确认无源", reason)

    # ---- 查询 ----
    def estimate(self, ch):
        """同 Q3：返回 (MEC圆心, 置信半径, 示向度条数)。"""
        bs = self.bearings.get(ch, [])
        key = (ch, len(bs))
        if key in self._est_cache:
            return self._est_cache[key]
        if len(bs) < 2:
            out = (None, 1e9, len(bs))
            self._est_cache[key] = out
            return out
        hpl = []
        for (x, y, th) in bs:
            hpl += sector_to_halfplanes(x, y, th, EPS_DEG)
        est = triangulate(bs)
        r = 0.0
        terms = []
        for (x, y, th) in bs:
            d = dist((x, y), est)
            delta = abs((th - bearing_deg((x, y), est) + 180) % 360 - 180)
            t = d * math.tan(math.radians(min(delta + EPS_DEG, 89.0)))
            terms.append(t)
            r = max(r, t)
        n = len(bs)
        for i in range(n):
            for j in range(i + 1, n):
                ang = abs(bs[i][2] - bs[j][2]) % 180.0
                ang = min(ang, 180.0 - ang)          # 两线夹角 ∈ [0,90]
                ang = max(ang, 3.0)                   # 防除零（近平行给出大 cr）
                r = max(r, (terms[i] + terms[j]) /
                        math.sin(math.radians(ang)))
        out = (est, max(r, 1.0), len(bs))
        self._est_cache[key] = out
        return out

    def region(self, ch):
        """M★ 核心（替代 v3 的 cr 置信半径）：由【全部示向度楔形】∩【目标圆盘】
        得到真源的【有界凸可行域】。
        每条示向度 (x,y,θ) 的可行域是以 (x,y) 为顶点、方向 θ±EPS、最大距离
        R_RECV_MAX(=1500m) 的凸扇形（源距必 ≤ 其接收半径 ≤1500m；截断半径
        必须取上界 1500，否则真源可能落在可行域之外 → 探针全失败）。
        多条示向度的可行域求交 ⇒ 多边形区域。
        返回 (poly, center, diam, nb)：
          * poly=None, center=None  ⇒ 示向度不足 2 条，无法交会；
          * poly=None, center!=None ⇒ 楔形交为空（数值退化），退回最小二乘点。
        性质：真源【必然】落在 poly 内（误差 ±1° 已计入，eps 再留 0.3° 余量；
        _sector_poly 采用外接多边形构造，真实扇形 ⊆ poly，故包含关系严格）。
        因此 diam 是【严格的定位误差上界】，可比 v3 的 (t_i+t_j)/sinθ 保守式
        小一到两个数量级（后者把两条"远距离、小夹角"示向度的最坏传播当上界，
        实测真误仅 ~4m，却给出 ~190m 的 cr，导致直清几乎从不触发）。
        """
        bs = self.bearings.get(ch, [])
        nb = len(bs)
        if nb < 2:
            return (None, None, None, nb)
        key = (ch, self._bver.get(ch, 0))
        if key in self._reg_cache:
            return self._reg_cache[key]
        base = _arena_poly()
        poly = None
        for eps in (EPS_DEG + 0.3, EPS_DEG + 1.5, EPS_DEG + 4.0):
            p = base
            for (x, y, th) in bs:
                p = _poly_clip_convex(p, _sector_poly((x, y), th, eps,
                                                      R_RECV_MAX))
                if len(p) < 3:
                    break
            if len(p) >= 3:
                poly = p
                break
        if poly is None:
            out = (None, triangulate(bs), 1e9, nb)
        else:
            out = (poly, _poly_center(poly), _poly_diam(poly), nb)
        self._reg_cache[key] = out
        return out

    def pending_channels(self):
        """已发现(≥1示向度)、未清除、未判空、未卡死的频道。"""
        return [ch for ch, bs in self.bearings.items()
                if ch not in self.cleared and ch not in self.empty
                and ch not in self.stuck and len(bs) >= 1]

    def unknown_channels(self):
        """从未检测到信号、且未清除、未判空的频道（还需去探测）。"""
        return [c for c in range(1, N_CH + 1)
                if c not in self.bearings and c not in self.cleared
                and c not in self.empty]

    def localized_channels(self, conf_thr=CONF_GOOD):
        out = []
        for ch in self.pending_channels():
            est, cr, nb = self.estimate(ch)
            if est is not None and cr <= conf_thr:
                out.append(ch)
        return out

    # ---- M8 统一角度证书：判空 ----
    def local_empty_check(self, ch):
        """增量判空（统一角度证书）：对网格点 p，若 p 满足
           p ∈ conv( N_c ∩ B(p, R_RECV_MAX) )   （N_c = 该频道全部 no_signal 点）
        则 p 处必无源。
        【正确性证明（分离超平面）】设 S = N_c ∩ B(p,R_RECV_MAX) 且 p ∉ conv(S)。
        则存在过 p 的直线使 S 全在某一侧；取单位法向 u 由 p 指向 S 的反侧，
        令定向源方向 θ = arg(u)、半径取 R_RECV_MAX。对任意 m ∈ N_c：
          · m ∈ S ⇒ (m−p)·u < 0 ⇒ 不在 θ 的 ±90° 半平面内 ⇒ 无信号
            （与 dist(m,p) ≤ R_RECV_MAX = r 无关，方向条件已否定）；
          · m ∉ S ⇒ dist(m,p) > R_RECV_MAX ≥ r ⇒ 超距无信号。
        即"p 处存在与全部 no_signal 相容的源"⇒ p 不可排除。逆否即得：
        p ∈ conv(S) ⟹ p 处必无源（对定向/全向、对任意 r∈[R_RECV_MIN,R_RECV_MAX] 均成立）。
        ★为什么半径取 R_RECV_MAX 而不是 R_RECV_MIN：半径上界 r ≤ R_RECV_MAX 决定
        了"哪些 no_signal 点携带信息"——dist>R_RECV_MAX 的点无论源朝哪都不可能
        响应，不含信息；而 dist ≤ R_RECV_MAX 的点一律可用。用 R_RECV_MIN=1000
        只取了一部分可用点，证书更弱（不完整但安全）；用 R_RECV_MAX=1500 是
        同一判据的最强形式（取满全部携带信息的点），且仍严格可靠。
        网格点全部被排除 ⇒ 该频道在盘内任何位置都不可能有源 ⇒ 判空。
        这与"发现保证"是同一个几何判据的两面（M8）。"""
        ns = self.no_signal.get(ch, [])
        if len(ns) < 3:
            return False
        keep = []
        for p in self.EMPTY_GRID:
            near = [n for n in ns if dist(n, p) <= R_RECV_MAX + 1e-9]
            if len(near) < 3:
                keep.append(p)
                continue
            hull = convex_hull(near)
            if not point_in_convex(p, hull):
                keep.append(p)
        self.possible_grid[ch] = keep
        return len(keep) == 0

    # ---- 可解释日志 ----
    def log(self, atype, trigger, inputs, decision, reason):
        self._seq += 1
        rec = {"seq": self._seq, "vt": round(self.vt, 2), "action": atype,
               "trigger": trigger, "inputs": inputs,
               "decision": decision, "reason": reason}
        self.logs.append(rec)
        return rec


# ==============================================================================
# 5. Executor：低层执行（可行域直清 / 探针梯 / 垂直偏移补测 / 沿示向线推进）
# ==============================================================================
class _BudgetExceeded(Exception):
    pass


class Executor(object):
    def __init__(self, backend, world, verbose=True):
        self.bk = backend
        self.w = world
        self.verbose = verbose
        self._rescue = False           # True 时 _clear_probes 强制高档探针数

    def _budget_ok(self):
        if self.w.budget_real_s is None or self.w.enter_wall is None:
            return True
        return (time.time() - self.w.enter_wall) < (self.w.budget_real_s - 20.0)

    @staticmethod
    def _clamp_pos(pos):
        x = max(-MAX_COORD, min(MAX_COORD, pos[0]))
        y = max(-MAX_COORD, min(MAX_COORD, pos[1]))
        return (x, y)

    # ---- 基础动作：测量并统一处理三种结果 ----
    def measure_and_handle(self, pos, ch, note=""):
        """移动到 pos 并对频道 ch 检测；near 即清（Q4 中 near 仍需在覆盖角内，
        但直清已不依赖 near）。返回 'direction'/'near_cleared'/'no_signal'/'near_clear_fail'。"""
        if not self._budget_ok():
            raise _BudgetExceeded()
        pos = self._clamp_pos(pos)
        resp = self.bk.measure(pos[0], pos[1], ch)
        self.w.n_action += 1
        self.w.n_measure += 1
        self.w.chan_meas[ch] = self.w.chan_meas.get(ch, 0) + 1
        self.w.on_accepted(resp, pos)
        self.w.channel = ch
        mr = resp.get("measure_result")
        if mr == "direction":
            svd = float(resp["svd_deg"])
            self.w.add_bearing(ch, pos[0], pos[1], svd)
            self.w.log("measure", note or "探测/逼近", {"ch": ch, "pos": _r2(pos)},
                       "示向度=%.2f°" % svd, "记录示向度用于交会定位")
            return "direction"
        elif mr == "near":
            cresp = self.bk.clear(pos[0], pos[1], ch)
            self.w.n_action += 1
            self.w.n_clear += 1
            self.w.on_accepted(cresp, pos)
            if cresp.get("clear_result") == "success":
                self.w.mark_cleared(ch)
                self.w.log("clear", "测量返回near(≤5m且在覆盖角内)",
                           {"ch": ch, "pos": _r2(pos)}, "成功",
                           "near即源在5m内,就地清除代价最小")
                return "near_cleared"
            self.w.log("clear", "near后清除", {"ch": ch}, "失败",
                       "罕见:near后清除未中,转入环绕/局部搜索")
            return "near_clear_fail"
        else:  # no_signal
            self.w.add_no_signal(ch, pos)
            self.w.log("measure", note or "探测", {"ch": ch, "pos": _r2(pos)},
                       "无信号", "该位置收不到此频道(超距/无此源/定向盲区)")
            return "no_signal"

    # ---- 路点扫描 ----
    def scan_waypoint(self, pos, channels, note="发现扫描"):
        """在 pos 对 channels 逐个 /measure（已判空/已清除频道由调用方排除）。"""
        new_found = 0
        for ch in channels:
            if ch in self.w.cleared or ch in self.w.empty:
                continue
            before = len(self.w.bearings.get(ch, []))
            r = self.measure_and_handle(pos, ch, note=note)
            if r in ("direction", "near_cleared") and before == 0:
                new_found += 1
        self.w.log("scan", note, {"pos": _r2(pos), "探测频道数": len(channels)},
                   "新发现%d个" % new_found,
                   "覆盖路点扫描以保证角度覆盖一个不漏")
        return new_found

    # ---- 基础清除：走到 pos 执行 /clear，返回是否真正清除 ----
    def _direct_clear(self, ch, pos, cr, note):
        """朝 pos 执行一次 /clear 并处理结果。
        Q4 保证：若 pos 落在【可行域】内（真源必在域内）且与真源距离 ≤20m，
        则清除必中。因此本动作失败即意味着 pos 不是真源（用于排除锥）。"""
        if not self._budget_ok():
            raise _BudgetExceeded()
        cresp = self.bk.clear(pos[0], pos[1], ch)
        self.w.n_action += 1
        self.w.n_clear += 1
        self.w.on_accepted(cresp, pos)
        if cresp.get("clear_result") == "success":
            self.w.mark_cleared(ch)
            self.w.log("clear", note, {"ch": ch, "点": _r2(pos),
                       "上界": round(cr, 1) if cr < 1e8 else "-"}, "成功",
                       "清除半径20m且与朝向无关；点的定位上界"
                       "≤20m ⟹ 真源必在半径内")
            return True
        self.w.log("clear", note + "未中", {"ch": ch, "点": _r2(pos)}, "失败",
                   "清除未中 ⟹ 真源不在该点20m内，缩小可行域后继续")
        return False

    # ---- M★ 探针梯：在可行域内铺 /clear 点，直到域被 20m 半径完全覆盖 ----
    def _clear_probes(self, ch, poly, center, max_probes=None):
        """在可行域 poly 内打 /clear 探针。策略：先打面积质心（最可能命中、
        路程最短）；若 max_probes>1，之后每次取"离已试探针最远"的采样点
        （最远点采样）继续铺开。

        【为什么默认条数要"分档自适应"】固定条数的两端都不好：
          * 固定 =5（v4 行为）：每一次失败探针都是一次"域直径量级"的长途，
            实测 16/40 局对照 vt/源 587.5（而 =1 只要 557.9）——多数简单源根本
            不需要铺 5 个点，纯浪费；
          * 固定 =1：省了移动，但对"两条示向度近平行 ⇒ 可行域巨大"的难源，
            质心一击不中就没有后续手段，8 次尝试耗尽后被判 stuck 永不再试
            ⇒ 高定向占比下漏检（实测 dratio=0.8 时成功率掉到 0.9922）。
        折中：按"该频道已被尝试次数"分档升级 —— 简单源走 1 段快速路，
        难源自动升级为铺开式搜索：
            尝试 ≤2  → 1 个（只打质心）
            尝试 ≤5  → 3 个
            否则      → 6 个（把域铺满，最远点采样保证覆盖）
        于是"快"和"不漏"同时拿到：dratio=0.4 时几乎所有源都在第 1 档解决，
        高定向占比时才触发升级。

        终止判据（max_probes>1 时生效）：若域内任一点到最近探针的距离
        ≤ R_CLEAR−6m，则真源（必在域内）到某个已试探针 ≤20m，清除必中
        ⟹再试也只会失败，直接退出换策略。
        每次"失败"都是严格信息：该点 20m 内无源。"""
        if max_probes is None:
            if self._rescue:
                max_probes = Q4_PROBES_HARD
            else:
                n_att = self.w.attempt.get(ch, 0)
                max_probes = Q4_PROBES_EASY if n_att <= 2 else (
                    Q4_PROBES_MID if n_att <= 5 else Q4_PROBES_HARD)
        samples = None
        probes = []
        for k in range(max_probes):
            if k == 0:
                q = center
            else:
                if samples is None:
                    samples = _poly_samples(poly, step=5.0, cap=320)
                q, qd = _farthest_in_poly(samples, probes)
                if q is None or qd <= R_CLEAR - 6.0:
                    break
            if self._direct_clear(ch, q, _poly_diam(poly) / 2.0,
                                  "可行域探针#%d" % k):
                return True
            probes.append(q)
        return False

    # ---- 定向源补测（示向度条数 <2 时用）：垂直偏移拿"大交会角"的第二条 ----
    def get_second_bearing(self, ch):
        """补一条角度多样的示向度。

        Q4 几何：以锚点（检测点或估计点）为中心做【垂直偏移 ±q】。
        ±q 两个候选从源看近似对跖（角差 ≈ 180° - atan(err/q)，err 为估计误差
        ≈4m、q=150m 时偏差仅 ~1.5°）。定向扇形半宽 90°（覆盖 180°闭区间），
        两个对跖点不可能都落在一个 180°闭区间外 ⟹ 至少一个候选在扇形内。
        偏移点距源 √(q²+d²)，对 d ≤ √(1000²−q²) 恒在接收半径内 ⇒ 命中率高。
        ★时间优化（vs v3）：候选从 6 个减到 2 个，且 q 由 300m 降到
        150m（近距更省往返；定向源一旦拿到 2 条示向度就由"可行域+探针"
        接管，不需要很高的交会角）。
        ★锚点选择：nb≥2 且 est 合理时用 est 作锚（候选在源附近，船不必飞回
        远处的检测点，实测比"始终用检测点"省 51~69% 时间）；否则退回最近检测点
        作锚（防近平行线把 est 抛到场地外）。两种锚点下对跖性均成立。
        ★±q 两候选按 dist(当前船位, 候选) 升序尝试 —— 命中率不变（对称等价），
        但把"绕远的那一侧"排到后面，平均少飞一段。"""
        bs = self.w.bearings.get(ch, [])
        if not bs:
            return False
        est, cr, nb = self.w.estimate(ch)
        cands = []
        # ★退化门槛：只有"估计点确实是一个可能的源位置"时，才允许把它当锚点。
        # 近平行两条示向度会让 est 被抛到场地外十几公里，若照此锚点偏移补测，
        # 船就会飞出场地再飞回（实测单次浪费 31.7km）。此时退回
        # "以某条示向度检测点为锚"的分支——该锚点必定在场地内。
        if nb < 2 or est is None or not _sane_pos(est):
            # 锚点取"离当前船位最近的一条示向度检测点"：几何上与取 bs[-1]
            # 完全等价（任取一条线做垂直偏移都能得到 ~90° 交会的新线），
            # 但按路程升序选锚点可以少飞。
            bx, by, bth = min(bs, key=lambda b: dist(self.w.pos, (b[0], b[1])))
            perp = math.radians(bth) + math.pi / 2.0
            for off in (Q4_REFINE_OFF, -Q4_REFINE_OFF):
                cands.append((bx + off * math.cos(perp),
                              by + off * math.sin(perp)))
        else:
            ref = math.radians(bearing_deg(self.w.pos, est))
            base0 = max(60.0, min(250.0, cr if cr < 1e8 else 150.0))
            perp = ref + math.pi / 2.0
            for off in (base0, -base0):
                cands.append((est[0] + off * math.cos(perp),
                              est[1] + off * math.sin(perp)))
        cands = [c for c in cands if _sane_pos(c, slack=600.0)]
        cands.sort(key=lambda c: dist(self.w.pos, c))
        for cand in cands:
            r = self.measure_and_handle(cand, ch, note="补示向度(垂直偏移)")
            if r in ("direction", "near_cleared"):
                self.w.n_refine += 1
                self.w.log("refine", "补测(垂直偏移)", {"ch": ch, "到": _r2(cand)},
                           "补测成功", "角偏<90°保证仍在定向扇形内,"
                           "距源≤1000m 保证在接收半径内")
                return True
        return False

    def push_along_bearing(self, ch, step=350.0):
        """单示向度兜底：沿最后一条示向线推进 step（不越过源：step<d 时
        半径条件恒满足、角偏≈0 仍在扇形内），再垂直偏移 ±250 取第二条示向度。
        仅在示向度<2、可行域无法建立时使用（频次极低）。返回是否拿到新示向度。"""
        bs = self.w.bearings.get(ch, [])
        if not bs:
            return False
        (x1, y1, th) = bs[-1]
        a = math.radians(th)
        nxt = (x1 + step * math.cos(a), y1 + step * math.sin(a))
        r = self.measure_and_handle(nxt, ch, note="沿示向线推进")
        if r == "near_cleared":
            return True
        if r == "no_signal":
            return False                      # 越过源（d<step）或扇形丢失
        (x2, y2, th2) = self.w.bearings[ch][-1]
        perp = math.radians(th2) + math.pi / 2.0
        for off in (250.0, -250.0):
            c = (x2 + off * math.cos(perp), y2 + off * math.sin(perp))
            if self.measure_and_handle(c, ch, note="推进后垂直偏移") in (
                    "direction", "near_cleared"):
                self.w.n_refine += 1
                return True
        return False

    # ---- M★ 统一清除：可行域 → 直清/探针 → 补测收缩 → 沿示向线推进 ----
    def resolve_channel(self, ch, max_round=4):
        """对频道 ch 尽最大努力清除，返回是否真正清除。

        流程（每一轮都由"可行域"驱动，不再有网格空转）：
          1) 可行域 (poly,center,diam)：
             * 直径 ≤ 2·清除半径(40m) → 直接在质心 /clear（必中）；
             * 否则在域内铺 /clear 探针（最多 5 个，最远点采样）；
          2) 探针全失败 ⇒ 该域内无源（矛盾）或域过大：补一条示向度
             （nb<2 时先 get_second_bearing；否则由 get_second_bearing 的
             "估计点垂直偏移"分支补测）使域收缩后重来；
          3) 仍不行 ⇒ 沿示向线推进取更近的示向度（push_along_bearing）。
        全程无"500m 硬推进/30点网格扫描/8点环绕"这类大往返兜底，
        这也是 v3 里 78% 移动耗时的主要来源。"""
        w = self.w
        if ch in w.cleared:
            return True
        if ch in w.stuck:
            return False
        w.attempt[ch] = w.attempt.get(ch, 0) + 1
        if w.attempt[ch] > Q4_MAX_ATTEMPT:
            w.stuck.add(ch)
            w.log("resolve", "尝试超限", {"ch": ch, "次数": w.attempt[ch]},
                  "本轮判卡死跳过", "防止单频道拖死全局；因探针条数按尝试次数"
                  "自适应升级(1→3→6)，8 次调用已含 2×1+3×3+3×6=29 次域内清除尝试。"
                  "仍未清除的留待收尾补救轮(_rescue_stuck)再打一遍")
            return False
        for _ in range(max_round):
            if ch in w.cleared:
                return True
            poly, est, diam, nb = w.region(ch)
            if poly is None:
                # est==None ⇒ 楔形交为空且交会点不可用；est 越界 ⇒ 数值退化
                # （近平行线的交会被抛到场地外，见 _sane_pos）。两种都走补测路径，
                # 绝不对一个"场地外十几公里"的点发 /clear。
                if not _sane_pos(est):
                    if self.get_second_bearing(ch):
                        continue
                    if self.push_along_bearing(ch):
                        continue
                    w.log("resolve", "示向度不足/退化且补测失败", {"ch": ch},
                          "本轮放弃", "退化为单线源,留待后续路点再测")
                    return False
                if self._direct_clear(ch, est, diam, "退化可行域直清"):
                    return True
                if self.get_second_bearing(ch):
                    continue
                if self.push_along_bearing(ch):
                    continue
                return False
            if not _sane_pos(est):
                # 域有界（受场地多边形约束）但质心算出场地外：不可能，保守走补测
                if self.get_second_bearing(ch):
                    continue
                if self.push_along_bearing(ch):
                    continue
                return False
            if diam <= 2.0 * R_CLEAR:
                if self._direct_clear(ch, est, diam / 2.0,
                                      "区域直清(域直径%.1fm)" % diam):
                    return True
            if self._clear_probes(ch, poly, est):
                return True
            # 探针全失败：用补测把域做小，再来一轮
            if len(w.bearings.get(ch, [])) < MAX_BEARINGS - 2 \
                    and self.get_second_bearing(ch):
                continue
            if self.push_along_bearing(ch):
                continue
            return False
        return False

    # 兼容旧调用名（HeuristicPlanner / 阶段2 等）
    def approach_and_clear(self, ch, depth=0):
        return self.resolve_channel(ch)

    # ---- 退化楔形域判定与饱和清扫（死锁兜底；正常局 stuck 为空，绝不触发）----
    def _max_cross_angle(self, ch):
        """该频道所有示向度两两夹角的最大值（度，∈[0,90]）。越小越退化。"""
        bs = self.w.bearings.get(ch, [])
        best = 0.0
        for i in range(len(bs)):
            for j in range(i + 1, len(bs)):
                d = abs(bs[i][2] - bs[j][2]) % 180.0
                d = min(d, 180.0 - d)
                if d > best:
                    best = d
        return best

    def _saturate_clear(self, ch):
        """饱和清扫：把可行域用 /clear 探针【铺满】（间距 ~2·R_CLEAR），真源必在
        域内（region 是严格上界），故必有一个探针落入 20m 清除半径 ⟹ 必清除。
        """
        w = self.w
        poly, center, diam, nb = w.region(ch)
        if poly is None or len(poly) < 3:
            # 楔形交为空（数值退化）：退回"最新一条示向度的扇形"作可行域。
            # 但该扇形半径 R_RECV_MAX、直径可达 ~3000m，90 个探针铺不满，
            # 硬铺只会白费 ~1200s。故仅当扇形直径也在可铺满范围内才继续。
            bs = w.bearings.get(ch, [])
            if not bs:
                return False
            (x1, y1, th) = bs[-1]
            poly = _sector_poly((x1, y1), th, EPS_DEG + 0.3, R_RECV_MAX)
            if len(poly) < 3:
                return False
            center = _poly_center(poly)
            diam = _poly_diam(poly)
            if diam > Q4_SATURATE_MAX * (R_CLEAR - 6.0):
                self.w.log("clear", "饱和清扫放弃", {"ch": ch, "域直径": round(diam)},
                           "放弃", "可行域过大，90 探针铺不满，保持 stuck 供人工复核")
                return False
        samples = _poly_samples_dense(poly, step=Q4_SATURATE_STEP)
        probes = []
        for k in range(Q4_SATURATE_MAX):
            if ch in w.cleared:
                return True
            if k == 0:
                q = center
            else:
                q, qd = _farthest_in_poly(samples, probes)
                if q is None or qd <= R_CLEAR - 6.0:
                    break          # 域已被探针 20m 半径铺满，再试也只会失败
            if not _sane_pos(q, slack=600.0):
                probes.append(q)   # 标记已考察，避免 _farthest_in_poly 重复选它而死循环
                continue
            if self._direct_clear(ch, q, (diam / 2.0) if diam < 1e8 else 0.0,
                                  "饱和探针#%d" % k):
                self.w.log("clear", "饱和清扫命中", {"ch": ch, "探针数": k + 1},
                           "成功", "退化楔形域铺满后真源必在 20m 内")
                return True
            probes.append(q)
        return False
# ==============================================================================
# 6. Planner：高层决策（Q4 版：判空调度 + 先验终止）
# ==============================================================================
def _square_spiral(center, radius, spacing, max_pts):
    pts = [center]
    x, y = center
    step = 1
    cx, cy = x, y
    dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    di = 0
    while len(pts) < max_pts and (step * spacing) <= 2 * radius + spacing:
        for _ in range(2):
            dx, dy = dirs[di % 4]
            for _ in range(step):
                cx += dx * spacing
                cy += dy * spacing
                if dist((cx, cy), center) <= radius + spacing:
                    pts.append((cx, cy))
                if len(pts) >= max_pts:
                    return pts
            di += 1
        step += 1
    return pts


class HeuristicPlanner(object):
    """基线 A：阿基米德螺旋全覆盖扫描 + 最近邻贪心清除。
    """
    name = "A_heuristic_spiral"

    def __init__(self, arm=800.0, step=800.0):
        self.spiral_pts = _gen_spiral(arm=arm, step=step, rmax=ARENA_R)

    def coverage_points(self):
        return list(self.spiral_pts)

    def run(self, ex):
        w = ex.w
        for pt in self.spiral_pts:
            chs = [c for c in range(1, N_CH + 1)
                   if c not in w.cleared and c not in w.empty]
            ex.scan_waypoint(pt, chs, note="螺旋扫描")
            ex.try_empty_declares()
        guard = 0
        while True:
            guard += 1
            if guard > 50:
                break
            if len(w.cleared) >= N_SRC_HI:
                w.log("plan", "先验终止", {}, "任务完成",
                      "已清除16个=源总数上限,提前/exit(M5)")
                break
            pend = []
            for ch in w.pending_channels():
                est, cr, nb = w.estimate(ch)
                if est is not None:
                    pend.append((dist(w.pos, est), ch))
            if not pend:
                break
            pend.sort()
            _, ch = pend[0]
            ex.approach_and_clear(ch)
        w.scan_complete = (len(w.pending_channels()) == 0)


class ParamPlanner(object):
    """24点双环构型 + rolling NN-TSP 顺路清除 + M8判空 + M5先验终止。
    核心机制：
      * rolling NN-TSP：每步把 [未访问路点 ∪ 已可清除源(可行域直径≤Q4_CLEAR_ADMIT)]
        放一起做最近邻排序，只执行第一步后立即重排——自适应源发现，
        源清除变成"顺路"而非"扫完绕回"；
      * 扫描排除已有≥2示向度的频道（全向源多路点自然获得多线；定向源扫完仍
        1 线才由 resolve_channel 内的补测分支处理）；
      * M8 判空：no_signal 点集按 conv(N_c∩B(p,R_RECV_MAX)) 证书逐网格点排除，
        全排除即判空，之后不再测该频道；
      * M5 先验终止：已清除数达到源总数上界 N_SRC_HI=16 即收尾。

    构型 = 中心 + 10环@1000(相位0) + 13环@1880(相位半格)，共 24 点，
    极密复算最坏角度间隙 173.545°@(1800,0)，余量 6.45°。"""
    name = "B_param_cem"

    def __init__(self, weights=None, waypoints=None):
        if waypoints is None:
            waypoints = _build_q4_waypoints()
        # NN + 2-opt TSP 排序，起点为原点
        self.waypoints = _tsp_optimized(list(waypoints), start=(0.0, 0.0))

    def coverage_points(self):
        return list(self.waypoints)

    def _scan_channels(self, w):
        """扫描时要测的频道：排除已清除、已判空、已有≥2示向度的。"""
        return [c for c in range(1, N_CH + 1)
                if c not in w.cleared and c not in w.empty
                and len(w.bearings.get(c, [])) < 2
                and w.chan_meas.get(c, 0) < Q4_SCAN_MAX_MEAS]

    def run(self, ex):
        w = ex.w
        guard = 0
        # ===== 单阶段 rolling TSP：路点 + 所有已定位源（不限制cr）混合排序 =====
        # 关键：阶段1+阶段2 统一 rolling，消除阶段分割的路径冗余。
        # rolling NN 每次只执行 TSP 第一项，立即更新状态重排——自适应源发现。
        # 路点 scan 后立即尝试清附近任意 cr 的源（不一定cr≤GATE），只要测完路点
        # 已经获得 ≥2 示向度的源就立即清。
        while True:
            guard += 1
            if guard > 400:
                w.log("plan", "主循环", {}, "强制收尾", "迭代超上限,防死循环")
                break
            if len(w.cleared) >= N_SRC_HI:
                w.log("plan", "先验终止", {"已清除": len(w.cleared)}, "任务完成",
                      "清除数=源总数上限,剩余必为无源频道,提前/exit(M5)")
                w.scan_complete = True
                break
            # 节点：未扫路点 + cr≤100 的源估计点（混合排序）
            nodes = []
            for i, wp in enumerate(self.waypoints):
                if i not in w.waypoints_done:
                    nodes.append(("wp", i, wp))
            for ch in w.pending_channels():
                est, cr, nb = w.estimate(ch)
                # 加 _sane_pos：退化交会会把 est 抛到场地外十几公里，
                # 一旦被当成"顺路清除节点"，rolling NN 就会把它排进路线里，
                # 船会真的飞出去（实测单次 31.7km）。母线交会点必须在场内。
                if _sane_pos(est) and cr <= 100.0 and nb >= 2:
                    nodes.append(("clear", ch, est))
            if not nodes:
                break  # 所有路点扫完且无可清的源
            # rolling NN
            tour = _nn_tour_nodes(nodes, w.pos)
            kind, key, pos = tour[0]
            if kind == "wp":
                chs = self._scan_channels(w)
                ex.scan_waypoint(pos, chs, note="rolling扫描")
                w.waypoints_done.add(key)
            else:  # clear
                ex.resolve_channel(key)
            ex.try_empty_declares()
        w.scan_complete = True
        # ===== 阶段2：剩余源（nb<2, 即 1-线频道需补测）批量 NN TSP =====
        guard2 = 0
        for _ in range(150):
            guard2 += 1
            if len(w.cleared) >= N_SRC_HI:
                break
            pend = w.pending_channels()
            if not pend:
                break
            one_line = [ch for ch in pend if w.region(ch)[3] < 2]
            multi = [(w.region(ch)[1], ch) for ch in pend
                     if w.region(ch)[1] is not None and w.region(ch)[3] >= 2]
            if one_line:
                one_line.sort(key=lambda ch: min(
                    dist(w.pos, (b[0], b[1])) for b in w.bearings.get(ch, [(0, 0, 0)])))
                ch = one_line[0]
                if not ex.get_second_bearing(ch):
                    ex.push_along_bearing(ch)
                ex.resolve_channel(ch)
                ex.try_empty_declares()
                continue
            if multi:
                tour = _nn_tour_nodes(
                    [("clear", ch, est) for est, ch in multi], w.pos)
                for _kind, ch, _pos in tour:
                    if ch in w.cleared:
                        continue
                    ex.resolve_channel(ch)
                    ex.try_empty_declares()
                    if len(w.cleared) >= N_SRC_HI:
                        break
                continue
            break
        # ===== 收尾补救轮：对被判 stuck 的频道再打一遍（保证"一个不漏"）=====
        # 主循环与阶段2都靠"尝试次数上限"防拖死，代价是极难源会被放弃。
        # 本轮的职责就是把它们捡回来：重置尝试计数并强制走"铺满可行域"的
        # 高档探针策略。stuck 集合在正常情况下为空（dratio=0.4 实测 40 局零触发），
        # 所以这一步对常规耗时无影响，只在难例上补时间买成功率。
        self._rescue_stuck(ex)

    def _rescue_stuck(self, ex):
        """收尾补救：把 stuck 频道捡回来，直到清除或确认无法清除。

        每个 stuck 频道按其几何退化程度分两类处理：
          * 退化楔形域（最大交会角 < Q4_DEGEN_CROSS_DEG）：常规 _clear_probes
            的少量最远点探针根本铺不满狭长楔形，且真源紧贴楔形边缘，反复撒网
            必然失败 ⟹ 直接 _saturate_clear 饱和铺满（真源必在 region 域内）；
          * 非退化难源：先 rescue 高档探针常规 resolve 一次，仍失败再饱和铺满。
        真源必在 region 返回的可行域内（严格上界，误差已含 ±1°+余量），饱和
        铺满 ⟹ 清除，故一次性处理即可，无需历史的多轮重试。"""
        w = ex.w
        for ch in sorted(list(w.stuck)):
            if ch in w.cleared or len(w.cleared) >= N_SRC_HI:
                w.stuck.discard(ch)
                continue
            w.stuck.discard(ch)
            w.attempt[ch] = 0
            ok = False
            if len(w.bearings.get(ch, [])) < 2 or \
                    ex._max_cross_angle(ch) >= Q4_DEGEN_CROSS_DEG:
                # 单线（需先补测拿第2条示向度）或非退化（探针能铺满）：
                # 先常规 resolve（rescue 强制高档探针）一次
                ex._rescue = True
                try:
                    ok = ex.resolve_channel(ch)
                finally:
                    ex._rescue = False
            if not ok and ch not in w.cleared and \
                    len(w.bearings.get(ch, [])) >= 2:
                # 退化楔形域，或常规 resolve 仍失败且域已有界（≥2示向度）：
                # 饱和铺满可行域（真源必在域内）。单线扇形域太大不铺（铺不满）。
                ok = ex._saturate_clear(ch)
            ex.try_empty_declares()
            if ch not in w.cleared:
                # 仍失败（单线补测不进，或 region 域罕见算错）：保持 stuck 供如实报告
                w.stuck.add(ch)


# ---- M8 判空调度（挂在 Executor 上，供两个 planner 复用）----
def _try_empty_declares(ex):
    """全局判空 + 局部增量判空（统一角度证书 M8）。
    全局版：全部路点扫描完成且某频道从未有信号 ⇒ 无源（角度覆盖保证的推论）。
    局部版：未扫描完但该频道 no_signal 点已足够多时，用 conv(N_c∩B(p,R_RECV_MAX))
    排除全部判空网格点 ⇒ 判空（与发现保证同一判据，见 World.local_empty_check）。"""
    w = ex.w
    for ch in range(1, N_CH + 1):
        if ch in w.cleared or ch in w.empty or ch in w.bearings:
            continue
        if w.scan_complete:
            w.declare_empty(ch, "全局判空: 全部路点扫描后仍无信号,双判据覆盖⟹无源(M8)")
            continue
        ns = w.no_signal.get(ch, [])
        if len(w.waypoints_done) >= LOCAL_EMPTY_MIN_WP and len(ns) >= LOCAL_EMPTY_MIN_NS:
            if w.local_empty_check(ch):
                w.declare_empty(ch, "局部判空: conv(N_c∩B(p,1000))排除全网格,统一角度证书(M8)")


Executor.try_empty_declares = _try_empty_declares


# ==============================================================================
# 7. Metrics：指标统计 + CSV 导出（Q4 新增判空/定向口径）
# ==============================================================================
def _gen_spiral(arm=800.0, step=800.0, rmax=1800.0):
    b = arm / (2 * math.pi)
    pts = []
    phi = 0.0
    while True:
        r = b * phi
        if r > rmax:
            break
        pts.append((r * math.cos(phi), r * math.sin(phi)))
        ds = math.sqrt(r * r + b * b)
        phi += step / max(ds, 1e-6)
    return pts


def _overlap_rate(path):
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    cells = set()
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        seg = dist(a, b)
        total += seg
        n = max(1, int(seg / 5.0))
        for k in range(n + 1):
            t = k / n
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            cells.add((round(x / 5.0), round(y / 5.0)))
    union_len = len(cells) * 5.0
    if total <= 0:
        return 0.0
    return max(0.0, 1.0 - union_len / total)


def _out_path(name):
    try:
        base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "generated", "q4")
    except NameError:
        base = os.getcwd()
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, name)


def run_once(backend, planner, verbose=False):
    world = World()
    ex = Executor(backend, world, verbose=verbose)
    t0 = time.time()
    ent = backend.enter()
    world.budget_real_s = float(ent.get("remaining_real_duration_s", 0))
    world.max_virtual_s = float(ent.get("max_virtual_duration_s", 360000))
    world.enter_wall = time.time()
    world.log("enter", "进入目标区域", {}, "开始任务",
              "初始位置(0,0),初始频道1")
    abort_reason = ""
    try:
        planner.run(ex)
    except _BudgetExceeded:
        world.log("budget", "现实预算将尽", {}, "主动收尾",
                  "按/enter返回的remaining_real_duration_s提前退出,避免超窗判负")
    except (RuntimeError, OSError, ValueError) as e:
        abort_reason = str(e)[:200]
        world.log("abort", "与模拟器通信异常", {}, "提前结束本轮", abort_reason)
    backend.exit()
    world.log("exit", "任务结束", {}, "主动退出", "覆盖完成或预算将尽")
    real_elapsed = time.time() - t0

    total_src = backend.total_sources()
    total_dir = backend.total_directional()
    cleared = len(world.cleared)
    cov_gap, cov_blind, cov_ok = (0.0, 0.0, True)
    if hasattr(planner, "coverage_points"):
        cov_gap, cov_blind, cov_ok = report_coverage_q4(planner, verbose=False)
    unresolved = sorted(ch for ch in world.bearings
                        if ch not in world.cleared and ch not in world.empty)
    basis = "simulator"
    if total_src is None and world.scan_complete and cov_ok and not unresolved:
        # 真实模拟器不返回源总数：由【双判据覆盖自检合格 + 扫描完成 + 无遗留
        # 未清除信号】反推：每个源必被某路点发现（角度覆盖），被发现过的又
        # 全部被清除 ⇒ 清除数 = 总源数。硬前提是双判据覆盖合格（Q4 比 Q3
        # 多一条角度判据，缺一不可）。新增第三前提：无任何未解决频道。
        total_src = cleared
        basis = "coverage-inferred(角度%.3f°≤180°,距离盲区%.1fm≤1000m)" % (
            cov_gap, cov_blind)
    elif total_src is None:
        basis = "unknown(未解决%d个频道:%s)" % (len(unresolved), unresolved)
    # 定向源清除情况（仅演练可查；正式测试不返回）
    dir_cleared = ""
    if total_dir is not None:
        dir_cleared = sum(1 for s in backend.sources
                          if s["directional"] is not None and s["cleared"])
    m = {
        "planner": planner.name,
        "n_sources": total_src if total_src is not None else "",
        "n_sources_basis": basis,
        "n_directional": total_dir if total_dir is not None else "",
        "n_directional_cleared": dir_cleared,
        "cleared": cleared,
        "success_rate": (cleared / total_src) if total_src else "",
        "total_virtual_time_s": round(world.vt, 2),
        "avg_clear_time_s": round(world.vt / cleared, 2) if cleared else "",
        "path_length_m": round(world.path_len, 1),
        "overlap_rate": round(_overlap_rate(world.path), 4),
        "angle_gap_deg": round(cov_gap, 3),
        "cover_gap_m": round(cov_blind, 2),
        "coverage_ok": cov_ok,
        "scan_complete": world.scan_complete,
        "n_empty": len(world.empty),
        "n_unresolved": len(unresolved),
        "unresolved_channels": unresolved,
        "n_stuck": len(world.stuck),
        "n_measure": world.n_measure,
        "n_clear": world.n_clear,
        "n_refine": world.n_refine,
        "n_orbit": world.n_orbit,
        "n_action": world.n_action,
        "real_elapsed_s": round(real_elapsed, 2),
        "cleared_channels": sorted(world.cleared),
        "abort": abort_reason,
    }
    return m, world


def run_batch(planner_factory, n_runs, seed0=1000, dratio=0.4, csv_path=None,
              verbose=False):
    rows = []
    for r in range(n_runs):
        bk = MockSimulator(seed=seed0 + r, directional_ratio=dratio)
        planner = planner_factory()
        m, world = run_once(bk, planner, verbose=verbose)
        m["run_id"] = r + 1
        rows.append(m)
        print("[run %d/%d] %s | 源=%s(定向%s) 清除=%s 成功率=%s | 判空=%s | "
              "总虚拟时=%ss 均=%ss | 路程=%sm" % (
                  r + 1, n_runs, m["planner"], m["n_sources"],
                  m["n_directional"], m["cleared"], m["success_rate"],
                  m["n_empty"], m["total_virtual_time_s"],
                  m["avg_clear_time_s"], m["path_length_m"]))
    import statistics as st
    keys = ["success_rate", "total_virtual_time_s", "avg_clear_time_s",
            "path_length_m", "overlap_rate"]
    agg = {"planner": rows[0]["planner"], "n_runs": n_runs,
           "directional_ratio": dratio}
    for k in keys:
        vals = [row[k] for row in rows if isinstance(row[k], (int, float))]
        if vals:
            agg[k + "_mean"] = round(st.mean(vals), 3)
            agg[k + "_std"] = round(st.pstdev(vals), 3) if len(vals) > 1 else 0.0
    if csv_path:
        _write_csv(csv_path, rows, agg)
    return rows, agg


def _write_csv(csv_path, rows, agg):
    cols = ["run_id", "planner", "n_sources", "n_sources_basis", "n_directional",
            "n_directional_cleared", "cleared", "success_rate",
            "total_virtual_time_s", "avg_clear_time_s", "path_length_m",
            "overlap_rate", "angle_gap_deg", "cover_gap_m", "coverage_ok",
            "scan_complete", "n_empty", "n_measure", "n_clear", "n_refine",
            "n_orbit", "n_action", "real_elapsed_s", "abort"]
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        wcsv = csv.DictWriter(f, fieldnames=cols)
        wcsv.writeheader()
        for row in rows:
            wcsv.writerow({c: row.get(c, "") for c in cols})
        agg_row = {c: "" for c in cols}
        agg_row["planner"] = agg["planner"] + " [MEAN±STD]"
        agg_row["n_sources"] = "n_runs=%d dratio=%.2f" % (
            agg["n_runs"], agg["directional_ratio"])
        agg_row["success_rate"] = "%.3f±%.3f" % (
            agg.get("success_rate_mean", 0), agg.get("success_rate_std", 0))
        agg_row["total_virtual_time_s"] = "%.1f±%.1f" % (
            agg.get("total_virtual_time_s_mean", 0),
            agg.get("total_virtual_time_s_std", 0))
        agg_row["avg_clear_time_s"] = "%.1f±%.1f" % (
            agg.get("avg_clear_time_s_mean", 0),
            agg.get("avg_clear_time_s_std", 0))
        agg_row["path_length_m"] = "%.0f±%.0f" % (
            agg.get("path_length_m_mean", 0), agg.get("path_length_m_std", 0))
        wcsv.writerow(agg_row)
    print("CSV 已写出: %s" % csv_path)


# ==============================================================================
# 8. CEM 调参（可选；默认用 Q3 调优权重）
# ==============================================================================
def tune_cem(BASE_URL=None, n_iter=12, pop=16, elite=4, runs_per_eval=3,
             seed0=5000, dratio=0.4, **kw):
    
    print("[--tune] v5 的 ParamPlanner 无打分权重可调，跳过 CEM。")
    print("         结构性旋钮已实测选定：_clear_probes=1 / Q4_REFINE_OFF=150 / "
          "Q4_CLEAR_ADMIT=300 / 24点构型。")
    print("         若确需重扫，请用 work/bench.py + work/expq8.py 这类外部脚本"
          "（不污染交付文件）。")
    return None, []


# ==============================================================================
# 9. 主流程 / 命令行（Q4 新增 --dratio / --no-local-empty）
# ==============================================================================
def _in_ipython():
    try:
        return get_ipython() is not None
    except NameError:
        return False


_ARG_TAKES_VALUE = ("--mode", "--planner", "--runs", "--seed0", "--csv",
                    "--weights", "--dratio")
_ARG_FLAGS = ("--tune", "--verbose", "--no-local-empty", "-h", "--help")


def _resolve_argv():
    cleaned = []
    raw = sys.argv[1:]
    n = len(raw)
    i = 0
    while i < n:
        a = raw[i]
        name = a.split("=", 1)[0]
        if name in _ARG_TAKES_VALUE and "=" not in a:
            cleaned.append(a)
            if i + 1 < n:
                cleaned.append(raw[i + 1])
                i += 2
            else:
                i += 1
            continue
        if name in _ARG_TAKES_VALUE or name in _ARG_FLAGS:
            cleaned.append(a)
            i += 1
            continue
        i += 2 if name == "-f" else 1
    if not cleaned and _in_ipython():
        cleaned = NOTEBOOK_ARGS.split()
    return cleaned


RANGE_SKIP_ENABLE = True   # 主改动开关
RANGE_SKIP_MARGIN = 0.0    # dist(pos,Sec) 需 > R_RECV_MAX + margin 才跳过


def _v6_acute(t1, t2):
    """两个方位角的锐夹角（度），∈[0,90]。"""
    g = abs(t1 - t2) % 180.0
    return min(g, 180.0 - g)


def _v6_dist_point_convex(p, poly):
    """点 p 到凸多边形 poly 的最短距离（p 在内部时为 0）。"""
    if poly is None or len(poly) < 3:
        return 1e18
    if point_in_convex(p, poly):
        return 0.0
    best = 1e18
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            t = 0.0
        else:
            t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2
            t = max(0.0, min(1.0, t))
        d = dist(p, (ax + t * dx, ay + t * dy))
        if d < best:
            best = d
    return best


class ParamPlannerV6(ParamPlanner):
    """v6 排程器：与 v5 完全相同的 rolling NN-TSP + 阶段2 + 收尾补救，
    只把"在某个路点扫描哪些频道"这一处替换为带可证超距跳过的版本。"""

    name = "B6_rangeskip"

    def __init__(self, weights=None, waypoints=None):
        ParamPlanner.__init__(self, weights=weights, waypoints=waypoints)

    def _range_skip(self, w, c, pos):
        """可证超距判据（定理级，非启发式）。详见本段开头注释。"""
        if not RANGE_SKIP_ENABLE:
            return False
        bs = w.bearings.get(c, [])
        if not bs:
            return False
        # 已有 ≥2 条示向度：直接用 v5 的凸可行域（更紧）
        poly = w.region(c)[0]
        if poly is None or len(poly) < 3:
            # 单线：用"最新一条示向度对应的扇形"作可行域（凸、有界）
            (x1, y1, th) = bs[-1]
            poly = _sector_poly((x1, y1), th, EPS_DEG + 0.3, R_RECV_MAX)
        d = _v6_dist_point_convex(pos, poly)
        return d > R_RECV_MAX + RANGE_SKIP_MARGIN

    def _scan_channels_v6(self, w, pos):
        """在 v5 的 _scan_channels 条件上叠加【可证超距跳过】。

        v5 条件：ch 未清除、未判空、示向度 <2、测量次数 < Q4_SCAN_MAX_MEAS。
        v6 追加：若该频道已有 ≥1 条示向度且 pos 距其可行域 > R_RECV_MAX，
                 则本次测量必然超距返回 no_signal，严格跳过。
        """
        out = []
        for c in range(1, N_CH + 1):
            if c in w.cleared or c in w.empty:
                continue
            bs = w.bearings.get(c, [])
            if len(bs) >= 2:
                continue
            if w.chan_meas.get(c, 0) >= Q4_SCAN_MAX_MEAS:
                continue
            if len(bs) >= 1 and self._range_skip(w, c, pos):
                continue
            out.append(c)
        return out

    def run(self, ex):
        """复用 v5 的 run，仅把 scan_waypoint 的频道清单换成 v6 版。

        最小侵入做法：包一层 scan_waypoint，忽略 planner 原本传入的 channels，
        改用 _scan_channels_v6(w, pos)。v5 的 Executor 与排程逻辑零改动。"""
        orig_scan = ex.scan_waypoint
        w = ex.w

        def v6_scan(pos, channels, note="发现扫描"):
            return orig_scan(pos, self._scan_channels_v6(w, pos), note=note)

        ex.scan_waypoint = v6_scan
        try:
            return ParamPlanner.run(self, ex)
        finally:
            ex.scan_waypoint = orig_scan


# ==============================================================================
# 22. 入口：单文件直接运行 / JupyterLab 单元格直接运行
# ==============================================================================
def _v6_run_batch(n_runs, seed0=1000, dratio=0.4, verbose=True, planner_name="B"):
    rows = []
    for i in range(n_runs):
        seed = seed0 + i
        bk = MockSimulator(seed=seed, directional_ratio=dratio)
        pl = HeuristicPlanner() if planner_name == "A" else ParamPlannerV6()
        m, w = run_once(bk, pl, verbose=False)
        m["seed"] = seed
        rows.append(m)
        if verbose:
            sr = float(m["success_rate"]) if m["success_rate"] != "" else -1.0
            ac = float(m["avg_clear_time_s"]) if m["avg_clear_time_s"] != "" else -1.0
            print("  [%2d/%2d] seed=%-5d src=%-3s cleared=%-3d ok=%.4f "
                  "vt=%8.1f s/源=%6.1f path=%5.1fkm n_meas=%3d" % (
                      i + 1, n_runs, seed, m["n_sources"], m["cleared"], sr,
                      m["total_virtual_time_s"], ac,
                      m["path_length_m"] / 1000.0, m["n_measure"]))
    return rows


def _v6_agg(rows):
    tot_src = sum(int(r["n_sources"]) for r in rows if r["n_sources"] != "")
    tot_cl = sum(int(r["cleared"]) for r in rows)
    tot_vt = sum(float(r["total_virtual_time_s"]) for r in rows)
    return dict(
        n_runs=len(rows), sources=tot_src, cleared=tot_cl,
        success_rate=(tot_cl / tot_src if tot_src else float("nan")),
        avg_per_source=(tot_vt / tot_cl if tot_cl else float("nan")),
        avg_per_run=tot_vt / len(rows),
        coverage_ok=all(r["coverage_ok"] for r in rows))


def main_v6():
    """v6 主入口：支持 --mode mock/real，Jupyter 中无参数时用 NOTEBOOK_ARGS。

    【重要】--mode real 才会连真实模拟器；--mode mock 只在本机跑离线仿真。
    早期合并版本漏接了 mode 分支，导致"Jupyter 有反应、模拟器无反应"
    （程序从未发出任何 HTTP 请求）。本版已完整恢复 v5 的 real 通道。
    """
    # _resolve_argv() 已在无命令行参数且处于 IPython 时回退到 NOTEBOOK_ARGS
    argv = _resolve_argv()

    def _val(flag, default, cast):
        for i, a in enumerate(argv):
            if a == flag and i + 1 < len(argv):
                try:
                    return cast(argv[i + 1])
                except Exception:
                    return default
            if a.startswith(flag + "="):
                try:
                    return cast(a.split("=", 1)[1])
                except Exception:
                    return default
        return default

    mode = _val("--mode", "mock", str)
    planner = _val("--planner", "B", str)
    n_runs = _val("--runs", 40, int)
    dratio = _val("--dratio", 0.4, float)
    seed0 = _val("--seed0", 1000, int)  
    if mode not in ("mock", "real"):
        mode = "mock"

    print("=" * 78)
    print("2026 国赛 B 题 · 问题 4 策略 v6（可证超距跳过）单文件版")
    print("=" * 78)
    print("[运行配置] mode=%s  planner=%s  robot_id=%s  dratio=%.2f%s" % (
        mode, planner, "configured" if "<" not in ROBOT_ID else "unconfigured", dratio,
        "   (Jupyter 单元格模式)" if _in_ipython() else ""))

    # ==========================================================================
    # A. 真实模拟器通道（--mode real）
    # ==========================================================================
    if mode == "real":
        if (not ROBOT_ID) or ("<" in ROBOT_ID):
            print("!! 请设置环境变量 ROBOT_ID 为模拟器登录身份!!")
            return None
        print("!! 正在连接真实模拟器 %s …（请确认界面该局已开始）" % BASE_URL)
        bk = Client(base_url=BASE_URL, robot_id=ROBOT_ID, verbose=True)
        pl = HeuristicPlanner() if planner == "A" else ParamPlannerV6()
        gap, cgap, ok = report_coverage_q4(pl)
        print("覆盖自检：最坏角度间隙 %.3f°（判据 ≤180°），最大距离盲区 %.1f m"
              "（判据 ≤1000m），合格 = %s" % (gap, cgap, ok))
        if not ok:
            print("!! 覆盖不合格，本局不保证一个不漏；请检查路点构型。")
        try:
            m, world = run_once(bk, pl, verbose=True)
        except Exception as e:
            print("\n!! 与模拟器通信失败：%s" % e)
            print("   逐条排查：")
            print("     1) 模拟器界面该局是否已开始、机器狗状态是否显示'已进入'；")
            print("     2) 检查环境变量 ROBOT_ID 是否与模拟器登录身份一致；")
            print("     3) 是否已过 25 分钟窗口 / 20 分钟程序时限（倒计时归零接口即关闭）；")
            print("     4) 接口地址端口是否仍是 %s。" % BASE_URL)
            return None
        print("\n==== 本次运行指标 ====")
        for k, v in m.items():
            print("  %s = %s" % (k, v))
        log_path = _out_path("q4_action_log.json")
        with open(log_path, "w", encoding="utf-8") as f:
            json.dump(world.logs, f, ensure_ascii=False, indent=1)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        arch = _out_path("q4_action_log_%s_%s.json" % (planner, stamp))
        with open(arch, "w", encoding="utf-8") as f:
            json.dump(world.logs, f, ensure_ascii=False, indent=1)
        print("可解释动作日志已存：%s（共 %d 条）" % (log_path, len(world.logs)))
        print("                   归档：%s" % arch)
        print("提示：正式测试请从模拟器界面导出加密日志放入支撑材料。")
        return m

    # ==========================================================================
    # B. 离线仿真通道（--mode mock）
    # ==========================================================================
    _cov_pl = HeuristicPlanner() if planner == "A" else ParamPlannerV6()
    gap, cgap, ok = report_coverage_q4(_cov_pl)
    print("覆盖自检：最坏角度间隙 %.3f°（判据 ≤180°），最大距离盲区 %.1f m"
          "（判据 ≤1000m），合格 = %s" % (gap, cgap, ok))
    if not ok:
        print("!! 覆盖不合格，本局不保证一个不漏，请检查路点构型。")

    print("\n[1] dratio=%.1f, %d 局 (seed%d 起)" % (dratio, n_runs, seed0))
    r1 = _v6_run_batch(n_runs, seed0=seed0, dratio=dratio,
                       verbose=(n_runs <= 20), planner_name=planner)
    a1 = _v6_agg(r1)
    print("    -> 源=%d 清除=%d 成功率=%.4f  vt/源=%.1f s  vt/局=%.1f s  覆盖合格=%s"
          % (a1["sources"], a1["cleared"], a1["success_rate"],
             a1["avg_per_source"], a1["avg_per_run"], a1["coverage_ok"]))

    if n_runs >= 20:
        print("\n[2] 跨定向占比压测 (各 20 局, seed2000 起)")
        for dr in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
            rr = _v6_run_batch(20, seed0=2000, dratio=dr, verbose=False,
                               planner_name=planner)
            aa = _v6_agg(rr)
            print("    dratio=%.1f  源=%3d 清除=%3d 成功率=%.4f  vt/源=%.1f s"
                  % (dr, aa["sources"], aa["cleared"], aa["success_rate"],
                     aa["avg_per_source"]))
    return a1


if __name__ == "__main__":
    main_v6()
