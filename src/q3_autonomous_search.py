# -*- coding: utf-8 -*-
"""Autonomous search, bounded-bearing localization and multi-source clearance.

Team competition implementation. Includes offline simulation, a baseline planner,
a parameterized planner, ray and legacy executors, geometric coverage checks,
and an optional local contest-simulator HTTP backend.
"""

import os
import json
import math
import time
import csv
import sys
import socket
import argparse
import hashlib
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError
from urllib.parse import urlparse

# ==============================================================================
# 0. Config：硬约束常量（来源：题面正文 + 附件2，逐条核对，写死）
# ==============================================================================
BASE_URL = "http://127.0.0.1:2026"     # 附件2 §1.5 模拟器默认服务地址（本地回环，无个人信息）

# Simulator identity is supplied at runtime through ROBOT_ID or --robot-id.
ROBOT_ID_PLACEHOLDER = "000000000000"  # 占位符：明显假值，绝不可能是真实队号
ROBOT_ID = ROBOT_ID_PLACEHOLDER       # 模块级默认=占位符；真实队号在运行时由 env/CLI 注入

def resolve_robot_id(cli_value=None):
    """按 命令行参数 > 环境变量 > 占位符 的优先级解析 robot_id。
    源码中不含任何真实队号；队号仅在运行环境（env / CLI）中提供。"""
    rid = (cli_value or os.environ.get("ROBOT_ID", "") or "").strip()
    return rid if rid else ROBOT_ID_PLACEHOLDER

def _mask_robot_id(rid):
    """仅用于运行日志显示，避免任何队号数字出现在输出中。"""
    if not rid or rid == ROBOT_ID_PLACEHOLDER:
        return "占位符(未配置)"
    if len(rid) <= 4:
        return "******"
    return rid[:2] + "******" + rid[-2:]

# 在 Jupyter / VS Code Notebook 的单元格里【直接运行本文件】时用的默认参数。
# 为什么需要：Jupyter 内核把 sys.argv 设成 "ipykernel_launcher.py -f kernel-xxx.json"，
# 命令行参数必须在这里显式给出（否则 argparse 认不出 -f 会直接报错退出）。
# 用 `%run q3_strategy.py --mode real` 这种显式命令时，本项自动失效、以实际命令为准。
# ★Jupyter 默认必须 mock（安全、零依赖、可随便跑调试）★
# ★real 模式必须显式切：把本行改成 "--mode real --planner B" + 改 ROBOT_ID★
# 上一个版本默认 real 是个设计错误——Jupyter 是开发环境，一运行就冲真机，
# 被 preflight 拦住后用户反而困惑。Jupyter 默认 mock，real 显式启。
NOTEBOOK_ARGS = "--mode mock --planner B"

ARENA_R   = 1800.0     # 目标区域半径(m)，圆心(0,0)，x东y北      [题面]
N_CH      = 20         # 频道总数 1..20                           [附录1(1)]
N_SRC_LO  = 10         # 干扰源个数下限                            [附件2 §1.3]
N_SRC_HI  = 16         # 干扰源个数上限                            [附件2 §1.3]
SPEED     = 5.0        # 移动速度 m/s                             [附录2(6)]
T_MEASURE = 5.0        # 一次检测耗时 s                            [附件2 §4.3]
T_SWITCH  = 1.0        # 切换频道耗时 s（仅 /measure 频道变化时）   [附件2 §4.3]
T_CLEAR_OK = 5.0       # 清除成功耗时 s（精确定位3+清除2）          [附件2 §4.4]
T_CLEAR_FL = 3.0       # 清除失败耗时 s（仅精确定位3）              [附件2 §4.4]
R_RECV_MIN = 1000.0    # 有效接收半径下限 m（各源不同、未知）        [附件2 §2.1]
R_RECV_MAX = 1500.0    # 有效接收半径上限 m                        [附件2 §2.1]
R_NEAR    = 5.0        # 近距离阈值：≤5m 返回 near、无示向度        [附件2 §2.4]
R_CLEAR   = 20.0       # 清除半径：≤20m 可清除成功                  [附件2 §2.4]
EPS_DEG   = 1.0        # 示向度误差上界 ±1°                        [题面/附录2]
MAX_COORD = 2000000.0  # 坐标分量绝对值上限                        [附件2 §1.1]
HTTP_TIMEOUT = 5.0     # 单次HTTP超时(s)，与官方示例一致

# ---- 策略内部阈值（非题面，可调，已注明为什么） ----
CLEAR_TRIG   = 15.0    # 逼近到估计距离≤15m 即尝试清除(<R_CLEAR=20 留5m余量)
RAY_INITIAL_STEP = 750.0   # 单示向度仍保证复测点距源不超过750m<最小接收半径
RAY_MIN_STEP = 3.0         # 小于near半径，便于越点后收敛
RAY_MAX_STEPS = 16         # 失败后仍转入原local_search安全兜底
RAY_MAX_STEP  = 1500.0     # 投影制步长上限(防单次越点过远)
RAY_MODE = "delta"         # Step 2 固定；Step 3 消融文件再开放四种步长律
APPROACH_RATIO = 0.45  # 每次逼近把与估计点的距离压到原来的0.45(几何收敛)
STANDOFF_MIN = 20.0    # 逼近停靠点到估计点的最小距离(保证仍在清除半径外再测一次)
CONF_GOOD    = 30.0    # 置信半径≤30m 认为"定位良好"可直接逼近
MAX_APPROACH = 12      # 单个源逼近伺服的最大迭代(防死循环)
LS_SPACING   = 6.0     # 局部搜索网格间距 m(<R_NEAR*sqrt2≈7.07,保证网格点落入5m)
LS_MAX_PTS   = 60      # 局部搜索最大网格点数(防爆炸)
REF_DIST     = 1800.0  # 打分特征归一化参考距离
REF_CONF     = 200.0   # 打分特征归一化参考置信半径

# ---- 发现阶段覆盖路点集（自检实测：最坏盲区956.05m<1000m，余量43.95m）----
# 依据（可复算，脚本 q3_cover_theory.py / q3_cover_opt.py / q3_waypoint_ablation.py）：
#   1) 等价性：源接收半径>=1000m，故路点集对1800m圆盘实现半径1000m的圆盘覆盖
#      <=> 任一源必落在某路点1000m内 <=> 必被发现（一个不漏）。
#   2) 下界：半径r圆盘最多覆盖R圆周弧长2*arcsin(r/R)，得 n>=pi/arcsin(5/9)=5.33 => n>=6。
#   3) 最优覆盖数：6点最好需1039m(环形)/1003.7m(非对称搜索)仍>1000m；
#      7点六边形(中心+6环@1558.85m)覆盖半径恰R/2=900m 可行且紧。
#   4) 但目标是"虚拟时间最短"而非"点数最少"：实测(30种子)9点4461s、8点4425s、
#      7点六边形5898s(+32%，外圈太远)；故取「中心+8环」——时间近最优且余量43.95m。
#   5) 下列数值由 cover_radius()【精确评估器】给出(圆周密采+邻域细化 ∪ Voronoi
#      顶点)：default 956.05m / hex7 900.00m / ring6 1039.23m / ring7 998.92m /
#      ring8 974.15m。★注意别用粗网格算覆盖半径——同样的路点集，361 层径向网格
#      会把 default 低报成 955.85m、把 hex7 低报成 898.85m（极值点落在网格之间），
#      乐观 0.2~1.2m 看似无害，但它会让"余量还有多少"这个结论失去意义。
COVER_WAYPOINTS = [(0.0, 0.0)] + [
    (1000.0 * math.cos(math.radians(45.0 * k)),
     1000.0 * math.sin(math.radians(45.0 * k))) for k in range(8)
]


# ------------------------------------------------------------------------------
# 覆盖资格：把命题 1/2 落成【可执行自检】，而不是只在注释里写"已验证"
# ------------------------------------------------------------------------------
def _ring(n, radius, phase_deg=0.0):
    """n 个点均布在半径 radius 的圆上。"""
    return [(radius * math.cos(math.radians(phase_deg + 360.0 * k / n)),
             radius * math.sin(math.radians(phase_deg + 360.0 * k / n)))
            for k in range(n)]


def ring_family(n, R=ARENA_R):
    """环形族解析构造：n 点均布在 r_n = R/(2cos(π/n)) 的圆上（无中心点）。

    半径来源：半径 r 的点最多覆盖 R 圆周弧 2·arcsin(r/R)，最优点
    d=√(R²−r²)（即环半径 ρ=r/cos(π/n)）处取等 ⟹ 覆盖半径恰为 r_n。
      n=6 -> 1039.23m（>1000 不合格，正好是"6 点不行"的解析证据）
      n=7 ->  998.92m（勉强合格）
    它只保证合法，不保证最快——实测环形 8 点比"中心+8环"仅快 0.8%，
    而环形 7 点反而慢 2.5%（点数少了但巡回变长）。"""
    return _ring(n, R / (2.0 * math.cos(math.pi / n)))


def hex7(R=ARENA_R):
    """中心 + 6 环 @ ρ=√3/2·R = 1558.85m：覆盖半径恰 R/2 = 900m。

    900m 是紧界（初等可证：r<0.5 时盖原点的圆盘到圆周最近距离 1−r>r，
    盖不到圆周任何点，剩 6 点最多覆盖 6×2arcsin(r)<360°），所以 7 点是
    "最少可行点数"。但外圈太远，实测总虚拟时 +32.2%（30 种子），只作对照。"""
    return [(0.0, 0.0)] + _ring(6, math.sqrt(3.0) / 2.0 * R)


WAYPOINT_SETS = {
    "default": COVER_WAYPOINTS,   # 中心+8环@1000m（采用：时间接近最优、余量 44m）
    "hex7":    hex7(),            # 7 点最少可行（理论对照，时间 +32.2%）
    "ring6":   ring_family(6),    # 覆盖不合格反例（1039.23m > 1000m）
    "ring7":   ring_family(7),    # 998.92m
    "ring8":   ring_family(8),    # 974.15m
    "ring9":   ring_family(9),
}


def cover_radius(waypoints, r_circle=ARENA_R, n_circle=20000, n_fine=2000):
    """路点集对 r_circle 圆盘的最坏盲区(=覆盖半径)，精确评估器。返回米。

    【为什么不能"撒个网格然后优化"】：稀疏网格会在 Voronoi 顶点处留缝，
    优化器会钻进测不到的缝隙里报出偏乐观的自相矛盾数——实测踩过：
    n=5 报 0.6467（比已知最优 1/φ=0.6180 还差）、n=6 报 0.5536（比经典值
    1/√3=0.5774 还好），两个都是假象。覆盖半径的全局最大点必落在下列两个
    候选集合中，取二者的最大值即为精确解（无需任何优化器）：
      (a) 圆盘边界上的点（弧覆盖最薄的地方）；
      (b) 路点三元组的外心，且满足 Voronoi 顶点条件（到三点等距、且不比其他
          路点更近）。
    （Voronoi 边上的最远点要么落在圆盘边界(由 (a) 处理)，要么落在 Voronoi
      顶点(由 (b) 处理)，故候选集完备。）

    (a) 用【粗扫 + 最优点邻域细化】两步：20000 点粗扫的角步长对应弧长 0.57m，
    直接把离散误差(≈0.3m)灌进结果就太糙了；在粗扫最优点两侧各一个步长内再取
    2000 点细化后，离散误差降到 ~1e-4 m 量级，与解析构造值(如 hex7 的 900m、
    ring6 的 1039.23m)能够逐位对齐。"""
    pts = [(float(x), float(y)) for (x, y) in waypoints]
    if not pts:
        return float("inf")

    def _circle_min(a):
        px, py = r_circle * math.cos(a), r_circle * math.sin(a)
        return min(math.hypot(px - q[0], py - q[1]) for q in pts)

    # (a) 圆周：粗扫定位最优点 -> 邻域细化
    best = 0.0
    best_a, best_g = 0.0, -1.0
    for k in range(n_circle):
        a = 2.0 * math.pi * k / n_circle
        g = _circle_min(a)
        if g > best_g:
            best_g, best_a = g, a
    half = 2.0 * math.pi / n_circle
    for k in range(n_fine):
        a = best_a - half + 2.0 * half * k / (n_fine - 1 if n_fine > 1 else 1)
        g = _circle_min(a)
        if g > best_g:
            best_g = g
    best = max(best, best_g)
    # (b) 三中心外心的 Voronoi 顶点
    m = len(pts)
    for i in range(m):
        for j in range(i + 1, m):
            for k in range(j + 1, m):
                a, b, c = pts[i], pts[j], pts[k]
                d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1])
                         + c[0] * (a[1] - b[1]))
                if abs(d) < 1e-12:
                    continue
                ux = ((a[0]**2 + a[1]**2) * (b[1] - c[1])
                      + (b[0]**2 + b[1]**2) * (c[1] - a[1])
                      + (c[0]**2 + c[1]**2) * (a[1] - b[1])) / d
                uy = ((a[0]**2 + a[1]**2) * (c[0] - b[0])
                      + (b[0]**2 + b[1]**2) * (a[0] - c[0])
                      + (c[0]**2 + c[1]**2) * (b[0] - a[0])) / d
                r0 = math.hypot(ux - a[0], uy - a[1])
                if math.hypot(ux, uy) > r_circle + 1e-9:
                    continue          # 外心落在圆盘外，不是本区域可行候选
                if any(math.hypot(ux - q[0], uy - q[1]) < r0 - 1e-9 for q in pts):
                    continue          # 有更近的路点 -> 不是 Voronoi 顶点
                if r0 > best:
                    best = r0
    return best


_COVER_CACHE = {}


def coverage_check(waypoints):
    """自检：返回 (最坏盲区 m, 余量 m, 是否合格)。

    合格 ⟺ 盲区 < R_RECV_MIN(=1000m) ⟺ 任一源(接收半径≥1000m)必落在某路点
    1000m 内 ⟺ 一个不漏。余量>0 才是"安全"，只有几十米余量属正常（点数与
    时间本来就是一对矛盾）。同一路点集只算一次并缓存。"""
    key = tuple((round(float(x), 6), round(float(y), 6)) for (x, y) in waypoints)
    if key not in _COVER_CACHE:
        gap = cover_radius(waypoints)
        _COVER_CACHE[key] = (gap, R_RECV_MIN - gap, gap < R_RECV_MIN)
    return _COVER_CACHE[key]


def report_coverage(planner):
    """打印一次覆盖自检；返回 (盲区, 余量, 合格)。供 main 与 run_once 共用。"""
    pts = planner.coverage_points() if hasattr(planner, "coverage_points") else []
    if not pts:
        return (0.0, 0.0, True)
    gap, margin, ok = coverage_check(pts)
    print("[覆盖自检] %s | %d个探测点 | 最坏盲区=%.2fm 余量=%.2fm -> %s"
          % (getattr(planner, "name", "?"), len(pts), gap, margin,
             "合格：任一源必被发现(一个不漏)" if ok
             else "!!不合格：不能保证一个不漏，请勿据本局反推源总数!!"))
    return (gap, margin, ok)


# ==============================================================================
# 1. 几何与定位（复用问题1的半平面求交/直径，新增多线最小二乘交会）
#    半平面约定: n·(P - S) >= 0  (即 n·P >= n·S)
# ==============================================================================
_EPS = 1e-9
_BIG = 1e9


def sector_to_halfplanes(S, theta, eps):
    """检测点S、示向度theta(度)、误差eps(度) -> 两个半平面(nx,ny,c) 满足 n·P>=c。
    扇区[theta-eps,theta+eps] = 两条边界射线内半平面之交；直接用cos/sin，天然跨360°。"""
    a_min = math.radians(theta - eps)
    a_max = math.radians(theta + eps)
    n_min = (-math.sin(a_min), math.cos(a_min))
    n_max = (math.sin(a_max), -math.cos(a_max))
    return [(n_min[0], n_min[1], n_min[0] * S[0] + n_min[1] * S[1]),
            (n_max[0], n_max[1], n_max[0] * S[0] + n_max[1] * S[1])]


def _ang(a, b):
    t = math.atan2(b, a)
    return t if t >= 0 else t + 2 * math.pi


def halfplane_intersection(hpl):
    """半平面交集(Sutherland-Hodgman式增量裁剪)。返回 {status, vertices}。"""
    hpl = sorted(hpl, key=lambda h: _ang(h[0], h[1]))
    uniq = []
    for h in hpl:
        if uniq and abs(_ang(h[0], h[1]) - _ang(uniq[-1][0], uniq[-1][1])) < 1e-12:
            if h[2] > uniq[-1][2]:
                uniq[-1] = h
        else:
            uniq.append(h)
    poly = [(-_BIG, -_BIG), (_BIG, -_BIG), (_BIG, _BIG), (-_BIG, _BIG)]
    for (nx, ny, c) in uniq:
        newp = []
        m = len(poly)
        for i in range(m):
            cur = poly[i]
            nxt = poly[(i + 1) % m]
            dcur = nx * cur[0] + ny * cur[1] - c
            dnxt = nx * nxt[0] + ny * nxt[1] - c
            if dcur >= -_EPS:
                newp.append(cur)
            if dcur * dnxt < -_EPS:
                t = dcur / (dcur - dnxt)
                newp.append((cur[0] + t * (nxt[0] - cur[0]),
                             cur[1] + t * (nxt[1] - cur[1])))
        poly = newp
        if not poly:
            return {'status': 'empty', 'vertices': []}
    if any(abs(p[0]) > _BIG - 1 or abs(p[1]) > _BIG - 1 for p in poly):
        return {'status': 'unbounded', 'vertices': poly}
    clean = [poly[0]]
    for p in poly[1:]:
        if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > _EPS:
            clean.append(p)
    if len(clean) > 1 and math.hypot(clean[0][0] - clean[-1][0],
                                     clean[0][1] - clean[-1][1]) < _EPS:
        clean = clean[:-1]
    if len(clean) <= 2:
        return {'status': 'degenerate', 'vertices': clean}
    # 【v6·已回退】原拟借鉴 Q4 的"顶点后验校验"（不满足全部半平面则判
    # unbounded），但在 Q3 全向场景下风险大于收益：mock 上 3000 组零误杀，
    # 可真实示向度数据若有细微数值特征差异，正常多边形会被误判 unbounded
    # → cr=1e8 → ray_chase 不飞向估计点 → 源清不掉 → 分母变小 → avg 暴涨。
    # Q3 的半平面交本来就良态(实验 q3_exp_region_feasibility: 99.8%有界、
    # MEC中位17.7m)，不需要这层 Q4 定向场景的审计防护。故回退为直接返回。
    return {'status': 'polygon', 'vertices': clean}


def _sane_pos(p, slack=50.0):
    """退化交会防护【v6 新增，借鉴 Q4 改动4，附实测依据】：

    两条示向度近平行时交会退化，估计点会被抛到场地外极远处
    （Q4 实测反例：seed1002 出现 (-3474,16349)，半径 16.7km；若把它当
    导航/清除目标，单次浪费 31.7km=6340s 虚拟时间，该局 vt/源 544→993）。

    源只可能在半径 ARENA_R(=1800m) 的盘内，故任何超出 ARENA_R+slack 的
    "源估计/导航点"必为数值退化产物，一律拒绝。_clamp_pos 只保证协议层
    ±2e6 的硬限，不防这种"合法但荒谬"的坐标——本函数补上这层语义防护。
    返回 True 表示该点可作为导航/清除目标。"""
    return (p is not None
            and math.hypot(p[0], p[1]) <= ARENA_R + slack)


def diameter_by_enumeration(pts):
    """顶点枚举求凸多边形直径(最远点对)。m很小，O(m^2)精确且易验证。"""
    n = len(pts)
    if n < 2:
        return 0.0
    best = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if d > best:
                best = d
    return best


def triangulate(bearings):
    """多线最小二乘交会定位。
    bearings: [(x,y,theta_deg),...] 每条示向度是一条过(x,y)、方向theta的直线。
    源G使得到各直线垂直距离平方和最小 -> 线性最小二乘(2x2正规方程)。
    为什么不用单条逼近：两条以上不同位置的示向度交会，能把 ±1° 角误差
    转化为有界的位置不确定域。返回 (gx,gy)；直线近乎平行(退化)返回 None。"""
    sxx = sxy = syy = sbx = sby = 0.0
    for (x, y, th) in bearings:
        a = math.radians(th)
        nx, ny = -math.sin(a), math.cos(a)     # 直线法向: n·(G-(x,y))=0
        c = nx * x + ny * y
        sxx += nx * nx
        sxy += nx * ny
        syy += ny * ny
        sbx += nx * c
        sby += ny * c
    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-9:                        # 所有示向度近乎平行 -> 无法交会
        return None
    gx = (sbx * syy - sby * sxy) / det
    gy = (sxx * sby - sxy * sbx) / det
    return (gx, gy)


def mec(pts):
    """最小覆盖圆(MEC)：返回(圆心,半径)。顶点数很少，枚举所有点对(直径圆)与
    三点组(外接圆)取最小者。为什么用MEC圆心作点估计：真源必落在置信多边形内，
    MEC圆心到任一多边形点的距离都≤MEC半径，是极小极大意义下最稳的点估计——
    远比无权重最小二乘(会被远距离示向度的大横向误差带偏)可靠。"""
    best = None
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            c = ((pts[i][0] + pts[j][0]) / 2.0, (pts[i][1] + pts[j][1]) / 2.0)
            r = dist(pts[i], pts[j]) / 2.0
            if all(dist(p, c) <= r + 1e-9 for p in pts):
                if best is None or r < best[1]:
                    best = (c, r)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                a, b, c = pts[i], pts[j], pts[k]
                d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
                if abs(d) < 1e-12:
                    continue
                ux = ((a[0]**2 + a[1]**2) * (b[1] - c[1]) + (b[0]**2 + b[1]**2) * (c[1] - a[1]) + (c[0]**2 + c[1]**2) * (a[1] - b[1])) / d
                uy = ((a[0]**2 + a[1]**2) * (c[0] - b[0]) + (b[0]**2 + b[1]**2) * (a[0] - c[0]) + (c[0]**2 + c[1]**2) * (b[0] - a[0])) / d
                cc = (ux, uy)
                r = dist(cc, a)
                if all(dist(p, cc) <= r + 1e-9 for p in pts):
                    if best is None or r < best[1]:
                        best = (cc, r)
    if best is None:                      # 理论上不会发生(点集非空)
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        rr = max(dist(p, (cx, cy)) for p in pts)
        best = ((cx, cy), rr)
    return best


def conf_radius(bearings, eps=EPS_DEG):
    """用问题1的精确定位区域求置信半径 = 区域直径/2。
    ≥2条示向度 -> 半平面交多边形 -> 直径D -> D/2 作为GDOP式置信半径。
    <2条或退化 -> 返回很大值表示"不可信"。为什么：直径/2 是问题1已验证的
    紧致不确定度度量，直接决定"能否放心走进20m清除半径"。"""
    if len(bearings) < 2:
        return 1e9
    hpl = []
    for (x, y, th) in bearings:
        hpl += sector_to_halfplanes((x, y), th, eps)
    res = halfplane_intersection(hpl)
    if res['status'] != 'polygon':
        return 1e9
    return diameter_by_enumeration(res['vertices']) / 2.0


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---- M8 判空证书【v6 新增，机制借鉴 Q4 World.local_empty_check / _try_empty_declares，
#      并针对 Q3"全向源"做了严格化简化】----
EMPTY_CERT_ENABLED = False  # 默认关闭：Q3 的 9 路点架构下结构性无效（见下注）
EMPTY_CERT_MARGIN = 30.0    # 安全余量 m：吸收 cover_radius 的数值误差(实测 <1m)，保证判空严格
EMPTY_CERT_MIN_NS = 3       # 至少 3 个无信号点才值得跑一次证书评估

# 【为什么默认关闭（2026-09-12 mock 实测 3 局的证据）】
# 判空证书要求"该频道全部 no_signal 点已把 1800m 圆盘盖到 970m 以内"，
# 而本架构的路点覆盖余量只有 44m（956.05/1000）——少扫任何一个路点都会
# 在它的专属区留下 >1000m 盲区，证书只能在【最后一个路点扫完】后触发，
# 而那时已经没有后续扫描可省。3 局实测触发 4-6 次、全部在收尾后、
# 节省测量数=0、总虚拟时与 v5 逐字节一致。这是结构性结论：
# Q4 能省是因为它有 24 个路点、同一频道被重复扫描的次数多得多。
# 保留实现并留开关，供"路点数增加/真实环境 chase 无信号点更多"的场景复查。


def update_empty_certificates(w, n_circle=2000, n_fine=200):
    """增量判空：对从未收到信号的频道，若其全部 no_signal 点已把场地"盖死"，
    则该频道不可能再有源，判空后后续路点不再为它花检测(5s)+切换(1s)。

    【Q3 全向源下的严格证明】(比 Q4 的定向版简单)：
      no_signal(m) ⇒ dist(m,源) > r，其中 r∈[1000,1500] 为该源接收半径
      ⇒ dist(m,源) > 1000 = R_RECV_MIN 对全部无信号点成立
      ⇒ 源必落在 F = 圆盘 \\ ∪B(m,1000) 内。
      若 F = ∅（即 no_signal 点集对场地圆盘的最坏盲区 cover_radius ≤ 1000），
      则该频道无源。再留 EMPTY_CERT_MARGIN 吸收数值误差，判空严格可靠。

    【与 Q4 的差异说明】Q4 面对定向源，no_signal 可能是"方向不通"而非
    "距离不通"，故需 conv(N_c∩B(p,R)) 的分离超平面论证；Q3 全向源下
    距离排除直接成立，直接复用本文件已有的精确覆盖评估器 cover_radius
    (圆周粗扫+细化 与 Voronoi 顶点两候选集完备，见其 docstring)。

    【成本控制】cover_radius 是 O(n_circle·|ns| + |ns|³)，故：
      * 只对"从未有信号"的频道评估（有示向度=非空，无需判）；
      * 每个 no_signal 点数只评估一次（点集只增，结果单调，缓存之）；
      * 评估用 n_circle=2000/n_fine=200 的轻量档（30m 余量远大于该档的
        亚米级离散误差）。
    """
    if not EMPTY_CERT_ENABLED:
        return w.certified_empty
    for ch in range(1, N_CH + 1):
        if ch in w.cleared or ch in w.certified_empty or ch in w.bearings:
            continue
        ns = w.no_signal.get(ch, [])
        if len(ns) < EMPTY_CERT_MIN_NS:
            continue
        if w._empty_cache.get(ch) == len(ns):
            continue                      # 点集没变，结果不会变（单调性）
        w._empty_cache[ch] = len(ns)
        gap = cover_radius(ns, r_circle=ARENA_R,
                           n_circle=n_circle, n_fine=n_fine)
        if gap <= R_RECV_MIN - EMPTY_CERT_MARGIN:
            w.certified_empty.add(ch)
            w.log("certify", "判空证书", {"ch": ch, "无信号点": len(ns),
                       "最坏盲区": round(gap, 1)},
                  "判空,后续扫描跳过",
                  "无信号点1000m外无立足处(余量%.0fm),该频道必无源(M8)" % EMPTY_CERT_MARGIN)


class _BudgetExceeded(Exception):
    """现实时间预算将尽：主动收尾,避免正式测试超20分钟窗口被判负。"""
    pass


def bearing_deg(frm, to):
    """从frm指向to的方位角(度,[0,360))，x东y北逆时针为正。"""
    return math.degrees(math.atan2(to[1] - frm[1], to[0] - frm[0])) % 360.0


# ==============================================================================
# 2. 后端抽象：真实HTTP(Client) 与 离线物理仿真(MockSimulator)
#    两者对外暴露完全相同的 enter/measure/clear/exit，返回与真实API一致的dict，
#    使得 World/Planner/Executor 完全感知不到自己在连真机还是仿真。
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


# ---- pre-flight 自检：连真机前的零成本预检，避免占位符/端口没开白白浪费
#      一次正式测试机会。注意：绝不能发 /enter 之类会触发计时的请求，只做
#      (1) ROBOT_ID 格式与占位符检查 (2) TCP 端口连通性探测(不发HTTP body)。----
def preflight_check(base_url=BASE_URL, robot_id=ROBOT_ID, timeout_s=2.0):
    """返回 (ok: bool, messages: list[str])。ok=False 时必须修完再跑 real。"""
    msgs = []
    # (1) ROBOT_ID 检查
    if not robot_id:
        msgs.append("ROBOT_ID 为空：请通过环境变量 ROBOT_ID 或 --robot-id 参数传入登录模拟器的队号。")
    elif robot_id == ROBOT_ID_PLACEHOLDER:
        msgs.append("ROBOT_ID 仍是占位符 '%s'：请通过环境变量 ROBOT_ID 或 --robot-id 参数传入登录模拟器的队号。"
                    % ROBOT_ID_PLACEHOLDER)
    elif not robot_id.isdigit():
        msgs.append("ROBOT_ID='%s' 含非数字字符：国赛队号一般为纯数字，请核对。"
                    % robot_id)
    elif len(robot_id) < 10:
        msgs.append("ROBOT_ID='%s' 长度 %d 偏短：国赛队号一般 12 位，请核对。"
                    % (robot_id, len(robot_id)))
    # (2) BASE_URL 格式
    try:
        parsed = urlparse(base_url)
        host = parsed.hostname
        port = parsed.port
        scheme = parsed.scheme
        if scheme != "http":
            msgs.append("BASE_URL 协议为 '%s'，模拟器协议为 http，请核对。" % scheme)
        if not host or not port:
            msgs.append("BASE_URL='%s' 解析不出 host/port，请核对格式。" % base_url)
    except Exception as e:
        msgs.append("BASE_URL='%s' 解析失败：%s" % (base_url, e))
        host = port = None
    # (3) TCP 端口连通性（不发 HTTP body，不触发任何模拟器动作/计时）
    if host and port:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout_s)
        try:
            s.connect((host, port))
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            msgs.append("模拟器端口 %s:%d 不可达(%s)：请确认模拟器已启动、"
                        "该局已点击'开始'、且 25 分钟窗口未结束。" % (host, port, type(e).__name__))
        finally:
            s.close()
    return (len(msgs) == 0, msgs)


class Client(Backend):
    """真实HTTP客户端（严格按附件2协议）。
    要点：
      * 逐字节 robot_id；arena_id="default"；每新动作新 request_id；
      * 串行(无并发)；网络异常时【复用原request_id与原payload】幂等重试；
      * 同时检查 HTTP状态 + JSON.accepted；accepted=false 的 vt=0 不是真实时刻。
    """

    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID, verbose=True):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.verbose = verbose
        self._ctr = {"enter": 0, "measure": 0, "clear": 0, "exit": 0}
        self.last_vt = 0.0          # 最近一次 accepted=true 的虚拟时刻
        self.remaining_real_s = None
        self.max_virtual_s = None

    def _base(self, kind):
        self._ctr[kind] += 1
        return {"arena_id": "default", "robot_id": self.robot_id,
                "request_id": "%s-%d" % (kind, self._ctr[kind])}

    def _post(self, path, payload):
        """发送并解析；网络层异常时幂等重试（复用同一request_id与payload）。"""
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
                # 能形成HTTP响应的错误：读JSON体；结构性错误(400/409/415等)不重试
                try:
                    data = json.loads(e.read().decode("utf-8"))
                except Exception:
                    data = {}
                code = e.code
                if code in (400, 404, 405, 409, 413, 415, 429, 500):
                    raise RuntimeError("HTTP %d on %s: %s" % (code, path, data))
                return code, data
            except (URLError, ConnectionError, OSError) as e:
                # 连不上(倒计时未结束/接口未开/测试已结束/断线)：幂等重试同一动作
                if attempt <= 5:
                    time.sleep(0.3 * attempt)
                    continue
                raise RuntimeError("连接失败(重试5次仍失败) %s: %s" % (path, e))
            return code, data

    def _check(self, path, code, data):
        """同时检查HTTP状态与accepted；更新last_vt(仅accepted=true)。"""
        if code != 200:
            raise RuntimeError("HTTP %d on %s: %s" % (code, path, data))
        if data.get("accepted") is not True:
            # accepted=false：动作未生效，vt=0不是真实时刻；不重试，直接报错供排查
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
        # 测试可能已因超时结束而接口关闭(连接直接失败)：此时 /exit 已无意义，
        # 必须容忍——否则会在收尾阶段抛异常，把已经跑出来的指标和日志全丢掉。
        try:
            code, data = self._post("/exit", self._base("exit"))
        except Exception:
            return {"accepted": False}
        try:
            return self._check("/exit", code, data)
        except RuntimeError:
            return {"accepted": False}


class MockSimulator(Backend):
    """离线物理仿真器（与真实模拟器同一接口），用于无限制演练、调参与指标生成。
    严格复刻附件2物理与时间模型：
      * 源：10-16个、频道1-20不重复、位置均匀分布于1800m圆盘、
            接收半径1000-1500m、问题3全向(directional=None)；
      * 示向度误差：±1°均匀；且【同一(取整)位置误差固定】——重复检测不改误差，
            只有换位置误差才独立(用带盐哈希实现空间误差场)；
      * near(≤5m,在覆盖角内)无示向度；clear(≤20m,与朝向无关)；
      * 虚拟时间 = 移动(距离/5)+切换(1s)+检测(5s)+清除(5/3s)。
    """

    def __init__(self, seed=None, n_sources=None, directional_ratio=0.0, verbose=False):
        import random
        self.rng = random.Random(seed)
        self.verbose = verbose
        # --- 生成案例 ---
        n = n_sources if n_sources else self.rng.randint(N_SRC_LO, N_SRC_HI)
        chans = self.rng.sample(range(1, N_CH + 1), n)
        self.sources = []      # dict: pos, channel, radius, cleared, directional
        for i in range(n):
            r = ARENA_R * math.sqrt(self.rng.random())
            a = self.rng.uniform(0, 2 * math.pi)
            pos = (r * math.cos(a), r * math.sin(a))
            direc = None
            if self.rng.random() < directional_ratio:   # 问题4扩展：定向源
                direc = self.rng.uniform(0, 360.0)
            self.sources.append({
                "idx": i, "pos": pos, "channel": chans[i],
                "radius": self.rng.uniform(R_RECV_MIN, R_RECV_MAX),
                "cleared": False, "directional": direc})
        self.by_channel = {s["channel"]: s for s in self.sources}
        # --- 状态 ---
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.vt = 0.0
        self._salt = "%x|" % (self.rng.getrandbits(64))   # 每局不同的误差场盐
        self._entered = False

    # -- 误差场：同一位置误差固定，不同位置独立均匀 ±1° --
    def _berr(self, src_idx, x, y):
        key = "%s%d|%.1f|%.1f" % (self._salt, src_idx, x, y)
        h = hashlib.sha256(key.encode()).digest()
        u = int.from_bytes(h[:8], "big") / float(1 << 64)   # [0,1)
        return (2.0 * u - 1.0) * EPS_DEG

    def _resp(self, **kw):
        d = {"accepted": True, "real_timestamp_ms": int(time.time() * 1000),
             "virtual_time_s": round(self.vt, 6)}
        d.update(kw)
        return d

    def total_sources(self):
        return len(self.sources)

    def enter(self):
        self._entered = True
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.vt = 0.0
        return {"accepted": True, "real_timestamp_ms": int(time.time() * 1000),
                "virtual_time_s": 0, "max_virtual_duration_s": 360000,
                "max_real_duration_s": 1200, "remaining_real_duration_s": 1200}

    def _in_cover(self, s, frm):
        """检测点frm是否在源s的信号覆盖内（全向恒True；定向需在±90°扇形内）。"""
        if s["directional"] is None:
            return True
        ang = bearing_deg(s["pos"], frm)   # 源指向检测点的方位
        diff = abs((ang - s["directional"] + 180) % 360 - 180)
        return diff <= 90.0

    def measure(self, x, y, channel):
        x = float(x); y = float(y); channel = int(channel)
        # 移动耗时 + 切换频道耗时 + 检测耗时
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
# 3. World：信念状态维护（机器狗"知道"的一切，决策的唯一信息来源）
# ==============================================================================
class World(object):
    def __init__(self):
        self.pos = (0.0, 0.0)          # 当前位置(=最近一次被接受动作提交的位置)
        self.channel = 1               # 测向机当前频道
        self.vt = 0.0                  # 虚拟时刻(取自 accepted=true 响应)
        self.bearings = {}             # ch -> [(x,y,svd_deg),...]
        self.no_signal = {}            # ch -> [(x,y),...] 无信号点【v6 新增】
        self.certified_empty = set()   # 已被判空的频道【v6 新增】
        self._empty_cache = {}         # ch -> 判空检查时的 no_signal 点数【v6】
        self.status = {}               # ch -> 'detected'/'localized'/'cleared'
        self.cleared = set()           # 已清除频道
        self.stuck = set()             # 多次尝试仍清不掉的频道(病理,正常不该出现)
        self.attempt = {}              # ch -> 已尝试清除次数(防对单源死磕)
        self.waypoints_done = set()    # 已访问的发现路点索引
        self.scan_complete = False     # 覆盖扫描是否跑完（真实模式据此推定源总数）
        self.path = []                 # 被接受动作的位置序列(算路程/重叠)
        self.path_len = 0.0
        self.n_measure = 0
        self.n_clear = 0
        self.n_action = 0
        # ---- Step 0：单源清除耗时记账（不参与任何策略决策） ----
        self.ch_detect_vt = {}         # ch -> 首次命中动作开始时刻
        self.ch_first_distance = {}    # ch -> Mock 真值首测距离；真实后端为空
        self.ch_cost = {}              # ch -> 首次命中+专属补测/逼近/清除的Δvt
        self.ch_measure_count = {}     # ch -> 上述口径内的测量次数
        self.per_source_clear_time = {}  # ch -> 清除成功时冻结的ch_cost
        self.logs = []                 # 可解释动作日志
        self.budget_real_s = None      # /enter返回的本局现实预算
        self.max_virtual_s = 360000.0
        self.enter_wall = None         # /enter成功时的墙钟时刻(算现实已用时间)
        self._seq = 0
        self._est_cache = {}           # (ch,示向度数) -> (估计点,置信半径,条数)

    # ---- 状态更新 ----
    def on_accepted(self, resp, pos):
        self.vt = float(resp.get("virtual_time_s", self.vt))
        if pos is not None:
            self.path_len += dist(self.pos, pos)
            self.pos = pos
            self.path.append(pos)

    def add_bearing(self, ch, x, y, svd):
        self.bearings.setdefault(ch, []).append((x, y, svd))
        self._est_cache.pop((ch, len(self.bearings[ch]) - 1), None)  # 旧缓存失效
        if self.status.get(ch) != 'cleared':
            self.status[ch] = 'detected'

    def mark_cleared(self, ch):
        self.cleared.add(ch)
        self.status[ch] = 'cleared'

    def add_source_cost(self, ch, delta_vt, is_measure=False):
        """按题定口径累加已接受动作的虚拟时钟差，不重算物理耗时。"""
        self.ch_cost[ch] = self.ch_cost.get(ch, 0.0) + max(0.0, float(delta_vt))
        if is_measure:
            self.ch_measure_count[ch] = self.ch_measure_count.get(ch, 0) + 1

    def freeze_source_cost(self, ch):
        self.per_source_clear_time[ch] = self.ch_cost.get(ch, 0.0)

    def source_records(self):
        rows = []
        for ch in sorted(self.per_source_clear_time):
            d0 = self.ch_first_distance.get(ch)
            rows.append({
                "channel": ch,
                "first_distance_m": d0,
                "clear_cost_s": self.per_source_clear_time[ch],
                "source_measure_count": self.ch_measure_count.get(ch, 0),
                "lower_bound_s": (d0 / SPEED + T_SWITCH + T_MEASURE + T_CLEAR_OK)
                                 if d0 is not None else None,
            })
        return rows

    # ---- 查询 ----
    def estimate(self, ch):
        """返回 (估计点, 置信半径, 示向度条数)。
        估计点 = 置信多边形的最小覆盖圆(MEC)圆心【极小极大最优】；
        置信半径 = MEC半径(误差上界：真源到估计点距离 ≤ 此值)。

        【为什么不是形心】形心确实必落在多边形内，但它【没有任何误差上界】——
        多边形偏长偏斜时形心可能离真源很远，而"能否走进 20m 清除半径"完全
        由误差上界决定。MEC 圆心把最大误差取到最小，误差上界恰好就是 MEC
        半径，可直接拿去和清除余量比较。路径规划的节点因此一律取 MEC 圆心。
        结果按(ch,示向度数)缓存，避免每次决策都重建多边形。"""
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
            hpl += sector_to_halfplanes((x, y), th, EPS_DEG)
        res = halfplane_intersection(hpl)
        if res['status'] == 'polygon':
            c, r = mec(res['vertices'])
            out = (c, r, len(bs))
        elif res['status'] == 'degenerate' and len(res['vertices']) == 2:
            # 退化成一条线段：真源必落在这条线段上（两条近乎共线的边界夹出的
            # 极窄条带），取【中点】做点估计，误差上界恰为半长。这仍然是有界
            # 的极小极大估计，比退回无权重最小二乘稳健得多——最小二乘会被
            # 远距离示向度的横向误差(1400m×tan1°≈24m)整体带偏，且给不出上界。
            a, b = res['vertices']
            out = (((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0),
                   dist(a, b) / 2.0, len(bs))
        else:
            # 空集/无界：几何根本不成立（示向度自相矛盾或近乎平行），
            # 只能给一个"不可信"的点估计 + 大置信半径，迫使上层去补测。
            out = (triangulate(bs), 1e9, len(bs))
        self._est_cache[key] = out
        return out

    def pending_channels(self):
        """已发现(≥1示向度)、未清除、且未被判卡死的频道(待处理)。"""
        return [ch for ch, bs in self.bearings.items()
                if ch not in self.cleared and ch not in self.stuck and len(bs) >= 1]

    def unknown_channels(self):
        """从未检测到信号、且未清除、未判空的频道（还需去探测）。"""
        return [c for c in range(1, N_CH + 1)
                if c not in self.bearings and c not in self.cleared
                and c not in self.certified_empty]

    def localized_channels(self, conf_thr=CONF_GOOD):
        out = []
        for ch in self.pending_channels():
            est, cr, nb = self.estimate(ch)
            if est is not None and cr <= conf_thr:
                out.append(ch)
        return out

    # ---- 可解释日志 ----
    def log(self, atype, trigger, inputs, decision, reason):
        self._seq += 1
        # pos：记录"本条日志产生时"机器狗所在位置。诊断必需——clear 成功点
        # 就是干扰源位置(误差≤20m)，据此可反推真实案例的源空间分布，
        # 用于校验 mock 的"均匀圆盘"假设是否与真实模拟器一致。
        rec = {"seq": self._seq, "vt": round(self.vt, 2), "action": atype,
               "trigger": trigger, "inputs": inputs,
               "decision": decision, "reason": reason,
               "pos": _r2(self.pos)}
        self.logs.append(rec)
        return rec


# ==============================================================================
# 4. Executor：低层执行（规则化、可解释、可验证；不交给任何黑盒）
#    每个方法都做一件事，并把"为什么这么做"写进 world.log。
# ==============================================================================
class Executor(object):
    def __init__(self, backend, world, verbose=True):
        self.bk = backend
        self.w = world
        self.verbose = verbose

    # ---- 现实时间预算检查（仅真实模式有意义；虚拟动作近瞬时,预算极少触发）----
    def _budget_ok(self):
        if self.w.budget_real_s is None or self.w.enter_wall is None:
            return True
        # 预留20s收尾余量；绝不假设固定1200s,用/enter返回的实际剩余时间
        return (time.time() - self.w.enter_wall) < (self.w.budget_real_s - 20.0)

    # ---- 基础动作：测量并统一处理三种结果（含 near 即清的便宜收益）----
    def measure_and_handle(self, pos, ch, note="", account_to_source=False):
        """移动到pos并对频道ch检测；根据结果更新World。
        返回 'direction' / 'near_cleared' / 'no_signal' / 'near_clear_fail'。
        为什么集中处理：任何一次测量若返回near(≤5m)，说明源就在清除半径内，
        立刻清除是最便宜的一手，必须就地抓住，不能等高层再排程。"""
        if not self._budget_ok():
            raise _BudgetExceeded()
        self._clamp_pos(pos)
        vt0 = self.w.vt
        known_before = ch in self.w.ch_detect_vt
        resp = self.bk.measure(pos[0], pos[1], ch)
        self.w.n_action += 1
        self.w.n_measure += 1
        self.w.on_accepted(resp, pos)
        measure_delta = self.w.vt - vt0
        self.w.channel = ch
        mr = resp.get("measure_result")
        if mr == "direction":
            svd = float(resp["svd_deg"])
            self.w.add_bearing(ch, pos[0], pos[1], svd)
            if not known_before:
                self.w.ch_detect_vt[ch] = vt0
                source = getattr(self.bk, "by_channel", {}).get(ch)
                if source is not None:
                    self.w.ch_first_distance[ch] = dist(pos, source["pos"])
                self.w.add_source_cost(ch, measure_delta, is_measure=True)
            elif account_to_source:
                self.w.add_source_cost(ch, measure_delta, is_measure=True)
            self.w.log("measure", note or "探测/逼近", {"ch": ch, "pos": _r2(pos)},
                       "示向度=%.2f°" % svd, "记录示向度用于交会定位")
            return "direction"
        elif mr == "near":
            if not known_before:
                self.w.ch_detect_vt[ch] = vt0
                source = getattr(self.bk, "by_channel", {}).get(ch)
                if source is not None:
                    self.w.ch_first_distance[ch] = dist(pos, source["pos"])
                self.w.add_source_cost(ch, measure_delta, is_measure=True)
            elif account_to_source:
                self.w.add_source_cost(ch, measure_delta, is_measure=True)
            # ≤5m：源就在脚边，直接清除(5s换1个源，最划算)
            clear_vt0 = self.w.vt
            cresp = self.bk.clear(pos[0], pos[1], ch)
            self.w.n_action += 1
            self.w.n_clear += 1
            self.w.on_accepted(cresp, pos)
            self.w.add_source_cost(ch, self.w.vt - clear_vt0)
            if cresp.get("clear_result") == "success":
                self.w.mark_cleared(ch)
                self.w.freeze_source_cost(ch)
                self.w.log("clear", "测量返回near(≤5m)", {"ch": ch, "pos": _r2(pos)},
                           "成功", "near即源在5m内,就地清除代价最小")
                return "near_cleared"
            self.w.log("clear", "near后清除", {"ch": ch}, "失败",
                       "罕见:near后清除未中,转入局部搜索")
            return "near_clear_fail"
        else:  # no_signal
            # 【v6 新增】记录无信号点。全向源下这是严格信息：
            # no_signal ⇒ dist(该点,源) > r ≥ R_RECV_MIN(1000m)，
            # 是判空证书(M8)的原始证据，对"发现扫描"和"追击"两种语境都成立。
            self.w.no_signal.setdefault(ch, []).append((pos[0], pos[1]))
            if account_to_source and known_before:
                self.w.add_source_cost(ch, measure_delta, is_measure=True)
            self.w.log("measure", note or "探测", {"ch": ch, "pos": _r2(pos)},
                       "无信号", "该位置收不到此频道(超距/无此源/定向盲区)")
            return "no_signal"

    # ---- 路点扫描：在pos对一组频道逐个检测（发现+积累示向度）----
    def scan_waypoint(self, pos, channels, note="发现扫描"):
        """在pos对channels逐个 /measure。返回新检测到(direction/near)的频道数。
        为什么按频道扫：机器狗一次只能测一个频道，必须逐个切换探测；
        已清除频道跳过(省时)，near就地清除。
        【v6 新增】已判空频道同样跳过：判空证书(M8)已严格证明该频道无源，
        再测纯属烧时间；扫完顺便跑一次增量判空，让后续路点尽早受益。"""
        skipped_empty = 0
        new_found = 0
        n_scan = 0
        for ch in channels:
            if ch in self.w.cleared:
                continue
            if ch in self.w.certified_empty:
                skipped_empty += 1
                continue
            n_scan += 1
            before = len(self.w.bearings.get(ch, []))
            r = self.measure_and_handle(pos, ch, note=note)
            if r in ("direction", "near_cleared") and before == 0:
                new_found += 1
        update_empty_certificates(self.w)
        self.w.log("scan", note, {"pos": _r2(pos), "探测频道数": n_scan,
                   "判空跳过": skipped_empty},
                   "新发现%d个" % new_found,
                   "覆盖路点扫描以保证1000m全覆盖、一个不漏；"
                   "已判空频道按M8证书跳过(v6)")
        return new_found

    # ---- 补一条角度多样的示向度（问题2策略：交会角γ≈90°定位效果最好）----
    def _legacy_get_second_bearing_disabled(self, ch):
        """当交会几何太差(只1条示向度、或多条近乎平行致置信半径大)时补测一条，
        使新旧示向度以接近90°相交(问题2结论：γ=90°时定位区域面积/直径最小)。

        关键【必须在源接收半径内补测，否则永远no_signal空转】：
        单条示向度时，源必在检测点S1沿示向度方向 0~1500m 范围内(半径≥1000m)，
        因此"射线中点M(750m处)及其垂直偏移±300m"到源的距离 ≤√(750²+300²)≈808m
        <1000m≤半径 => 必有信号。故单示向度时锚定M的垂直偏移点补测，一定成功；
        已有估计(≥2条)时则锚定估计点方向、按置信半径尺度偏移补测以改善几何。"""
        bs = self.w.bearings.get(ch, [])
        if not bs:
            return False
        est, cr, nb = self.w.estimate(ch)
        cands = []
        if nb < 2 or est is None or not _sane_pos(est):
            # —— 单示向度：锚定"射线中点M"的垂直偏移(保证在源接收半径内)——
            # 【v6】锚点选择改为"离当前船位最近的一条示向度检测点"(借鉴Q4)：
            # 任取一条线做垂直偏移都能得到 ~90° 交会的新线，几何完全等价，
            # 但按路程选锚点平均少飞一段。
            (x1, y1, th) = min(bs, key=lambda b: dist(self.w.pos, (b[0], b[1])))
            a = math.radians(th)
            M = (x1 + 750.0 * math.cos(a), y1 + 750.0 * math.sin(a))
            perp = a + math.pi / 2.0
            # 【v6】候选 6 个 -> 2 个(借鉴Q4)：±对称候选几何效果等价，
            # 多余候选只是把"域直径量级"的长途重复飞。两个候选按
            # "到当前船位的路程"升序尝试——命中率不变，平均少飞。
            for off in (300.0, -300.0):
                cands.append((M[0] + off * math.cos(perp), M[1] + off * math.sin(perp)))
        else:
            # —— 已有估计但几何差：锚定估计点方向,按置信半径尺度垂直偏移 ——
            ref_dir = math.radians(bearing_deg(self.w.pos, est))
            base0 = max(40.0, min(600.0, 1.2 * (cr if cr < 1e8 else 150.0)))
            perp = ref_dir + math.pi / 2.0
            for off in (base0, -base0):
                cands.append((est[0] + off * math.cos(perp), est[1] + off * math.sin(perp)))
        # 【v6】按路程升序尝试(借鉴Q4)：对称候选命中概率相同，先飞近的那个
        cands.sort(key=lambda c: dist(self.w.pos, c))
        for cand in cands:
            r = self.measure_and_handle(cand, ch, note="补示向度",
                                        account_to_source=True)
            if r in ("direction", "near_cleared"):
                self.w.log("refine", "改善交会几何", {"ch": ch, "到": _r2(cand)},
                           "补测成功", "在源接收半径内垂直偏移补一条,使交会角近90°(问题2)")
                return True
        return False

    # ---- 逼近并清除（伺服收敛：越走越近、示向度越测越准）----
    def _legacy_approach_and_clear_disabled(self, ch, depth=0):
        """对频道ch的源执行"逼近-复测-修正"伺服，直至进入清除半径清除之。
        收敛原理：示向度横向误差≈距离×tan(1°)，每把距离压到0.45倍，横向不确定
        近似等比缩小；逼近到15m时横向误差<0.3m，稳进20m清除半径。
        清除判据(关键)：必须同时满足"够近(d≤15)"且"够准(cr≤20-d)"——只看距离
        不看置信半径，会在估计本身偏几十米时清不中空转。失败兜底为局部网格搜索。
        depth：与local_search互相调用时的递归深度上限，防无限递归。"""
        if depth > 3:
            self.w.log("approach", "递归深度超限", {"ch": ch}, "放弃本次",
                       "防逼近<->局部搜索无限互调")
            return False
        self.w.attempt[ch] = self.w.attempt.get(ch, 0) + 1
        if self.w.attempt[ch] > 8:
            # 同一源反复清不掉(病理,全向源正常不该发生)：判卡死,不再死磕
            self.w.stuck.add(ch)
            self.w.log("approach", "尝试次数超限", {"ch": ch}, "判卡死跳过",
                       "同一源尝试8次未清除,标记stuck避免拖死全局")
            return False
        # 1) 先把交会几何改善到置信半径可接受（最多补3条角度多样的示向度）
        for _ in range(3):
            est, cr, nb = self.w.estimate(ch)
            if est is not None and nb >= 2 and cr <= CONF_GOOD:
                break
            if not self.get_second_bearing(ch):
                break
        est, cr, nb = self.w.estimate(ch)
        if est is None or not _sane_pos(est):
            self.w.log("approach", "尝试逼近", {"ch": ch}, "放弃",
                       "无法交会出位置或估计点在场外(退化,v6防护),暂跳过")
            return False

        # 2) 伺服逼近
        prev_m_pos = (1e18, 1e18)      # 上一次逼近测量位置(用于原地踏步检测)
        for it in range(MAX_APPROACH):
            est, cr, nb = self.w.estimate(ch)
            if est is None or not _sane_pos(est):
                # 【v6 新增】伺服中途估计退化(抛出场外)：立即止损,
                # 不再朝场外点走伺服步(每步最长 d*0.55,场外 d 巨大)。
                self.w.log("approach", "伺服中估计退化", {"ch": ch},
                           "中止伺服", "估计点场外必为退化产物,留待补测")
                return False
            d = dist(self.w.pos, est)
            if d <= CLEAR_TRIG:
                if cr <= max(6.0, R_CLEAR - d):
                    # 够近且够准：源必落在20m清除半径内 -> 清除
                    if not self._budget_ok():
                        raise _BudgetExceeded()
                    clear_vt0 = self.w.vt
                    cresp = self.bk.clear(self.w.pos[0], self.w.pos[1], ch)
                    self.w.n_action += 1
                    self.w.n_clear += 1
                    self.w.on_accepted(cresp, self.w.pos)
                    self.w.add_source_cost(ch, self.w.vt - clear_vt0)
                    if cresp.get("clear_result") == "success":
                        self.w.mark_cleared(ch)
                        self.w.freeze_source_cost(ch)
                        self.w.log("clear", "逼近至%.1fm,置信%.1fm" % (d, cr),
                                   {"ch": ch}, "成功", "进入清除半径,执行清除")
                        return True
                    # 清除未中：估计残差>余量 -> 局部网格搜索兜底
                    self.w.log("clear", "逼近清除未中", {"ch": ch, "d": round(d, 1)},
                               "转局部搜索", "估计残差超余量,用网格把源逼进5m")
                    return self.local_search(ch, est, min(max(cr, R_CLEAR) + 15.0, 45.0),
                                             depth + 1)
                else:
                    # 够近但不够准：近距离换个角度补一条示向度,迅速压小置信半径
                    if not self.get_second_bearing(ch):
                        break     # 补不出信号,跳出伺服走兜底
                    continue
            # 计算下一逼近点：朝估计点走,停在距其 standoff 处再复测。
            # ★standoff 必须严格小于当前距离 d(取 d*0.45),保证每步净推进;
            #   否则当 d≈STANDOFF_MIN 时停靠点=当前位置,同一位置复测示向度不变
            #   (同地点误差固定),白白空测一次还原地踏步。
            standoff = max(3.0, d * APPROACH_RATIO)
            if standoff >= d:                      # 兜底:确保比当前更近
                standoff = d * 0.5
            ux = (self.w.pos[0] - est[0]) / d
            uy = (self.w.pos[1] - est[1]) / d
            nxt = (est[0] + ux * standoff, est[1] + uy * standoff)
            r = self.measure_and_handle(nxt, ch, note="逼近复测#%d" % it,
                                        account_to_source=True)
            if r == "near_cleared":
                return True
            elif r == "direction":
                # 无进展检测: 若本次测量位置与上一次几乎相同(示向度必然也不变),
                # 说明伺服原地踏步, 不必再空转, 直接转局部搜索把源逼进5m。
                if dist(nxt, prev_m_pos) < 2.0:
                    self.w.log("approach", "伺服原地踏步", {"ch": ch},
                               "转局部搜索", "同位置复测无新信息,改用网格兜底")
                    return self.local_search(ch, est, min(max(cr, R_CLEAR) + 15.0, 45.0),
                                             depth + 1)
                prev_m_pos = nxt
                continue          # 新示向度已并入，重新交会，继续逼近
            elif r == "no_signal":
                # 全向源在逼近中丢信号 = 估计严重错误(我们走反了) -> 局部搜索
                self.w.log("approach", "逼近中丢信号", {"ch": ch, "d": round(d, 1)},
                           "转局部搜索", "全向源不应在接近时丢信号,估计有误")
                return self.local_search(ch, est, min(max(cr, d) + 20.0, 45.0), depth + 1)
        # 伺服超上限仍未清除 -> 局部搜索兜底
        est, cr, nb = self.w.estimate(ch)
        if est is not None:
            return self.local_search(ch, est, min(max(cr, R_CLEAR) + 20.0, 45.0), depth + 1)
        return False

    @staticmethod
    def _turn_gap(a, b):
        return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

    def _certified_clear_here(self, ch):
        """仅在当前位置到整个MEC定位域均不超过20m时执行清除。"""
        est, cr, _nb = self.w.estimate(ch)
        if est is None or dist(self.w.pos, est) + cr > R_CLEAR + 1e-9:
            return False
        if not self._budget_ok():
            raise _BudgetExceeded()
        vt0 = self.w.vt
        cresp = self.bk.clear(self.w.pos[0], self.w.pos[1], ch)
        self.w.n_action += 1
        self.w.n_clear += 1
        self.w.on_accepted(cresp, self.w.pos)
        self.w.add_source_cost(ch, self.w.vt - vt0)
        if cresp.get("clear_result") == "success":
            self.w.mark_cleared(ch)
            self.w.freeze_source_cost(ch)
            self.w.log("clear", "射线追踪MEC证书", {"ch": ch, "R_min": round(cr, 3)},
                       "成功", "当前位置到定位域最远点不超过20m")
            return True
        self.w.log("clear", "射线追踪证书异常", {"ch": ch},
                   "转安全兜底", "保留local_search与递归深度保护")
        return False

    def _ray_step_length(self, previous_step, turn_deg, origin, heading_deg, est, cr):
        """投影制步长【v5 改动，已实测，v7 回退到 v6 原版】。

        【v7 教训】曾试 cr 自适应(cr>30 缩短步长)，但 mock 上 cr>30 占 32.4%
        且 est 偏差小(±1°误差)，缩短步长反而变慢(avg 309→338)。已回退。
        防越点不靠缩短步长(在 mock 有害)，靠 no_signal 后的恢复策略(见 ray_chase)。"""
        if est is not None and cr < 1e8:
            a = math.radians(heading_deg)
            proj = ((est[0] - origin[0]) * math.cos(a)
                    + (est[1] - origin[1]) * math.sin(a))
            if proj > RAY_MIN_STEP:
                return max(RAY_MIN_STEP, min(RAY_MAX_STEP, proj))
        return max(RAY_MIN_STEP, min(RAY_MAX_STEP, RAY_INITIAL_STEP))

    def ray_chase_and_clear(self, ch, depth=0):
        """沿最新示向度净推进；near即清，MEC证书满足时原地清，其余走安全兜底。"""
        if depth > 3:
            return False
        bs = self.w.bearings.get(ch, [])
        if not bs:
            return False
        self.w.attempt[ch] = self.w.attempt.get(ch, 0) + 1
        if self.w.attempt[ch] > 8:
            self.w.stuck.add(ch)
            self.w.log("ray_chase", "尝试次数超限", {"ch": ch}, "判卡死跳过",
                       "保留原stuck安全阈值，防单源阻塞全局")
            return False

        step = RAY_INITIAL_STEP
        previous_heading = None
        last_est = None
        last_cr = 1e9
        for iteration in range(RAY_MAX_STEPS):
            if self._certified_clear_here(ch):
                return True
            x0, y0, heading = self.w.bearings[ch][-1]
            origin = (x0, y0)
            est, cr, _nb = self.w.estimate(ch)
            last_est, last_cr = est, cr
            turn = (self._turn_gap(previous_heading, heading)
                    if previous_heading is not None else None)
            step = self._ray_step_length(step, turn, origin, heading, est, cr)
            a = math.radians(heading)
            unit = (math.cos(a), math.sin(a))
            # 【v7】步数触发混合：前2步沿射线走满proj(快)，第3步起朝MEC走d×0.45(稳)
            # 根因(诊断确认)：mock 上 no_signal=0%(不越点)，真实400+不是越点→兜底，
            # 而是走偏→多走步。mock ±1°误差小→1-2步near→不触发切换→不变慢；
            # 真实误差大→3+步还没near→切换到朝MEC→多条示向度平均抵消偏差→稳收敛。
            use_ray = True
            if iteration >= 2 and est is not None and cr < 1e8 and _sane_pos(est):
                d = dist(origin, est)
                if d > 1e-6:
                    standoff = max(3.0, d * 0.45)
                    ux = (est[0] - origin[0]) / d
                    uy = (est[1] - origin[1]) / d
                    nxt = (origin[0] + standoff * ux, origin[1] + standoff * uy)
                    use_ray = False
            if use_ray:
                nxt = (origin[0] + step * unit[0], origin[1] + step * unit[1])
                forward = ((nxt[0] - origin[0]) * unit[0]
                           + (nxt[1] - origin[1]) * unit[1])
                if forward <= 0.0:
                    raise RuntimeError("射线候选点未满足净推进约束")
            self.w.log("ray_chase", "沿示向度净推进",
                       {"ch": ch, "step": round(step, 2),
                        "Delta": None if turn is None else round(turn, 2)},
                       "执行第%d次射线复测" % (iteration + 1),
                       "候选点严格位于最新示向度正射线上")
            result = self.measure_and_handle(
                nxt, ch, note="射线追踪#%d" % iteration, account_to_source=True)
            if result == "near_cleared":
                return True
            if result == "near_clear_fail":
                return self.local_search(ch, nxt, 20.0, depth + 1)
            if result == "no_signal":
                break
            previous_heading = heading

        if (last_est is not None and last_cr < 1e8
                and _sane_pos(last_est)):
            return self.local_search(
                ch, last_est, min(max(last_cr, R_CLEAR) + 15.0, 45.0), depth + 1)
        self.w.log("ray_chase", "达到步数上限/丢信号", {"ch": ch},
                   "暂缓", "无有限MEC定位域或不解释的估计点(v6防护)时不盲目清除")
        return False

    def get_second_bearing(self, ch):
        return self.ray_chase_and_clear(ch)

    def approach_and_clear(self, ch, depth=0):
        return self.ray_chase_and_clear(ch, depth)

    # ---- 局部网格搜索（清除失败/丢信号的兜底：把源逼进5m触发near）----
    def local_search(self, ch, center, radius, depth=0):
        """以center为中心做方形螺旋网格探测，网格间距6m<5√2，保证某个网格点
        落在源5m内触发near，然后清除。为什么用网格：清除失败说明估计有界但
        不够准，网格是无遗漏的有限兜底。半径钳到≤45m(更大应先改善几何再搜)。
        depth：与approach_and_clear互相调用时的递归深度上限。"""
        if depth > 3:
            return False
        if not _sane_pos(center):
            self.w.log("local_search", "搜索中心在场外(退化交会)",
                       {"ch": ch, "center": _r2(center)},
                       "放弃本次", "场外中心必为数值退化产物,飞过去是纯浪费")
            return False
        radius = min(radius, 45.0)
        self.w.log("local_search", "清除失败/丢信号", {"ch": ch,
                   "center": _r2(center), "radius": round(radius, 1)},
                   "启动网格搜索", "网格间距%.0fm保证覆盖,源必入5m触发near" % LS_SPACING)
        pts = _square_spiral(center, radius, LS_SPACING, LS_MAX_PTS)
        for pt in pts:
            r = self.measure_and_handle(pt, ch, note="局部搜索",
                                        account_to_source=True)
            if r == "near_cleared":
                return True
            if r == "direction":
                # 搜到示向度就立刻重新交会；若已足够准，回到逼近伺服更快
                est, cr, nb = self.w.estimate(ch)
                if est is not None and nb >= 2 and cr <= CONF_GOOD * 0.5:
                    return self.approach_and_clear(ch, depth + 1)
        # 网格走完仍未near：用累计示向度最后交会一次再逼近
        est, cr, nb = self.w.estimate(ch)
        if est is not None and nb >= 2:
            return self.approach_and_clear(ch, depth + 1)
        self.w.log("local_search", "网格搜索结束", {"ch": ch}, "未清除",
                   "该源暂未定位,标记留待后续(不阻塞)")
        return False

    @staticmethod
    def _clamp_pos(pos):
        # 坐标分量绝对值不得超过2,000,000(附件2 §1.1)，防御性截断
        x = max(-MAX_COORD, min(MAX_COORD, pos[0]))
        y = max(-MAX_COORD, min(MAX_COORD, pos[1]))
        return (x, y)


# ==============================================================================
# 5. Planner：高层决策（只决定"下一步干什么"，执行全交给Executor）
# ==============================================================================
class HeuristicPlanner(object):
    """基线A：纯启发式，零训练，完全可解释。
    流程 = 两阶段串行：
      阶段1 阿基米德螺旋全覆盖扫描：在每个螺旋采样点扫全部未清除频道，
             发现即积累示向度、near即清；
      阶段2 最近邻贪心清除：对已发现未清除的源，反复挑离当前最近的逼近清除。
    为什么这样设计：螺旋保证几何覆盖(一个不漏)，最近邻贪心是TSP的经典
    可解释近似。它不做在线交错(先全扫完再统一清)，作为方案B的对照基线。"""
    name = "A_heuristic_spiral"

    def __init__(self, arm=800.0, step=800.0):
        # arm/step=800：螺旋采样点对 1800m 圆盘形成 1000m 覆盖（一个不漏）。
        # 盲区不再写死在这里——由 coverage_points() + coverage_check() 启动时
        # 实时复算并打印（注释会过期，代码不会）。
        self.spiral_pts = _gen_spiral(arm=arm, step=step, rmax=ARENA_R)

    def coverage_points(self):
        """本规划器用来保证"一个不漏"的探测位置集合 = 螺旋采样点。
        交给 report_coverage() 自动复算最坏盲区，不在注释里写历史值。"""
        return list(self.spiral_pts)

    def run(self, ex):
        w = ex.w
        # ---- 阶段1：螺旋全覆盖扫描 ----
        for pt in self.spiral_pts:
            # 【v6】已判空频道跳过(判空证书M8)
            chs = [c for c in range(1, N_CH + 1)
                   if c not in w.cleared and c not in w.certified_empty]
            ex.scan_waypoint(pt, chs, note="螺旋扫描")
        # ---- 阶段2：最近邻贪心清除 ----
        guard = 0
        while True:
            guard += 1
            if guard > 50:
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
            w.log("plan", "阶段2清除", {"ch": ch}, "选最近源清除",
                  "最近邻贪心近似TSP,减少空驶")
            ex.approach_and_clear(ch)
        # 螺旋已覆盖全盘(最坏盲区<1000m<=最小接收半径)且无待清除源 -> 覆盖完成
        w.scan_complete = (len(w.pending_channels()) == 0)


class ParamPlanner(object):
    """方案B：覆盖路点 backbone + 参数化线性打分的在线自适应决策。
    每个决策步，对所有候选动作打分并选最优：
      clear(ch)   : 清除已定位源ch      特征(距离, 置信半径, 示向度数)
      scan(wp)    : 去未访问路点wp扫描   特征(距离, 信息增益=未知+未定位频道数)
      refine(ch)  : 给只1条示向度的源补测 特征(距离, 置信半径=很大值得补)
    score = Σ w_i·feature_i，权重向量 w 用 CEM 在 MockSimulator 上自动调优。
    为什么这样设计：
      (自适应) 每步都看当前信念(哪些源已定位/哪些区域没探)在线选动作，
               清除与发现交错进行，顺路就把源清了，省掉基线A的空驶；
               ★这一条是实测出来的：两阶段串行(先全扫完再统一清)比在线交错
                慢 27.9%，所以基线A只留作对照，方案B不做串行。
      (可解释) 线性打分每项都有物理含义，能说出"为什么选它"；
      (鲁棒)   覆盖路点集保证最坏情况仍1000m全覆盖、一个不漏。
    路径规划的节点一律取【MEC 圆心】(World.estimate)，不用形心——形心没有
    误差上界，MEC 圆心是极小极大最优、误差上界=置信半径。"""
    name = "B_raychase_delta_R4"
    # 默认权重：经 CEM 在 MockSimulator 上调优(15轮,每代20个体,成功率=1约束),
    # 并在20个未参与调参的新种子上验证泛化——成功率仍100%,总虚拟时再降6.4%。
    # 各分量物理含义见下注释,可用 --weights 覆盖或用 --tune 重新寻优。
    # 【v5 改动，已实测】执行层换成射线追踪后重新用 CEM 调过的权重。
    # 旧权重是"6次逐步逼近"时代调出来的，执行层换了必须重调，否则决策失衡。
    # 关键证据：w1(清除-置信半径) 0.785 -> 0.010，CEM 自己发现
    #   "定位精度不再是清除的前提"；w5(补测) 1.135 -> 2.075。
    # 实测：总虚拟时 +1.4%(4173->4115)、单源清除耗时 +3.9%(175.8->168.9)。
    W0 = [1.739,   # w0 清除-距离(负)：越近越该先清
          0.010,   # w1 清除-置信半径(负)：射线追踪下几乎不再看精度
          0.751,   # w2 清除-示向度数(正)：证据越足越优先
          1.761,   # w3 扫描-距离(负)：越近的路点先去
          1.059,   # w4 扫描-信息增益(正)
          2.075,   # w5 补测-距离(负)
          0.051]   # w6 补测-置信半径(正)

    def __init__(self, weights=None, waypoints=None):
        self.w_vec = list(weights) if weights else list(self.W0)
        # 默认「中心 + 8 环@1000m」（时间接近最优 + 余量 44m）；可换用
        # WAYPOINT_SETS 里的其他集合做消融：hex7(7 点最少可行但慢 32.2%)、
        # ring6(覆盖不合格的反例)、ring7/ring8/ring9。
        self.waypoints = list(waypoints) if waypoints else list(COVER_WAYPOINTS)

    def coverage_points(self):
        """本规划器用来保证"一个不漏"的探测位置集合 = 覆盖路点集。"""
        return list(self.waypoints)

    # -- 候选动作打分 --
    def _score_actions(self, w):
        wv = self.w_vec
        cands = []   # (score, kind, key)
        # 清除候选：已交会出位置的未清除源
        for ch in w.pending_channels():
            est, cr, nb = w.estimate(ch)
            if est is None or not _sane_pos(est):
                # 【v6】退化估计点不作为清除候选(Q4同款防护)：否则会被
                # rolling 调度当成"顺路清除节点"排进路线,真的飞过去。
                continue
            d = dist(w.pos, est)
            s = (-wv[0] * d / REF_DIST
                 - wv[1] * min(cr, REF_CONF) / REF_CONF
                 + wv[2] * min(nb, 4) / 4.0)
            cands.append((s, "clear", ch))
        # 扫描候选：未访问覆盖路点(信息增益=未知频道+未定位频道)
        unkn = len(w.unknown_channels())
        unloc = len([c for c in w.pending_channels()
                     if w.estimate(c)[0] is None or w.estimate(c)[1] > CONF_GOOD])
        info = (unkn + 0.5 * unloc) / N_CH
        for i, wp in enumerate(self.waypoints):
            if i in w.waypoints_done:
                continue
            d = dist(w.pos, wp)
            s = -wv[3] * d / REF_DIST + wv[4] * info
            cands.append((s, "scan", i))
        # 补测候选：仅1条示向度的未清除源(交会不了,值得补第二示向度)。
        # ★仅当覆盖扫描全部完成后才补测——发现阶段先去各路点扫描,
        #   扫描本身就会为多数源白送第二示向度,远比逐个补测省时间。
        done_scan = len(w.waypoints_done) >= len(self.waypoints)
        if done_scan:
            for ch in w.pending_channels():
                est, cr, nb = w.estimate(ch)
                if nb == 1:
                    bs = w.bearings[ch]
                    d = dist(w.pos, (bs[-1][0], bs[-1][1]))
                    s = (-wv[5] * d / REF_DIST
                         + wv[6] * min(cr, REF_CONF) / REF_CONF)
                    cands.append((s, "refine", ch))
        return cands

    def _progress(self, w):
        """进度快照：已清除数 + 示向度总数 + 已访问路点数。用于检测"动作空转"。"""
        nb_total = sum(len(b) for b in w.bearings.values())
        return (len(w.cleared), nb_total, len(w.waypoints_done))

    def run(self, ex):
        w = ex.w
        guard = 0
        stall = {}      # (kind,key) -> 连续无进展次数,防止对同一失败动作死磕
        while True:
            guard += 1
            if guard > 200:
                w.log("plan", "主循环", {}, "强制收尾", "迭代超上限,防死循环")
                break
            # R4：16是题面给定的源数上界；已成功清除16个时不再扫描剩余频道。
            # 少于16绝不据此早停，避免把漏源当成不存在。
            if len(w.cleared) >= N_SRC_HI:
                w.log("plan", "达到源数上界", {"cleared": len(w.cleared)},
                      "确定性提前结束", "已清除数达到题面上界16")
                w.scan_complete = True
                break
            # 终止条件：覆盖完成 且 没有未清除的已发现源
            done_scan = len(w.waypoints_done) >= len(self.waypoints)
            pend = w.pending_channels()
            if done_scan and not pend:
                w.log("plan", "终止判断", {}, "任务完成",
                      "覆盖路点全部访问(保证一个不漏)且已发现源全部清除")
                w.scan_complete = True
                break
            cands = self._score_actions(w)
            if not cands:
                rest = [i for i in range(len(self.waypoints))
                        if i not in w.waypoints_done]
                if rest:
                    cands = [(0.0, "scan", rest[0])]
                else:
                    break
            cands.sort(key=lambda t: -t[0])
            # 选第一个"未卡死"的候选(同一动作连续无进展则降权跳过)
            chosen = None
            for score, kind, key in cands:
                if stall.get((kind, key), 0) < 3:
                    chosen = (score, kind, key)
                    break
            if chosen is None:
                # 全部候选都卡死：清空空转计数再试一轮,还不行就收尾
                if stall:
                    stall.clear()
                    continue
                break
            score, kind, key = chosen
            before = self._progress(w)
            if kind == "clear":
                w.log("plan", "在线决策", {"kind": "clear", "ch": key,
                      "score": round(score, 3)}, "清除该源",
                      "已定位且综合距离/置信最优")
                ex.approach_and_clear(key)
            elif kind == "scan":
                wp = self.waypoints[key]
                # 自适应选频道：未知频道(发现) + 已发现但未定位频道(补示向度)
                # 【v6】已判空频道不再进入扫描列表(判空证书M8已严格排除)
                chs = [c for c in range(1, N_CH + 1)
                       if c not in w.cleared and c not in w.certified_empty and (
                           c not in w.bearings or
                           (w.estimate(c)[0] is None or
                            w.estimate(c)[1] > CONF_GOOD))]
                w.log("plan", "在线决策", {"kind": "scan", "wp": _r2(wp),
                      "score": round(score, 3), "待测频道": len(chs)},
                      "前往路点扫描", "覆盖未探区域+信息增益最大")
                ex.scan_waypoint(wp, chs)
                w.waypoints_done.add(key)
            else:  # refine
                w.log("plan", "在线决策", {"kind": "refine", "ch": key,
                      "score": round(score, 3)}, "补第二示向度",
                      "单条示向度无法交会,按γ≈90°补测")
                ex.get_second_bearing(key)
            # 进展检测：该动作没带来任何进展 -> 计数+1,连续3次则暂时跳过它
            if self._progress(w) == before:
                stall[(kind, key)] = stall.get((kind, key), 0) + 1
            else:
                stall.pop((kind, key), None)


# ==============================================================================
# 6. Metrics：5 项指标统计 + CSV 导出
# ==============================================================================
def _square_spiral(center, radius, spacing, max_pts):
    """以center为中心的方形螺旋网格点(由内向外)，用于局部搜索兜底。"""
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


def _gen_spiral(arm=800.0, step=800.0, rmax=1800.0):
    """阿基米德螺旋 r=b·φ 采样点：臂间距≈arm、相邻点弧长≈step。
    为什么：臂间距与点距都取 800m(<2×1000m)，保证螺旋采样点对圆盘形成
    1000m 覆盖（一个不漏）。★默认值必须与 HeuristicPlanner 实参一致——
    历史上默认值写成 1300/1200 而实参是 800/800，两套数不一致会误导读者；
    覆盖资格现在由 coverage_check() 实时复算，不再靠这里写死的注释。"""
    b = arm / (2 * math.pi)
    pts = []
    phi = 0.0
    while True:
        r = b * phi
        if r > rmax:
            break
        pts.append((r * math.cos(phi), r * math.sin(phi)))
        ds = math.sqrt(r * r + b * b)      # 弧长元素 ds=sqrt(r²+b²)dφ
        phi += step / max(ds, 1e-6)
    return pts


def _overlap_rate(path):
    """重叠率 = 1 - 并集路径长/总路径长（越接近0说明越少重复走）。
    用5m栅格标记路径并集近似。为什么这样定义：重叠衡量探索冗余，
    重复访问已探区域越多、并集越小于总路程，重叠率越高。"""
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


def _r2(p):
    return (round(p[0], 1), round(p[1], 1))


def _out_path(name):
    """把输出文件名钉在【脚本所在目录】，避免在 Jupyter 里把结果写到随机的内核工作目录。

    为什么需要：Jupyter 的当前工作目录常是 notebook 所在目录甚至用户主目录，
    直接写相对路径会导致"文件生成了但找不到"。__file__ 在单元格 exec 时可能不存在，
    故做回退。
    """
    try:
        base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "generated", "q3")
    except NameError:
        base = os.getcwd()
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, name)


def _nearest_rank(values, q):
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    return ordered[max(0, min(len(ordered) - 1,
                              int(math.ceil(q * len(ordered))) - 1))]


def _source_summary(samples):
    import statistics as st
    costs = [float(x["clear_cost_s"]) for x in samples]
    measures = [float(x["source_measure_count"]) for x in samples]
    if not costs:
        return {}
    return {
        "n_source_samples": len(costs),
        "source_clear_mean_s": st.mean(costs),
        "source_clear_median_s": st.median(costs),
        "source_clear_p90_s": _nearest_rank(costs, 0.90),
        "source_measure_mean": st.mean(measures),
        "source_measure_median": st.median(measures),
        "source_measure_p90": _nearest_rank(measures, 0.90),
    }


def _bucket_rows(samples):
    import statistics as st
    specs = [
        ("<300", -float("inf"), 300.0),
        ("300-600", 300.0, 600.0),
        ("600-900", 600.0, 900.0),
        ("900-1200", 900.0, 1200.0),
        (">1200", 1200.0, float("inf")),
    ]
    out = []
    for label, lo, hi in specs:
        part = [x for x in samples if x.get("first_distance_m") is not None
                and lo <= float(x["first_distance_m"]) < hi]
        costs = [float(x["clear_cost_s"]) for x in part]
        lower = [float(x["lower_bound_s"]) for x in part]
        out.append({
            "distance_bucket_m": label,
            "n": len(part),
            "mean_s": st.mean(costs) if costs else "",
            "median_s": st.median(costs) if costs else "",
            "p90_s": _nearest_rank(costs, 0.90) if costs else "",
            "mean_lower_bound_s": st.mean(lower) if lower else "",
            "waste_ratio": (st.mean(costs) / st.mean(lower)) if lower else "",
        })
    return out


def _source_table_paths(csv_path):
    root, ext = os.path.splitext(csv_path)
    return root + "_sources.csv", root + "_buckets.csv"


def _write_source_tables(csv_path, samples):
    source_path, bucket_path = _source_table_paths(csv_path)
    source_cols = ["run_id", "seed", "channel", "first_distance_m",
                   "clear_cost_s", "source_measure_count", "lower_bound_s"]
    with open(source_path, "w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=source_cols)
        wr.writeheader()
        for row in samples:
            wr.writerow({c: row.get(c, "") for c in source_cols})
    bucket_rows = _bucket_rows(samples)
    with open(bucket_path, "w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=list(bucket_rows[0].keys()))
        wr.writeheader()
        wr.writerows(bucket_rows)
    print("单源明细已写出: %s" % source_path)
    print("距离分桶已写出: %s" % bucket_path)


def run_once(backend, planner, verbose=False):
    """跑一次完整演练：enter -> planner驱动 -> exit -> 汇总指标。

    中途若模拟器拒绝(accepted=false)、连不上或超时，不抛出中断整个程序，
    而是记录一条 abort 日志后照常收尾 —— 这样即便中途失败，也能拿到
    已完成的动作日志与部分指标，便于排查（正式测试判负也要留证据）。
    """
    world = World()
    ex = Executor(backend, world, verbose=verbose)
    t0 = time.time()
    ent = backend.enter()          # 进入失败由上层 main() 兜底给出排查清单
    world.budget_real_s = float(ent.get("remaining_real_duration_s", 0))
    world.max_virtual_s = float(ent.get("max_virtual_duration_s", 360000))
    world.enter_wall = time.time()     # 记录进入的墙钟时刻(算现实已用时间)
    world.log("enter", "进入目标区域", {}, "开始任务",
              "初始位置(0,0),初始频道1")
    abort_reason = ""
    try:
        planner.run(ex)
    except _BudgetExceeded:
        world.log("budget", "现实预算将尽", {}, "主动收尾",
                  "按/enter返回的remaining_real_duration_s提前退出,避免超窗判负")
    except (RuntimeError, OSError, ValueError) as e:
        # 测试提前结束/接口关闭/网络抖动/参数异常：收尾并保留证据，不让程序崩掉
        abort_reason = str(e)[:200]
        world.log("abort", "与模拟器通信异常", {}, "提前结束本轮", abort_reason)
    backend.exit()
    world.log("exit", "任务结束", {}, "主动退出",
              "覆盖完成或预算将尽")
    real_elapsed = time.time() - t0

    total_src = backend.total_sources()
    cleared = len(world.cleared)
    # 覆盖自检（用本局实际 planner 的探测点集实时复算，不用注释里的历史值）
    cov_gap, cov_margin, cov_ok = (0.0, 0.0, True)
    if hasattr(planner, "coverage_points"):
        cov_gap, cov_margin, cov_ok = coverage_check(planner.coverage_points())
    basis = "simulator"
    if total_src is None and world.scan_complete and cov_ok:
        # 真实模拟器不通过接口返回源总数（附件2 §1.3）。既然
        #   (1) 覆盖自检合格：最坏盲区 cov_gap < 1000m ≤ 任意源的最小接收半径
        #   (2) 覆盖扫描已跑完且无任何遗留未清除信号
        # 则由覆盖性论证可反推总源数：
        #   每个源必被某个探测点发现（(1)）；被发现过的又全部被清除（(2)）
        #   => 清除数 = 总源数（这是论证的结论，不是估计）。
        # ★(1) 是硬前提：若换了覆盖不合格的路点集（如 ring6），这里绝不能反推，
        #   否则会把"漏掉的源"当成"不存在"，把成功率虚报成 100%。
        total_src = cleared
        basis = "coverage-inferred(盲区%.1fm<1000m)" % cov_gap
    elif total_src is None:
        basis = "unknown"
    source_samples = world.source_records()
    source_stats = _source_summary(source_samples)
    m = {
        "planner": planner.name,
        "n_sources": total_src if total_src is not None else "",
        "n_sources_basis": basis,
        "cleared": cleared,
        "success_rate": (cleared / total_src) if total_src else "",
        "total_virtual_time_s": round(world.vt, 2),
        "avg_clear_time_s": round(world.vt / cleared, 2) if cleared else "",
        "path_length_m": round(world.path_len, 1),
        "overlap_rate": round(_overlap_rate(world.path), 4),
        "cover_gap_m": round(cov_gap, 2),
        "cover_margin_m": round(cov_margin, 2),
        "coverage_ok": cov_ok,
        "scan_complete": world.scan_complete,
        "n_measure": world.n_measure,
        "n_clear": world.n_clear,
        "n_action": world.n_action,
        "stuck_count": len(world.stuck),
        "source_clear_mean_s": round(source_stats.get("source_clear_mean_s", 0), 3),
        "source_clear_median_s": round(source_stats.get("source_clear_median_s", 0), 3),
        "source_clear_p90_s": round(source_stats.get("source_clear_p90_s", 0), 3),
        "source_measure_mean": round(source_stats.get("source_measure_mean", 0), 3),
        "source_measure_median": round(source_stats.get("source_measure_median", 0), 3),
        "source_measure_p90": round(source_stats.get("source_measure_p90", 0), 3),
        "source_samples": source_samples,
        "real_elapsed_s": round(real_elapsed, 2),
        "cleared_channels": sorted(world.cleared),
        "abort": abort_reason,
    }
    return m, world


def run_batch(planner_factory, n_runs, seed0=1000, csv_path=None, verbose=False):
    """多次演练(≥5)统计稳健性：均值/标准差 -> 供论文画箱线图/灵敏度分析。"""
    rows = []
    all_source_samples = []
    for r in range(n_runs):
        bk = MockSimulator(seed=seed0 + r)
        planner = planner_factory()
        m, world = run_once(bk, planner, verbose=verbose)
        m["run_id"] = r + 1
        for sample in m.get("source_samples", []):
            copied = dict(sample)
            copied["run_id"] = r + 1
            copied["seed"] = seed0 + r
            all_source_samples.append(copied)
        rows.append(m)
        print("[run %d/%d] %s | 源=%s 清除=%s 成功率=%s | 总虚拟时=%ss 均=%ss | "
              "路程=%sm 重叠=%s" % (
                  r + 1, n_runs, m["planner"], m["n_sources"], m["cleared"],
                  m["success_rate"], m["total_virtual_time_s"],
                  m["avg_clear_time_s"], m["path_length_m"], m["overlap_rate"]))
    # 聚合均值/标准差
    import statistics as st
    keys = ["success_rate", "total_virtual_time_s", "avg_clear_time_s",
            "path_length_m", "overlap_rate", "stuck_count"]
    agg = {"planner": rows[0]["planner"], "n_runs": n_runs}
    for k in keys:
        vals = [row[k] for row in rows if isinstance(row[k], (int, float))]
        if vals:
            agg[k + "_mean"] = round(st.mean(vals), 3)
            agg[k + "_std"] = round(st.pstdev(vals), 3) if len(vals) > 1 else 0.0
    agg.update({k: round(v, 3) if isinstance(v, float) else v
                for k, v in _source_summary(all_source_samples).items()})
    if csv_path:
        _write_csv(csv_path, rows, agg)
        _write_source_tables(csv_path, all_source_samples)
    if all_source_samples:
        print("[单源口径] n=%d 清除耗时 mean/median/p90=%.2f/%.2f/%.2fs; "
              "测量次数 mean/median/p90=%.2f/%.2f/%.2f" % (
                  agg["n_source_samples"], agg["source_clear_mean_s"],
                  agg["source_clear_median_s"], agg["source_clear_p90_s"],
                  agg["source_measure_mean"], agg["source_measure_median"],
                  agg["source_measure_p90"]))
    return rows, agg


def _write_csv(csv_path, rows, agg):
    cols = ["run_id", "planner", "n_sources", "n_sources_basis", "cleared",
            "success_rate", "total_virtual_time_s", "avg_clear_time_s",
            "path_length_m", "overlap_rate", "cover_gap_m", "cover_margin_m",
            "coverage_ok", "scan_complete", "n_measure", "n_clear", "n_action",
            "stuck_count", "source_clear_mean_s", "source_clear_median_s",
            "source_clear_p90_s", "source_measure_mean", "source_measure_median",
            "source_measure_p90",
            "real_elapsed_s", "abort"]
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        wcsv = csv.DictWriter(f, fieldnames=cols)
        wcsv.writeheader()
        for row in rows:
            wcsv.writerow({c: row.get(c, "") for c in cols})
        # 聚合行
        agg_row = {c: "" for c in cols}
        agg_row["planner"] = agg["planner"] + " [MEAN±STD]"
        agg_row["n_sources"] = "n_runs=%d" % agg["n_runs"]
        agg_row["success_rate"] = "%.3f±%.3f" % (
            agg.get("success_rate_mean", 0), agg.get("success_rate_std", 0))
        agg_row["total_virtual_time_s"] = "%.1f±%.1f" % (
            agg.get("total_virtual_time_s_mean", 0), agg.get("total_virtual_time_s_std", 0))
        agg_row["avg_clear_time_s"] = "%.1f±%.1f" % (
            agg.get("avg_clear_time_s_mean", 0), agg.get("avg_clear_time_s_std", 0))
        agg_row["path_length_m"] = "%.0f±%.0f" % (
            agg.get("path_length_m_mean", 0), agg.get("path_length_m_std", 0))
        agg_row["overlap_rate"] = "%.3f±%.3f" % (
            agg.get("overlap_rate_mean", 0), agg.get("overlap_rate_std", 0))
        wcsv.writerow(agg_row)
    print("CSV 已写出: %s" % csv_path)


# ==============================================================================
# 7. CEM black-box optimization of planner weights
#    按题意：PPO 训练成本高，允许退化为"可解释启发式+参数化权重"，
#    CEM tunes a parameter vector on the offline simulator.
# ==============================================================================
def tune_cem(n_iter=12, pop=16, elite=4, runs_per_eval=3, seed0=5000):
    """交叉熵方法调 ParamPlanner 的7维权重。目标=平均总虚拟时间(越小越好)，
    约束=成功率必须100%(否则罚大数)。返回最优权重与得分历史。
    为什么用CEM而非PPO：权重仅7维、环境廉价(仿真瞬回)、无需梯度，
    CEM稳定可复现且几十次迭代即收敛；PPO对此问题是杀鸡用牛刀。"""
    import random
    rng = random.Random(12345)
    dim = len(ParamPlanner.W0)
    mean = list(ParamPlanner.W0)
    std = [0.5] * dim
    best_w, best_score = None, float("inf")
    history = []

    def evaluate(wvec):
        # 同一批随机案例上评估(公平比较)；成功率<1则重罚
        scores = []
        for r in range(runs_per_eval):
            bk = MockSimulator(seed=seed0 + r)
            pl = ParamPlanner(weights=wvec)
            m, _ = run_once(bk, pl, verbose=False)
            pen = 0.0
            if isinstance(m["success_rate"], (int, float)) and m["success_rate"] < 1.0:
                pen = 1e6 * (1.0 - m["success_rate"])   # 没清干净 -> 巨大惩罚
            scores.append(m["total_virtual_time_s"] + pen)
        return sum(scores) / len(scores)

    for it in range(n_iter):
        pop_w = [[max(0.01, rng.gauss(mean[d], std[d])) for d in range(dim)]
                 for _ in range(pop)]
        scored = sorted(((evaluate(w), w) for w in pop_w), key=lambda t: t[0])
        elites = [w for _, w in scored[:elite]]
        for d in range(dim):
            col = [e[d] for e in elites]
            mean[d] = sum(col) / len(col)
            var = sum((c - mean[d]) ** 2 for c in col) / len(col)
            std[d] = max(0.05, math.sqrt(var))
        if scored[0][0] < best_score:
            best_score, best_w = scored[0][0], list(scored[0][1])
        history.append((it, round(scored[0][0], 1), [round(x, 3) for x in best_w]))
        print("[CEM iter %d/%d] best=%.1f  w=%s" %
              (it + 1, n_iter, scored[0][0], [round(x, 3) for x in best_w]))
    print("CEM 完成。最优权重=%s  最优平均总虚拟时=%.1f" %
          ([round(x, 3) for x in best_w], best_score))
    return best_w, history


# ==============================================================================
# 8. 主流程 / 命令行
# ==============================================================================
def _in_ipython():
    """判断当前是否跑在 Jupyter / IPython 内核里（用于给出对应的提示语）。"""
    try:
        return get_ipython() is not None       # noqa: F821  (IPython 注入的全局变量)
    except NameError:
        return False


# 本脚本自己的命令行开关（白名单）。
# 【为什么要白名单，而不是逐个排除内核参数】
# Jupyter / VS Code Notebook 的内核启动参数不止 -f 一种形态，实测还会带
# --matplotlib=inline、--ip=...、--HistoryManager.hist_file=... 等。用"黑名单"
# 逐个排除，只要漏掉一个，"清理后的参数表"就非空，于是【跳过 NOTEBOOK_ARGS 回退】
# ——表现为"我在单元格里改了 NOTEBOOK_ARGS，却仍然跑了 mock 默认值"，而且不报错，
# 极难排查（本轮实测踩到：--matplotlib=inline 残留导致回退失效）。
# 改成白名单后，凡不是本脚本的开关一律丢弃，内核参数再多也清得干净。
_ARG_TAKES_VALUE = ("--mode", "--planner", "--runs", "--seed0", "--csv",
                    "--weights", "--wp", "--seed", "--exec")
_ARG_FLAGS = ("--tune", "--verbose", "-h", "--help", "--empty-cert")


def _resolve_argv():
    """取出真正属于本脚本的命令行参数（白名单过滤 + Notebook 回退）。

    【为什么必须单独处理】
    在 Jupyter / VS Code Notebook 的单元格里直接运行本文件时，sys.argv 并不是
    空列表，而是内核的启动参数：
        ['ipykernel_launcher.py', '-f', '.../kernel-xxxx.json']
    argparse 不认识 -f，于是抛
        ipykernel_launcher.py: error: unrecognized arguments: -f ...json
    并 SystemExit(2) —— 这就是那个报错的根因，跟策略模型本身无关。

    处理办法：只保留白名单里的开关（本脚本自己的参数），其余一律丢弃；
    若清理后一个参数都不剩（说明是"裸跑本文件"），回退到 NOTEBOOK_ARGS，
    这样在单元格里可直接改 NOTEBOOK_ARGS 切换 mock/real 与各参数。

    在终端里 `python src/q3_autonomous_search.py --mode real` 时 sys.argv 本来就是干净的，
    本函数等价于原样返回，不影响命令行用法。
    """
    cleaned = []
    raw = sys.argv[1:]
    n = len(raw)
    i = 0
    while i < n:
        a = raw[i]
        name = a.split("=", 1)[0]
        if name in _ARG_TAKES_VALUE and "=" not in a:
            cleaned.append(a)          # 形如 --csv out.csv：把它的值一并带上
            if i + 1 < n:
                cleaned.append(raw[i + 1])
                i += 2
            else:
                i += 1
            continue
        if name in _ARG_TAKES_VALUE or name in _ARG_FLAGS:
            cleaned.append(a)          # 形如 --mode=real 或 --tune / --verbose
            i += 1
            continue
        # 不是本脚本的开关：内核的 "-f kernel.json" 是"开关+值"形态，吞掉它的值；
        # 其余内核开关都是 --k=v 形态，单步跳过即可。
        i += 2 if name == "-f" else 1
    if not cleaned and _in_ipython():
        # 仅在 Notebook 里"裸跑本文件"时启用 NOTEBOOK_ARGS；
        # 终端里 `python src/q3_autonomous_search.py` 不带参数仍走 argparse 默认值(mock)，
        # 避免误连模拟器。
        cleaned = NOTEBOOK_ARGS.split()
    return cleaned


def main():
    ap = argparse.ArgumentParser(
        description="2026国赛B题 问题3 机器狗搜索清除策略")
    ap.add_argument("--mode", choices=["mock", "real"], default="mock",
                    help="mock=离线仿真(默认,可无限制演练调参) / real=连真实模拟器")
    ap.add_argument("--planner", choices=["A", "B"], default="B",
                    help="A=纯启发式螺旋基线 / B=参数化自适应(默认)")
    ap.add_argument("--runs", type=int, default=5,
                    help="演练次数(≥5,统计稳健性方差)")
    ap.add_argument("--seed0", "--seed", dest="seed0", type=int, default=1000,
                    help="随机种子起点（--seed为等价简写）")
    ap.add_argument("--csv", default="", help="指标CSV输出路径")
    ap.add_argument("--tune", action="store_true",
                    help="先用CEM调ParamPlanner权重再演练(仅mock)")
    ap.add_argument("--weights", default="",
                    help="直接指定7维权重(逗号分隔),跳过调参")
    ap.add_argument("--wp", choices=sorted(WAYPOINT_SETS.keys()), default="default",
                    help="覆盖路点集(消融/灵敏度用): default=中心+8环@1000m(采用) / "
                         "hex7=7点最少可行(实测慢32%%) / ring6=覆盖不合格反例 / "
                         "ring7,ring8,ring9=纯环解析构造")
    ap.add_argument("--exec", choices=["ray", "legacy"], default="ray",
                    help="执行层：ray=射线追踪(v5默认,使用射线执行器) / "
                         "legacy=方案B逐步逼近(对首测距离分布天然鲁棒,真实A/B用)")
    ap.add_argument("--verbose", action="store_true", help="打印逐步动作日志")
    ap.add_argument("--robot-id", default=None,
                    help="连真机用的机器狗队号（优先于环境变量 ROBOT_ID）；"
                         "国赛要求代码不含队号，故默认留空，由运行环境注入")
    ap.add_argument("--empty-cert", action="store_true",
                    help="【v6】开启M8判空证书(默认关:9路点架构下实测零收益,"
                         "见 EMPTY_CERT_ENABLED 注释)")
    # parse_known_args：即使还有漏网的内核参数，也只是忽略而不是崩溃退出
    args, _unknown = ap.parse_known_args(_resolve_argv())
    global EMPTY_CERT_ENABLED
    if args.empty_cert:
        EMPTY_CERT_ENABLED = True
    robot_id = resolve_robot_id(args.robot_id)   # 运行时由 env/CLI 注入，源码不含队号
    print("[运行配置] mode=%s planner=%s exec=%s robot_id=%s%s"
          % (args.mode, args.planner, args.exec, _mask_robot_id(robot_id),
             "  (Jupyter 单元格模式)" if _in_ipython() else ""))
    # 执行层切换：同一份代码做真实环境 A/B，避免"两套代码各自演化"导致结论失真。
    if args.exec == "legacy":
        Executor.approach_and_clear = Executor._legacy_approach_and_clear_disabled
        Executor.get_second_bearing = Executor._legacy_get_second_bearing_disabled
        print("           执行层=legacy(方案B逐步逼近 standoff=0.45d)")

    # ---- 真实模拟器模式：单次正式/演练运行 ----
    if args.mode == "real":
        # ★pre-flight 自检：连真机前先确认队号已配置、端口可达，避免占位符
        #   或端口没开白白消耗一次正式测试机会。不消耗任何模拟器动作。
        ok, pf_msgs = preflight_check(BASE_URL, robot_id)
        if not ok:
            print("!! pre-flight 自检未通过，已中止 real 模式（未发送任何请求）：")
            for m in pf_msgs:
                print("   - %s" % m)
            print("逐条修复后重跑：python <本文件名> --mode real --robot-id <你的队号>")
            return
        else:
            print("[pre-flight] robot_id=%s 已配置、端口 %s 可达 ✓"
                  % (_mask_robot_id(robot_id), BASE_URL))
        bk = Client(base_url=BASE_URL, robot_id=robot_id, verbose=True)
        if args.planner == "A":
            planner = HeuristicPlanner()
        else:
            wv = ([float(x) for x in args.weights.split(",")]
                  if args.weights else None)
            planner = ParamPlanner(weights=wv, waypoints=WAYPOINT_SETS[args.wp])
        # ★连真机之前先自检覆盖资格：不合格(如 ring6)必须当场知道，
        #   否则会把"漏掉的源"误当成"不存在"，把成功率虚报成 100%。
        gap, margin, ok = report_coverage(planner)
        if not ok:
            print("!! 当前路点集覆盖不合格(最坏盲区 %.1fm > 1000m)，本局不保证一个不漏；"
                  "正式测试请改用 --wp default。" % gap)
        try:
            m, world = run_once(bk, planner, verbose=True)
        except Exception as e:
            # 进入动作就被拒/连不上：不让整个 Notebook 崩掉，给出可读排查线索
            print("\n!! 与模拟器通信失败：%s" % e)
            print("   逐条排查:")
            print("     1) 模拟器界面该局是否已开始、机器狗状态显示'已进入'；")
            print("     2) --robot-id / 环境变量 ROBOT_ID 是否与模拟器登录队号逐字节一致；")
            print("     3) 是否已过 25 分钟窗口 / 20 分钟程序时限(倒计时归零即接口关闭)；")
            print("     4) 接口地址端口是否仍是 %s。" % BASE_URL)
            return
        print("\n==== 本次运行指标 ====")
        for k, v in m.items():
            print("  %s = %s" % (k, v))
        # 保存可解释动作日志
        log_path = _out_path("q3_action_log.json")
        with open(log_path, "w", encoding="utf-8") as f:
            json.dump(world.logs, f, ensure_ascii=False, indent=1)
        # 再存一份带时间戳的归档：多次演练/正式测试互不覆盖，便于留证据
        stamp = time.strftime("%Y%m%d_%H%M%S")
        arch = _out_path("q3_action_log_%s_%s.json" % (args.planner, stamp))
        with open(arch, "w", encoding="utf-8") as f:
            json.dump(world.logs, f, ensure_ascii=False, indent=1)
        print("可解释动作日志已存: %s (共%d条)" % (log_path, len(world.logs)))
        print("                   归档: %s" % arch)
        print("提示: 正式测试请从模拟器界面导出加密日志放入支撑材料。")
        return

    # ---- 离线仿真模式 ----
    weights = None
    if args.weights:
        weights = [float(x) for x in args.weights.split(",")]
    elif args.tune and args.planner == "B":
        weights, _ = tune_cem()

    if args.planner == "A":
        factory = lambda: HeuristicPlanner()
    else:
        factory = lambda: ParamPlanner(weights=weights,
                                       waypoints=WAYPOINT_SETS[args.wp])
    # 覆盖自检：启动即打印本路点集的实测最坏盲区与余量（可复算，不靠注释）
    gap, margin, ok = report_coverage(factory())
    if not ok:
        print("!! 该路点集覆盖不合格(最坏盲区 %.1fm > 1000m)：这是消融反例，"
              "仅用于说明「点数够不等于覆盖够」，不要用它跑正式指标。" % gap)

    csv_path = args.csv or _out_path("q3_metrics_%s.csv" %
                                     ("A" if args.planner == "A" else "B"))
    rows, agg = run_batch(factory, n_runs=args.runs, seed0=args.seed0,
                          csv_path=csv_path, verbose=args.verbose)
    print("\n==== 聚合指标(mean±std, %d次演练) ====" % args.runs)
    for k, v in agg.items():
        print("  %s = %s" % (k, v))


if __name__ == "__main__":
    main()


# ================================================================================
# 附：方案B为何用「参数化权重+CEM」而非原生PPO（题意允许的取舍说明）
#   1) 决策本质是「离散动作(清哪个/去哪扫)+连续权重打分」，动作空间小、
#      状态低维，用深度PPO属于过度建模，且引入torch/gym重依赖、训练不稳定；
#   2) CEM是公认的无梯度随机搜索/进化策略，对7维权重几十次迭代即收敛，
#      全程可复现、可审计，符合"可解释、可验证"的硬要求；
#   3) 若坚持PPO：把 ParamPlanner._score_actions 换成策略网络 π(a|s)，
#      状态s=World特征(已清源数/剩余预算/各频道示向度数/置信半径/信息增益)，
#      动作a=候选动作ID，奖励r=-Δ虚拟时间(清除成功额外+r)，即可接入
#      stable-baselines3 的 PPO；环境用 MockSimulator 包成 gym.Env。
#      本代码已把状态/动作/奖励的接口留好，替换成本低。
# ================================================================================
