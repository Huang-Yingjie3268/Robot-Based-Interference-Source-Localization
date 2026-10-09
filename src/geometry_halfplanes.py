# -*- coding: utf-8 -*-
"""Half-plane localization using incremental convex-polygon clipping.

Evaluates region diameter and coverage. Convention: n dot (P - S) >= 0."""
from math import cos, sin, radians, atan2, sqrt, pi

EPS = 1e-9
BIG = 1e9  # 包围域半边长；用于检测"无界区域"（触及包围域边界则视为无界）


def sector_to_halfplanes(S, theta, eps):
    """
    检测点 S=(x,y), 示向度 theta(度), 误差 eps(度) -> 两个半平面。
    扇区 [theta-eps, theta+eps] 等价于两条边界射线的内半平面之交：
      下界: cross(d_min, P-S) >= 0  =>  (-sin a_min, cos a_min) . (P-S) >= 0
      上界: cross(d_max, P-S) <= 0  =>  ( sin a_max,-cos a_max) . (P-S) >= 0
    角度直接经 cos/sin 计算，天然处理跨 360° 环绕。
    """
    a_min = radians(theta - eps)
    a_max = radians(theta + eps)
    n_min = (-sin(a_min), cos(a_min))
    n_max = (sin(a_max), -cos(a_max))
    c_min = n_min[0] * S[0] + n_min[1] * S[1]
    c_max = n_max[0] * S[0] + n_max[1] * S[1]
    return [(n_min[0], n_min[1], c_min), (n_max[0], n_max[1], c_max)]


def _norm_angle(a, b):
    t = atan2(b, a)
    return t if t >= 0 else t + 2 * pi


def halfplane_intersection(hpl):
    """
    半平面交集。hpl: list of (nx, ny, c), 约束 n.P >= c。
    返回 dict: {status, vertices, diameter}
      status: 'polygon' | 'empty' | 'segment' | 'point' | 'unbounded'
    """
    # 1. 按法向极角排序
    hpl.sort(key=lambda h: _norm_angle(h[0], h[1]))

    # 2. 相同法向去重: 约束 n.P >= c, 保留 c 最大(最紧)者
    uniq = []
    for h in hpl:
        if uniq and abs(_norm_angle(h[0], h[1]) - _norm_angle(uniq[-1][0], uniq[-1][1])) < 1e-12:
            if h[2] > uniq[-1][2]:
                uniq[-1] = h
        else:
            uniq.append(h)
    hpl = uniq

    # 3. 基于包围域初始化的增量半平面裁剪 (Sutherland-Hodgman 式)
    poly = [(-BIG, -BIG), (BIG, -BIG), (BIG, BIG), (-BIG, BIG)]
    for (nx, ny, c) in hpl:
        newpoly = []
        m = len(poly)
        for i in range(m):
            cur = poly[i]
            nxt = poly[(i + 1) % m]
            dcur = nx * cur[0] + ny * cur[1] - c
            dnxt = nx * nxt[0] + ny * nxt[1] - c
            if dcur >= -EPS:
                newpoly.append(cur)
            if dcur * dnxt < -EPS:
                t = dcur / (dcur - dnxt)
                newpoly.append((cur[0] + t * (nxt[0] - cur[0]),
                                cur[1] + t * (nxt[1] - cur[1])))
        poly = newpoly
        if len(poly) == 0:
            return {'status': 'empty', 'vertices': [], 'diameter': None}

    # 4. 无界判定: 若裁剪结果触及包围域边界, 说明半平面交不闭合 -> 无界
    if any(abs(p[0]) > BIG - 1 or abs(p[1]) > BIG - 1 for p in poly):
        return {'status': 'unbounded', 'vertices': poly, 'diameter': None}

    # 5. 退化判定: 点 / 线段
    clean = [poly[0]]
    for p in poly[1:]:
        if sqrt((p[0] - clean[-1][0]) ** 2 + (p[1] - clean[-1][1]) ** 2) > EPS:
            clean.append(p)
    if len(clean) > 1 and sqrt((clean[0][0] - clean[-1][0]) ** 2 +
                               (clean[0][1] - clean[-1][1]) ** 2) < EPS:
        clean = clean[:-1]

    if len(clean) == 1:
        return {'status': 'point', 'vertices': clean, 'diameter': 0.0}
    if len(clean) == 2:
        d = sqrt((clean[0][0] - clean[1][0]) ** 2 + (clean[0][1] - clean[1][1]) ** 2)
        return {'status': 'segment', 'vertices': clean, 'diameter': d}

    return {'status': 'polygon', 'vertices': clean, 'diameter': None}


def diameter_by_enumeration(pts):
    """
    顶点枚举求凸多边形最远点对（直径）。
    本题定位区域顶点数 m <= 2N（N 为检测点个数，通常很小），
    O(m^2) 枚举既精确又易验证，作为主算法；规模很大时可改用旋转卡壳 O(m)。
    """
    n = len(pts)
    if n == 1:
        return 0, 0, 0.0
    if n == 2:
        return 0, 1, sqrt((pts[0][0] - pts[1][0]) ** 2 + (pts[0][1] - pts[1][1]) ** 2)
    best, bi, bj = -1.0, 0, 0
    for i in range(n):
        for j in range(i + 1, n):
            d = (pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2
            if d > best:
                best, bi, bj = d, i, j
    return bi, bj, sqrt(best)


def rotating_calipers(pts):
    """
    旋转卡壳求凸多边形最远点对（pts 逆时针，凸）。
    仅作为"规模较大时的复杂度优化"备选，主算法用顶点枚举。
    """
    n = len(pts)
    if n == 1:
        return 0, 0, 0.0
    if n == 2:
        return 0, 1, sqrt((pts[0][0] - pts[1][0]) ** 2 + (pts[0][1] - pts[1][1]) ** 2)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    j, best, bi, bj = 1, 0.0, 0, 0
    for i in range(n):
        while True:
            nxt = (j + 1) % n
            if abs(cross(pts[i], pts[(i + 1) % n], pts[nxt])) > abs(cross(pts[i], pts[(i + 1) % n], pts[j])):
                j = nxt
            else:
                break
        d = (pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2
        if d > best:
            best, bi, bj = d, i, j
    return bi, bj, sqrt(best)


def solve_q1(stations, eps=1.0):
    """
    主求解。
    stations: list of (x, y, theta_deg)
    eps: 示向度误差上界(度)
    返回 dict, 关键字段:
      status, diameter, diameter_pair(A,B), center O, radius R,
      max_vertex_dist, leakage(Delta), covered
    """
    hpl = []
    for (x, y, th) in stations:
        hpl += sector_to_halfplanes((x, y), th, eps)
    res = halfplane_intersection(hpl)

    if res['status'] == 'polygon':
        pts = res['vertices']
        # 主算法: 顶点枚举求直径
        bi, bj, D = diameter_by_enumeration(pts)
        A = pts[bi]
        B = pts[bj]
        O = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
        R = D / 2
        max_out = 0.0
        for v in pts:
            dv = sqrt((v[0] - O[0]) ** 2 + (v[1] - O[1]) ** 2)
            if dv > max_out:
                max_out = dv
        # 漏出量: 直径端点 A,B 本身到 O 的距离恰为 D/2, 故 Delta >= 0 恒成立
        leakage = max_out - R
        # 覆盖判据: Delta = 0 (容差内); 用相对容差吸收浮点误差
        covered = leakage <= R * 1e-6 + 1e-9
        res.update({
            'diameter': D,
            'diameter_pair': (A, B),
            'center': O,
            'radius': R,
            'max_vertex_dist': max_out,
            'leakage': leakage,
            'covered': covered,
        })
    return res
