# -*- coding: utf-8 -*-
"""Export objective grids and geometric points for Q2 visualization.

Uses the retained observation-optimization model without changing its parameters."""
import sys, os, csv
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from q2_observation_optimization import (support_boundary, max_dist_to_support, in_frec, worst_diam,
                         build_P1, rbound, frec_margin, N_ARC)

S1 = (0.0, 0.0)
theta1 = 0.0
_HERE = os.path.dirname(os.path.abspath(__file__))
_DATA = os.path.join(os.path.dirname(_HERE), 'results', 'generated', 'q2')
os.makedirs(_DATA, exist_ok=True)
out = _DATA

p1 = build_P1(S1, theta1)
samples = support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801)
rb = rbound(samples, S1)
print(f'P1 凸多边形顶点数（n_arc=N_ARC={N_ARC}）= {len(p1)}')
print(f'R*(g) 范围 = [{min(rb):.4f}, {max(rb):.4f}] m（耦合口径）')

# ---- maxdist / margin 网格（10 m；窗口与 grid_J 完全一致，MATLAB 侧据此断言同格）----
# 故取 x∈[-50,1050]、y∈[-1000,1000]；与 q2_results.WIN_* 逐字一致。
XS = np.arange(-50.0, 1050.1, 10.0)
YS = np.arange(-1000.0, 1000.1, 10.0)
with open(os.path.join(out, 'grid_maxdist.csv'), 'w', newline='', encoding='utf-8') as f:
    w = csv.writer(f)
    w.writerow(['x', 'y', 'maxdist', 'margin'])
    for x in XS:
        for y in YS:
            P = (float(x), float(y))
            w.writerow([x, y, round(max_dist_to_support(P, samples), 4),
                        round(frec_margin(P, samples, rb), 4)])
print(f'maxdist/margin 网格 {len(XS)}x{len(YS)} 已写出')

# ---- grid_J.csv 一致性核对（由 q2_optimization_experiments.py 写出，本脚本不重算）----
gj = os.path.join(out, 'grid_J.csv')
if not os.path.isfile(gj):
    raise SystemExit('缺少 data/grid_J.csv —— 请先运行 code/q2_optimization_experiments.py（它同时产出候选区域三层与 J 网格）')
with open(gj, encoding='utf-8') as f:
    rows = list(csv.DictReader(f))
assert len(rows) == len(XS) * len(YS), \
    f'grid_J 行数 {len(rows)} ≠ {len(XS)}*{len(YS)}，与 grid_maxdist 不同格'
nJ = sum(1 for r in rows if r['J'] not in ('', None))
print(f'grid_J.csv 核对通过：{len(rows)} 行，其中 {nJ} 点有 J（= F_rec 格点数）')

# ---- 关键点（供 MATLAB 标注）：全部读自 CSV，不写死坐标 ----
def read_center():
    with open(os.path.join(out, 'q2_center_result.csv'), encoding='utf-8') as f:
        r = next(csv.DictReader(f))
    return r


def read_worst():
    d = {}
    with open(os.path.join(out, 'q2_worst_branch.csv'), encoding='utf-8') as f:
        for r in csv.DictReader(f):
            d[r['item']] = r['value']
    return d


def _pair(s):
    return [float(v) for v in s.strip('()').split(',')]


cr = read_center()
s2x, s2y = float(cr['S2_x']), float(cr['S2_y'])
wb = read_worst()
e1 = _pair(wb['diam_end_1'])
e2 = _pair(wb['diam_end_2'])
# 基线选点也不写死：中心构型 CSV 里就存着 baseline_x / baseline_y
bx, by = float(cr['baseline_x']), float(cr['baseline_y'])
# ★ 坐标一律按双精度原样写出（不 round）：q2_data.m 有一条
#   chk('S_2^* 与 J* 同源一致', norm(D.S2c - D.S2dn), 0, 1e-9)，
#   S2c 来自 q2_center_result.csv（全精度）、S2dn 来自本文件的 keypoints.csv；
#   早先这里写成 round(s2x, 6) 会引入 ~1.5e-7 的截断误差，使该断言失败——
#   而它本来是"两份数据必须同源"这一红线的最直接检查，不能靠放宽容差蒙过去。
with open(os.path.join(out, 'keypoints.csv'), 'w', newline='', encoding='utf-8') as f:
    w = csv.writer(f)
    w.writerow(['name', 'x', 'y'])
    w.writerow(['S1', S1[0], S1[1]])
    w.writerow(['S2_opt_up', s2x, abs(s2y)])
    w.writerow(['S2_opt_dn', s2x, s2y])
    w.writerow(['baseline', bx, by])
    w.writerow(['worst_end_1', e1[0], e1[1]])
    w.writerow(['worst_end_2', e2[0], e2[1]])
print('keypoints.csv 已写出（S2* = (%.6f, %.6f)，最坏分支端点 (%.6f,%.6f)/(%.6f,%.6f)）'
      % (s2x, s2y, e1[0], e1[1], e2[0], e2[1]))
