# -*- coding: utf-8 -*-
"""Cross-check clipped-polygon and vertex-enumeration Q2 diameters.

Runs 50 fixed-seed samples within the reception-feasible region."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from q2_observation_optimization import (region_diam, region_diam_bruteforce, support_boundary,
                         in_frec, theta2_range, build_P1, N_ARC)

S1 = (0.0, 0.0); theta1 = 0.0
SEED = 20260911
N = 50

samples = support_boundary(S1, theta1, n_ang=N_ARC, n_rad=801)
p1 = build_P1(S1, theta1)


def main():
    rng = np.random.default_rng(SEED)
    diffs = []
    n_match = 0
    n_poly = 0
    n_try = 0
    for _ in range(N):
        S2 = None
        for _ in range(100):
            x = float(rng.uniform(0.0, 1010.0))
            y = float(rng.uniform(-950.0, 950.0))
            if in_frec((x, y), samples, S1):
                S2 = (x, y)
                break
        if S2 is None:
            continue
        lo, hi = theta2_range(S2, samples)
        th = float(rng.uniform(lo, hi))
        st1, D1 = region_diam(S1, theta1, S2, th, p1=p1)
        st2, D2 = region_diam_bruteforce(S1, theta1, S2, th, p1=p1)
        n_try += 1
        if st1 == st2:
            n_poly += 1
            d = abs(D1 - D2)
            diffs.append(d)
            if d < 1e-5:
                n_match += 1
    assert n_try == N, f'有效样本数 {n_try} ≠ {N}'
    assert n_poly == N, f'两算法状态一致样本数 {n_poly} ≠ {N}'
    print(f'有效样本数 {n_try}（全部 S2∈F_rec、θ2∈Θ(S2)）')
    print(f'两算法状态一致 {n_poly}/{N}')
    print(f'最大直径差 = {max(diffs):.3e} m')
    print(f'均值 {sum(diffs)/len(diffs):.3e} m，{n_match}/{len(diffs)} 差值 < 1e-5')
    # 机器可读输出，供正文引用
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'generated', 'q2')
    os.makedirs(out, exist_ok=True)
    if os.path.isdir(out):
        with open(os.path.join(out, 'q2_verify_result.txt'), 'w', encoding='utf-8') as f:
            f.write(f'seed={SEED}\nn_samples={n_try}\nstatus_match={n_poly}\n')
            f.write(f'max_abs_diff_m={max(diffs):.10e}\nmean_abs_diff_m={sum(diffs)/len(diffs):.10e}\n')
            f.write(f'n_diff_lt_1e-5={n_match}\n')
    print('独立复核完成。')


if __name__ == '__main__':
    main()
