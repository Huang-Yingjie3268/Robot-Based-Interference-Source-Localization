#!/usr/bin/env python3
"""Question 1: bounded-bearing set-intersection localization.

Primary numerical path:
1) convert each bearing cone into two directed half-planes;
2) compute a bounded polygon by angular sorting + deque half-plane intersection;
3) compute its diameter with rotating calipers;
4) compute the minimum enclosing circle for the coverage test.

The pairwise boundary-intersection routine is retained only as an independent
O(n^2) verifier.  This script also writes the original validation CSV files;
their numerical contents are intentionally kept unchanged.
"""
from __future__ import annotations

import csv
import math
import random
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

TOL = 1.0e-9
FEAS_TOL = 2.0e-7


def cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def cone_halfplanes(
    sensors: np.ndarray, bearings_deg: np.ndarray, delta_deg: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return A,b for all direction cones in the form A @ p <= b."""
    rows: list[np.ndarray] = []
    rhs: list[float] = []
    delta = math.radians(delta_deg)
    for sensor, bearing_deg in zip(sensors, bearings_deg):
        theta = math.radians(float(bearing_deg))
        low = np.array([math.cos(theta - delta), math.sin(theta - delta)])
        high = np.array([math.cos(theta + delta), math.sin(theta + delta)])
        # lower ray: low x (p-S) >= 0
        a_low = np.array([low[1], -low[0]])
        # upper ray: high x (p-S) <= 0
        a_high = np.array([-high[1], high[0]])
        rows.extend((a_low, a_high))
        rhs.extend((float(a_low @ sensor), float(a_high @ sensor)))
    return np.array(rows, dtype=float), np.array(rhs, dtype=float)


def feasible_point(a: np.ndarray, b: np.ndarray) -> np.ndarray | None:
    result = linprog(np.zeros(2), A_ub=a, b_ub=b, bounds=[(None, None)] * 2, method="highs")
    return result.x if result.success else None


def is_bounded(a: np.ndarray, b: np.ndarray) -> bool:
    """A 2-D polyhedron is bounded iff both coordinates have finite extrema."""
    for objective in (
        np.array([1.0, 0.0]), np.array([-1.0, 0.0]),
        np.array([0.0, 1.0]), np.array([0.0, -1.0]),
    ):
        result = linprog(objective, A_ub=a, b_ub=b, bounds=[(None, None)] * 2, method="highs")
        if result.status == 3:
            return False
        if not result.success:
            raise RuntimeError(f"Unexpected linear-program status: {result.message}")
    return True


def unique_rows(points: list[np.ndarray], tol: float = 1.0e-7) -> np.ndarray:
    unique: list[np.ndarray] = []
    for point in points:
        if not any(np.linalg.norm(point - old) <= tol for old in unique):
            unique.append(point)
    return np.array(unique) if unique else np.empty((0, 2))


def feasible_vertices(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """O(n^2) pairwise boundary enumeration used only as a verifier."""
    points: list[np.ndarray] = []
    for i in range(len(a)):
        for j in range(i + 1, len(a)):
            matrix = np.vstack((a[i], a[j]))
            det = float(np.linalg.det(matrix))
            if abs(det) <= TOL:
                continue
            point = np.linalg.solve(matrix, np.array([b[i], b[j]]))
            if np.all(a @ point <= b + FEAS_TOL):
                points.append(point)
    vertices = unique_rows(points)
    if len(vertices) >= 3:
        center = vertices.mean(axis=0)
        order = np.argsort(np.arctan2(vertices[:, 1] - center[1], vertices[:, 0] - center[0]))
        vertices = vertices[order]
    elif len(vertices) == 2:
        vertices = vertices[np.lexsort((vertices[:, 1], vertices[:, 0]))]
    return vertices


@dataclass(frozen=True)
class HalfPlane:
    a: np.ndarray
    b: float
    direction: np.ndarray
    angle: float
    point: np.ndarray


def _make_halfplanes(a: np.ndarray, b: np.ndarray) -> list[HalfPlane]:
    hps: list[HalfPlane] = []
    for ai, bi in zip(a, b):
        norm = float(np.linalg.norm(ai))
        if norm <= TOL:
            continue
        an = ai / norm
        bn = float(bi / norm)
        # d=(-a_y,a_x) makes feasible side the left side of the directed line.
        d = np.array([-an[1], an[0]])
        p0 = an * bn
        hps.append(HalfPlane(an, bn, d, math.atan2(d[1], d[0]), p0))
    hps.sort(key=lambda h: h.angle)

    # 同向去重：法向量单位化后，b 越小表示约束越严格。
    # Same-direction deduplication: for normalized a, smaller b is tighter.
    dedup: list[HalfPlane] = []
    for h in hps:
        if dedup and abs(cross(dedup[-1].direction, h.direction)) <= TOL and float(dedup[-1].direction @ h.direction) > 0:
            if h.b < dedup[-1].b:
                dedup[-1] = h
        else:
            dedup.append(h)
    return dedup


def _line_intersection(h1: HalfPlane, h2: HalfPlane) -> np.ndarray | None:
    matrix = np.vstack((h1.a, h2.a))
    det = float(np.linalg.det(matrix))
    if abs(det) <= TOL:
        return None
    return np.linalg.solve(matrix, np.array([h1.b, h2.b]))


def _outside(h: HalfPlane, p: np.ndarray) -> bool:
    return float(h.a @ p - h.b) > FEAS_TOL


def halfplane_intersection_deque(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """O(n log n) angular-sort + deque half-plane intersection for 2-D polygons."""
    hps = _make_halfplanes(a, b)
    q: deque[HalfPlane] = deque()

    for h in hps:
        while len(q) >= 2:
            p = _line_intersection(q[-2], q[-1])
            if p is not None and _outside(h, p):
                q.pop()
            else:
                break
        while len(q) >= 2:
            p = _line_intersection(q[0], q[1])
            if p is not None and _outside(h, p):
                q.popleft()
            else:
                break
        q.append(h)

    while len(q) >= 3:
        p = _line_intersection(q[-2], q[-1])
        if p is not None and _outside(q[0], p):
            q.pop()
        else:
            break
    while len(q) >= 3:
        p = _line_intersection(q[0], q[1])
        if p is not None and _outside(q[-1], p):
            q.popleft()
        else:
            break

    if len(q) < 3:
        return np.empty((0, 2))
    vertices: list[np.ndarray] = []
    q_list = list(q)
    for i in range(len(q_list)):
        p = _line_intersection(q_list[i], q_list[(i + 1) % len(q_list)])
        if p is None or np.any(a @ p > b + FEAS_TOL):
            return np.empty((0, 2))
        vertices.append(p)
    vertices_arr = unique_rows(vertices)
    if len(vertices_arr) >= 3:
        center = vertices_arr.mean(axis=0)
        order = np.argsort(np.arctan2(vertices_arr[:, 1] - center[1], vertices_arr[:, 0] - center[0]))
        vertices_arr = vertices_arr[order]
    return vertices_arr


def polygon_area(vertices: np.ndarray) -> float:
    if len(vertices) < 3:
        return 0.0
    return 0.5 * abs(float(np.dot(vertices[:, 0], np.roll(vertices[:, 1], -1)))
                     - float(np.dot(vertices[:, 1], np.roll(vertices[:, 0], -1))))


def diameter_bruteforce(vertices: np.ndarray) -> tuple[float, tuple[int, int]]:
    if len(vertices) < 2:
        return 0.0, (0, 0)
    best, pair = -1.0, (0, 1)
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            value = float(np.linalg.norm(vertices[i] - vertices[j]))
            if value > best:
                best, pair = value, (i, j)
    return best, pair


def diameter_rotating_calipers(vertices: np.ndarray) -> tuple[float, tuple[int, int]]:
    n = len(vertices)
    if n <= 2:
        return diameter_bruteforce(vertices)
    j, best, pair = 1, 0.0, (0, 1)
    for i in range(n):
        ni = (i + 1) % n
        while True:
            nj = (j + 1) % n
            current = abs(cross(vertices[ni] - vertices[i], vertices[j] - vertices[i]))
            upcoming = abs(cross(vertices[ni] - vertices[i], vertices[nj] - vertices[i]))
            if upcoming > current + TOL:
                j = nj
            else:
                break
        for first, second in ((i, j), (ni, j)):
            value = float(np.linalg.norm(vertices[first] - vertices[second]))
            if value > best:
                best, pair = value, (first, second)
    return best, pair


@dataclass
class Circle:
    center: np.ndarray
    radius: float


def circle_from_two(a: np.ndarray, b: np.ndarray) -> Circle:
    center = (a + b) / 2.0
    return Circle(center, float(np.linalg.norm(a - center)))


def circle_from_three(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> Circle:
    matrix = np.array([2.0 * (b - a), 2.0 * (c - a)])
    rhs = np.array([float(b @ b - a @ a), float(c @ c - a @ a)])
    if abs(float(np.linalg.det(matrix))) <= TOL:
        candidates = (circle_from_two(a, b), circle_from_two(a, c), circle_from_two(b, c))
        valid = [circle for circle in candidates
                 if all(np.linalg.norm(point - circle.center) <= circle.radius + 1.0e-8 for point in (a, b, c))]
        return min(valid, key=lambda circle: circle.radius)
    center = np.linalg.solve(matrix, rhs)
    return Circle(center, float(np.linalg.norm(a - center)))


def minimum_enclosing_circle(points: np.ndarray, seed: int = 2026) -> Circle:
    if len(points) == 0:
        raise ValueError("The minimum enclosing circle is undefined for an empty set")
    order = list(range(len(points)))
    random.Random(seed).shuffle(order)
    circle: Circle | None = None
    for ii, i in enumerate(order):
        p = points[i]
        if circle and np.linalg.norm(p - circle.center) <= circle.radius + 1.0e-8:
            continue
        circle = Circle(p.copy(), 0.0)
        for jj in range(ii):
            q = points[order[jj]]
            if np.linalg.norm(q - circle.center) <= circle.radius + 1.0e-8:
                continue
            circle = circle_from_two(p, q)
            for kk in range(jj):
                r = points[order[kk]]
                if np.linalg.norm(r - circle.center) > circle.radius + 1.0e-8:
                    circle = circle_from_three(p, q, r)
    assert circle is not None
    return circle


def classify_and_solve(sensors: np.ndarray, bearings_deg: np.ndarray, delta_deg: float = 1.0) -> dict:
    a, b = cone_halfplanes(sensors, bearings_deg, delta_deg)
    witness = feasible_point(a, b)
    if witness is None:
        return {"state": "empty", "diameter": None, "vertices": np.empty((0, 2))}
    if not is_bounded(a, b):
        return {"state": "unbounded", "diameter": math.inf, "vertices": np.empty((0, 2))}

    # 主路径：双端队列半平面交；两两边界交点枚举只承担独立复核。
    # Main path: deque HPI. Pairwise enumeration is an independent verifier and
    # a robust fallback for lower-dimensional degeneracies (point/segment).
    vertices = halfplane_intersection_deque(a, b)
    enum_vertices = feasible_vertices(a, b)
    if len(vertices) < 3:
        vertices = enum_vertices
    elif len(enum_vertices) >= 3:
        d_main, _ = diameter_bruteforce(vertices)
        d_enum, _ = diameter_bruteforce(enum_vertices)
        if abs(d_main - d_enum) > 1.0e-6 or len(vertices) != len(enum_vertices):
            raise AssertionError("Deque HPI disagrees with pairwise verifier")

    if len(vertices) == 0:
        vertices = np.array([witness])
    area = polygon_area(vertices)
    if len(vertices) == 1:
        state = "point"
    elif len(vertices) == 2 or area <= 1.0e-8:
        state = "segment"
        if len(vertices) > 2:
            _, pair0 = diameter_bruteforce(vertices)
            vertices = vertices[list(pair0)]
    else:
        state = "bounded_polygon"

    diameter, pair = diameter_rotating_calipers(vertices)
    brute_diameter, _ = diameter_bruteforce(vertices)
    if abs(diameter - brute_diameter) > 1.0e-7:
        raise AssertionError("Rotating calipers disagrees with brute force")
    max_residual = float(np.max(a @ vertices.T - b[:, None]))
    if max_residual > FEAS_TOL:
        raise AssertionError("A reported vertex violates a direction-cone constraint")
    mec = minimum_enclosing_circle(vertices)
    center_ab = (vertices[pair[0]] + vertices[pair[1]]) / 2.0
    rho = max(float(np.linalg.norm(v - center_ab)) for v in vertices)
    return {
        "state": state, "vertices": vertices, "area": area, "diameter": diameter,
        "diameter_pair": pair, "mec": mec, "specific_circle_rho": rho,
        "specific_circle_covers": rho <= diameter / 2.0 + 1.0e-8,
        "any_diameter_circle_covers": mec.radius <= diameter / 2.0 + 1.0e-8,
        "max_constraint_residual": max_residual,
        "diameter_crosscheck_error": abs(diameter - brute_diameter),
    }


def bearing(sensor: np.ndarray, source: np.ndarray, offset_deg: float) -> float:
    true_value = math.degrees(math.atan2(source[1] - sensor[1], source[0] - sensor[0]))
    return (true_value + offset_deg) % 360.0


def save_csv(path: Path, columns: list[str], rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def build_cases() -> dict[str, tuple[np.ndarray, np.ndarray, float]]:
    source = np.array([0.0, 0.0])
    tri_sensors = np.array([[1000.0, 0.0], [-500.0, 500.0 * math.sqrt(3.0)], [-500.0, -500.0 * math.sqrt(3.0)]])
    tri_bearings = np.array([bearing(sensor, source, 0.5) for sensor in tri_sensors])
    return {
        "two_point": (np.array([[0.0, 0.0], [100.0, 0.0]]), np.array([45.0, 135.0]), 1.0),
        "three_point": (np.array([[0.0, 0.0], [100.0, 0.0], [50.0, 100.0]]), np.array([45.0, 135.0, 270.0]), 1.0),
        "equilateral_counterexample": (tri_sensors, tri_bearings, 1.0),
        "point_degenerate": (np.array([[0.0, 0.0], [0.0, 0.0]]), np.array([0.0, 180.0]), 1.0),
        "unbounded": (np.array([[0.0, 0.0]]), np.array([0.0]), 1.0),
    }


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "results" / "generated" / "q1"
    data_dir.mkdir(parents=True, exist_ok=True)

    cases = build_cases()
    results: dict[str, dict] = {}
    start = time.perf_counter()
    for name, (sensors, bearings, delta) in cases.items():
        results[name] = classify_and_solve(sensors, bearings, delta)
    elapsed_ms = (time.perf_counter() - start) * 1000.0

    # Preserve the original validation CSVs exactly from the legal equilateral case.
    sensors, bearings, _ = cases["equilateral_counterexample"]
    result = results["equilateral_counterexample"]
    input_rows = [{"detector": f"S{i}", "x_m": f"{sensor[0]:.6f}", "y_m": f"{sensor[1]:.6f}",
                   "bearing_deg": f"{angle:.6f}", "offset_deg": "0.500000"}
                  for i, (sensor, angle) in enumerate(zip(sensors, bearings), start=1)]
    save_csv(data_dir / "validation_input.csv", list(input_rows[0]), input_rows)
    vertex_rows = [{"vertex": f"V{i}", "x_m": f"{point[0]:.9f}", "y_m": f"{point[1]:.9f}"}
                   for i, point in enumerate(result["vertices"], start=1)]
    save_csv(data_dir / "validation_vertices.csv", ["vertex", "x_m", "y_m"], vertex_rows)
    summary_rows = [
        {"metric": "state", "value": result["state"], "unit": "-"},
        {"metric": "diameter", "value": f"{result['diameter']:.9f}", "unit": "m"},
        {"metric": "minimum_enclosing_radius", "value": f"{result['mec'].radius:.9f}", "unit": "m"},
        {"metric": "D_over_2", "value": f"{result['diameter']/2:.9f}", "unit": "m"},
        {"metric": "any_D_diameter_circle_covers", "value": str(result["any_diameter_circle_covers"]), "unit": "-"},
    ]
    save_csv(data_dir / "validation_summary.csv", ["metric", "value", "unit"], summary_rows)

    sensitivity_rows: list[dict] = []
    previous = -math.inf
    for delta in (0.8, 0.9, 1.0, 1.1, 1.2):
        current = classify_and_solve(sensors, bearings, delta)
        if current["diameter"] + 1.0e-7 < previous:
            raise AssertionError("Diameter should not decrease when the error bound expands")
        previous = current["diameter"]
        sensitivity_rows.append({"delta_deg": f"{delta:.1f}", "diameter_m": f"{current['diameter']:.9f}",
                                 "area_m2": f"{current['area']:.9f}",
                                 "minimum_enclosing_radius_m": f"{current['mec'].radius:.9f}"})
    save_csv(data_dir / "sensitivity_delta.csv", ["delta_deg", "diameter_m", "area_m2", "minimum_enclosing_radius_m"], sensitivity_rows)

    labels = {
        "two_point": "two-point intersection",
        "three_point": "three-point intersection",
        "equilateral_counterexample": "equilateral counterexample",
        "point_degenerate": "point degeneracy",
        "unbounded": "unbounded configuration",
    }
    print("case,state,vertices,D_m,Rmin_m,covers")
    for name in cases:
        r = results[name]
        if r["state"] in {"empty", "unbounded"}:
            d = "+inf" if r["state"] == "unbounded" else "--"
            print(f"{labels[name]},{r['state']},{len(r['vertices'])},{d},--,--")
        else:
            print(f"{labels[name]},{r['state']},{len(r['vertices'])},{r['diameter']:.9f},{r['mec'].radius:.9f},{r['any_diameter_circle_covers']}")
    print(f"3.4921 reproducibility check: {results['two_point']['diameter']:.4f} m")
    print(f"counterexample max residual: {result['max_constraint_residual']:.12e}")
    print(f"calipers-vs-bruteforce error: {result['diameter_crosscheck_error']:.12e}")
    print(f"five-case runtime: {elapsed_ms:.3f} ms")


if __name__ == "__main__":
    main()
