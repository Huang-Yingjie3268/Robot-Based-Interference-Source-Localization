"""Export repeatable offline experiments using the archived planners unchanged.

This driver only selects existing implementations and writes their outputs.
It does not tune weights or modify search/localization/clearance algorithms.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=['q3', 'q4'], required=True)
    parser.add_argument('--planner', choices=['A', 'B'], default='B')
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--seed0', type=int, default=1000)
    parser.add_argument('--directional-ratio', type=float, default=0.4,
                        help='Q4 only: fraction of directional sources')
    args = parser.parse_args()
    if args.runs <= 0:
        parser.error('--runs must be positive')
    if not 0 <= args.directional_ratio <= 1:
        parser.error('--directional-ratio must be within [0, 1]')
    root = Path(__file__).resolve().parents[1]
    out = root / 'results' / 'generated' / args.task
    out.mkdir(parents=True, exist_ok=True)
    if args.task == 'q3':
        import q3_autonomous_search as model
        factory = model.HeuristicPlanner if args.planner == 'A' else model.ParamPlanner
        model.report_coverage(factory())
        rows, _ = model.run_batch(factory, n_runs=args.runs, seed0=args.seed0,
                                  csv_path=str(out / f'{args.planner}_runs.csv'))
    else:
        import q4_directional_coverage as model
        model.report_coverage_q4(model.HeuristicPlanner() if args.planner == 'A'
                                 else model.ParamPlannerV6())
        rows = model._v6_run_batch(args.runs, seed0=args.seed0,
                                  dratio=args.directional_ratio,
                                  verbose=True, planner_name=args.planner)
    # One rectangular table for both tasks. Q3's original CLI additionally writes
    # summary sections in its CSV; summaries here have a separate JSON file.
    for index, row in enumerate(rows):
        row.setdefault('seed', args.seed0 + index)
    with (out / f'{args.planner}_runs.csv').open('w', newline='', encoding='utf-8') as f:
        # Detailed Q3 source samples already have their own CSV, avoiding a
        # duplicate nested Python-list representation inside the per-run table.
        writer = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k != 'source_samples'],
                                extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    total_sources = sum(int(row['n_sources']) for row in rows)
    cleared = sum(int(row['cleared']) for row in rows)
    total_time = sum(float(row['total_virtual_time_s']) for row in rows)
    summary = {
        'evidence_type': 'offline_mock_reproduction',
        'task': args.task, 'planner': args.planner,
        'runs': args.runs, 'seed_start': args.seed0,
        'seed_end': args.seed0 + args.runs - 1,
        'directional_ratio': args.directional_ratio if args.task == 'q4' else None,
        'python': platform.python_version(),
        'model_file': Path(model.__file__).name,
        'model_sha256': hashlib.sha256(Path(model.__file__).read_bytes()).hexdigest(),
        'total_sources': total_sources, 'cleared': cleared,
        'clearance_rate': cleared / total_sources if total_sources else None,
        'mean_run_virtual_time_s': total_time / len(rows),
        # Whole-run costs, including search, divided by cleared-source count.
        # This differs from per-source detection-to-clear latency in Q3 source tables.
        'total_virtual_time_per_cleared_source_s': total_time / cleared if cleared else None,
        'mean_path_length_m': statistics.mean(float(row['path_length_m']) for row in rows),
        'coverage_checks_passed': all(bool(row['coverage_ok']) for row in rows),
    }
    (out / f'{args.planner}_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
