# Reproducibility and evidence boundaries

## Checked environment

Executed checks used Python 3.12.14 and NumPy from the desktop runtime;
the Q4 Python figure renderer also used ReportLab and Poppler.
The minimal dependency list is inferred from retained source imports, not from
the unrelated libraries installed on the packaging machine.

SciPy was unavailable in that runtime, and installation was unsuccessful. The
SciPy-dependent Q1 main entry point was preserved and syntax-checked, but not
executed. Its independent NumPy-based verifier passed all five archived cases.
The intended full environment can be installed with `pip install -r requirements.txt`.
The original archive mentions Python 3.13 and MATLAB R2025b; that statement is
historical metadata, not proof of a tested package lock.

## Validation performed

| Check | Evidence |
|---|---|
| Python source syntax | All retained modules compile. |
| Numerical algorithm preservation | Original and cleaned ASTs compared; retained numerical function/class bodies are unchanged. Changes are paths, imports, configuration, output formatting and documentation. |
| Q1 independent verifier | Five archived cases pass: two-point, three-point, equilateral, point-degenerate and unbounded. |
| Q2 geometric cross-check | 50 samples, seed 20260911; 50 matching states; maximum diameter difference 4.5474735089e-13 m. Both methods use the same discretized support polygon. |
| Q2 archived optimum/baseline | Re-evaluated at `(843.0442010349119, -545.5139116553951)` and `(750, 500)`; results match archived values. This checks objective evaluation, not global optimality. |
| Q2 reception-margin grid | Selected grid entries recomputed; agree within stored four-decimal rounding. The archived grid has 12,187 feasible cells. |
| Q3/Q4 offline validation | 30 runs per retained setting, seeds 1000–1029; raw tables and source hashes in `results/reproduced/`. |
| Renamed simulation entry point | Q3 B and Q4 B Quick Start commands each rerun for five seeds (1000–1004); both cleared 72/72 sources. These generated files do not replace the 30-run snapshots. |
| Figure QA | Five archived figures retained (two rasterized from standalone PDFs); Q4 regenerated from its model with plain-text annotations and visually checked. |

Not executed: the Q1 primary SciPy path, complete Q2 optimization/sensitivity
regeneration, CEM tuning, MATLAB rendering, or the external contest simulator.
No physical robot measurements are claimed.

## Reproduce the offline tables

From the repository root:

```bash
python src/run_simulation_experiments.py --task q3 --planner A --runs 30 --seed0 1000
python src/run_simulation_experiments.py --task q3 --planner B --runs 30 --seed0 1000
python src/run_simulation_experiments.py --task q4 --planner B --runs 30 --seed0 1000 --directional-ratio 0.4
```

New outputs go to `results/generated/<task>/`. The checked-in snapshots are in
`results/reproduced/<task>/`, separately from the original archive's Q1/Q2 files
in `results/reference/`. Regeneration does not overwrite those snapshots.

Q3 B uses the default nine-waypoint parameterized planner and ray executor;
Q3 A uses the original spiral baseline. Q4 B uses `ParamPlannerV6`, a 24-point
layout, and a directional-source probability of 0.4. The driver invokes the
original simulator/planner code and only standardizes exports. It does not
retune weights or introduce a new algorithm.

## Metric definitions

- **Clearance rate:** total cleared sources divided by total generated sources
  across runs. The Q3 A failure remains in the table (383/384).
- **Mean path:** arithmetic mean of per-run `path_length_m`.
- **Mean virtual time:** arithmetic mean of per-run simulator time, including
  travel, measurements, switching and clearance. It is not wall-clock runtime.
- **Time per cleared source:** total virtual time divided by total clearances;
  this includes coverage/search overhead. It is not the mean of per-run ratios.
- **Q3 source latency:** detection-to-clearance time in the separate source
  table. Unobserved or uncleared sources are not samples in that table, so its
  latency mean must not substitute for whole-task costs or clearance rate.
- **Q2 objective:** worst posterior feasible-region diameter, not Euclidean
  localization error relative to a known true source.
- **Coverage checks:** numerical geometry diagnostics; Q4 samples a finite
  interior grid, boundary points, and local refinements. No interval or
  branch-and-bound proof of continuous-domain angular coverage is implemented.

Seeds fix the synthetic worlds and sensing randomness. Wall-clock timing can vary
across machines and should not be treated as a deterministic benchmark. Full
Q2 optimization can also differ slightly with numerical/environment changes.

## MATLAB

MATLAB scripts and required style/data helpers are retained. The original
scripts target R2025b and use recent `exportgraphics` features. MATLAB was not
available during packaging, so updated paths have only been statically checked.

Each task has its own directory because Q1 and Q2 use differently implemented
helpers with the same names. Run their batch commands in separate MATLAB processes:

```bash
matlab -batch "cd('matlab/q1'); run_all_figs"
matlab -batch "cd('matlab/q2'); run_all_figs"
matlab -batch "cd('matlab/q3'); plot_coverage_geometry"
matlab -batch "cd('matlab/q4'); plot_directional_coverage_geometry"
```

Q1/Q2 plotting reads **archived** reference CSVs by default. To visualize a new
full Q2 run, explicitly update the data-directory selection in `q2_data_dir.m`
after reviewing the new CSVs. `q2_data.m` intentionally asserts archived numeric
values; update those checks consciously if the experiment settings change.

## External simulator

The HTTP clients refer to a local contest service at `127.0.0.1:2026`. This
repository does not include that service. A successful mock run does not
verify external simulator interoperability. Runtime identity is supplied via
`ROBOT_ID` (Q3 also accepts `--robot-id`); no real team ID is stored in the repo.
Terminal and notebook defaults are offline. Old competition logs, notebooks,
papers and intermediate archives are excluded.

## Maintenance choices

Output paths and module references were adjusted for the new layout. An
overridden duplicate `support_candidates` definition and an obsolete Q2 model
diagnostic main block were removed; the active numerical implementations were
retained. Q4's plotting script now includes only geometric plotting, excluding
statistical plots based on manually embedded values without raw-run provenance.

Python and MATLAB geometry routines remain separate where they have different
interfaces or serve as independent verifiers. Broader deduplication would need
additional numerical regression checks and was deliberately left for future work.


## Q4 figure regeneration

The Python renderer uses ReportLab and the external Poppler `pdftoppm` executable. With `pdftoppm` on PATH, run:

```bash
python src/plot_directional_coverage.py
```

Alternatively provide its executable path with `--renderer PATH_TO_PDFTOPPM`. The default image is written to `results/generated/figures/q4_directional_coverage.png`; use `--output figures/q4_directional_coverage.png` only to deliberately replace the displayed figure. A temporary vector PDF is rendered and removed; no paper PDF is included.

This command was executed successfully. It calls the retained Q4 waypoint builder and sampled angular-gap check, rather than tracing or editing the archived image. The MATLAB alternative fixes escaped annotations, RGB styling, and clipping of the reception disk; it has not been executed here.

The Q4 model header was clarified after the saved simulation runs. The stored `model_sha256` identifies the source used at execution time; the subsequent edit changes only its module docstring. Numerical code preservation was checked again. Snapshot hashes were not rewritten to imply a new execution.
