# Evaluation protocol and preserved variants

The software contains 16 task families and 32 independently selectable settings.
A new public suffix, `_variant01`, names the previously registered `_v3` setting.
Resolution returns the identical configuration and implementation. Legacy scene
IDs, TOML names, run IDs and recorded snapshots remain unchanged. Selecting both
aliases of one task in the same batch is rejected as a duplicate.

## What remains fixed

The release preparation changes no task TOML, prompt, scene/texture/mesh, physical
material, collision geometry, initial pose, camera calibration/appearance,
scoring implementation or controller setting. The pre-change contract in
`configs/benchmark/task_contract.json` captures 32 configurations and hashes
247 protected files. Derived numeric fingerprints use 12 significant digits (magnitudes below 1e-12 treated as zero) to
avoid Python/NumPy last-bit differences; the protected source/configuration
files still require exact SHA-256 equality. This normalization is only in the
checker and never changes a runtime value or model prompt. `tools/release/check_task_contract.py` recomputes effective
robot, simulation, execution, layout, instruction and scoring fingerprints.
`assets/runtime.lock.json` pins the complete 325-file asset closure separately.

## Existing differences are part of the protocol

| Task group | Original / alternative relationship |
|---|---|
| L1 (5 tasks) | Original placement and fixed selected P03 placement. The stored selection indices are 3, 3, 2, 1, 1 in task-ID order. |
| L2 (6 tasks) | Original placement and fixed P01 placement; existing task rules are retained. |
| L3-03 / 05 / 11 | Separate delivered scene settings, with their existing prompts, cameras and implementations. |
| L3-14 | Original scores containment of both books; alternative uses the lower book boundary. This is a protocol difference, not just a pose change. |
| L3-15 | Separate delivered alternative scene. Existing success criterion requires all three blocks. |

L2-04 requires the source cup to be empty and at least **6 of 12** beads to be in
the receiving cup. It does not succeed merely because six beads have arrived
while others are still in the source cup.

L1 and L3 retain gravity compensation disabled and Cartesian completion tolerance
0.005 m; L2 retains gravity compensation enabled and tolerance 0.001 m. These are
existing settings, not newly recommended universal values.

## Budgets

`configs/benchmark/experiments.json` lists all current defaults. It describes
this software version, not reconstructed historical records.

| Settings | Effective call cap | Wall-time cap | Call accounting |
|---|---:|---:|---|
| L1 originals / variants | 100 | disabled | all requests |
| L2 originals / variants | 100 | disabled | all requests |
| L3-03 / 05 / 11 originals / variants | 100 | disabled | completed generations |
| L3-14 / 15 originals | disabled | 18,000 s | completed generations |
| L3-14 / 15 alternatives | 150 | 18,000 s | completed generations |

A zero cap means disabled. `tools/reproduce.py` passes the preserved per-task
limits explicitly so that `batch_run.py` defaults cannot replace them. CLI
budget/model/seed overrides must be reported as distinct conditions.

## Policy and execution

Model input comprises the task instruction, shared operating instructions,
configured camera images, robot state and execution feedback. It does not
receive evaluator object-state tables as a direct privileged input. The motion
planner and evaluator do use simulator geometry. Task-specific scoring and
execution safeguards are part of the system; do not describe it as just a raw
text prompt with no supporting machinery.

Within an episode, configured request history is available. Repeating a task
does not automatically add learned task skills or append past run archives to
model context. Real-robot zero-shot versus skill/history-enabled conditions must
be labeled independently; they are not interchangeable experimental populations.

## Reporting

Use the stored instruction, full policy instructions, effective configuration,
seed, model/transport/effort, planner revision, asset hash, calls, elapsed time,
actions, outcome and termination reason for each attempt. Keep all attempted
runs in the denominator and identify excluded infrastructure failures explicitly.
Do not combine different scoring rules, variants or budgets in a single success
rate without stratification. Videos are supporting evidence, not the sole source
of truth for the evaluator.

`batch_run.py --summarize PATH` writes JSON/CSV/Markdown and separates succeeded,
failed, invalid, timeout, interruption, infrastructure error, skipped and dry-run
outcomes. The release does not substitute invented totals for missing raw runs.
Raw experiment logs and real-robot materials are distributed separately when a
paper results release is finalized.
