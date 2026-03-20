# Stage-1A Checkpoint-02 Smoke Manifest

- Archive: `stage1a_checkpoint_20260320_02_smoke`
- Date: 2026-03-20
- Scope: Stage-1A smoke run (`easy`, `seed=42`)

## Commit Base
- See `base_commit.txt`

## Command
- `conda run -n lerobot python run_baseline_batch.py --batch-name Stage1A_Geom_Smoke --difficulties easy --seeds 42`

## Results Snapshot
- success_rate: `1.0` (n=1)
- parallel_makespan_s: `23.312`
- speedup_vs_serial: `1.9684`

## Baseline Same-Point Comparison
- Baseline (`B0_Baseline_Run_v2`, easy seed42): makespan `23.276`, speedup `1.9706`
- Smoke delta: makespan `+0.036s`, speedup `-0.0022`

## Artifacts
- `results/baseline_summary.csv`
- `results/baseline_trials.csv`
- `results/baseline_summary.md`
- `docs/STAGE_PROGRESS_LOG.md`
- `docs/Stage1_Optimization_Log.md`
