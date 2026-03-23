# High-Conflict Case Study（Hard, with auto-delay）

## 选择规则
- 数据来源：
  - `xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_Run/tables/baseline_trials.csv`
  - `xml_pipeline_run/baseline_runs/CtrlC_Reactive_AutoDelay_HardFull30_42_51/tables/baseline_trials.csv`
- 规则：在 hard full30 中，按 `rrt_nodes_max_ur5e` 从高到低取 Top-3 作为高冲突子集。
- 选中 seeds：`47, 44, 46`（对应 `rrt_nodes_max_ur5e = 126, 87, 72`）。

## 子集结果（Ctrl-C vs Ours，统一 with auto-delay）

| method | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|
| Ctrl-C Reactive (with auto-delay) | 1.0 | 55.7780 | 1.7364 | 1.0760 | 224.8435 |
| Ours Stage-1C (with auto-delay) | 1.0 | 55.7780 | 1.7282 | 1.5320 | 103.2575 |

## seed 级明细（用于复核）

### Ours Stage-1C (with auto-delay)
| seed | rrt_nodes_max_ur5e | parallel_makespan_s | total_wall_time_s |
|---:|---:|---:|---:|
| 44 | 87 | 55.8000 | 103.5702 |
| 46 | 72 | 55.7460 | 103.4952 |
| 47 | 126 | 55.7880 | 102.7072 |

### Ctrl-C Reactive (with auto-delay)
| seed | rrt_nodes_max_ur5e | parallel_makespan_s | total_wall_time_s |
|---:|---:|---:|---:|
| 44 | 87 | 55.8000 | 219.2817 |
| 46 | 72 | 55.7460 | 220.0318 |
| 47 | 126 | 55.7880 | 235.2168 |

## 结论
1. 在高冲突子集上，Ctrl-C 与 Ours 的任务质量仍持平（success/makespan 一致）。
2. Ours 的计算墙钟保持显著优势（约 103s vs 225s，约 `2.18x` 更快）。
3. 该结果与 hard full30 主表结论一致：Stage-1C 的主要收益在计算效率，而非进一步压低 makespan。
