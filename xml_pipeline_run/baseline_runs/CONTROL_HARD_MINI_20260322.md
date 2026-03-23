# A/B/C vs Ours（Hard Mini, seeds=42/46）

## 批次目录
- A 串行基线: `xml_pipeline_run/baseline_runs/CtrlA_Serial_HardMini_42_46`
- B 启发式基线: `xml_pipeline_run/baseline_runs/CtrlB_Heuristic_HardMini_42_46_v2`
- C 解耦+反应式: `xml_pipeline_run/baseline_runs/CtrlC_Reactive_HardMini_42_46`
- Ours Stage-1C Opt-v3: `xml_pipeline_run/baseline_runs/Ours_Stage1C_OptV3_HardMini_42_46_v2`

## 统一指标（hard）

| method | success_rate | parallel_makespan_mean (s) | speedup_mean | deadlock_wait_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|
| Ctrl-A Serial | 1.0 | 95.737 | 2.000 | 0.0 | 0.000 | 11.573 |
| Ctrl-B Heuristic | 0.0 | nan | 0.000 | 201.0 | 0.000 | 5.018 |
| Ctrl-C Reactive | 1.0 | 56.233 | 1.702 | 0.0 | 1.765 | 9.049 |
| Ours (Stage-1C Opt-v3) | 1.0 | 55.775 | 1.728 | 0.0 | 1.529 | 116.312 |

## 读数说明
- Ctrl-B 在 hard mini 上两次均失败（`success_rate=0`），并出现显著 `deadlock_wait_mean`。
- Ctrl-C 与 Ours 都成功；Ours 的 `parallel_makespan_mean` 与 `speedup_mean` 略优于 Ctrl-C。
- `total_wall_mean` 不可直接横向比较：
  - Ours 启用了 Stage-1C 预调度推演（`preview_wall_mean` 显著非零）；
  - Ctrl-A/B/C 这轮使用了 `--disable-auto-delay`，`preview_wall_mean=0`。
- 若需要公平比较墙钟，下一轮建议统一两种口径之一：
  1) 全部开启 auto-delay；或
  2) 全部关闭 auto-delay，仅比较 `planning + execution`。
