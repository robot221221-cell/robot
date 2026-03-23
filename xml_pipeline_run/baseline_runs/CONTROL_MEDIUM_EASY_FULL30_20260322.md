# A/B/C vs Ours（Medium+Easy Full30, seeds=42-51）

## 批次目录
- A 串行基线（无 auto-delay）: xml_pipeline_run/baseline_runs/CtrlA_Serial_MediumEasyFull30_42_51
- B 启发式基线（无 auto-delay、无恢复）: xml_pipeline_run/baseline_runs/CtrlB_Heuristic_MediumEasyFull30_42_51
- C 解耦+反应式（无 auto-delay）: xml_pipeline_run/baseline_runs/CtrlC_Reactive_MediumEasyFull30_42_51
- C 解耦+反应式（with auto-delay）: xml_pipeline_run/baseline_runs/CtrlC_Reactive_AutoDelay_MediumEasyFull30_42_51
- Ours（Stage-1C Opt-v3，无 auto-delay）: xml_pipeline_run/baseline_runs/Ours_Stage1C_OptV3_NoAutoDelay_MediumEasyFull30_42_51
- Ours（Stage-1C Opt-v3，默认 auto-delay）: xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_Run

## 结果摘要（no auto-delay）

| difficulty | method | success_rate | parallel_makespan_mean (s) | speedup_mean | deadlock_wait_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| easy | Ctrl-A Serial | 1.0 | 45.7018 | 2.0000 | 0.0 | 0.0000 | 6.5518 |
| easy | Ctrl-B Heuristic | 0.0 | nan | 0.0000 | 201.0 | 0.0000 | 3.6963 |
| easy | Ctrl-C Reactive | 1.0 | 23.4518 | 1.9487 | 0.0 | 0.1080 | 5.0952 |
| easy | Ours Stage-1C | 1.0 | 23.2978 | 1.9616 | 0.0 | 0.2100 | 4.8338 |
| medium | Ctrl-A Serial | 1.0 | 79.2182 | 2.0000 | 0.0 | 0.0000 | 9.3134 |
| medium | Ctrl-B Heuristic | 0.0 | nan | 0.0000 | 201.0 | 0.0000 | 4.2846 |
| medium | Ctrl-C Reactive | 1.0 | 46.1094 | 1.7182 | 0.0 | 1.3926 | 7.2774 |
| medium | Ours Stage-1C | 0.0 | nan | 0.0000 | 201.0 | 0.8680 | 5.0646 |

## 主表建议（统一 with auto-delay 口径）

| difficulty | method | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|---:|
| easy | Ctrl-C Reactive (with auto-delay) | 1.0 | 23.3058 | 1.9687 | 0.0360 | 133.1354 |
| easy | Ours Stage-1C (with auto-delay) | 1.0 | 23.2978 | 1.9616 | 0.2100 | 60.5627 |
| medium | Ctrl-C Reactive (with auto-delay) | 1.0 | 45.6514 | 1.7598 | 0.7006 | 200.7553 |
| medium | Ours Stage-1C (with auto-delay) | 1.0 | 45.6514 | 1.7498 | 1.1566 | 98.2462 |

## 关键结论
1. no auto-delay 口径下，B 仍为稳定失败负对照；Ours 在 medium 出现失败，说明 `auto-delay` 仍是关键组成。
2. 统一 with auto-delay 后，medium/easy 与 hard 一致：Ours 与 Ctrl-C 在任务质量上基本持平（makespan 接近或相同）。
3. Ours 在 with auto-delay 口径下保持明显墙钟优势（easy/medium 均约为 Ctrl-C 的一半）。

## 备注
- 统计口径与 hard full30 报告保持一致，均来自各批次 `tables/baseline_summary.csv`。
- hard 报告见：`xml_pipeline_run/baseline_runs/CONTROL_HARD_FULL30_20260322.md`。
