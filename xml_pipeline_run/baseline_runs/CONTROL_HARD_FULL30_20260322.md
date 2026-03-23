# A/B/C vs Ours（Hard Full30, seeds=42-51）

## 批次目录
- A 串行基线（无 auto-delay）: xml_pipeline_run/baseline_runs/CtrlA_Serial_HardFull30_42_51
- B 启发式基线（无 auto-delay、无恢复）: xml_pipeline_run/baseline_runs/CtrlB_Heuristic_HardFull30_42_51
- C 解耦+反应式（无 auto-delay）: xml_pipeline_run/baseline_runs/CtrlC_Reactive_HardFull30_42_51
- C 解耦+反应式（with auto-delay）: xml_pipeline_run/baseline_runs/CtrlC_Reactive_AutoDelay_HardFull30_42_51
- Ours（Stage-1C Opt-v3，无 auto-delay）: xml_pipeline_run/baseline_runs/Ours_Stage1C_OptV3_NoAutoDelay_HardFull30_42_51
- Ours（Stage-1C Opt-v3，默认 auto-delay）: xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_Run

## 结果摘要（hard）

| method | success_rate | parallel_makespan_mean (s) | speedup_mean | deadlock_wait_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|
| Ctrl-A Serial (no auto-delay) | 1.0 | 95.7534 | 2.0000 | 0.0 | 0.0000 | 11.6722 |
| Ctrl-B Heuristic (no auto-delay) | 0.0 | nan | 0.0000 | 201.0 | 0.0000 | 5.3236 |
| Ctrl-C Reactive (no auto-delay) | 1.0 | 56.2494 | 1.7023 | 0.0 | 1.7814 | 9.2449 |
| Ours Stage-1C (no auto-delay) | 0.0 | nan | 0.0000 | 201.0 | 0.8680 | 6.1932 |
| Ours Stage-1C (with auto-delay) | 1.0 | 55.7914 | 1.7280 | 0.0 | 1.5454 | 103.2772 |

## 关键结论
1. 在 hard full30 上，`auto-delay` 是 Ours 的关键组成：关闭后成功率显著下降。
2. 在成功运行的组别中，Ours（with auto-delay）相比 Ctrl-C Reactive 有更优 makespan / speedup。
3. Ctrl-B（无恢复）稳定失败，作为“弱启发式”负对照有效。

## 主表建议（统一 with auto-delay 口径）

| method | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|
| Ctrl-C Reactive (with auto-delay) | 1.0 | 55.7914 | 1.7362 | 1.0894 | 224.0143 |
| Ours Stage-1C (with auto-delay) | 1.0 | 55.7914 | 1.7280 | 1.5454 | 103.2772 |

说明：
- 在 hard full30 下，Ctrl-C(with auto-delay) 与 Ours(with auto-delay) 的 `parallel_makespan_mean` 相同；
- Ours 的 `preview_wall_mean / total_wall_mean` 显著更低，说明在同等任务质量下，Stage-1C 漏斗机制带来计算效率优势。

## 下一步（按计划）
1. （已完成）统一“with auto-delay”口径补跑 Ctrl-C，形成公平主表。
2. （已完成）在 hard full30 主表稳定后，扩展到 medium/easy（见 `CONTROL_MEDIUM_EASY_FULL30_20260322.md`）。
3. （已完成）增加高冲突子集（High-Conflict Case Study）用于亮点对照（见 `CONTROL_HIGH_CONFLICT_20260322.md`）。
