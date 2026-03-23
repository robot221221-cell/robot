# Paper-Ready Main Table（with auto-delay, seeds=42-51）

## 统一口径
- 对照方法：Ctrl-C Reactive vs Ours Stage-1C Opt-v3
- 统计口径：全部使用 with auto-delay
- 数据来源：
  - hard: `CONTROL_HARD_FULL30_20260322.md`
  - medium/easy: `CONTROL_MEDIUM_EASY_FULL30_20260322.md`
  - high-conflict: `CONTROL_HIGH_CONFLICT_20260322.md`

## 主表（可直接放论文）

| split | method | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|---:|
| easy | Ctrl-C Reactive (with auto-delay) | 1.0 | 23.3058 | 1.9687 | 0.0360 | 133.1354 |
| easy | Ours Stage-1C (with auto-delay) | 1.0 | 23.2978 | 1.9616 | 0.2100 | 60.5627 |
| medium | Ctrl-C Reactive (with auto-delay) | 1.0 | 45.6514 | 1.7598 | 0.7006 | 200.7553 |
| medium | Ours Stage-1C (with auto-delay) | 1.0 | 45.6514 | 1.7498 | 1.1566 | 98.2462 |
| hard | Ctrl-C Reactive (with auto-delay) | 1.0 | 55.7914 | 1.7362 | 1.0894 | 224.0143 |
| hard | Ours Stage-1C (with auto-delay) | 1.0 | 55.7914 | 1.7280 | 1.5454 | 103.2772 |

## 高冲突子集（亮点表，hard Top-3 by `rrt_nodes_max_ur5e`）

| split | method | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|---:|
| hard-high-conflict | Ctrl-C Reactive (with auto-delay) | 1.0 | 55.7780 | 1.7364 | 1.0760 | 224.8435 |
| hard-high-conflict | Ours Stage-1C (with auto-delay) | 1.0 | 55.7780 | 1.7282 | 1.5320 | 103.2575 |

## 可直接复用的论文结论（中文）
1. 在 easy/medium/hard 三个 full30 分组中，Ours 与 Ctrl-C 在任务质量上基本持平（`success_rate` 均为 1.0，`parallel_makespan_mean` 接近或相同）。
2. 在统一 with auto-delay 口径下，Ours 的计算墙钟显著更低：
   - easy：60.56s vs 133.14s
   - medium：98.25s vs 200.76s
   - hard：103.28s vs 224.01s
3. 在 hard 的高冲突子集中，上述结论保持一致（55.78s makespan 持平，墙钟约 103s vs 225s），说明效率优势在冲突更强的样本上同样稳定。

## 英文一句话版本（可放摘要/结论）
Under a unified auto-delay setting, our Stage-1C pipeline matches Ctrl-C in task quality (success and makespan) while consistently reducing computation wall-time by about 2× across easy/medium/hard full30 and the high-conflict hard subset.

## Innovation-2（GA + Priority Delay）严格对照进展（2026-03-23）
- hard full30（GA vs strict no_evo）：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_GA_ONOFF_EXPLICIT_FULL30_20260323.md`
- medium/easy full30（GA vs strict no_evo）：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_MEDIUM_EASY_FULL30_20260323.md`
- 显著性检验（paired sign test + bootstrap CI）：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md`
- medium 机制扫参（`sync_balance_weight` mini）：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_MECHANISM_SWEEP_MEDIUM_MINI_20260323.md`
- GA seeding mini 消融：`xml_pipeline_run/baseline_runs/CONTROL_GA_SEEDING_MINI_20260323.md`
- 最终归档汇总：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_FINAL_ARCHIVE_20260323.md`

简述：严格 GA-off（no_evo）对照与显著性检验已完成。结论为“难度相关收益”：hard 上 GA 对 makespan 有显著正收益（p=0.002），medium 上 no_evo 更优（p=0.002），easy 差异不显著（p=0.50）。

## 增量小创新收口决策（2026-03-23）
- 决策总表：`xml_pipeline_run/baseline_runs/CONTROL_INCREMENTAL_INNOVATIONS_DECISION_20260323.md`
- 采用口径：
  - 保留主创新：Innovation-2（难度相关收益）
  - 保留为辅助证据：medium 机制扫参、GA seeding（弱正向）
  - 不并入主线：GA objective alignment（未扭转 medium）与 Innovation-1（收益弱）
