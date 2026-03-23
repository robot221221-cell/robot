# Innovation-2 机制验证小扫参（Medium mini, seeds=42/46/51）

## 目标
验证 `sync_balance_weight` 是否能解释 medium 场景中 `no_evo` 优于 `ga` 的现象。

## 设定
- difficulty: `medium`
- seeds: `42,46,51`
- 对照：`sequence-mode=ga` vs `sequence-mode=no_evo`
- 扫参：`sync_balance_weight ∈ {0.0, 0.3, 0.5}`
- 其余参数固定为 Stage-1C 口径（Distance-LOD + Priority-LOD + auto-delay）

## 结果汇总（summary.csv）

| sequence_mode | sync_balance_weight | success_rate | parallel_makespan_mean (s) | wait_time_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|
| ga | 0.0 | 0.0 | inf | 0.8680 | 0.1606 | 5.4968 |
| ga | 0.3 | 1.0 | 46.2167 | 1.1507 | 103.4646 | 110.6466 |
| ga | 0.5 | 1.0 | 46.2167 | 1.1507 | 103.2867 | 110.4568 |
| no_evo | 0.0 | 1.0 | 43.0700 | 0.2100 | 0.2574 | 7.0987 |
| no_evo | 0.3 | 1.0 | 43.0700 | 0.2100 | 55.8612 | 62.7390 |
| no_evo | 0.5 | 1.0 | 43.0700 | 0.2100 | 55.3396 | 62.0359 |

## 关键观察
1. `sync_balance_weight=0.0` 时，GA 组出现 `success_rate=0.0`（3/3 失败，`deadlock_wait_count=201`），而 no_evo 组 3/3 成功。
2. 在 0.3 与 0.5 下，GA 组恢复成功，但 `makespan` 仍劣于 no_evo（46.22s vs 43.07s，约 +7.3%）。
3. no_evo 的 `wait_time_mean` 在所有权重都稳定为 0.21s，明显低于 GA（约 1.15s），说明 medium 反转主要由冲突等待差异驱动。
4. 提高 `sync_balance_weight`（0.3→0.5）对两组 `makespan` 几乎无影响，说明该参数不是本次 medium 反转的主导因子。

## 结论
- 该扫参**不支持**“仅靠调 `sync_balance_weight` 即可让 GA 在 medium 反超”的假设。
- 更可能的机制是：GA 当前优化目标（pairwise 成本主导）与在线调度中的冲突等待代价存在偏差，导致在 medium 分布下 no_evo 的局部聚类序列更占优。

## 产物目录
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w0_GA_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w03_GA_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w05_GA_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w0_NoEvo_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w03_NoEvo_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/MechSweep_MediumMini_w05_NoEvo_42_46_51_r1/`
