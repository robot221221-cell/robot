# GA 目标函数对齐（Objective Alignment）mini 验证（2026-03-23）

## 目标
验证“将 GA 适应度改为 makespan/wait 代理主导”是否能缓解 medium 场景下 GA 劣于 no_evo 的问题。

## 改动
- 文件：`ga_task_allocation.py`
- 新增：
  - `--objective-mode {legacy, aligned}`
  - `--weight-makespan`, `--weight-total-cost`, `--weight-balance`, `--weight-wait-proxy`
  - `--base-solution`（支持按现有 solution 的重复任务多重集进行重排）
- aligned 适应度：
  - 主项：`max(ur5e_cost, fr3_cost)`（makespan 代理）
  - 次项：`total_cost`
  - 弱项：`balance_gap`
  - 额外：全局臂切换次数作为 wait 代理

## 试验设置（mini）
- difficulties: medium + hard
- seeds: 42/46/51
- 对照组：
  1) `ga (legacy objective)`
  2) `ga (aligned objective)`
  3) `no_evo`
- 调度参数：Stage-1C + auto-delay（与主线一致）

## 结果（summary.csv）

| split | method | success_rate | parallel_makespan_mean (s) | wait_time_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|
| medium | GA legacy | 1.0 | 55.6100 | 4.3633 | 107.3337 |
| medium | GA aligned | 1.0 | 55.6100 | 4.3633 | 107.8530 |
| medium | no_evo | 1.0 | 43.0700 | 0.2100 | 61.5562 |
| hard | GA legacy | 1.0 | 65.6080 | 4.3633 | 109.7515 |
| hard | GA aligned | 1.0 | 61.9200 | 1.4780 | 166.7408 |
| hard | no_evo | 1.0 | 57.6820 | 1.1540 | 85.6187 |

## 结论
1. 在本轮 mini 中，`GA aligned` **未能**解决 medium 反转：medium 的 makespan 与 legacy 相同，且明显差于 no_evo。
2. hard 上 `GA aligned` 相比 legacy 有改善（65.61 → 61.92），但仍落后 no_evo（57.68）。
3. 说明“仅微调离线适应度权重”不足以让 GA 价值函数与执行期耗时完全对齐。
4. 下一步应考虑：
   - 直接把在线调度预估（wait/delay 代理）嵌入 GA 评估回路；或
   - 重建覆盖完整任务集（含 top3/top4/front3/front4 等）的 pairwise 成本矩阵，再做目标对齐。

## 产物路径
- `xml_pipeline_run/baseline_runs/ObjAlignV3_Legacy_GA_MedHardMini_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/ObjAlignV3_Aligned_GA_MedHardMini_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/ObjAlign_NoEvo_MedHardMini_42_46_51_r1/`
- `xml_pipeline_run/baseline_runs/GAObjectiveAlign_Mini_20260323_v3/`
