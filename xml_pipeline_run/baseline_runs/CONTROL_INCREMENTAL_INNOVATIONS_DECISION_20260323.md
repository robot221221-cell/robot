# 增量创新优化：最终决策表（2026-03-23）

## 目标
对“可并入主线/仅保留归档”的小创新做最终收口决策，避免论文叙述与工程实现混淆。

## 决策总表

| 模块 | 改动 | 证据状态 | 结果 | 主线决策 |
|---|---|---|---|---|
| Innovation-2 主体 | GA + Priority Delay（严格 GA vs no_evo，full30 全难度 + 显著性） | 已完成 | hard 正向、medium 反向、easy不显著（难度相关） | **保留（论文主创新）** |
| 机制扫参 | `sync_balance_weight` in medium mini（0/0.3/0.5） | 已完成 | 无法消除 medium 反转 | **保留为机制解释证据** |
| GA seeding | 启发式种子注入（`--disable-greedy-seed` 对照） | 已完成 mini | 收敛代数略提前（2~3 代→2代），幅度小 | **保留为可选增强，不列主结论** |
| GA objective alignment | makespan/wait 代理主导适应度（mini） | 已完成 mini | 未扭转 medium，hard 仅部分改善 | **不并入主线（负结果归档）** |
| Innovation-1 | Priority-Aware Lazy Validation（med+hard mini 复验） | 已完成 | makespan 不变，wall-time 仅微小波动 | **不扩 full30，保留分支** |

## 最终采用口径（论文）
1. 主创新结论采用“**难度相关收益**”口径：GA 在 hard（高冲突）对并行质量更有优势。
2. 机制解释使用 `sync_balance_weight` 扫参与显著性报告支撑，不把 GA seeding / objective alignment 作为主贡献。
3. Innovation-1 以“可行但当前收益弱”作为负结果说明，避免过度主张。

## 对应报告索引
- 主创新严格对照归档：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_FINAL_ARCHIVE_20260323.md`
- 显著性统计：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md`
- 机制扫参：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_MECHANISM_SWEEP_MEDIUM_MINI_20260323.md`
- GA seeding mini：`xml_pipeline_run/baseline_runs/CONTROL_GA_SEEDING_MINI_20260323.md`
- GA objective alignment mini：`xml_pipeline_run/baseline_runs/CONTROL_GA_OBJECTIVE_ALIGNMENT_MINI_20260323.md`
- Innovation-1 med+hard mini：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION1_MEDHARD_MINI_20260323.md`
