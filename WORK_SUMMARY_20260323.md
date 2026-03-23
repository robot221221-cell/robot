# 工作总结（2026-03-23）

## 1. 总体目标与阶段结论
本阶段围绕双臂调度主线 Stage-1C（Distance-LOD + Priority-LOD + auto-delay）完成了：
- 主对照体系整理（A/B/C vs Ours）
- Innovation-2（GA + Priority Delay）严格因果验证
- 增量小优化（机制扫参、GA seeding、GA目标函数对齐、RRT lazy复核）
- 结果归档、文档收口、仓库同步

最终主结论：
- Innovation-2 的收益呈“难度相关”而非“全局单调”
- hard 场景下 GA 对并行质量（makespan）有显著收益
- medium 场景下 no_evo 更优，easy 差异不显著

## 2. 已完成实验与产物

### 2.1 主线与严格对照
- hard full30：GA vs strict no_evo
- medium+easy full30：GA vs strict no_evo
- paired 显著性检验（sign test + bootstrap CI）

主要文档：
- xml_pipeline_run/baseline_runs/CONTROL_PAPER_MAIN_TABLE_20260322.md
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_FINAL_ARCHIVE_20260323.md
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.json

### 2.2 medium 机制解释
- sync_balance_weight 小扫参（0.0 / 0.3 / 0.5）
- 结论：仅调该参数不能消除 medium 反转

文档：
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_MECHANISM_SWEEP_MEDIUM_MINI_20260323.md

### 2.3 增量小优化
1) GA seeding（启发式种子注入）
- 已实现开关并完成 mini 消融
- 结论：收敛代数略有提前，收益小，作为可选增强保留

文档：
- xml_pipeline_run/baseline_runs/CONTROL_GA_SEEDING_MINI_20260323.md

2) GA 目标函数对齐（makespan/wait 代理主导）
- 已实现并 mini 验证
- 结论：未扭转 medium 反转，不纳入主线默认

文档：
- xml_pipeline_run/baseline_runs/CONTROL_GA_OBJECTIVE_ALIGNMENT_MINI_20260323.md

3) Innovation-1（RRT Priority-Aware Lazy Validation）
- hard mini 与 med+hard mini 复验
- 结论：功能可用但收益弱，不扩 full30

文档：
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION1_MINI_20260323.md
- xml_pipeline_run/baseline_runs/CONTROL_INNOVATION1_MEDHARD_MINI_20260323.md

## 3. 代码与工程改动摘要
- 调度与批跑：
  - schedule_ga_with_priority_delay.py
  - run_baseline_batch.py
- 路径规划：
  - rrt_planner.py
- GA：
  - ga_task_allocation.py（保留 seeding 开关；不采纳 objective-align 主线入口）
- 日志与阶段记录：
  - STAGE_PROGRESS_LOG.md

## 4. 最终收口决策
增量创新决策总表：
- xml_pipeline_run/baseline_runs/CONTROL_INCREMENTAL_INNOVATIONS_DECISION_20260323.md

采纳策略：
- 主线采纳：Innovation-2 严格结论（难度相关收益）
- 辅助证据：机制扫参与显著性统计
- 不并入主线默认：GA objective alignment、Innovation-1（当前收益弱）

## 5. 仓库状态
- 工作区已清理并提交
- 远程 main 分支已同步
- 当前可直接进入论文写作阶段
