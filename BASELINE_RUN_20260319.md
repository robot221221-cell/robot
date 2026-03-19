# Baseline 运行记录（2026-03-19）

> 本文件由模板 `BASELINE_EXECUTION_CHECKLIST_TEMPLATE.md` 实例化，用于本次正式基线批次。

## A. 实验批次信息

- [x] 批次名称：`b0_system_20260319_motionfix_seed42_51`
- [x] 执行日期：`2026-03-19`
- [x] 执行人：`jiangshuo`
- [x] 机器/系统：`Linux`
- [x] Python 环境：`lerobot`
- [x] Git commit：`1c6b639`（当前本地基线提交）
- [x] 目标定义：`B0-System`

## B. 冻结项检查（不通过不得开跑）

- [x] 已冻结代码版本（commit 已记录）
- [x] 已冻结参数文件（见 `xml_pipeline_run/baseline_runs/b0_system_20260319_seed42_51/configs/baseline_config_snapshot.json`）
- [x] 已冻结场景集（easy/medium/hard）
- [x] 已冻结 seeds（每个难度 10 个：42~51）
- [x] 已生成并冻结 GA 最优解（每个场景固定 solution JSON，后续 RRT/调度实验必须直接读取）
- [x] 已记录固定解路径（easy/medium/hard）：
  - [x] easy：`xml_pipeline_run/difficulty_experiments_v6/solution_easy.json`
  - [x] medium：`xml_pipeline_run/difficulty_experiments_v6/solution_medium.json`
  - [x] hard：`xml_pipeline_run/difficulty_experiments_v6/solution_hard.json`
- [x] 已确认本轮不引入任何新优化
- [x] 输出目录已创建：`xml_pipeline_run/baseline_runs/b0_system_20260319_motionfix_seed42_51/`

## C. 预运行健康检查

- [x] 环境检查通过（`conda run -n lerobot ...` 可执行）
- [x] 关键脚本可执行（`run_baseline_batch.py` / `schedule_ga_with_priority_delay.py`）
- [x] 关键输入文件存在（solution/config/cost matrix）

## D. Baseline 主实验（执行中）

- [ ] easy 已运行（固定 GA 解 + 多 seed）
- [ ] medium 已运行（固定 GA 解 + 多 seed）
- [ ] hard 已运行（固定 GA 解 + 多 seed）

执行命令：

```bash
conda run -n lerobot python run_baseline_batch.py --batch-name b0_system_20260319_seed42_51
```

```bash
conda run -n lerobot python run_baseline_batch.py --batch-name b0_system_20260319_motionfix_seed42_51
```

- [x] 已启动正式批量运行（当前执行中）
- [x] 已中止旧批次并重启（原因：修复“瞬移/抽搐”根因后需保证数据同版本可比）

运行输出位置：
- `xml_pipeline_run/baseline_runs/b0_system_20260319_motionfix_seed42_51/raw_stats/`
- `xml_pipeline_run/baseline_runs/b0_system_20260319_motionfix_seed42_51/logs/`
- `xml_pipeline_run/baseline_runs/b0_system_20260319_motionfix_seed42_51/tables/`

## E/F/G/H

待批次运行完成后回填。
