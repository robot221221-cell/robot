# Baseline 运行记录（2026-03-19）

> 本文件由模板 `BASELINE_EXECUTION_CHECKLIST_TEMPLATE.md` 实例化，用于本次正式基线批次。

## A. 实验批次信息

- [x] 批次名称：`B0_Baseline_Run_v2`
- [x] 执行日期：`2026-03-19`
- [x] 执行人：`jiangshuo`
- [x] 机器/系统：`Linux`
- [x] Python 环境：`lerobot`
- [x] Git commit：`fbe12c9`（Code Freeze：motionfix 基线）
- [x] 目标定义：`B0-System`

## B. 冻结项检查（不通过不得开跑）

- [x] 已冻结代码版本（commit 已记录）
- [x] 已冻结参数文件（见 `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/configs/baseline_config_snapshot.json`）
- [x] 已冻结场景集（easy/medium/hard）
- [x] 已冻结 seeds（每个难度 10 个：42~51）
- [x] 已生成并冻结 GA 最优解（每个场景固定 solution JSON，后续 RRT/调度实验必须直接读取）
- [x] 已记录固定解路径（easy/medium/hard）：
  - [x] easy：`xml_pipeline_run/difficulty_experiments_v6/solution_easy.json`
  - [x] medium：`xml_pipeline_run/difficulty_experiments_v6/solution_medium.json`
  - [x] hard：`xml_pipeline_run/difficulty_experiments_v6/solution_hard.json`
- [x] 已确认本轮不引入任何新优化
- [x] 输出目录已创建：`xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/`
- [x] 已落实新增指标：`planning_time_s` / `scheduling_time_s` 分离统计
- [x] 已落实新增指标：`rrt_nodes` / `rrt_iterations` 统计（UR5e/FR3）

## C. 预运行健康检查

- [x] 环境检查通过（`conda run -n lerobot ...` 可执行）
- [x] 关键脚本可执行（`run_baseline_batch.py` / `schedule_ga_with_priority_delay.py`）
- [x] 关键输入文件存在（solution/config/cost matrix）

## D. Baseline 主实验（已完成）

- [x] easy 已运行（固定 GA 解 + 多 seed）
- [x] medium 已运行（固定 GA 解 + 多 seed）
- [x] hard 已运行（固定 GA 解 + 多 seed）
- 当前进度（最终）：easy 10/10，medium 10/10，hard 10/10（总计 30/30）

执行命令：

```bash
conda run -n lerobot python run_baseline_batch.py --batch-name B0_Baseline_Run
```

```bash
conda run -n lerobot python run_baseline_batch.py --batch-name B0_Baseline_Run_v2
```

- [x] 已启动正式批量运行
- [x] 已中止旧批次并重启（原因：修复“瞬移/抽搐”根因后需保证数据同版本可比）
- [x] Step 1 Code Freeze 已完成（commit: `fbe12c9`）
- [x] Step 2 Checklist 已填写并开始按步骤打勾
- [x] Step 3 多难度多 seed 跑分已启动
- [x] 已记录一次中断：`KeyboardInterrupt` 发生在 auto-delay 预演阶段（非算法崩溃）
- [x] 已重新启动 `B0_Baseline_Run_v2` 继续全量执行

运行输出位置：
- `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/raw_stats/`
- `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/logs/`
- `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/tables/`

## E/F/G/H

- [x] 已完成数据归档：`raw_stats/`、`logs/`、`tables/` 完整生成。
- [x] 已生成汇总结果：
  - `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/tables/baseline_trials.csv`
  - `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/tables/baseline_summary.csv`
  - `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/tables/baseline_summary.md`
- [x] 当前批次进程已结束（无残留运行进程）。

### 最终关键结果（B0_Baseline_Run_v2）

- easy（n=10）：
  - success_rate = 1.000
  - makespan = 23.270 ± 0.020 s
  - speedup = 1.971 ± 0.001
- medium（n=10）：
  - success_rate = 1.000
  - makespan = 45.547 ± 0.738 s
  - speedup = 1.761 ± 0.012
- hard（n=10）：
  - success_rate = 1.000
  - makespan = 55.655 ± 0.091 s
  - speedup = 1.746 ± 0.007

### 阶段结论

- [x] Baseline 阶段通过（可复现、可对照、统计口径完整）。
- [x] 下一步进入 Stage-1：模型几何减负（胶囊体替换）对照实验准备。
- [x] 已完成阶段归档（按时间段保存）：`stage_archives/baseline_stage_20260320/`
- [x] 已切换为“分阶段 + 强对照 + 严格过滤与对齐验证”执行模式。
- [x] 已建立阶段日志：`STAGE_PROGRESS_LOG.md`
- [x] 已建立归档协议：`STAGE_ARCHIVE_PROTOCOL.md`

## 进展日志（持续更新）

- [x] 已确认可视化 motionfix 版本无瞬移/抽搐。
- [x] 已执行 Code Freeze（`fbe12c9`）。
- [x] 已停止旧批次 `B0_Baseline_Run`，避免口径混杂。
- [x] 已清理并重启 `B0_Baseline_Run_v2` 全量实验。
- [x] 当前 `run_easy_seed_42.log` 已开始写入，说明 v2 批次正在执行。
- [x] 已确认当前批次仍在推进，待生成 `raw_stats/*.json` 后持续更新进度。
- [x] 2026-03-20 已再次确认并重启 `B0_Baseline_Run_v2`，当前从 easy/seed42 开始执行。
