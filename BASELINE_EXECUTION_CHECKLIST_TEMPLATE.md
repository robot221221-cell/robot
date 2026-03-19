# Baseline 实验可执行清单模板（可直接打勾执行）

> 用法：复制本模板作为一次正式实验批次的执行单，逐项打勾。
> 建议文件名：`BASELINE_RUN_YYYYMMDD.md`

---

## A. 实验批次信息

- [ ] 批次名称：`__________________________`
- [ ] 执行日期：`__________________________`
- [ ] 执行人：`__________________________`
- [ ] 机器/系统：`__________________________`
- [ ] Python 环境：`__________________________`（例如 `lerobot`）
- [ ] Git commit：`__________________________`
- [ ] 目标定义：`B0-Algorithm` / `B0-System`（二选一或都做）

---

## B. 冻结项检查（不通过不得开跑）

- [ ] 已冻结代码版本（commit 已记录）
- [ ] 已冻结参数文件（路径：`__________________`）
- [ ] 已冻结场景集（easy/medium/hard）
- [ ] 已冻结 seeds（每个难度至少 5~10 个，建议：`42,43,44,45,46,47,48,49,50,51`）
- [ ] 已生成并冻结 GA 最优解（每个场景固定 solution JSON，后续 RRT/调度实验必须直接读取）
- [ ] 已记录固定解路径（easy/medium/hard）：
  - [ ] easy：`__________________________`
  - [ ] medium：`__________________________`
  - [ ] hard：`__________________________`
- [ ] 已确认本轮不引入任何新优化
- [ ] 输出目录已创建：`xml_pipeline_run/baseline_runs/<batch_name>/`

---

## C. 预运行健康检查

- [ ] 环境检查通过
- [ ] 关键脚本可执行
- [ ] 关键输入文件存在（solution/config/cost matrix）

可用命令（按需执行）：

```bash
conda run -n lerobot python verify_setup.py
```

---

## D. Baseline 主实验（必做）

## D1. 难度实验（easy/medium/hard）

- [ ] easy 已运行（固定 GA 解 + 多 seed）
- [ ] medium 已运行（固定 GA 解 + 多 seed）
- [ ] hard 已运行（固定 GA 解 + 多 seed）

建议命令模板（按项目现有脚本调整）：

```bash
conda run -n lerobot python run_difficulty_experiments.py
```

> 若你当前以 `schedule_ga_with_priority_delay.py` 为主入口，也可写循环脚本对 easy/medium/hard + seed 集批量运行。

## D2. hard 场景重点复核（必做）

- [ ] hard 场景至少 10 seeds
- [ ] 每次运行均产出 `stats_json`
- [ ] 无手工挑选成功样本行为
- [ ] 同时记录失败样本原因（如 `ik_infeasible` / `planning_failed` / `deadlock_timeout`）

参考命令模板：

```bash
conda run -n lerobot python schedule_ga_with_priority_delay.py \
  --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json \
  --headless \
  --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e \
  --velocity-scale 0.8 --acceleration-scale 0.7 \
  --max-start-delay 4000 --delay-coarse-step 20 \
  --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 \
  --max-total-ticks 120000 --max-no-progress-ticks 5000 \
  --retreat-cooldown-ticks 16 --max-cmd-step 0.02 \
  --jump-step-threshold 0.06 --jump-velocity-threshold 25 \
  --sync-balance-weight 0.3 --random-seed <SEED> \
  --stats-json xml_pipeline_run/baseline_runs/<batch_name>/stats_hard_seed_<SEED>.json
```

---

## E. 指标采集清单（缺一不可）

## E1 主指标（success-first）
- [ ] `success`
- [ ] `parallel_makespan_s`
- [ ] `speedup_vs_serial`
- [ ] `deadlock_wait_count`

## E2 次指标
- [ ] `wait_time_total_s`
- [ ] `finish_gap_s`
- [ ] `jump_summary`
- [ ] 路径长度 / 拐点数量

## E3 性能瓶颈指标（本轮重点）
- [ ] 碰撞检测调用次数
- [ ] 单次检测耗时统计（均值/P95）
- [ ] 总检测耗时占比
- [ ] 规划耗时 `planning_time_s`（RRT 几何路径生成耗时）
- [ ] 调度推演耗时 `scheduling_time_s`（dry_run / auto-delay 前向推演耗时）
- [ ] 总耗时中规划占比、调度占比、碰撞检测占比
- [ ] RRT 节点数 `rrt_nodes`
- [ ] RRT 迭代次数/树深度 `rrt_iterations` / `rrt_tree_depth`

## E4 统计口径约束（防止结论漂移）
- [ ] 每个场景报告 `Mean ± SD`
- [ ] success 同时报告成功率（%）与样本数（n）
- [ ] 对比表必须使用同一 seed 集

---

## F. 数据归档清单（可复现要求）

- [ ] 原始逐次结果（json）
- [ ] 批量汇总表（csv）
- [ ] 人类可读汇总（md）
- [ ] 可视化图（png）
- [ ] 本轮命令清单（txt/md）
- [ ] 参数快照（json/yaml）
- [ ] seed 列表文件

目录建议：

```text
xml_pipeline_run/baseline_runs/<batch_name>/
  ├─ raw_stats/
  ├─ tables/
  ├─ figures/
  ├─ configs/
  ├─ commands/
  └─ summary.md
```

---

## G. 统计与结论（必须填写）

- [ ] 各场景 success rate：`__________________________`
- [ ] hard 场景 success rate：`__________________________`
- [ ] success 统计样本数 n：`__________________________`
- [ ] makespan 均值 ± 标准差：`__________________________`
- [ ] speedup 均值 ± 标准差：`__________________________`
- [ ] planning_time 均值 ± 标准差：`__________________________`
- [ ] scheduling_time 均值 ± 标准差：`__________________________`
- [ ] collision_check_time 占比：`__________________________`
- [ ] RRT 节点数/迭代次数统计：`__________________________`
- [ ] 死锁/等待问题摘要：`__________________________`
- [ ] 碰撞检测瓶颈结论：`__________________________`

本轮结论（3 句话以内）：
1. `____________________________________________________`
2. `____________________________________________________`
3. `____________________________________________________`

---

## H. Baseline 通过门槛（满足后才能进入优化）

- [ ] 同一配置可重复运行并复现实验趋势
- [ ] 主指标统计稳定（无异常漂移）
- [ ] 数据归档完整可追溯
- [ ] 已形成 baseline 对照报告

> 全部勾选后，才允许进入下一阶段（模型简化 / Lazy 检测 / RRT* / Informed / CCD）。
