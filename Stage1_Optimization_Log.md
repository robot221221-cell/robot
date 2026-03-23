# Stage1 Optimization Log

## 2026-03-20 Stage-1A Checkpoint 01

### 目标
- 仅针对复杂末端执行 V-HACD：
  - UR5e: `wrist3.obj`
  - FR3: `link7.obj`
  - 额外候选 `hand.obj` / `finger.obj` 当前仓库不存在（跳过并记录）

### V-HACD 参数（PyBullet）
- `resolution=1000000`
- `concavity=0.0025`
- `maxNumVerticesPerCH=64`
- `minVolumePerCH=1e-5`

### 生成产物
- `universal_robots_ur5e/assets/wrist3_vhacd.obj`
- `franka_fr3/assets/link7_vhacd.obj`
- 报告：`artifacts/stage1a/vhacd_report.json`

### 对齐验证
- 可视化测试场景：`stage1a_alignment_view.xml`
- 数值验证脚本：`stage1a_alignment_validate.py`
- 对齐指标：`artifacts/stage1a/alignment_metrics.json`

结论（补偿前 vs 补偿后）：
- UR5e wrist3: 原始偏差 `pos=0.042345m`, `quat=1.588266rad`；补偿后 `pos≈0`, `quat=0`
- FR3 link7: 原始偏差 `pos=0.064118m`, `quat=0.950204rad`；补偿后 `pos=0`, `quat=0`

### 接入状态
- 已将 `wrist3_vhacd`、`link7_vhacd` 接入 `dual_arm_scene.xml`
- 已应用几何补偿（来自对齐指标）

### 可视化截图清单（待补充）
- [ ] UR5e wrist3 蓝色原始网格 + 红色 V-HACD 叠加图
- [ ] FR3 link7 蓝色原始网格 + 红色 V-HACD 叠加图
- [ ] 双末端同框叠加图

> 建议截图操作：打开 `stage1a_alignment_view.xml`，在 viewer 中开启碰撞几何显示并截取三张图，保存到 `artifacts/stage1a/screenshots/`。

## 2026-03-20 Stage-1A Checkpoint 02（Smoke）

### 冒烟运行
- 批次：`Stage1A_Geom_Smoke`
- 命令：`conda run -n lerobot python run_baseline_batch.py --batch-name Stage1A_Geom_Smoke --difficulties easy --seeds 42`
- 输出：`xml_pipeline_run/baseline_runs/Stage1A_Geom_Smoke/`

### 冒烟结果（easy, seed=42）
- success: `1`
- parallel_makespan_s: `23.312`
- speedup_vs_serial: `1.9684`
- planning_wall_time_s: `3.0123`
- scheduling_preview_wall_time_s: `143.4289`
- total_wall_time_s: `148.2753`

### 与基线同点对照（B0_Baseline_Run_v2 easy seed42）
- baseline parallel_makespan_s: `23.276`
- smoke parallel_makespan_s: `23.312`（+0.036s）
- baseline speedup: `1.9706`
- smoke speedup: `1.9684`（-0.0022）

### 结论
- 冒烟通过，功能正确性与可执行性通过。
- 性能端初看与 baseline 基本同量级，尚未体现加速（符合 Stage-1A 仅做末端重构的预期）。

## 2026-03-20 Stage-1A Checkpoint 03（Full 30-Run）

### 全量运行
- 批次：`Stage1A_Geom_Run`
- 规模：`easy/medium/hard × seed 42~51 = 30`
- 输出：`xml_pipeline_run/baseline_runs/Stage1A_Geom_Run/`

### 结果摘要（对照 B0_Baseline_Run_v2）
- 成功率：三档均 `1.0`
- `parallel_makespan_mean`：变化很小（约 `+0.15% ~ +0.25%`）
- `speedup_mean`：变化很小（约 `-0.08% ~ -0.55%`）
- `total_wall_mean`：上升明显
  - easy: `+19.79%`
  - medium: `+18.28%`
  - hard: `+17.02%`

### 结论
- Stage-1A 在正确性上通过（无成功率退化），但效率门槛未达。
- 进入 Stage-1B（Distance-Aware LOD）以降低全程高精度碰撞带来的墙钟开销。

## 2026-03-20 Stage-1B Checkpoint 01（Distance-LOD Smoke）

### 代码变更
- `schedule_ga_with_priority_delay.py`
  - 新增参数：`--enable-distance-lod`、`--lod-distance-in`、`--lod-distance-out`
  - 在 `PriorityDelayScheduler.will_conflict()` 增加双臂末端距离门控与滞回切换：
    - `distance <= D_in`：精细碰撞模式
    - `distance >= D_out`：粗模式（跳过双臂精细碰撞检测）
    - 且约束 `D_in < D_out`
- `run_baseline_batch.py`
  - 增加 Stage-1B 参数并透传到子进程

### 冒烟运行
- 批次：`Stage1B_DistanceLOD_Smoke`
- 命令：`conda run -n lerobot python run_baseline_batch.py --batch-name Stage1B_DistanceLOD_Smoke --difficulties easy --seeds 42 --enable-distance-lod --lod-distance-in 0.28 --lod-distance-out 0.35`

### 冒烟结果（easy, seed=42）
- success: `1`
- parallel_makespan_s: `23.312`
- speedup_vs_serial: `1.9684`
- planning_wall_time_s: `3.1249`
- scheduling_preview_wall_time_s: `147.1283`
- total_wall_time_s: `152.0616`

### 结论
- Stage-1B 代码路径已接通，功能可运行。
- 当前 smoke 点位未体现效率收益，需执行 Stage-1B 全量 30 组并进行阈值敏感性复核。

## 2026-03-20 Stage-1B Checkpoint 02（Full 30-Run）

### 全量运行
- 批次：`Stage1B_DistanceLOD_Run`
- 规模：`easy/medium/hard × seed 42~51 = 30`
- 参数：`enable_distance_lod=true, lod_distance_in=0.28, lod_distance_out=0.35`

### 结果摘要（对照 B0_Baseline_Run_v2）
- 成功率：三档均 `1.0`
- `parallel_makespan_mean`：变化很小（约 `+0.15% ~ +0.25%`）
- `speedup_mean`：变化很小（约 `-0.08% ~ -0.55%`）
- `preview_wall_mean`：显著上升（约 `+26% ~ +29%`）
- `total_wall_mean`：显著上升（约 `+25% ~ +29%`）

### 结论
- 正确性通过，但效率门槛不通过。
- 当前 Distance-LOD 实现与阈值下，未降低冲突检测开销，反而增加调度预览开销。

## 2026-03-21 Stage-1C Checkpoint 01（Priority-LOD Kickoff）

### 代码变更
- `schedule_ga_with_priority_delay.py`
  - 新增参数：`--enable-priority-lod`
  - 在冲突预测中加入 Priority-Aware LOD 选择逻辑（高优先级相关推进保持精细检测，低优先级单臂推进/回退允许粗粒度）
- `run_baseline_batch.py`
  - 新增并透传参数：`--enable-priority-lod`

### 验证
- 冒烟：`Stage1C_PriorityLOD_Smoke`（easy seed42）成功
- hard 探针：`Stage1C_PriorityLOD_HardProbe`（hard seed42）成功
- 但两次 `lod_stats.coarse_skips=0`，旁路率为 0%，表现出策略过保守

### 对照结论
- 功能可运行，但效率未达标：hard 探针下 `preview_wall_time_s`、`total_wall_time_s` 均劣于 Stage-1B v2。
- 下一步应放宽低优先级 hold/等待状态下的降级触发条件，再复测 smoke 后决定是否进全量。

