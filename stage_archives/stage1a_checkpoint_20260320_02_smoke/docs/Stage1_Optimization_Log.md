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

