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
