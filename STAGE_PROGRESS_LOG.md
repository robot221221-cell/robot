# 阶段执行与归档日志

> 规则：每次“修改-验证-归档”完成后追加一条记录。

## 记录模板
- 时间：YYYY-MM-DD HH:MM
- 阶段：Stage-1A / 1B / 1C / 1D
- 变更：
- 验证：
- 结果：通过 / 不通过
- 产物路径：
- 对照结论：

---

## 已记录

- 时间：2026-03-20
- 阶段：Baseline 完成收尾
- 变更：完成 B0_Baseline_Run_v2 全量 30 组统计与归档
- 验证：检查 raw_stats=30、tables 汇总完整、success_rate 全 1.0
- 结果：通过
- 产物路径：
  - `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/tables/`
  - `stage_archives/baseline_stage_20260320/`
- 对照结论：作为 Stage-1 全部子阶段统一对照基线

- 时间：2026-03-20
- 阶段：Stage-1A（Checkpoint-01）
- 变更：新增 `stage1a_generate_vhacd.py`，生成并接入末端 `wrist3_vhacd.obj`、`link7_vhacd.obj`；新增 `stage1a_alignment_view.xml` 和 `stage1a_alignment_validate.py`
- 验证：
  - `vhacd_report.json` 显示复杂末端 2/2 生成成功，`hand.obj/finger.obj` 缺失已登记
  - `alignment_metrics.json` 补偿后残差为 0（`all_pass=true`）
  - `dual_arm_scene.xml` 可加载（`mujoco.MjModel.from_xml_path` 通过）
- 结果：通过
- 产物路径：
  - `artifacts/stage1a/vhacd_report.json`
  - `artifacts/stage1a/alignment_metrics.json`
  - `Stage1_Optimization_Log.md`
  - `stage_archives/stage1a_checkpoint_20260320_01/`
- 对照结论：完成 Stage-1A 首次“资产生成+对齐验证”门槛，下一步进入可视化截图归档与冒烟测试
