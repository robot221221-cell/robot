# Freeze Cleanup Record (2026-03-22)

## 冻结目标
- 主方案冻结：Stage-1C Opt-v3
- 优先级结论冻结：`priority_arm=ur5e`（默认）与 `priority_arm=fr3`（对照）
- Stage-1D 结论：当前任务分布下负收益探索（不纳入主线）

## 保留目录（核心证据）
- `xml_pipeline_run/baseline_runs/B0_Baseline_Run_v2/`
- `xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_Run/`
- `xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_PriorityFlip_Run/`
- `xml_pipeline_run/baseline_runs/Stage1C_Opt_v3_HardMini_42_46/`

## 删除目录（中间探索 / 已失效）
- `xml_pipeline_run/baseline_runs/Stage1C_PriorityLOD_Smoke/`
- `xml_pipeline_run/baseline_runs/Stage1C_PriorityLOD_HardProbe/`
- `xml_pipeline_run/baseline_runs/Stage1C_PriorityLOD_Smoke_v2Relax/`
- `xml_pipeline_run/baseline_runs/Stage1C_PriorityLOD_HardProbe_v2Relax/`
- `xml_pipeline_run/baseline_runs/Stage1D_HorizonLOD_Smoke/`
- `xml_pipeline_run/baseline_runs/Stage1D_HorizonLOD_HardProbe/`
- `xml_pipeline_run/baseline_runs/Stage1D_Tune_SetA_HardMini_42_46/`
- `xml_pipeline_run/baseline_runs/Stage1D_Tune_SetB_HardMini_42_46/`
- `xml_pipeline_run/baseline_runs/Stage1D_Tune_SetC_HardMini_42_46/`

## 说明
- 删除对象均为 smoke/probe/tuning 中间结果，结论已固化到：
  - `STAGE_PROGRESS_LOG.md`
  - `README_CURRENT_STAGE.md`
  - `xml_pipeline_run/difficulty_experiments_v6/final_best_config.json`
