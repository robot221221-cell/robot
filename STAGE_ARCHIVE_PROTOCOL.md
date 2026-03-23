# 分阶段归档协议（强制）

## 1. 归档触发条件
每完成一次“可复现单元”必须归档：
- 一次代码修改 + 一次验证通过
- 一个子阶段（如 Stage-1A）全量跑完

## 2. 归档目录规范
`stage_archives/<stage_name>_<yyyymmdd_hhmm>/`

建议结构：
- `docs/`：阶段计划、日志、对照结论
- `scripts/`：关键脚本快照
- `results/`：raw_stats / logs / tables
- `configs/`：固定配置、seed、solution
- `manifest/`：提交哈希与文件清单

## 3. 最小归档清单
- Git commit hash（必须）
- 运行命令（必须）
- 参数快照（必须）
- 结果 summary（必须）
- 对照结论（必须）

## 4. 对照约束
- 任何 Stage-1 子阶段必须对照：
  1) `B0_Baseline_Run_v2`
  2) 前一子阶段（A→B→C→D）

## 5. 命名规则
- Stage-1A：`Stage1A_Geom_Run`
- Stage-1B：`Stage1B_DistanceLOD_Run`
- Stage-1C：`Stage1C_PriorityLOD_Run`
- Stage-1D：`Stage1D_HorizonLOD_Run`
