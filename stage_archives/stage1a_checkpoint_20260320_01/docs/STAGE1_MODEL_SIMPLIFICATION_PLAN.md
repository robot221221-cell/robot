# Stage-1 计划：V-HACD + Distance-Aware LOD（含 Priority/Horizon 扩展）

> 前置状态：Baseline 已完成（B0_Baseline_Run_v2，30/30）。

## 1. 目标

在不改变任务定义与调度参数的前提下，通过“异构几何包络 + 动态LOD策略”降低碰撞检测成本，并保持成功率与稳定性：

- 复杂末端使用 `V-HACD` 凸分解；
- 简单拓扑连杆保留数学解析碰撞体（capsule/cylinder）；
- 先做 `Distance-Aware LOD`，再逐步加入 `Priority-Aware`、`Horizon-Aware`。

对照关系：
- 对照组：`B0_Baseline_Run_v2`
- 实验组：`Stage1_VHACD_LOD_Run`

## 2. 变更边界（严格控制）

只允许改：
- MuJoCo 碰撞几何描述（V-HACD 资产 + LOD 切换规则）
- 碰撞过滤配置（`exclude` / `contype` / `conaffinity`）
- 仅用于碰撞优化的调度预测层选择逻辑（不改任务策略本身）

禁止改：
- `GA` 分配与序列
- `Bi-RRT` 规划策略
- `TOPP-RA` 参数
- 调度器参数
- seed 集（仍为 42~51）

## 3. 分阶段执行（分阶段 + 强对照）

### Stage-1A：异构几何包络（仅几何，不加LOD）
- Base/Shoulder/UpperArm/Forearm：解析几何（capsule/cylinder）
- Wrist/Hand/Finger：V-HACD
- 输出批次：`Stage1A_Geom_Run`

### Stage-1B：Distance-Aware LOD
- 按双臂距离切换粗/细碰撞模型
- 阈值必须满足：
	$$D_{threshold} > 2V_{max}\Delta t + \epsilon$$
- 使用滞回：`D_in < D_out`
- 输出批次：`Stage1B_DistanceLOD_Run`

### Stage-1C：Priority-Aware LOD
- 高优先级臂保持高精度
- 低优先级在 `retreat/hold` 状态降级粗模型
- 输出批次：`Stage1C_PriorityLOD_Run`

### Stage-1D：Horizon-Aware LOD
- 近视野（1~10步）高精度
- 远视野（>50步）粗粒度趋势判断
- 输出批次：`Stage1D_HorizonLOD_Run`

每个子阶段都必须对照前一阶段与 `B0_Baseline_Run_v2`。

## 4. 实验配置

- 难度：easy / medium / hard
- seeds：42~51
- 批次数：30（与 baseline 完全一致）
- 输出目录：`xml_pipeline_run/baseline_runs/Stage1*_*/`

## 5. 严格过滤与对齐验证（必须先过）

1) 坐标对齐验证（Frame Alignment）
- 开启半透明碰撞体可视化（红色）逐一对齐检查
- 末端 V-HACD 子凸包与视觉 mesh 必须重合

2) 自碰撞过滤验证（Self-Collision Filtering）
- 明确配置 `exclude body1 body2`
- 或按 `contype/conaffinity` 分组，避免末端内部凸包互炸

3) 近场安全验证
- hard 场景下抽样回放，检查是否出现动力学发散/异常接触爆炸

## 6. 验证门槛

1) 正确性门槛：
- success_rate 不低于 baseline（至少保持 1.0）
- 不出现新型碰撞漏检（重点抽查 hard）

2) 效率门槛：
- `planning_wall_mean` 或 `preview_wall_mean` 至少一项显著下降
- `total_wall_mean` 下降，且 makespan 不显著恶化

3) 稳定性门槛：
- `deadlock_wait_mean` 不上升
- 指令跳变统计不恶化

## 7. 产出物

- `Stage1A_Geom_Run/tables/baseline_summary.csv`
- `Stage1B_DistanceLOD_Run/tables/baseline_summary.csv`
- `Stage1C_PriorityLOD_Run/tables/baseline_summary.csv`
- `Stage1D_HorizonLOD_Run/tables/baseline_summary.csv`
- `Stage1_vs_B0_comparison.md`（单独对照结论）

## 8. 本阶段执行清单

- [ ] 完成 Stage-1A 几何资产替换并通过对齐验证
- [ ] 完成 Stage-1A 冒烟测试（easy seed42）
- [ ] 完成 Stage-1A 全量 30 组
- [ ] 完成 Stage-1B（Distance-LOD）并全量 30 组
- [ ] 完成 Stage-1C（Priority-LOD）并全量 30 组
- [ ] 完成 Stage-1D（Horizon-LOD）并全量 30 组
- [ ] 汇总 `B0` 与 `Stage-1A/B/C/D` 对照表（Mean ± SD）
- [ ] 出具阶段结论：通过 / 不通过
