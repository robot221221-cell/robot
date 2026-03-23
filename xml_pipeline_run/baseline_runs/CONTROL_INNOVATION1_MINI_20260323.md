# Innovation-1（Priority-Aware Lazy Validation）Mini 验证

## 实现范围（最小版）
- 文件：
  - `rrt_planner.py`
  - `schedule_ga_with_priority_delay.py`
  - `run_baseline_batch.py`
- 开关：`--enable-priority-lazy-validation`
- 机制：当**低优先级机械臂**执行 RRT 边验证时，将**高优先级机械臂**的复杂几何切换为胶囊包络（capsule）进行惰性复核，检查后恢复默认几何。

## 验证设置
- 难度/seed：hard mini（42,46）
- 统一使用 Stage-1C Opt-v3 + with auto-delay
- 对比批次：
  - lazy off：`Innovation1_LazyVal_Off_HardMini_42_46_r1`
  - lazy on：`Innovation1_LazyVal_On_HardMini_42_46_r1`

## 结果对比

| variant | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | planning_wall_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| lazy off | 1.0 | 55.7750 | 1.7282 | 1.5290 | 4.6864 | 108.2359 | 116.3536 |
| lazy on  | 1.0 | 55.7750 | 1.7282 | 1.5290 | 4.6592 | 106.4983 | 114.6965 |

补充（来自 raw_stats 中新增 RRT 统计）：
- `rrt_plan_wall_time_s_sum_ur5e`：0.0931s → 0.0941s（几乎不变）
- `rrt_plan_wall_time_s_sum_fr3`：0.0649s → 0.0645s（几乎不变）

## 结论
1. 正确性：通过（成功率不下降，makespan 不变）。
2. 增益：存在但较小（`preview_wall` 约 -1.6%，`total_wall` 约 -1.4%）。
3. 决策：当前 hard mini 未出现“RRT wall 显著下降”信号，**暂不扩 full30**；先保留为 Stage-2 分支，后续再扩大胶囊替代范围（不仅末端 link）并复测。
