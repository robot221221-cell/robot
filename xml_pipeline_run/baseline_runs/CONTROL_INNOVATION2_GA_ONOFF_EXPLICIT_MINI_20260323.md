# Innovation-2：显式 GA on/off（Hard mini）

## 本次目标
在 Stage-1C + with auto-delay 固定条件下，增加显式开关做 GA on/off 对照，并补充随机序列基线。

## 实现
- `schedule_ga_with_priority_delay.py` 新增参数：`--sequence-mode {ga, heuristic, random}`
  - `ga`：使用 solution 中的 `ur5e_sequence/fr3_sequence`
  - `heuristic`：对每臂序列做确定性字典序排序（保留重复项）
  - `random`：固定随机种子打乱每臂序列（保留重复项）
- `run_baseline_batch.py` 增加 `--sequence-mode` 透传。

## 批次
- GA on（显式）：`Innovation2_GAExplicit_On_HardMini_42_46_r1`
- GA off（显式 heuristic）：`Innovation2_GAExplicit_Off_HardMini_42_46_r1`
- GA off（显式 random）：`Innovation2_GAOffRandom_HardMini_42_46_r1`

## 结果

| variant | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | planning_wall_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| GA on (`sequence-mode=ga`) | 1.0 | 55.7750 | 1.7282 | 1.5290 | 4.5074 | 101.7199 | 109.4939 |
| GA off (`sequence-mode=heuristic`) | 1.0 | 49.8560 | 1.4856 | 5.3080 | 4.8791 | 95.6296 | 103.7939 |
| GA off (`sequence-mode=random`) | 1.0 | 55.9090 | 1.7617 | 1.0990 | 5.4756 | 102.6487 | 111.6083 |

## 解释与边界
1. `heuristic` 与 `random` 均属于 GA-off 代理定义（不做进化优化），用于打通显式开关实验管线。
2. `heuristic` 在当前任务多重集上出现异常偏优（可能由排序与重复任务结构耦合导致），可比性偏弱。
3. `random` 基线更稳健：相对 GA on，`parallel_makespan` 略差（55.909 vs 55.775），`total_wall` 略高（111.608 vs 109.494），方向上支持 GA 的收益但幅度不大。
4. 因此，本组可用于“显式 GA on/off 已接通 + 趋势初现”，但仍不作为论文最终因果证据。

## 下一步（用于论文闭环）
- 需要实现“严格 GA-off”定义：
  - 以同一任务集合与同一建模口径，构造“无进化迭代/仅初始解”的对照；
  - 保持与 GA on 同样的 Stage-1C、delay、随机种子与预算设置；
  - 再做 hard mini -> full30 扩展。
