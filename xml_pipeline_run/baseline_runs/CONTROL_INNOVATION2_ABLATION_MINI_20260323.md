# Innovation-2 Ablation（Hard mini, seeds=42/46）

## 目标
验证“创新2（GA + Priority Delay + Stage-1C 推演加速）”的作用链条，并先用 hard mini 快速闭环。

## 口径与批次
- 难度/seed：hard, {42,46}
- 统一调度脚本：`schedule_ga_with_priority_delay.py`
- 关键开关：
  - Stage-1C on/off（`--enable-distance-lod --enable-priority-lod ...`）
  - delay on/off（`--disable-auto-delay`）

对应批次：
- GA on, Stage-1C on, delay on：`Stage1C_Opt_v3_HardMini_42_46`
- GA on, Stage-1C off, delay on：`Ablation_GAon_Stage0_DelayOn_HardMini_42_46_r1`
- GA on, Stage-1C on, delay off：`Ablation_GAon_Stage1_DelayOff_HardMini_42_46_r1`
- GA on, Stage-1C off, delay off：`Ablation_GAon_Stage0_DelayOff_HardMini_42_46_r1`

GA-off 代理对照（用于短周期先验比较）：
- Ctrl-A 串行：`CtrlA_Serial_HardMini_42_46`
- Ctrl-B 启发式：`CtrlB_Heuristic_HardMini_42_46_v2`
- Ctrl-C 反应式：`CtrlC_Reactive_HardMini_42_46`

## 结果表（GA on 的 2×2）

| GA | Stage-1C | auto-delay | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---|---|---:|---:|---:|---:|---:|---:|
| on | on  | on  | 1.0 | 55.7750 | 1.7282 | 1.5290 | 106.8803 | 115.0188 |
| on | off | on  | 1.0 | 55.7750 | 1.7364 | 1.0730 | 226.8829 | 235.7322 |
| on | on  | off | 0.0 | nan     | 0.0000 | 0.8680 | 0.0000   | 6.0750   |
| on | off | off | 1.0 | 56.2330 | 1.7025 | 1.7650 | 0.0000   | 8.8580   |

## 关键观察
1. **Stage-1C 的计算加速在 delay=on 条件下非常明确**：
   - `preview_wall_mean`：226.88s → 106.88s（约 **-52.9%**）
   - `total_wall_mean`：235.73s → 115.02s（约 **-51.2%**）
   - 同时 `parallel_makespan_mean` 保持 55.775s 不变（任务质量未损失）。
2. **delay 是关键组成**：在 Stage-1C on 条件下关闭 auto-delay，hard mini 直接失败（success_rate=0.0）。
3. **交互效应可见**：Stage-1C 不是“独立万能开关”，需要与 delay 联用才能稳定兑现质量与效率。

## GA-off 代理对照（短周期）

| method (proxy) | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|
| Ctrl-A Serial | 1.0 | 95.7370 | 2.0000 | 0.0000 | 11.5732 |
| Ctrl-B Heuristic | 0.0 | nan | 0.0000 | 0.0000 | 5.0177 |
| Ctrl-C Reactive | 1.0 | 56.2330 | 1.7025 | 1.7650 | 9.0485 |

说明：当前代码主干没有显式 `--disable-ga-search`（序列来自预生成 solution），故 GA-off 先以 A/B/C 代理法做短周期对照；完整“GA on/off 机制因果”将在下一阶段补专门开关与同预算实验。

## 结论（当前可写入）
- 在 hard mini 上，创新2中“Stage-1C 推演加速 + Priority Delay”的联合作用已被直接观测：
  - 质量不降（成功率、makespan 保持），
  - 计算墙钟显著下降（约 50%+）。
- 下一步需补齐“显式 GA on/off”实验，完成创新2机制因果闭环。
