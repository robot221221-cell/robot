# README（现阶段实验速查）

> 说明：本文件是“现阶段 hard 场景实验”的精简记录，不替换原 README。

## 1) 当前目标
- 聚焦 hard 场景稳定执行。
- 指标优先级：`success` > `parallel_makespan_s` > `speedup_vs_serial` > `deadlock_wait_count`。

## 2) 已确认的最优配置（success-first）
- `sync_balance_weight = 0.3`
- `priority_arm = ur5e`
- `hold_steps = 80`
- `sim_steps_per_tick = 1`
- `velocity_scale = 0.8`
- `acceleration_scale = 0.7`
- `max_start_delay = 4000`
- `delay_coarse_step = 20`
- `anti_oscillation_trigger = 900`
- `anti_oscillation_hold = 80`
- `anti_oscillation_max_attempts = 3`
- `max_total_ticks = 120000`
- `max_no_progress_ticks = 5000`
- `retreat_cooldown_ticks = 16`
- `max_cmd_step = 0.02`
- `jump_step_threshold = 0.06`
- `jump_velocity_threshold = 25`
- 推荐可复现 seed：`46`

## 3) 最重要命令（直接可用）

### 3.1 最优配置可视化（hard，motionfix 版本）
```bash
conda run -n lerobot python schedule_ga_with_priority_delay.py \
  --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json \
  --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e \
  --velocity-scale 0.8 --acceleration-scale 0.7 \
  --max-start-delay 4000 --delay-coarse-step 20 \
  --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 \
  --max-total-ticks 120000 --max-no-progress-ticks 5000 \
  --retreat-cooldown-ticks 16 --max-cmd-step 0.02 \
  --jump-step-threshold 0.06 --jump-velocity-threshold 25 \
  --sync-balance-weight 0.3 --random-seed 42 \
  --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_visual_motionfix.json
```

### 3.2 最优配置 headless 复现（hard，motionfix 版本）
```bash
conda run -n lerobot python schedule_ga_with_priority_delay.py \
  --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json \
  --headless \
  --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e \
  --velocity-scale 0.8 --acceleration-scale 0.7 \
  --max-start-delay 4000 --delay-coarse-step 20 \
  --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 \
  --max-total-ticks 120000 --max-no-progress-ticks 5000 \
  --retreat-cooldown-ticks 16 --max-cmd-step 0.02 \
  --jump-step-threshold 0.06 --jump-velocity-threshold 25 \
  --sync-balance-weight 0.3 --random-seed 42 \
  --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_headless_motionfix.json
```

### 3.3 同步/非同步对比（可选）
```bash
conda run -n lerobot python run_sync_comparison_experiments.py \
  --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json \
  --output-dir xml_pipeline_run/difficulty_experiments_v6/sync_comparison \
  --lambda-sync 0.3 --random-seed 42 --max-retries 2
```

## 4) 关键结果文件
- 最终配置：`xml_pipeline_run/difficulty_experiments_v6/final_best_config.json`
- 5-seed 复核汇总：`xml_pipeline_run/difficulty_experiments_v6/final_best_validation_5seeds/summary.json`
- 5-seed 复核明细：`xml_pipeline_run/difficulty_experiments_v6/final_best_validation_5seeds/trials.json`

## 5) 相关脚本
- 主调度：`schedule_ga_with_priority_delay.py`
- λ 搜索：`run_final_parameter_search.py`
- 固定 λ 后参数搜索：`run_fixed_lambda_param_search.py`
- 同步/非同步对比：`run_sync_comparison_experiments.py`
- 输出整理：`organize_outputs.py`
