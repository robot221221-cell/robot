# README（当前冻结方案速查）

> 说明：本文件用于冻结当前主方案与实验结论，不替换项目主 README。

## 1) 冻结结论（2026-03-22）
- 主方法：**Stage-1C Opt-v3**（已完成 full30，作为默认方案）
- 优先级：**可配置**（`priority_arm=ur5e` 与 `priority_arm=fr3` 均完成 full30）
- Stage-1D：标注为 **“当前任务分布下负收益探索”**，暂不纳入主线

## 2) 主方案参数（Stage-1C Opt-v3）
- `sync_balance_weight = 0.3`
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
- `enable_distance_lod = true`
- `enable_priority_lod = true`
- `lod_distance_in = 0.22`
- `lod_distance_out = 0.30`
- `lod_distance_out_both_move = 0.26`
- `lod_check_interval = 5`
- `priority_lod_static_step_threshold = 0.0015`

## 3) Full30 结果摘要（主线 + 优先级翻转）

### 3.1 Stage-1C Opt-v3（`priority_arm=ur5e`）
- easy: `success=1.0`, `parallel_makespan_mean=23.2978`, `total_wall_mean=60.5627`
- medium: `success=1.0`, `parallel_makespan_mean=45.6514`, `total_wall_mean=98.2462`
- hard: `success=1.0`, `parallel_makespan_mean=55.7914`, `total_wall_mean=103.2772`

### 3.2 Stage-1C Opt-v3 PriorityFlip（`priority_arm=fr3`）
- easy: `success=1.0`, `parallel_makespan_mean=26.2620`, `total_wall_mean=30.4367`
- medium: `success=1.0`, `parallel_makespan_mean=44.0420`, `total_wall_mean=101.3038`
- hard: `success=1.0`, `parallel_makespan_mean=53.6434`, `total_wall_mean=104.3613`

### 3.3 结论
- `ur5e` 优先：easy/hard 墙钟更稳（主线默认）
- `fr3` 优先：hard/medium 的 makespan 更优（用于论文敏感性分析）

## 4) Stage-1D 负收益结论（冻结）
- hard mini（seed=42/46）Set-A/B/C 都保持 `success=1.0`
- 但相较 1C Opt-v3 hard mini：
  - `preview_wall_mean` 由 `106.88` 上升到 `127.81~128.97`
  - `total_wall_mean` 由 `115.02` 上升到 `136.10~137.36`
- 结论：在当前任务分布下，Horizon-LOD 额外开销大于收益，作为负结果归档

## 5) 冻结复现命令（headless）

### 5.1 主线默认（`priority_arm=ur5e`）
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
  --enable-distance-lod --enable-priority-lod \
  --lod-distance-in 0.22 --lod-distance-out 0.30 --lod-distance-out-both-move 0.26 \
  --lod-check-interval 5 --priority-lod-static-step-threshold 0.0015
```

### 5.2 优先级翻转复现（`priority_arm=fr3`）
```bash
conda run -n lerobot python schedule_ga_with_priority_delay.py \
  --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json \
  --headless \
  --hold-steps 80 --sim-steps-per-tick 1 --priority-arm fr3 \
  --velocity-scale 0.8 --acceleration-scale 0.7 \
  --max-start-delay 4000 --delay-coarse-step 20 \
  --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 \
  --max-total-ticks 120000 --max-no-progress-ticks 5000 \
  --retreat-cooldown-ticks 16 --max-cmd-step 0.02 \
  --jump-step-threshold 0.06 --jump-velocity-threshold 25 \
  --sync-balance-weight 0.3 --random-seed 42 \
  --enable-distance-lod --enable-priority-lod \
  --lod-distance-in 0.22 --lod-distance-out 0.30 --lod-distance-out-both-move 0.26 \
  --lod-check-interval 5 --priority-lod-static-step-threshold 0.0015
```

## 6) 关键文件
- 冻结配置：`xml_pipeline_run/difficulty_experiments_v6/final_best_config.json`
- 阶段日志：`STAGE_PROGRESS_LOG.md`
- 清理记录：`CLEANUP_FREEZE_20260322.md`
