# Innovation-2：严格 GA on/off（Medium+Easy full30）

## 本次目标
在严格 GA-off（`sequence-mode=no_evo`）口径下，将创新2验证从 hard 扩展到 medium+easy full30（seed 42~51）。

## 批次
- GA on：`Innovation2_GAExplicit_On_MediumEasyFull30_42_51_r1`
- GA off（严格 no_evo）：`Innovation2_GAOffNoEvo_MediumEasyFull30_42_51_r1`

固定设置：Stage-1C（Distance-LOD + Priority-LOD）+ with auto-delay，参数与 hard 批次保持一致。

## 结果（summary.csv）

| split | variant | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| easy | GA on (`ga`) | 1.0 | 23.2978 | 1.9616 | 0.2100 | 56.1341 | 60.9661 |
| easy | GA off (`no_evo`) | 1.0 | 23.6170 | 1.9516 | 0.2100 | 52.3023 | 57.2111 |
| medium | GA on (`ga`) | 1.0 | 45.6514 | 1.7498 | 1.1566 | 99.2911 | 106.1977 |
| medium | GA off (`no_evo`) | 1.0 | 42.9868 | 1.7921 | 0.2100 | 50.3299 | 56.6753 |

## 差异（no_evo - GA on）

| split | makespan Δ (s) | makespan Δ (%) | preview_wall Δ (s) | total_wall Δ (s) |
|---|---:|---:|---:|---:|
| easy | +0.3192 | +1.37% | -3.8318 | -3.7550 |
| medium | -2.6646 | -5.84% | -48.9613 | -49.5223 |

## 观察
1. easy：GA on 在质量上略优（makespan 更低），但 no_evo 仍保持较低 wall-time。
2. medium：no_evo 同时取得更低 makespan 与更低 wall-time。
3. 说明创新2（GA）在跨难度上并非单调优势：在 hard 上更能体现质量收益，在 medium/easy 上需要进一步解释“GA目标函数 vs 调度执行目标”的偏差。

## 产物路径
- `xml_pipeline_run/baseline_runs/Innovation2_GAExplicit_On_MediumEasyFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAOffNoEvo_MediumEasyFull30_42_51_r1/`
