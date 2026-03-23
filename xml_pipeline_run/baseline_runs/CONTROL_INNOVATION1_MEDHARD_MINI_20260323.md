# Innovation-1（Priority-Aware Lazy Validation）验证：Medium+Hard mini（seeds=42/46/51）

## 目标
在已冻结 Stage-1C 配置下，验证 RRT 层惰性复核在 medium+hard mini 上是否带来稳定收益。

## 批次
- lazy off：`Innovation1_LazyVal_Off_MedHardMini_42_46_51_r2`
- lazy on：`Innovation1_LazyVal_On_MedHardMini_42_46_51_r2`

## 结果（summary.csv）

| split | variant | success_rate | makespan (s) | planning_wall (s) | preview_wall (s) | total_wall (s) |
|---|---|---:|---:|---:|---:|---:|
| medium | lazy off | 1.0 | 46.2167 | 4.1895 | 103.7394 | 110.8580 |
| medium | lazy on | 1.0 | 46.2167 | 4.1725 | 103.8620 | 111.0201 |
| hard | lazy off | 1.0 | 55.7847 | 4.8583 | 108.5636 | 116.8731 |
| hard | lazy on | 1.0 | 55.7847 | 4.7989 | 108.5407 | 116.7365 |

## 差值（lazy on - lazy off）

| split | makespan Δ (s) | planning_wall Δ (s) | total_wall Δ (s) |
|---|---:|---:|---:|
| medium | +0.0000 | -0.0170 | +0.1621 |
| hard | +0.0000 | -0.0593 | -0.1366 |

## 结论
1. medium 与 hard 上，lazy on/off 的 `makespan` 完全一致，`success_rate` 均为 1.0。
2. wall-time 变化仅为 ±0.1~0.2s 量级，未形成稳定单向收益。
3. 该小创新在当前实现下仍属于“可用但收益弱”，建议继续保持为可选分支，不并入主线默认配置。

## 产物路径
- `xml_pipeline_run/baseline_runs/Innovation1_LazyVal_Off_MedHardMini_42_46_51_r2/`
- `xml_pipeline_run/baseline_runs/Innovation1_LazyVal_On_MedHardMini_42_46_51_r2/`
