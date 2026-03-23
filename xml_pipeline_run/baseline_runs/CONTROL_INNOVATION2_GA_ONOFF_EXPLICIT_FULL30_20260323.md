# Innovation-2：显式 GA on/off（Hard full30）

## 本次目标
在与 mini 相同口径下，将显式 `sequence-mode` 对照扩展到 hard full30（seed 42~51），验证趋势是否稳定。

## 批次
- GA on（显式）：`Innovation2_GAExplicit_On_HardFull30_42_51_r1`
- GA off（严格 no_evo）：`Innovation2_GAOffNoEvo_HardFull30_42_51_r1`
- GA off（代理 random，补充参考）：`Innovation2_GAOffRandom_HardFull30_42_51_r1`

固定设置：Stage-1C（Distance-LOD + Priority-LOD）+ with auto-delay，其他参数一致。

## 结果（summary.csv 原始口径，主对照 GA vs no_evo）

| variant | n | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | planning_wall_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| GA on (`sequence-mode=ga`) | 10 | 1.0 | 55.7914 | 1.7280 | 1.5454 | 5.0145 | 107.7949 | 116.2199 |
| GA off (`sequence-mode=no_evo`) | 10 | 1.0 | 58.2008 | 1.7476 | 1.0986 | 5.0280 | 72.8991 | 81.2361 |

## 补充参考（GA vs random 代理）

| variant | n | success_rate | parallel_makespan_mean (s) | speedup_mean | wait_time_mean (s) | preview_wall_mean (s) | total_wall_mean (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| GA on (`sequence-mode=ga`) | 10 | 1.0 | 55.7914 | 1.7280 | 1.5454 | 107.7949 | 116.2199 |
| GA off (`sequence-mode=random`) | 10 | 0.9 | 58.1927 | 1.5534 | 1.3988 | 102.7529 | 111.1348 |

## 观察
1. **严格 GA-off 已落地**：`no_evo` 采用“同一任务多重集 + pairwise 成本矩阵 + 无进化贪心重排”，不使用进化迭代。
2. **质量结论（主结论）**：在 hard full30，GA on 与 no_evo 均 `success_rate=1.0`，但 GA on 的 `parallel_makespan_mean` 更优（55.7914s vs 58.2008s，约 4.3%）。
3. **wall-time 现象（需谨慎）**：no_evo 的 `preview/total wall` 显著更低，说明当前实现下它生成了“更易规划但并行质量更差”的序列；这与创新2主张（质量优先）不冲突。
4. **定位**：创新2在 hard full30 的“GA 改善并行质量”证据已形成；下一步应扩展到 medium/easy full30 做跨难度稳定性验证。

## 产物路径
- `xml_pipeline_run/baseline_runs/Innovation2_GAExplicit_On_HardFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAOffNoEvo_HardFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAOffRandom_HardFull30_42_51_r1/`

## 扩展说明
- medium/easy full30 严格对照见：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_MEDIUM_EASY_FULL30_20260323.md`
- 最终归档汇总见：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_FINAL_ARCHIVE_20260323.md`
