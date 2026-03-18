# 同步化 vs 非同步化 对比结果

- 同步化软目标权重 λ = 0.5
- 说明：同步化方法在自动延迟搜索时最小化 `makespan_ticks + λ * finish_gap_ticks`

| method | success | parallel_makespan_s | speedup_vs_serial | wait_time_total_s | finish_gap_s | overhead_vs_ideal_parallel_s |
|---|---|---|---|---|---|---|
| nonsync_efficiency | 0 | inf | 0.0 | 0.7000000000000001 | 0.0 | inf |
| sync_soft_lambda_0.5 | 1 | 56.932 | 1.7434131946884002 | 0.066 | 0.0 | 0.0660000000000025 |
