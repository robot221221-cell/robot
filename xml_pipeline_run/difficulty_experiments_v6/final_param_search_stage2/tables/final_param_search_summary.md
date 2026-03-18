# hard 场景最终参数搜索（成功率优先）

- solution: xml_pipeline_run/difficulty_experiments_v6/solution_hard.json
- seeds: [42, 43, 44]
- 排序规则：success_rate 降序 -> avg_parallel_makespan 升序 -> avg_speedup 降序 -> avg_deadlock_wait_count 升序

| lambda | trials | success_rate | avg_parallel_makespan_s_on_success | p95_parallel_makespan_s_on_success | avg_speedup_on_success | avg_deadlock_wait_count |
|---|---|---|---|---|---|---|
| 0.3 | 3 | 1.0 | 57.398 | 59.108000000000004 | 1.7289687365836592 | 0.0 |
| 0.5 | 3 | 1.0 | 57.398 | 59.108000000000004 | 1.7289687365836592 | 0.0 |
| 0.7 | 3 | 1.0 | 57.398 | 59.108000000000004 | 1.7289687365836592 | 0.0 |
