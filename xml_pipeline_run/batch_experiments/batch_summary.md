# 批量调度实验汇总报告

| 场景 | 方法 | 重复次数 | 成功率 | 并行时长均值(s) | 加速比均值 | 总等待时间均值(s) | 死锁等待均值 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| S1_hold20_sim1_priour5e | AutoDelay关闭(预览) | 2 | 0.000 | - | 0.000 | 0.000 | 201.000 |
| S1_hold20_sim1_priour5e | AutoDelay开启(预览) | 2 | 1.000 | 11.428 | 1.793 | 0.020 | 0.000 |
| S1_hold20_sim1_priour5e | 实际执行 | 2 | 1.000 | 11.428 | 1.793 | 0.020 | 0.000 |
| S2_hold80_sim1_priour5e | AutoDelay关闭(预览) | 2 | 0.000 | - | 0.000 | 0.000 | 201.000 |
| S2_hold80_sim1_priour5e | AutoDelay开启(预览) | 2 | 1.000 | 11.908 | 1.791 | 0.020 | 0.000 |
| S2_hold80_sim1_priour5e | 实际执行 | 2 | 1.000 | 11.908 | 1.791 | 0.020 | 0.000 |
| S3_hold160_sim1_priour5e | AutoDelay关闭(预览) | 2 | 0.000 | - | 0.000 | 0.000 | 201.000 |
| S3_hold160_sim1_priour5e | AutoDelay开启(预览) | 2 | 1.000 | 12.548 | 1.789 | 0.020 | 0.000 |
| S3_hold160_sim1_priour5e | 实际执行 | 2 | 1.000 | 12.548 | 1.789 | 0.020 | 0.000 |

## 建议固定评价指标

1. 成功率 success_rate
2. 并行总时长 parallel_makespan_mean_s
3. 相对串行加速比 speedup_mean
4. 总等待时间 wait_time_total_mean_s
5. 死锁等待次数 deadlock_wait_mean
