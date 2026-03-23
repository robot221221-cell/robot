# GA 启发式种子注入（Seeding）mini 消融（2026-03-23）

## 目标
验证“启发式基因注入（greedy seed）”是否能加速 GA 收敛。

## 实现
- 文件：`ga_task_allocation.py`
- 新增参数：`--disable-greedy-seed`
  - 默认（开）：首代注入 `greedy_seed()`，其余随机
  - 关闭：首代全部随机

## 实验设置
- matrix: `xml_pipeline_run/data/cost_matrices/pairwise_transition_cost_ur5e.csv` / `..._fr3.csv`
- population=80, generations=200, elite=6
- seeds: 42/46/51
- 指标：首次达到最终最优 fitness 的代数（`first_best_gen`）

## 结果

| mode | seed | best_fitness | first_best_gen |
|---|---:|---:|---:|
| greedy_seed_on | 42 | 38.54045 | 2 |
| greedy_seed_on | 46 | 38.54045 | 2 |
| greedy_seed_on | 51 | 38.54045 | 2 |
| greedy_seed_off | 42 | 38.54045 | 2 |
| greedy_seed_off | 46 | 38.54045 | 2 |
| greedy_seed_off | 51 | 38.54045 | 3 |

## 结论
- 在当前问题规模上，GA 本身已很快收敛（2~3 代）。
- seeding 的收益方向正确（seed=51 提前 1 代），但总体幅度较小。
- 建议将其作为“低风险小优化”保留；若要形成更强图表，需要在更大任务集/更难矩阵上复验。

## 产物
- `xml_pipeline_run/baseline_runs/GASeeding_Ablation_Mini_20260323/seeding_ablation_summary.json`
- `xml_pipeline_run/baseline_runs/GASeeding_Ablation_Mini_20260323/solution_greedy_seed_on_seed*.json`
- `xml_pipeline_run/baseline_runs/GASeeding_Ablation_Mini_20260323/solution_greedy_seed_off_seed*.json`
