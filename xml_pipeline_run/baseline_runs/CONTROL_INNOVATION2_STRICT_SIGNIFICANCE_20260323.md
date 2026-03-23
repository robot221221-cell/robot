# Innovation-2 严格对照：显著性检验（2026-03-23）

## 数据与方法
- 对照：GA on (`sequence-mode=ga`) vs 严格 GA-off (`sequence-mode=no_evo`)
- 样本：按 difficulty 分组、seed 配对（每组 n=10）
- 方向：差值定义为 `Δ = no_evo - ga`
- 统计：
  - 主检验：双侧配对符号检验（exact sign test）
  - 区间：bootstrap 95% CI（50,000 次重采样）
  - 效应量：配对 Cohen's $d$

> 注：当前环境未启用 SciPy，因此采用 exact sign test；对本数据（多数组同号）足以支持方向性结论。

## 结果（核心指标）

### 1) `parallel_makespan`（越小越好）

| split | mean Δ(no_evo-ga) (s) | 95% CI | p-value | 结论 |
|---|---:|---:|---:|---|
| easy | +0.3192 | [0.0000, 0.9452] | 0.5000 | 未达显著，差异小 |
| medium | -2.6646 | [-3.1656, -2.3322] | 0.0020 | no_evo 显著更优 |
| hard | +2.4094 | [1.9120, 3.1260] | 0.0020 | GA 显著更优 |

### 2) `total_wall_time`（越小越好）

| split | mean Δ(no_evo-ga) (s) | 95% CI | p-value | 结论 |
|---|---:|---:|---:|---|
| easy | -3.7550 | [-4.2385, -3.2573] | 0.0020 | no_evo 显著更低 |
| medium | -49.5223 | [-51.5371, -47.6115] | 0.0020 | no_evo 显著更低 |
| hard | -34.9838 | [-36.6538, -33.0895] | 0.0020 | no_evo 显著更低 |

## 解释（论文口径）
1. 创新2（GA）对 `makespan` 的收益**具有难度相关性**：hard 显著正向，medium 反向，easy 不显著。
2. `total_wall_time` 上 no_evo 全难度显著更低，说明其生成序列更“易规划/易执行”，但不保证并行质量最优。
3. 因此建议主叙述使用：**GA 在高难度/高冲突场景提升并行质量；全局性能取决于优化目标与执行目标的一致性。**

## 产物
- 原始统计 JSON：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.json`
- 本报告：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md`
