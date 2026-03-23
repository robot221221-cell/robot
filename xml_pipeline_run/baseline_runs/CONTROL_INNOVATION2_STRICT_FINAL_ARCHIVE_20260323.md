# Innovation-2 严格验证：结果整理归档（2026-03-23）

## 归档范围
- 显式 GA on/off（严格 `no_evo`）
- full30：easy / medium / hard（seed 42~51）
- 统一设置：Stage-1C（Distance-LOD + Priority-LOD）+ with auto-delay

## 汇总表（GA on vs GA off=no_evo）

| split | GA on success_rate | no_evo success_rate | GA on makespan (s) | no_evo makespan (s) | makespan Δ%(no_evo-GA) | GA on total_wall (s) | no_evo total_wall (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| easy | 1.0 | 1.0 | 23.2978 | 23.6170 | +1.37% | 60.9661 | 57.2111 |
| medium | 1.0 | 1.0 | 45.6514 | 42.9868 | -5.84% | 106.1977 | 56.6753 |
| hard | 1.0 | 1.0 | 55.7914 | 58.2008 | +4.32% | 116.2199 | 81.2361 |

## 全30样本总览（easy+medium+hard）
- GA on：`success_rate=1.0`, `parallel_makespan_mean=41.5802`
- no_evo：`success_rate=1.0`, `parallel_makespan_mean=41.6015`
- 结论：全体平均 makespan 几乎持平（差约 0.05%）。

## 结论（归档口径）
1. 严格 GA-off（no_evo）实验通路已打通，创新2因果对照链条完整。
2. hard 上 GA 对并行质量（makespan）有正收益；medium 上 no_evo 反而更优；easy 差异很小。
3. 当前证据更支持“难度相关收益”而非“全局稳定优于 no_evo”。
4. 后续若用于论文主结论，建议将创新2表述为：**GA 在高冲突/高难度场景更具优势**，并补充目标函数一致性讨论。

## 显著性结论（配对统计）
- `makespan`：
	- easy：差异不显著（p=0.50）
	- medium：no_evo 显著更优（p=0.002）
	- hard：GA 显著更优（p=0.002）
- `total_wall_time`：easy/medium/hard 均为 no_evo 显著更低（p=0.002）
- 详细统计见：`xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md`

## 归档清单
### 报告
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_GA_ONOFF_EXPLICIT_MINI_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_GA_ONOFF_EXPLICIT_FULL30_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_MEDIUM_EASY_FULL30_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_SIGNIFICANCE_20260323.json`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_MECHANISM_SWEEP_MEDIUM_MINI_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_GA_SEEDING_MINI_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_GA_OBJECTIVE_ALIGNMENT_MINI_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INCREMENTAL_INNOVATIONS_DECISION_20260323.md`
- `xml_pipeline_run/baseline_runs/CONTROL_INNOVATION2_STRICT_FINAL_ARCHIVE_20260323.md`

### 批次目录
- `xml_pipeline_run/baseline_runs/Innovation2_GAExplicit_On_HardFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAOffNoEvo_HardFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAExplicit_On_MediumEasyFull30_42_51_r1/`
- `xml_pipeline_run/baseline_runs/Innovation2_GAOffNoEvo_MediumEasyFull30_42_51_r1/`
- （补充代理）`xml_pipeline_run/baseline_runs/Innovation2_GAOffRandom_HardFull30_42_51_r1/`
