# 异构双臂仿真环境 - 项目说明

## 项目概述
基于异构双臂（UR5e + FR3）的多位姿时间最优规划与防碰撞分拣控制系统

**总自由度**: 13 DOF (UR5e: 6-DOF + FR3: 7-DOF)

## 文件结构
```
/home/jiangshuo/robot/
├── dual_arm_scene.xml          # 双臂场景定义文件
├── dual_arm_simulation.py      # 主仿真脚本
├── artifacts/                  # 主目录输出归档（CSV/JSON）
│   └── root_outputs/
│       ├── csv/
│       └── json/
├── test_environment.py         # 环境测试脚本
├── run_dual_arm.sh            # 快速启动脚本
├── requirements.txt           # Python依赖
├── franka_fr3/               # FR3机械臂资源
│   ├── fr3.xml
│   └── assets/
└── universal_robots_ur5e/    # UR5e机械臂资源
    ├── ur5e.xml
    └── assets/
```

## 快速开始

### 1. 测试环境
```bash
cd /home/jiangshuo/robot
conda run -n lerobot python test_environment.py
```

### 2. 运行仿真
```bash
# 直接运行
conda run -n lerobot python dual_arm_simulation.py

# 控制 UR5e 到指定目标点
conda run -n lerobot python dual_arm_simulation.py --arm ur5e --targets target_top_1 target_left_1

# 控制 FR3 到指定目标点
conda run -n lerobot python dual_arm_simulation.py --arm fr3 --targets target_top_1 target_right_1

# 无界面快速验证
conda run -n lerobot python dual_arm_simulation.py --arm fr3 --targets target_top_1 --headless

# 查看全部可用目标点
conda run -n lerobot python dual_arm_simulation.py --list-targets

# 生成面向任务排序的 pairwise transition cost matrix
conda run -n lerobot python generate_pairwise_transition_cost_matrix.py

# 只针对部分目标点快速生成 pairwise 矩阵
conda run -n lerobot python generate_pairwise_transition_cost_matrix.py --arms ur5e fr3 --targets target_top_1 target_left_1

# 基于 pairwise 矩阵执行 GA 任务分配与排序优化
conda run -n lerobot python ga_task_allocation.py

# 仅对指定目标子集执行 GA 优化
conda run -n lerobot python ga_task_allocation.py --targets target_top_1 target_left_1

# 读取单点代价矩阵并自动分类目标点，生成可直接给 GA 使用的任务列表
conda run -n lerobot python classify_targets_for_ga.py

# 基于当前 XML 已定义目标点一键跑通完整流程
conda run -n lerobot python run_xml_ga_pipeline.py --output-dir xml_pipeline_run

# 读取 GA 最优结果并在可视化界面中执行双臂任务
conda run -n lerobot python execute_ga_solution.py --solution xml_pipeline_run/ga_best_solution.json

# 无界面快速验证 GA 最优结果执行
conda run -n lerobot python execute_ga_solution.py --solution xml_pipeline_run/ga_best_solution.json --headless

# 按 global_permutation 交替执行 GA 最优结果（可视化）
conda run -n lerobot python execute_ga_solution_global.py --solution xml_pipeline_run/ga_best_solution.json

# 按 global_permutation 交替执行 GA 最优结果（无界面）
conda run -n lerobot python execute_ga_solution_global.py --solution xml_pipeline_run/ga_best_solution.json --headless

# 基于 TOPP-RA：双臂并行执行 + 冲突检测 + 优先级等待时间调度
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json

# 可选：统一缩放速度/加速度约束
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --velocity-scale 0.8 --acceleration-scale 0.8

# 无界面快速验证 TOPP-RA 时间调度
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless

# 调试并行下发：打印前 20 个 tick 的双臂同步命令（验证“同一时间轴并行推进”）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless --debug-first-ticks 20

# 禁用自动起始延迟（放大等待/让行策略差异，便于 A/B 对比）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless --disable-auto-delay

# 输出统计指标（并行/串行 makespan、等待时间、加速比）到 JSON
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless --stats-json xml_pipeline_run/schedule_stats.json

# 进行 auto-delay 开/关 的消融对比（不改算法）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless --report-ablation --stats-json xml_pipeline_run/reports/schedule/schedule_ablation_stats.json

# 仿真结束后自动生成“方法对比表”（CSV + Markdown）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/ga_best_solution.json --headless --report-ablation --table-csv xml_pipeline_run/reports/schedule/schedule_comparison.csv --table-md xml_pipeline_run/reports/schedule/schedule_comparison.md

# 将统计 JSON 渲染为“可读版实验报告”（简洁 Markdown + 图表 PNG）
conda run -n lerobot python render_schedule_report.py --input xml_pipeline_run/reports/schedule/schedule_ablation_stats.json --output-md xml_pipeline_run/reports/schedule/schedule_report.md --output-png xml_pipeline_run/reports/schedule/schedule_report.png

# 批量实验（多场景×多次重复），自动输出中文汇总表和中文图表
conda run -n lerobot python run_batch_schedule_experiments.py --solution xml_pipeline_run/ga_best_solution.json --output-dir xml_pipeline_run/batch_experiments --repeats 2 --hold-steps-list 20 80 160 --sim-steps-list 1 --priority-arms ur5e --tick-sleep 0

# 难度分级实验（easy/medium/hard），自动覆盖全部可达目标点并输出中文报告与图表（推荐用 v6）
conda run -n lerobot python run_difficulty_experiments.py --solution xml_pipeline_run/ga_best_solution.json --output-dir xml_pipeline_run/difficulty_experiments_v6 --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20

# ========== 三种场景可视化启动（非 headless）==========

# easy 可视化
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_easy.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20

# medium 可视化
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_medium.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20

# hard 可视化
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20

# hard 可视化（去“瞬移感”稳定版：执行限幅 + 降速时参 + 更保守恢复）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_visual_smooth_stable.json

# hard 非同步化可视化（效率优先，λ=0）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --sync-balance-weight 0 --random-seed 42 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_visual_nonsync.json

# hard 同步化可视化（软目标，惩罚完工时间差，建议 λ=0.3）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --sync-balance-weight 0.3 --random-seed 46 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_visual_sync_soft.json

# 一键跑“同步化 vs 非同步化”对比实验（headless），自动输出 JSON/CSV/Markdown/PNG 图表
conda run -n lerobot python run_sync_comparison_experiments.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --output-dir xml_pipeline_run/difficulty_experiments_v6/sync_comparison --lambda-sync 0.5 --random-seed 42 --max-retries 2

# hard 最终参数搜索（成功率优先，搜索 λ）
conda run -n lerobot python run_final_parameter_search.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --output-dir xml_pipeline_run/difficulty_experiments_v6/final_param_search --lambdas 0 0.3 0.5 0.7 --seeds 42 43 44 45 46

# 快速版最终参数搜索（先小样本）
conda run -n lerobot python run_final_parameter_search.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --output-dir xml_pipeline_run/difficulty_experiments_v6/final_param_search_quick --lambdas 0 0.5 --seeds 42 43

# 固定 λ=0.3 后，搜索其余调度参数（阶段1）
conda run -n lerobot python run_fixed_lambda_param_search.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --output-dir xml_pipeline_run/difficulty_experiments_v6/fixed_lambda_param_search_stage1 --lambda-sync 0.3 --seeds 45 46 --anti-trigger-list 700 900 --anti-hold-list 80 --anti-attempts-list 3 4 --max-no-progress-list 5000 --max-cmd-step-list 0.02

# 若你只想先快速验证流程（更快返回）
conda run -n lerobot python run_sync_comparison_experiments.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --output-dir xml_pipeline_run/difficulty_experiments_v6/sync_comparison --lambda-sync 0.5 --random-seed 42 --max-retries 1 --max-total-ticks 80000 --allow-partial

# 对比输出目录结构（更整洁）
# - sync_comparison/stats/*.json
# - sync_comparison/tables/*.csv|*.md
# - sync_comparison/figures/*.png
# - final_param_search/stats/*.json
# - final_param_search/tables/*.csv|*.md|*.json
# - fixed_lambda_param_search_stage1/stats/*.json
# - fixed_lambda_param_search_stage1/tables/*.csv|*.json

# 对比实验关键指标说明：
# - success：任务是否完整成功（必须=1）
# - parallel_makespan_s：并行总耗时（越小越好）
# - speedup_vs_serial：相对串行加速比（越大越好）
# - deadlock_wait_count：死锁等待次数（越小越好）
# - finish_gap_s：双臂完工时间差（同步性参考项，可降级）
# - jump_summary.applied：两臂执行层跳变统计（越平滑越好）

# 成功率优先的 hard 推荐命令（可视化）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --sync-balance-weight 0.3 --random-seed 46 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_visual_success_first.json

# 注：final_param_search 阶段中 λ=0.3 与 λ=0.5 表现并列，默认取更小正则 λ=0.3

# 最终最优配置（success-first）已固化到：
# - xml_pipeline_run/difficulty_experiments_v6/final_best_config.json
# - 5种子复核汇总：xml_pipeline_run/difficulty_experiments_v6/final_best_validation_5seeds/summary.json

# 用最终最优配置一键复现（headless）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --headless --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --sync-balance-weight 0.3 --random-seed 46 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_best_config.json

# hard 调试（防振荡阈值可调，避免长时间循环）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --headless --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 600 --anti-oscillation-hold 120 --anti-oscillation-max-attempts 4 --max-total-ticks 120000 --max-no-progress-ticks 4000 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_debug.json

# hard 去“瞬移感”推荐配置（headless 稳定版，便于统计）
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_hard.json --headless --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --velocity-scale 0.8 --acceleration-scale 0.7 --max-start-delay 4000 --delay-coarse-step 20 --anti-oscillation-trigger 900 --anti-oscillation-hold 80 --anti-oscillation-max-attempts 3 --max-total-ticks 120000 --max-no-progress-ticks 5000 --retreat-cooldown-ticks 16 --max-cmd-step 0.02 --jump-step-threshold 0.06 --jump-velocity-threshold 25 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_hard_smooth_stable.json

# ========== 界面外查看“到达了哪些点/没到达哪些点/为什么”==========

# medium（headless）+ 目标到达摘要 JSON
conda run -n lerobot python schedule_ga_with_priority_delay.py --solution xml_pipeline_run/difficulty_experiments_v6/solution_medium.json --headless --hold-steps 80 --sim-steps-per-tick 1 --priority-arm ur5e --max-start-delay 4000 --delay-coarse-step 20 --stats-json xml_pipeline_run/difficulty_experiments_v6/stats/stats_medium_with_target_summary.json

# 说明：
# stats JSON 中 trajectory_info.target_summary 会记录：
# - reached_targets（到达目标）
# - skipped_targets（未到达目标）
# - reason/reason_cn（未到达原因，如 IK不可行、压入段碰撞风险、接近段规划失败）
```

### 3. 交互操作
- **鼠标左键拖拽**: 旋转视角
- **鼠标滚轮**: 缩放
- **Ctrl+C 或关闭窗口**: 退出仿真

### 4. 输出文件整理约定（建议）
- 主目录历史输出归档：
  - CSV: [artifacts/root_outputs/csv](artifacts/root_outputs/csv)
  - JSON: [artifacts/root_outputs/json](artifacts/root_outputs/json)
- 新实验结果统一放在 [xml_pipeline_run](xml_pipeline_run) 下按实验分目录。
- 同步化对比实验输出已自动分为：
  - `stats/*.json`
  - `tables/*.csv|*.md`
  - `figures/*.png`
- 目前 `schedule_ga_with_priority_delay.py`、`run_difficulty_experiments.py`、`run_batch_schedule_experiments.py`、`run_sync_comparison_experiments.py` 在结束后会默认自动整理输出；如需关闭可加：`--disable-auto-organize`
- 任意时间可一键整理历史/新增结果：
  - `conda run -n lerobot python organize_outputs.py --root /home/jiangshuo/robot --apply`

## 系统配置

### 机械臂布局
- **UR5e**: 位于工作空间左侧 (pos: -0.3, 0.4, 0.42)
- **FR3**: 位于工作空间右侧 (pos: 0.3, -0.4, 0.42)
- **工作台**: 中央位置，尺寸 1.6m × 1.2m × 0.04m

### 初始配置
- UR5e初始姿态: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
- FR3初始姿态: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

### 仿真参数
- 时间步长: 0.002s (500Hz)
- 积分器: implicitfast
- 碰撞检测: 启用

## 功能模块

### 当前已实现
- ✅ 双臂模型加载与可视化
- ✅ 基础运动学接口
- ✅ 关节位置/速度获取
- ✅ 末端执行器位置查询
- ✅ 碰撞检测框架
- ✅ 工作空间分析

### 待开发（论文章节对应）
- ⏳ **第一章**: 运动学与动力学建模
  - 正逆运动学求解
  - 雅可比矩阵计算
  - 动力学参数辨识

- ⏳ **第二章**: 连续碰撞检测
  - 胶囊体/距离场算法
  - 最小距离计算
  - 自碰撞检测

- ⏳ **第三章**: 多位姿时间最优规划
  - 任务分配算法（谁抓哪个）
  - 轨迹时间最优参数化(TOPP)
  - 速度/加速度约束

- ⏳ **第四章**: 实验验证
  - 性能指标统计
  - 对比算法实现
  - 可视化与数据记录

## API 接口

### DualArmEnvironment 类

```python
# 初始化
env = DualArmEnvironment("dual_arm_scene.xml")

# 重置环境
env.reset(ur5e_qpos, fr3_qpos)

# 执行仿真步
env.step(ur5e_ctrl, fr3_ctrl)

# 获取状态
ur5e_q, fr3_q = env.get_joint_positions()
ur5e_qvel, fr3_qvel = env.get_joint_velocities()
ur5e_ee, fr3_ee = env.get_ee_positions()

# 碰撞检测
has_collision, contacts = env.check_collision()

# 工作空间限制
limits = env.get_workspace_limits()
```

## 下一步工作

1. **运动学模块开发**
   - 实现基于雅可比的逆运动学
   - 添加关节限位检查
   - 奇异性避免

2. **碰撞检测优化**
   - 集成FCL或类似库
   - 实现胶囊体近似
   - 距离场计算加速

3. **轨迹规划器**
   - 路径点插值
   - TOPP-RA算法实现
   - 速度剖面优化

4. **任务分配**
   - 组合优化求解器
   - 启发式搜索算法
   - 成本函数设计

## 技术栈
- **仿真**: MuJoCo 3.0+
- **数值计算**: NumPy
- **Python环境**: lerobot (conda)
- **可视化**: MuJoCo Viewer

## 注意事项
- 所有脚本必须在 `lerobot` conda环境中运行
- 碰撞检测仅识别双臂之间的碰撞
- 当前物体为固定位置，后续将添加抓取功能

---
**作者**: 硕士论文项目  
**日期**: 2026-03-05  
**状态**: 基础框架完成，进入开发阶段
