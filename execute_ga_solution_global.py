"""
读取 GA 最优结果，并按 global_permutation 交替执行 UR5e / FR3 任务。

执行策略：
- 严格按照 global_permutation 顺序逐个执行任务
- 根据 assignment 判断当前任务由哪只手臂执行
- 若本次执行机械臂与上一次不同，则先让上一只手臂安全回到 home
- 非执行机械臂默认保持在 home，以尽量贴合当前 pairwise/GA 的建模假设

说明：
- 这是“全局顺序交替执行”版本，不是双臂同时同步执行版本
- 其目的在于把 GA 的 global_permutation 更直观地映射到可视化仿真中
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from dual_arm_simulation import ARM_HOME_Q, DualArmEnvironment, _HeadlessViewer
from execute_ga_solution import execute_single_task, load_solution, return_arm_home
from rrt_planner import BiRRTPlanner


def parse_args():
    parser = argparse.ArgumentParser(description="按 global_permutation 交替执行 GA 最优任务")
    parser.add_argument("--solution", default="xml_pipeline_run/ga_best_solution.json", help="GA 最优解 JSON 路径")
    parser.add_argument("--headless", action="store_true", help="无界面运行，用于快速验证")
    return parser.parse_args()


def main():
    args = parse_args()
    solution = load_solution(Path(args.solution))

    global_permutation = solution.get("global_permutation", [])
    assignment = solution.get("assignment", {})
    if not global_permutation or not assignment:
        raise ValueError("GA 解中缺少 global_permutation 或 assignment，无法按全局顺序执行")

    ur5e_home = ARM_HOME_Q["ur5e"].copy()
    fr3_home = ARM_HOME_Q["fr3"].copy()
    env = DualArmEnvironment("dual_arm_scene.xml")
    env.reset(ur5e_home, fr3_home)

    planners = {
        "ur5e": BiRRTPlanner(env, arm_name="ur5e", step_size=0.15),
        "fr3": BiRRTPlanner(env, arm_name="fr3", step_size=0.15),
    }
    current_q = {
        "ur5e": ur5e_home.copy(),
        "fr3": fr3_home.copy(),
    }
    home_q = {
        "ur5e": ur5e_home.copy(),
        "fr3": fr3_home.copy(),
    }

    viewer = _HeadlessViewer()
    viewer_manager = None
    if not args.headless:
        viewer_manager = mujoco.viewer.launch_passive(env.model, env.data)
        viewer = viewer_manager.__enter__()

    last_arm = None

    try:
        print("\n" + "=" * 72)
        print("开始按 global_permutation 交替执行 GA 最优任务")
        print(f"全局顺序: {global_permutation}")
        print("=" * 72)

        for idx, target in enumerate(global_permutation, start=1):
            if not viewer.is_running():
                break

            arm_name = assignment.get(target)
            if arm_name not in ("ur5e", "fr3"):
                print(f"⚠️ 任务 {target} 未找到合法机械臂分配，已跳过。")
                continue

            other_arm = "fr3" if arm_name == "ur5e" else "ur5e"

            print("\n" + "-" * 72)
            print(f"[全局步骤 {idx}/{len(global_permutation)}] {target} -> {arm_name.upper()}")
            print("-" * 72)

            if last_arm is not None and last_arm != arm_name:
                print(f"[{last_arm.upper()}] 与下一任务机械臂不同，先安全返回 Home...")
                current_q[last_arm] = return_arm_home(
                    env,
                    viewer,
                    last_arm,
                    planners[last_arm],
                    current_q[last_arm],
                    home_q[arm_name],
                )
                time.sleep(0.2)

            current_q[arm_name], success = execute_single_task(
                env,
                viewer,
                arm_name,
                planners[arm_name],
                current_q[arm_name],
                home_q[other_arm],
                target,
            )

            if not success:
                print(f"⚠️ {target} 执行失败，继续尝试后续任务。")

            last_arm = arm_name

        for arm_name in ["ur5e", "fr3"]:
            if not viewer.is_running():
                break
            current_q[arm_name] = return_arm_home(
                env,
                viewer,
                arm_name,
                planners[arm_name],
                current_q[arm_name],
                home_q["fr3" if arm_name == "ur5e" else "ur5e"],
            )

        print("\n✅ global_permutation 交替执行完毕！")
        while viewer.is_running() and not args.headless:
            env.step(home_q["ur5e"], home_q["fr3"])
            viewer.sync()
            time.sleep(0.01)

    finally:
        if viewer_manager is not None:
            viewer_manager.__exit__(None, None, None)


if __name__ == "__main__":
    main()
