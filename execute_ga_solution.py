"""
读取 GA 最优结果，并在 MuJoCo 可视化界面中按最优分配结果执行任务。

当前执行策略：
- 先执行 UR5e 的最优任务序列，再执行 FR3 的最优任务序列
- 非执行机械臂保持在 home 姿态
- 这样与当前 pairwise 矩阵和 GA 的建模假设保持一致，稳定性更高
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

from dual_arm_simulation import (
    ARM_HOME_Q,
    DualArmEnvironment,
    _HeadlessViewer,
    _env_step_for_arm,
    _get_allowed_workpiece_parts,
    _get_current_arm_q,
    _get_ee_site_id,
    _servo_to_waypoint,
    calculate_target_posture,
    execute_trajectory,
)
from rrt_planner import BiRRTPlanner
from trajectory_smoothing import b_spline_smooth


def load_solution(solution_path: Path) -> dict:
    return json.loads(solution_path.read_text(encoding="utf-8"))


def hold_and_validate(env, viewer, arm_name: str, q_target: np.ndarray, other_arm_q: np.ndarray, target_name: str, hold_steps: int = 300):
    target_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, target_name)
    target_pos = env.data.site_xpos[target_id].copy()
    allowed_parts = _get_allowed_workpiece_parts(arm_name)

    for _ in range(hold_steps):
        _env_step_for_arm(env, arm_name, q_target, other_arm_q)
        viewer.sync()
        time.sleep(0.002)

        for c_idx in range(env.data.ncon):
            contact = env.data.contact[c_idx]
            b1 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, env.model.geom_bodyid[contact.geom1]) or ""
            b2 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, env.model.geom_bodyid[contact.geom2]) or ""
            if 'workpiece' not in b1 and 'workpiece' not in b2:
                continue
            other_body = b1 if 'workpiece' in b2 else b2
            if not any(part in other_body for part in allowed_parts):
                return False, f"手臂非允许部位({other_body})强行挤压了箱子"

    actual_pos = env.data.site_xpos[_get_ee_site_id(env, arm_name)].copy()
    pos_error = np.linalg.norm(actual_pos - target_pos)
    if pos_error > 0.015:
        return False, f"末端被物理卡住，位置误差高达 {pos_error * 100:.1f} cm"

    return True, "ok"


def execute_single_task(env, viewer, arm_name: str, planner: BiRRTPlanner, current_arm_q: np.ndarray, other_arm_q: np.ndarray, target: str):
    print(f"\n[{arm_name.upper()} | {target}] ---------------------------------")
    print(f"[{arm_name.upper()} | {target}] 1. 正在求解逆运动学 (IK)...")
    q_approach, q_target = calculate_target_posture(env, arm_name, target)
    if q_approach is None or q_target is None:
        print(f"❌ {arm_name.upper()} 无法安全到达 {target}，已跳过。")
        return current_arm_q, False

    if not planner._is_edge_collision_free(q_approach, q_target, allow_workpiece_contact=True):
        print(f"❌ {arm_name.upper()} 的压入段对 {target} 存在碰撞风险，已跳过。")
        return current_arm_q, False

    print(f"[{arm_name.upper()} | {target}] 2. 正在进行 RRT 全局防碰撞路径规划...")
    rrt_path = planner.plan(current_arm_q, q_approach)
    if rrt_path is None:
        print(f"❌ {arm_name.upper()} 去往 {target} 的路径规划失败，已跳过。")
        return current_arm_q, False

    print(f"[{arm_name.upper()} | {target}] 3. 执行接近段...")
    smooth_rrt_path = b_spline_smooth(rrt_path, num_points=200, degree=1)
    if not execute_trajectory(env, viewer, arm_name, smooth_rrt_path, other_arm_q):
        return current_arm_q, False

    print(f"[{arm_name.upper()} | {target}] 4. 垂直压入作业...")
    smooth_insert_path = b_spline_smooth([q_approach, q_target], num_points=50, degree=1)
    if not execute_trajectory(env, viewer, arm_name, smooth_insert_path, other_arm_q):
        return current_arm_q, False
    if not _servo_to_waypoint(env, viewer, arm_name, q_target, other_arm_q, joint_tol=0.02, max_servo_steps=80):
        return current_arm_q, False

    ok, reason = hold_and_validate(env, viewer, arm_name, q_target, other_arm_q, target)
    if not ok:
        print(f"❌ [物理引擎警报] {arm_name.upper()} 执行 {target} 失败：{reason}！")
        print("👉 正在执行安全撤离...")
        smooth_retract_path = b_spline_smooth([_get_current_arm_q(env, arm_name), q_approach], num_points=50, degree=1)
        execute_trajectory(env, viewer, arm_name, smooth_retract_path, other_arm_q)
        return q_approach.copy(), False

    print(f"[{arm_name.upper()} | {target}] 5. 作业完成，安全拔出...")
    smooth_retract_path = b_spline_smooth([q_target, q_approach], num_points=50, degree=1)
    if not execute_trajectory(env, viewer, arm_name, smooth_retract_path, other_arm_q):
        return current_arm_q, False

    return q_approach.copy(), True


def return_arm_home(env, viewer, arm_name: str, planner: BiRRTPlanner, current_arm_q: np.ndarray, other_arm_q: np.ndarray):
    home_q = ARM_HOME_Q[arm_name].copy()
    if np.linalg.norm(current_arm_q - home_q) < 1e-3:
        return home_q

    print(f"\n[{arm_name.upper()}] 正在返回 Home 姿态...")
    path = planner.plan(current_arm_q, home_q)
    if path is None:
        print(f"⚠️ {arm_name.upper()} 返回 Home 的 RRT 失败，改用直连伺服回位。")
        _servo_to_waypoint(env, viewer, arm_name, home_q, other_arm_q, joint_tol=0.03, max_servo_steps=200)
        return home_q

    smooth_path = b_spline_smooth(path, num_points=150, degree=1)
    execute_trajectory(env, viewer, arm_name, smooth_path, other_arm_q)
    _servo_to_waypoint(env, viewer, arm_name, home_q, other_arm_q, joint_tol=0.02, max_servo_steps=120)
    return home_q


def execute_arm_sequence(env, viewer, arm_name: str, sequence: list):
    planner = BiRRTPlanner(env, arm_name=arm_name, step_size=0.15)
    other_arm_name = 'fr3' if arm_name == 'ur5e' else 'ur5e'
    current_arm_q = ARM_HOME_Q[arm_name].copy()
    other_arm_q = ARM_HOME_Q[other_arm_name].copy()

    print("\n" + "=" * 70)
    print(f"开始执行 {arm_name.upper()} 的最优任务序列: {sequence}")
    print("=" * 70)

    for target in sequence:
        if not viewer.is_running():
            break
        current_arm_q, _ = execute_single_task(env, viewer, arm_name, planner, current_arm_q, other_arm_q, target)

    current_arm_q = return_arm_home(env, viewer, arm_name, planner, current_arm_q, other_arm_q)
    for _ in range(100):
        if not viewer.is_running():
            break
        _env_step_for_arm(env, arm_name, current_arm_q, other_arm_q)
        viewer.sync()
        time.sleep(0.005)


def parse_args():
    parser = argparse.ArgumentParser(description="读取 GA 最优结果并在可视化界面中执行双臂任务")
    parser.add_argument("--solution", default="xml_pipeline_run/ga_best_solution.json", help="GA 最优解 JSON 路径")
    parser.add_argument("--headless", action="store_true", help="无界面运行，用于快速验证")
    return parser.parse_args()


def main():
    args = parse_args()
    solution = load_solution(Path(args.solution))

    ur5e_home = ARM_HOME_Q['ur5e'].copy()
    fr3_home = ARM_HOME_Q['fr3'].copy()
    env = DualArmEnvironment("dual_arm_scene.xml")
    env.reset(ur5e_home, fr3_home)

    viewer = _HeadlessViewer()
    viewer_manager = None
    if not args.headless:
        viewer_manager = mujoco.viewer.launch_passive(env.model, env.data)
        viewer = viewer_manager.__enter__()

    try:
        execute_arm_sequence(env, viewer, 'ur5e', solution.get('ur5e_sequence', []))
        env.reset(ur5e_home, fr3_home)
        execute_arm_sequence(env, viewer, 'fr3', solution.get('fr3_sequence', []))

        print("\n✅ GA 最优结果执行完毕！")
        while viewer.is_running() and not args.headless:
            env.step(ur5e_home, fr3_home)
            viewer.sync()
            time.sleep(0.01)
    finally:
        if viewer_manager is not None:
            viewer_manager.__exit__(None, None, None)


if __name__ == "__main__":
    main()
