"""
生成面向任务排序的 pairwise transition cost matrix。

输出内容：
1. pairwise_transition_cost_ur5e.csv
2. pairwise_transition_cost_fr3.csv
3. pairwise_transition_edges_ur5e.csv
4. pairwise_transition_edges_fr3.csv

矩阵定义：
- 行表示当前任务结束后的状态（home 或某个 target）
- 列表示下一步要执行的目标点（或返回 home）
- 若 source = target，则代价记为 inf（禁止自环）
- 若 source != home，则默认上一任务已完成“拔出”，因此起点取该任务的 approach 姿态
- 转移总代价包含：
    retract_cost(source) + transfer_cost(source_approach -> dest_approach) + insert_cost(dest)
"""

import argparse
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

from dual_arm_simulation import ALL_TARGETS, ARM_HOME_Q, DualArmEnvironment, calculate_target_posture
from rrt_planner import BiRRTPlanner


def calculate_path_cost(path: list) -> float:
    if not path or len(path) < 2:
        return float("inf")
    return float(sum(np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1)))


def round_or_inf(value: float, digits: int = 3):
    return round(value, digits) if np.isfinite(value) else float("inf")


def precompute_target_postures(
    env: DualArmEnvironment,
    arm_name: str,
    targets: List[str],
    ur5e_home: np.ndarray,
    fr3_home: np.ndarray,
) -> Dict[str, Optional[dict]]:
    posture_cache: Dict[str, Optional[dict]] = {}

    for target in targets:
        env.reset(ur5e_home, fr3_home)
        q_approach, q_target = calculate_target_posture(env, arm_name, target)
        if q_approach is None or q_target is None:
            posture_cache[target] = None
            print(f"  [{arm_name}] {target}: posture infeasible (IK/Collision)")
            continue

        insert_cost = float(np.linalg.norm(q_target - q_approach))
        posture_cache[target] = {
            "approach": q_approach,
            "target": q_target,
            "insert_cost": insert_cost,
            "retract_cost": insert_cost,
        }
        print(f"  [{arm_name}] {target}: posture feasible")

    return posture_cache


def compute_transition_cost(
    env: DualArmEnvironment,
    planner: BiRRTPlanner,
    arm_name: str,
    source: str,
    dest: str,
    posture_cache: Dict[str, Optional[dict]],
    ur5e_home: np.ndarray,
    fr3_home: np.ndarray,
) -> dict:
    active_home = ARM_HOME_Q[arm_name]

    if source == dest and source != "home":
        return {
            "cost": float("inf"),
            "feasible": False,
            "reason": "self_loop_blocked",
            "retract_cost": float("inf"),
            "transfer_cost": float("inf"),
            "insert_cost": float("inf"),
        }
    if source == "home" and dest == "home":
        return {
            "cost": 0.0,
            "feasible": True,
            "reason": "identity",
            "retract_cost": 0.0,
            "transfer_cost": 0.0,
            "insert_cost": 0.0,
        }

    if dest != "home" and posture_cache.get(dest) is None:
        return {
            "cost": float("inf"),
            "feasible": False,
            "reason": "dest_posture_infeasible",
            "retract_cost": float("inf"),
            "transfer_cost": float("inf"),
            "insert_cost": float("inf"),
        }
    if source != "home" and posture_cache.get(source) is None:
        return {
            "cost": float("inf"),
            "feasible": False,
            "reason": "source_posture_infeasible",
            "retract_cost": float("inf"),
            "transfer_cost": float("inf"),
            "insert_cost": float("inf"),
        }

    if source == "home":
        q_start = active_home
        retract_cost = 0.0
    else:
        q_start = posture_cache[source]["approach"]
        retract_cost = posture_cache[source]["retract_cost"]

    if dest == "home":
        q_goal = active_home
        insert_cost = 0.0
    else:
        q_goal = posture_cache[dest]["approach"]
        insert_cost = posture_cache[dest]["insert_cost"]

    env.reset(ur5e_home, fr3_home)
    allow_endpoint_workpiece_contact = source != "home" or dest != "home"
    path = planner.plan(q_start, q_goal, allow_endpoint_workpiece_contact=allow_endpoint_workpiece_contact)
    if path is None:
        return {
            "cost": float("inf"),
            "feasible": False,
            "reason": "transfer_path_blocked",
            "retract_cost": retract_cost,
            "transfer_cost": float("inf"),
            "insert_cost": insert_cost,
        }

    transfer_cost = calculate_path_cost(path)
    return {
        "cost": retract_cost + transfer_cost + insert_cost,
        "feasible": True,
        "reason": "ok",
        "retract_cost": retract_cost,
        "transfer_cost": transfer_cost,
        "insert_cost": insert_cost,
    }


def build_pairwise_matrix_for_arm(
    env: DualArmEnvironment,
    arm_name: str,
    targets: List[str],
    output_dir: Path,
    step_size: float = 0.15,
    max_iter: int = 2000,
):
    print("\n" + "=" * 70)
    print(f"开始生成 {arm_name.upper()} 的 pairwise transition cost matrix")
    print("=" * 70)

    ur5e_home = ARM_HOME_Q["ur5e"].copy()
    fr3_home = ARM_HOME_Q["fr3"].copy()
    planner = BiRRTPlanner(env, arm_name=arm_name, step_size=step_size, max_iter=max_iter)

    posture_cache = precompute_target_postures(env, arm_name, targets, ur5e_home, fr3_home)
    labels = ["home"] + targets
    matrix = pd.DataFrame(index=labels, columns=labels, dtype=float)
    edge_rows = []

    for source in labels:
        for dest in labels:
            result = compute_transition_cost(
                env,
                planner,
                arm_name,
                source,
                dest,
                posture_cache,
                ur5e_home,
                fr3_home,
            )
            cost = result["cost"]
            matrix.loc[source, dest] = round_or_inf(cost)
            edge_rows.append(
                {
                    "arm": arm_name,
                    "source": source,
                    "destination": dest,
                    "cost": round_or_inf(cost),
                    "feasible": result["feasible"],
                    "reason": result["reason"],
                    "retract_cost": round_or_inf(result["retract_cost"]),
                    "transfer_cost": round_or_inf(result["transfer_cost"]),
                    "insert_cost": round_or_inf(result["insert_cost"]),
                }
            )
            print(f"  [{arm_name}] {source:>14} -> {dest:<14} : {matrix.loc[source, dest]}")

    matrix_path = output_dir / f"pairwise_transition_cost_{arm_name}.csv"
    edges_path = output_dir / f"pairwise_transition_edges_{arm_name}.csv"
    matrix.to_csv(matrix_path, index=True)
    pd.DataFrame(edge_rows).to_csv(edges_path, index=False)

    print(f"\n✅ 已保存矩阵: {matrix_path.name}")
    print(f"✅ 已保存边表: {edges_path.name}")

    return matrix


def parse_args():
    parser = argparse.ArgumentParser(description="生成面向任务排序的 pairwise transition cost matrix")
    parser.add_argument(
        "--arms",
        nargs="*",
        choices=["ur5e", "fr3"],
        default=["ur5e", "fr3"],
        help="要生成矩阵的机械臂列表",
    )
    parser.add_argument(
        "--targets",
        nargs="*",
        default=ALL_TARGETS,
        help="参与排序优化的目标点列表",
    )
    parser.add_argument(
        "--output-dir",
        default=".",
        help="输出目录，默认当前工作区根目录",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    env = DualArmEnvironment("dual_arm_scene.xml")
    for arm_name in args.arms:
        build_pairwise_matrix_for_arm(env, arm_name, args.targets, output_dir)


if __name__ == "__main__":
    main()
