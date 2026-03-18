"""
基于 TOPP-RA 的“双臂并行 + 冲突检测 + 优先级等待”时间调度脚本。

特点：
- 读取 GA 最优结果
- 为 UR5e / FR3 分别生成满足关节速度/加速度约束的时间最优轨迹
- 在执行阶段按统一采样周期并行推进两条轨迹
- 若预测下一步会发生双臂碰撞，则低优先级机械臂等待，高优先级机械臂先行
- 若高优先级当前无法推进，则尝试让低优先级机械臂先移动以解除死锁

说明：
- 本脚本已将原来的“固定采样名义轨迹”升级为“TOPP-RA 时间参数化轨迹”
- 轨迹的几何路径仍由 RRT / 直线压入段给出，TOPP-RA 负责在速度/加速度约束下做时间最优参数化
"""

import argparse
import csv
import json
import random
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import mujoco
import mujoco.viewer
import toppra.algorithm as algo
import toppra.constraint as constraint
from toppra.interpolator import AbstractGeometricPath

from dual_arm_simulation import ARM_HOME_Q, DualArmEnvironment, _HeadlessViewer, calculate_target_posture
from execute_ga_solution import load_solution
from rrt_planner import BiRRTPlanner


ARM_JOINT_LIMITS = {
    # 采用保守关节约束，后续可替换为更精确的厂家参数/实验辨识参数
    "ur5e": {
        "velocity": np.array([3.14, 3.14, 3.14, 3.14, 3.14, 3.14], dtype=float),
        "acceleration": np.array([6.0, 6.0, 6.0, 6.0, 6.0, 6.0], dtype=float),
    },
    "fr3": {
        "velocity": np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61], dtype=float),
        "acceleration": np.array([5.0, 5.0, 5.0, 5.0, 6.0, 6.0, 6.0], dtype=float),
    },
}


class PiecewiseLinearPath(AbstractGeometricPath):
    """供 TOPP-RA 使用的分段线性几何路径，避免样条插值带来的拐角抄近路问题。"""

    def __init__(self, waypoints: np.ndarray):
        self._waypoints = np.asarray(waypoints, dtype=float)
        if self._waypoints.ndim != 2 or len(self._waypoints) < 2:
            raise ValueError("PiecewiseLinearPath 至少需要 2 个关节路点。")

        segment_vectors = np.diff(self._waypoints, axis=0)
        segment_lengths = np.linalg.norm(segment_vectors, axis=1)
        self._gridpoints = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        self._dof = self._waypoints.shape[1]

    @property
    def dof(self) -> int:
        return self._dof

    @property
    def path_interval(self) -> Tuple[float, float]:
        return float(self._gridpoints[0]), float(self._gridpoints[-1])

    @property
    def waypoints(self) -> np.ndarray:
        return self._waypoints

    def __call__(self, path_positions, order: int = 0) -> np.ndarray:
        if order == 0:
            return self.eval(path_positions)
        if order == 1:
            return self.evald(path_positions)
        if order == 2:
            return self.evaldd(path_positions)
        raise ValueError(f"不支持的导数阶数: {order}")

    def _locate_segment(self, s: float) -> Tuple[int, float]:
        if s <= self._gridpoints[0]:
            return 0, 0.0
        if s >= self._gridpoints[-1]:
            return len(self._gridpoints) - 2, 1.0

        seg_idx = np.searchsorted(self._gridpoints, s, side="right") - 1
        seg_len = self._gridpoints[seg_idx + 1] - self._gridpoints[seg_idx]
        local_u = 0.0 if seg_len <= 1e-12 else (s - self._gridpoints[seg_idx]) / seg_len
        return seg_idx, float(local_u)

    def eval(self, ss):
        is_scalar = np.isscalar(ss)
        s_values = np.atleast_1d(np.asarray(ss, dtype=float))
        q_values = []
        for s in s_values:
            seg_idx, local_u = self._locate_segment(float(s))
            q0 = self._waypoints[seg_idx]
            q1 = self._waypoints[seg_idx + 1]
            q_values.append((1.0 - local_u) * q0 + local_u * q1)

        q_values = np.vstack(q_values)
        return q_values[0] if is_scalar else q_values

    def evald(self, ss):
        is_scalar = np.isscalar(ss)
        s_values = np.atleast_1d(np.asarray(ss, dtype=float))
        dq_values = []
        for s in s_values:
            seg_idx, _ = self._locate_segment(float(s))
            seg_len = self._gridpoints[seg_idx + 1] - self._gridpoints[seg_idx]
            if seg_len <= 1e-12:
                dq_values.append(np.zeros(self._dof, dtype=float))
            else:
                dq_values.append((self._waypoints[seg_idx + 1] - self._waypoints[seg_idx]) / seg_len)

        dq_values = np.vstack(dq_values)
        return dq_values[0] if is_scalar else dq_values

    def evaldd(self, ss):
        is_scalar = np.isscalar(ss)
        s_values = np.atleast_1d(np.asarray(ss, dtype=float))
        ddq_values = np.zeros((len(s_values), self._dof), dtype=float)
        return ddq_values[0] if is_scalar else ddq_values


def deduplicate_waypoints(path: List[np.ndarray], atol: float = 1e-9) -> List[np.ndarray]:
    cleaned = []
    for q in path:
        q_arr = np.asarray(q, dtype=float).copy()
        if not cleaned or np.linalg.norm(q_arr - cleaned[-1]) > atol:
            cleaned.append(q_arr)
    return cleaned


def time_parameterize_path(
    path: List[np.ndarray],
    arm_name: str,
    sample_dt: float,
    velocity_scale: float = 1.0,
    acceleration_scale: float = 1.0,
) -> Tuple[List[np.ndarray], float]:
    waypoints = deduplicate_waypoints(path)
    if not waypoints:
        return [], 0.0
    if len(waypoints) == 1:
        return [waypoints[0].copy()], 0.0

    path_array = np.vstack(waypoints)
    segment_lengths = np.linalg.norm(np.diff(path_array, axis=0), axis=1)
    if np.all(segment_lengths < 1e-12):
        return [waypoints[0].copy()], 0.0

    geometric_path = PiecewiseLinearPath(path_array)
    constraints = [
        constraint.JointVelocityConstraint(ARM_JOINT_LIMITS[arm_name]["velocity"] * velocity_scale),
        constraint.JointAccelerationConstraint(ARM_JOINT_LIMITS[arm_name]["acceleration"] * acceleration_scale),
    ]
    instance = algo.TOPPRA(constraints, geometric_path, parametrizer="ParametrizeConstAccel")
    trajectory = instance.compute_trajectory(0.0, 0.0)
    if trajectory is None:
        raise RuntimeError(f"{arm_name} 的 TOPP-RA 时间参数化失败。")

    duration = float(trajectory.duration)
    if duration <= 1e-12:
        return [waypoints[0].copy(), waypoints[-1].copy()], 0.0

    ts = np.arange(0.0, duration, sample_dt, dtype=float)
    if len(ts) == 0 or duration - ts[-1] > 1e-9:
        ts = np.append(ts, duration)

    sampled_q = np.asarray(trajectory(ts), dtype=float)
    return [q.copy() for q in sampled_q], duration


def is_inter_arm_collision(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    for c_idx in range(data.ncon):
        contact = data.contact[c_idx]
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""
        is_ur5e_fr3 = ("ur5e" in b1 and "fr3" in b2) or ("ur5e" in b2 and "fr3" in b1)
        is_ur5e_panda = ("ur5e" in b1 and "panda" in b2) or ("ur5e" in b2 and "panda" in b1)
        if is_ur5e_fr3 or is_ur5e_panda:
            return True
    return False


class PriorityDelayScheduler:
    def __init__(
        self,
        env: DualArmEnvironment,
        priority_arm: str = "ur5e",
        enable_retreat_recovery: bool = True,
        retreat_cooldown_ticks: int = 8,
        allow_high_priority_retreat: bool = False,
    ):
        self.env = env
        self.model = env.model
        self.priority_arm = priority_arm
        self.enable_retreat_recovery = enable_retreat_recovery
        self.retreat_cooldown_ticks = max(0, int(retreat_cooldown_ticks))
        self.allow_high_priority_retreat = bool(allow_high_priority_retreat)
        self.retreat_cooldown = {"ur5e": 0, "fr3": 0}
        self.preview_data = mujoco.MjData(self.model)
        self.arm_joint_ids = {
            "ur5e": env.ur5e_joint_ids,
            "fr3": env.fr3_joint_ids,
        }

    def _set_joint_state(self, data: mujoco.MjData, arm_name: str, q: np.ndarray):
        for i, jid in enumerate(self.arm_joint_ids[arm_name]):
            data.qpos[jid] = q[i]

    def will_conflict(self, ur5e_q: np.ndarray, fr3_q: np.ndarray) -> bool:
        self.preview_data.qpos[:] = self.env.data.qpos[:]
        self.preview_data.qvel[:] = 0.0
        self.preview_data.ctrl[:] = self.env.data.ctrl[:]
        self._set_joint_state(self.preview_data, "ur5e", ur5e_q)
        self._set_joint_state(self.preview_data, "fr3", fr3_q)
        mujoco.mj_forward(self.model, self.preview_data)
        return is_inter_arm_collision(self.model, self.preview_data)

    def choose_next_indices(self, indices: Dict[str, int], trajectories: Dict[str, List[np.ndarray]]) -> Tuple[Dict[str, int], str]:
        for arm in ["ur5e", "fr3"]:
            if self.retreat_cooldown[arm] > 0:
                self.retreat_cooldown[arm] -= 1

        last_idx = {arm: len(trajectories[arm]) - 1 for arm in ["ur5e", "fr3"]}
        curr = {arm: indices[arm] for arm in ["ur5e", "fr3"]}
        nxt = {arm: min(indices[arm] + 1, last_idx[arm]) for arm in ["ur5e", "fr3"]}
        prv = {arm: max(indices[arm] - 1, 0) for arm in ["ur5e", "fr3"]}

        ur_curr = trajectories["ur5e"][curr["ur5e"]]
        fr_curr = trajectories["fr3"][curr["fr3"]]
        ur_next = trajectories["ur5e"][nxt["ur5e"]]
        fr_next = trajectories["fr3"][nxt["fr3"]]

        both_done = curr["ur5e"] == last_idx["ur5e"] and curr["fr3"] == last_idx["fr3"]
        if both_done:
            return curr, "done"

        if not self.will_conflict(ur_next, fr_next):
            return nxt, "both_move"

        high = self.priority_arm
        low = "fr3" if high == "ur5e" else "ur5e"
        high_curr = ur_curr if high == "ur5e" else fr_curr
        high_next = ur_next if high == "ur5e" else fr_next
        low_curr = fr_curr if low == "fr3" else ur_curr
        low_next = fr_next if low == "fr3" else ur_next
        high_can_advance = curr[high] < last_idx[high] and self.retreat_cooldown[high] <= 0
        low_can_advance = curr[low] < last_idx[low] and self.retreat_cooldown[low] <= 0

        if high_can_advance:
            if high == "ur5e":
                if not self.will_conflict(high_next, low_curr):
                    new_idx = curr.copy()
                    new_idx[high] = nxt[high]
                    return new_idx, f"{low}_wait"
            else:
                if not self.will_conflict(low_curr, high_next):
                    new_idx = curr.copy()
                    new_idx[high] = nxt[high]
                    return new_idx, f"{low}_wait"

        if low_can_advance:
            if low == "ur5e":
                if not self.will_conflict(low_next, high_curr):
                    new_idx = curr.copy()
                    new_idx[low] = nxt[low]
                    return new_idx, f"{high}_blocked_low_moves"
            else:
                if not self.will_conflict(high_curr, low_next):
                    new_idx = curr.copy()
                    new_idx[low] = nxt[low]
                    return new_idx, f"{low}_moves_to_resolve"

        # 死锁恢复：若双方“下一步都冲突”，尝试让低优先级机械臂后退一步
        # 用轻微回撤释放空间，再继续按原优先级前进。
        if self.enable_retreat_recovery and curr[low] > 0:
            low_prev = trajectories[low][prv[low]]
            if high == "ur5e":
                # low = fr3
                if not self.will_conflict(high_curr, low_prev):
                    new_idx = curr.copy()
                    new_idx[low] = prv[low]
                    self.retreat_cooldown[low] = self.retreat_cooldown_ticks
                    return new_idx, f"{low}_retreat"
            else:
                # low = ur5e
                if not self.will_conflict(low_prev, high_curr):
                    new_idx = curr.copy()
                    new_idx[low] = prv[low]
                    self.retreat_cooldown[low] = self.retreat_cooldown_ticks
                    return new_idx, f"{low}_retreat"

        # 次级恢复：低优先级无法回退时，尝试高优先级回退一步
        if self.enable_retreat_recovery and self.allow_high_priority_retreat and curr[high] > 0:
            high_prev = trajectories[high][prv[high]]
            if high == "ur5e":
                if not self.will_conflict(high_prev, low_curr):
                    new_idx = curr.copy()
                    new_idx[high] = prv[high]
                    self.retreat_cooldown[high] = self.retreat_cooldown_ticks
                    return new_idx, f"{high}_retreat"
            else:
                if not self.will_conflict(low_curr, high_prev):
                    new_idx = curr.copy()
                    new_idx[high] = prv[high]
                    self.retreat_cooldown[high] = self.retreat_cooldown_ticks
                    return new_idx, f"{high}_retreat"

        return curr, "deadlock_wait"


def append_segment(trajectory: List[np.ndarray], segment: List[np.ndarray]):
    if not segment:
        return
    if not trajectory:
        trajectory.extend(segment)
        return
    trajectory.extend(segment[1:])


def append_hold_segment(trajectory: List[np.ndarray], q_hold: np.ndarray, hold_steps: int):
    if hold_steps <= 0:
        return
    trajectory.extend([q_hold.copy() for _ in range(hold_steps)])


def prepend_wait_steps(trajectory: List[np.ndarray], q_hold: np.ndarray, wait_steps: int) -> List[np.ndarray]:
    if wait_steps <= 0:
        return [q.copy() for q in trajectory]
    return [q_hold.copy() for _ in range(wait_steps)] + [q.copy() for q in trajectory]


def build_time_optimal_trajectory(
    env: DualArmEnvironment,
    arm_name: str,
    sequence: List[str],
    sample_dt: float,
    hold_steps: int = 160,
    velocity_scale: float = 1.0,
    acceleration_scale: float = 1.0,
    return_home: bool = True,
) -> Tuple[List[np.ndarray], Dict[str, float], List[Dict[str, object]]]:
    planner = BiRRTPlanner(env, arm_name=arm_name, step_size=0.15)
    current_q = ARM_HOME_Q[arm_name].copy()
    trajectory = [current_q.copy()]
    timing = {
        "motion_time": 0.0,
        "hold_time": 0.0,
    }
    target_records: List[Dict[str, object]] = []

    for target in sequence:
        env.reset(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
        q_approach, q_target = calculate_target_posture(env, arm_name, target)
        if q_approach is None or q_target is None:
            print(f"⚠️ [{arm_name.upper()}] {target} 姿态不可行，已跳过该任务的 TOPP-RA 轨迹构建。")
            target_records.append({
                "target": target,
                "status": "skipped",
                "reason": "ik_infeasible",
            })
            continue

        if not planner._is_edge_collision_free(q_approach, q_target, allow_workpiece_contact=True):
            print(f"⚠️ [{arm_name.upper()}] {target} 压入段存在碰撞风险，已跳过。")
            target_records.append({
                "target": target,
                "status": "skipped",
                "reason": "insert_collision_risk",
            })
            continue

        path = planner.plan(current_q, q_approach)
        if path is None:
            print(f"⚠️ [{arm_name.upper()}] {target} 接近段路径规划失败，已跳过。")
            target_records.append({
                "target": target,
                "status": "skipped",
                "reason": "approach_plan_failed",
            })
            continue

        approach_segment, approach_duration = time_parameterize_path(
            path,
            arm_name,
            sample_dt=sample_dt,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        insert_segment, insert_duration = time_parameterize_path(
            [q_approach, q_target],
            arm_name,
            sample_dt=sample_dt,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        retract_segment, retract_duration = time_parameterize_path(
            [q_target, q_approach],
            arm_name,
            sample_dt=sample_dt,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )

        append_segment(trajectory, approach_segment)
        append_segment(trajectory, insert_segment)
        append_hold_segment(trajectory, q_target, hold_steps)
        append_segment(trajectory, retract_segment)
        timing["motion_time"] += approach_duration + insert_duration + retract_duration
        timing["hold_time"] += hold_steps * sample_dt
        current_q = q_approach.copy()
        target_records.append({
            "target": target,
            "status": "reached",
            "reason": "ok",
        })

    if return_home and np.linalg.norm(current_q - ARM_HOME_Q[arm_name]) > 1e-3:
        path_home = planner.plan(current_q, ARM_HOME_Q[arm_name].copy())
        if path_home is not None:
            home_segment, home_duration = time_parameterize_path(
                path_home,
                arm_name,
                sample_dt=sample_dt,
                velocity_scale=velocity_scale,
                acceleration_scale=acceleration_scale,
            )
            append_segment(trajectory, home_segment)
            timing["motion_time"] += home_duration
        else:
            trajectory.append(ARM_HOME_Q[arm_name].copy())

    return trajectory, timing, target_records


def summarize_target_records(arm_name: str, records: List[Dict[str, object]]) -> Dict[str, object]:
    reached = [r["target"] for r in records if r.get("status") == "reached"]
    skipped = [r for r in records if r.get("status") == "skipped"]

    skip_reason_cn = {
        "ik_infeasible": "IK不可行",
        "insert_collision_risk": "压入段碰撞风险",
        "approach_plan_failed": "接近段规划失败",
    }
    skipped_detail = [
        {
            "target": r.get("target", ""),
            "reason": r.get("reason", "unknown"),
            "reason_cn": skip_reason_cn.get(r.get("reason", ""), "未知原因"),
        }
        for r in skipped
    ]

    print("\n" + "-" * 72)
    print(f"[{arm_name.upper()}] 目标点执行摘要")
    print("-" * 72)
    print(f"到达数量: {len(reached)}")
    if reached:
        print(f"到达目标: {reached}")
    print(f"未到达数量: {len(skipped_detail)}")
    if skipped_detail:
        for item in skipped_detail:
            print(f"- {item['target']}: {item['reason_cn']} ({item['reason']})")

    return {
        "reached_count": len(reached),
        "reached_targets": reached,
        "skipped_count": len(skipped_detail),
        "skipped_targets": skipped_detail,
    }


def dry_run_schedule(
    env: DualArmEnvironment,
    ur5e_traj: List[np.ndarray],
    fr3_traj: List[np.ndarray],
    priority_arm: str,
    enable_retreat_recovery: bool = True,
    max_deadlock: int = 200,
    max_total_ticks: int = 120000,
    max_no_progress_ticks: int = 4000,
    anti_oscillation_trigger: int = 600,
    anti_oscillation_hold: int = 120,
    anti_oscillation_max_attempts: int = 4,
    disable_anti_oscillation: bool = False,
    retreat_cooldown_ticks: int = 8,
    allow_high_priority_retreat: bool = False,
) -> Dict[str, object]:
    env.reset(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
    scheduler = PriorityDelayScheduler(
        env,
        priority_arm=priority_arm,
        enable_retreat_recovery=enable_retreat_recovery,
        retreat_cooldown_ticks=retreat_cooldown_ticks,
        allow_high_priority_retreat=allow_high_priority_retreat,
    )
    indices = {"ur5e": 0, "fr3": 0}
    trajectories = {"ur5e": ur5e_traj, "fr3": fr3_traj}
    last_idx = {arm: len(trajectories[arm]) - 1 for arm in ["ur5e", "fr3"]}
    deadlock_counter = 0
    wait_counter = {"ur5e": 0, "fr3": 0}
    mode_counter: Dict[str, int] = {}
    tick = 0
    finish_tick: Dict[str, Optional[int]] = {
        "ur5e": 0 if indices["ur5e"] == last_idx["ur5e"] else None,
        "fr3": 0 if indices["fr3"] == last_idx["fr3"] else None,
    }
    best_progress = 0
    no_progress_counter = 0
    recovery_remaining = 0
    recovery_frozen_arm = ""
    recovery_attempts = 0
    base_priority = priority_arm

    while True:
        if tick > max_total_ticks:
            return {
                "success": False,
                "tick": tick,
                "wait_counter": wait_counter,
                "mode_counter": mode_counter,
                "reason": "max_total_ticks_exceeded",
            }

        if recovery_remaining > 0 and recovery_frozen_arm in ["ur5e", "fr3"]:
            frozen = recovery_frozen_arm
            mover = "fr3" if frozen == "ur5e" else "ur5e"
            last_idx = {arm: len(trajectories[arm]) - 1 for arm in ["ur5e", "fr3"]}
            candidate = indices.copy()
            if candidate[mover] < last_idx[mover]:
                candidate[mover] += 1
                ur_q = trajectories["ur5e"][candidate["ur5e"]]
                fr_q = trajectories["fr3"][candidate["fr3"]]
                if not scheduler.will_conflict(ur_q, fr_q):
                    next_indices, mode = candidate, f"{frozen}_forced_wait"
                    wait_counter[frozen] += 1
                else:
                    next_indices, mode = indices, "forced_wait_blocked"
            else:
                next_indices, mode = indices, "forced_wait_blocked"
            recovery_remaining -= 1
            if recovery_remaining <= 0:
                scheduler.priority_arm = base_priority
                scheduler.enable_retreat_recovery = enable_retreat_recovery
        else:
            next_indices, mode = scheduler.choose_next_indices(indices, trajectories)
        mode_counter[mode] = mode_counter.get(mode, 0) + 1
        if mode == "done":
            ur_finish = int(finish_tick["ur5e"] if finish_tick["ur5e"] is not None else tick)
            fr_finish = int(finish_tick["fr3"] if finish_tick["fr3"] is not None else tick)
            return {
                "success": True,
                "tick": tick,
                "wait_counter": wait_counter,
                "mode_counter": mode_counter,
                "finish_tick": {
                    "ur5e": ur_finish,
                    "fr3": fr_finish,
                },
                "finish_gap_ticks": abs(ur_finish - fr_finish),
            }

        if mode == "ur5e_wait":
            wait_counter["ur5e"] += 1
        elif mode == "fr3_wait":
            wait_counter["fr3"] += 1
        elif mode == "deadlock_wait":
            deadlock_counter += 1
            if deadlock_counter > max_deadlock:
                return {
                    "success": False,
                    "tick": tick,
                    "wait_counter": wait_counter,
                    "mode_counter": mode_counter,
                    "reason": "deadlock_wait",
                }
        else:
            deadlock_counter = 0

        indices = next_indices
        for arm in ["ur5e", "fr3"]:
            if finish_tick[arm] is None and indices[arm] == last_idx[arm]:
                finish_tick[arm] = tick + 1
        progress = indices["ur5e"] + indices["fr3"]
        if progress > best_progress:
            best_progress = progress
            no_progress_counter = 0
        else:
            no_progress_counter += 1
            trigger = max(1, min(anti_oscillation_trigger, max_no_progress_ticks))
            if (
                not disable_anti_oscillation
                and recovery_remaining <= 0
                and no_progress_counter > trigger
                and recovery_attempts < anti_oscillation_max_attempts
            ):
                # 先尝试“强制单臂等待 + 临时切换优先级”打破振荡
                recovery_attempts += 1
                recovery_remaining = max(1, anti_oscillation_hold)
                recovery_frozen_arm = "fr3" if scheduler.priority_arm == "ur5e" else "ur5e"
                scheduler.priority_arm = recovery_frozen_arm
                scheduler.enable_retreat_recovery = False
                mode_counter["anti_oscillation_triggered"] = mode_counter.get("anti_oscillation_triggered", 0) + 1
                no_progress_counter = 0
            elif no_progress_counter > max_no_progress_ticks:
                return {
                    "success": False,
                    "tick": tick,
                    "wait_counter": wait_counter,
                    "mode_counter": mode_counter,
                    "reason": "no_progress_loop",
                }
        tick += 1


def auto_delay_low_priority_trajectory(
    env: DualArmEnvironment,
    ur5e_traj: List[np.ndarray],
    fr3_traj: List[np.ndarray],
    priority_arm: str,
    enable_retreat_recovery: bool = True,
    max_start_delay: int = 600,
    coarse_step: int = 20,
    max_total_ticks: int = 120000,
    max_no_progress_ticks: int = 4000,
    anti_oscillation_trigger: int = 600,
    anti_oscillation_hold: int = 120,
    anti_oscillation_max_attempts: int = 4,
    disable_anti_oscillation: bool = False,
    retreat_cooldown_ticks: int = 8,
    allow_high_priority_retreat: bool = False,
    sync_balance_weight: float = 0.0,
) -> Tuple[List[np.ndarray], List[np.ndarray], Dict[str, object]]:
    baseline = dry_run_schedule(
        env,
        ur5e_traj,
        fr3_traj,
        priority_arm=priority_arm,
        enable_retreat_recovery=enable_retreat_recovery,
        max_total_ticks=max_total_ticks,
        max_no_progress_ticks=max_no_progress_ticks,
        anti_oscillation_trigger=anti_oscillation_trigger,
        anti_oscillation_hold=anti_oscillation_hold,
        anti_oscillation_max_attempts=anti_oscillation_max_attempts,
        disable_anti_oscillation=disable_anti_oscillation,
        retreat_cooldown_ticks=retreat_cooldown_ticks,
        allow_high_priority_retreat=allow_high_priority_retreat,
    )
    if baseline["success"]:
        if sync_balance_weight <= 0.0:
            return ur5e_traj, fr3_traj, {
                "applied": False,
                "delayed_arm": None,
                "delay_steps": 0,
                "preview": baseline,
            }
    else:
        if sync_balance_weight <= 0.0:
            return ur5e_traj, fr3_traj, {
                "applied": False,
                "delayed_arm": None,
                "delay_steps": 0,
                "preview": baseline,
            }

    low_priority_arm = "fr3" if priority_arm == "ur5e" else "ur5e"

    def _build_delayed_pair(delay_steps: int) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        if low_priority_arm == "ur5e":
            delayed_ur5e = prepend_wait_steps(ur5e_traj, ARM_HOME_Q["ur5e"], delay_steps)
            delayed_fr3 = [q.copy() for q in fr3_traj]
        else:
            delayed_ur5e = [q.copy() for q in ur5e_traj]
            delayed_fr3 = prepend_wait_steps(fr3_traj, ARM_HOME_Q["fr3"], delay_steps)
        return delayed_ur5e, delayed_fr3

    def _score(preview: Dict[str, object]) -> float:
        finish_gap_ticks = float(preview.get("finish_gap_ticks", 0.0))
        makespan_ticks = float(preview.get("tick", float("inf")))
        return makespan_ticks + float(sync_balance_weight) * finish_gap_ticks

    if sync_balance_weight > 0.0:
        coarse_candidates = [0] + list(range(coarse_step, max_start_delay + 1, coarse_step))
        best_record: Optional[Dict[str, object]] = None

        for delay_steps in coarse_candidates:
            delayed_ur5e, delayed_fr3 = _build_delayed_pair(delay_steps)
            preview = dry_run_schedule(
                env,
                delayed_ur5e,
                delayed_fr3,
                priority_arm=priority_arm,
                enable_retreat_recovery=enable_retreat_recovery,
                max_total_ticks=max_total_ticks,
                max_no_progress_ticks=max_no_progress_ticks,
                anti_oscillation_trigger=anti_oscillation_trigger,
                anti_oscillation_hold=anti_oscillation_hold,
                anti_oscillation_max_attempts=anti_oscillation_max_attempts,
                disable_anti_oscillation=disable_anti_oscillation,
                retreat_cooldown_ticks=retreat_cooldown_ticks,
                allow_high_priority_retreat=allow_high_priority_retreat,
            )
            if not preview["success"]:
                continue
            score = _score(preview)
            record = {
                "delay_steps": delay_steps,
                "preview": preview,
                "ur5e_traj": delayed_ur5e,
                "fr3_traj": delayed_fr3,
                "score": score,
            }
            if (
                best_record is None
                or score < best_record["score"]
                or (
                    abs(score - float(best_record["score"])) < 1e-9
                    and int(delay_steps) < int(best_record["delay_steps"])
                )
            ):
                best_record = record

        if best_record is None:
            return ur5e_traj, fr3_traj, {
                "applied": False,
                "delayed_arm": low_priority_arm,
                "delay_steps": 0,
                "preview": baseline,
                "warning": "同步化软目标搜索未找到可行解，回退到原始轨迹。",
            }

        center = int(best_record["delay_steps"])
        refine_start = max(0, center - coarse_step + 1)
        refine_end = min(max_start_delay, center + coarse_step - 1)
        for refined_delay in range(refine_start, refine_end + 1):
            if refined_delay == center:
                continue
            candidate_ur5e, candidate_fr3 = _build_delayed_pair(refined_delay)
            refined_preview = dry_run_schedule(
                env,
                candidate_ur5e,
                candidate_fr3,
                priority_arm=priority_arm,
                enable_retreat_recovery=enable_retreat_recovery,
                max_total_ticks=max_total_ticks,
                max_no_progress_ticks=max_no_progress_ticks,
                anti_oscillation_trigger=anti_oscillation_trigger,
                anti_oscillation_hold=anti_oscillation_hold,
                anti_oscillation_max_attempts=anti_oscillation_max_attempts,
                disable_anti_oscillation=disable_anti_oscillation,
                retreat_cooldown_ticks=retreat_cooldown_ticks,
                allow_high_priority_retreat=allow_high_priority_retreat,
            )
            if not refined_preview["success"]:
                continue
            refined_score = _score(refined_preview)
            if (
                refined_score < float(best_record["score"])
                or (
                    abs(refined_score - float(best_record["score"])) < 1e-9
                    and refined_delay < int(best_record["delay_steps"])
                )
            ):
                best_record = {
                    "delay_steps": refined_delay,
                    "preview": refined_preview,
                    "ur5e_traj": candidate_ur5e,
                    "fr3_traj": candidate_fr3,
                    "score": refined_score,
                }

        best_delay = int(best_record["delay_steps"])
        return best_record["ur5e_traj"], best_record["fr3_traj"], {
            "applied": best_delay > 0,
            "delayed_arm": low_priority_arm,
            "delay_steps": best_delay,
            "preview": best_record["preview"],
            "sync_balance_weight": float(sync_balance_weight),
            "selection_objective": float(best_record["score"]),
            "selection_objective_name": "makespan_ticks + lambda * finish_gap_ticks",
        }

    if baseline["success"]:
        return ur5e_traj, fr3_traj, {
            "applied": False,
            "delayed_arm": None,
            "delay_steps": 0,
            "preview": baseline,
        }

    coarse_candidates = list(range(coarse_step, max_start_delay + 1, coarse_step))
    for delay_steps in coarse_candidates:
        if low_priority_arm == "ur5e":
            delayed_ur5e = prepend_wait_steps(ur5e_traj, ARM_HOME_Q["ur5e"], delay_steps)
            delayed_fr3 = [q.copy() for q in fr3_traj]
        else:
            delayed_ur5e = [q.copy() for q in ur5e_traj]
            delayed_fr3 = prepend_wait_steps(fr3_traj, ARM_HOME_Q["fr3"], delay_steps)

        preview = dry_run_schedule(
            env,
            delayed_ur5e,
            delayed_fr3,
            priority_arm=priority_arm,
            enable_retreat_recovery=enable_retreat_recovery,
            max_total_ticks=max_total_ticks,
            max_no_progress_ticks=max_no_progress_ticks,
            anti_oscillation_trigger=anti_oscillation_trigger,
            anti_oscillation_hold=anti_oscillation_hold,
            anti_oscillation_max_attempts=anti_oscillation_max_attempts,
            disable_anti_oscillation=disable_anti_oscillation,
            retreat_cooldown_ticks=retreat_cooldown_ticks,
            allow_high_priority_retreat=allow_high_priority_retreat,
        )
        if not preview["success"]:
            continue

        refine_start = max(0, delay_steps - coarse_step + 1)
        best_delay = delay_steps
        best_preview = preview
        best_ur5e = delayed_ur5e
        best_fr3 = delayed_fr3

        for refined_delay in range(refine_start, delay_steps):
            if low_priority_arm == "ur5e":
                candidate_ur5e = prepend_wait_steps(ur5e_traj, ARM_HOME_Q["ur5e"], refined_delay)
                candidate_fr3 = [q.copy() for q in fr3_traj]
            else:
                candidate_ur5e = [q.copy() for q in ur5e_traj]
                candidate_fr3 = prepend_wait_steps(fr3_traj, ARM_HOME_Q["fr3"], refined_delay)

            refined_preview = dry_run_schedule(
                env,
                candidate_ur5e,
                candidate_fr3,
                priority_arm=priority_arm,
                enable_retreat_recovery=enable_retreat_recovery,
                max_total_ticks=max_total_ticks,
                max_no_progress_ticks=max_no_progress_ticks,
                anti_oscillation_trigger=anti_oscillation_trigger,
                anti_oscillation_hold=anti_oscillation_hold,
                anti_oscillation_max_attempts=anti_oscillation_max_attempts,
                disable_anti_oscillation=disable_anti_oscillation,
                retreat_cooldown_ticks=retreat_cooldown_ticks,
                allow_high_priority_retreat=allow_high_priority_retreat,
            )
            if refined_preview["success"]:
                best_delay = refined_delay
                best_preview = refined_preview
                best_ur5e = candidate_ur5e
                best_fr3 = candidate_fr3
                break

        return best_ur5e, best_fr3, {
            "applied": True,
            "delayed_arm": low_priority_arm,
            "delay_steps": best_delay,
            "preview": best_preview,
        }

    return ur5e_traj, fr3_traj, {
        "applied": False,
        "delayed_arm": low_priority_arm,
        "delay_steps": 0,
        "preview": baseline,
        "warning": "未找到可消除死锁的起始延迟。",
    }


def simulate_synchronized_execution(
    env: DualArmEnvironment,
    viewer,
    ur5e_traj: List[np.ndarray],
    fr3_traj: List[np.ndarray],
    priority_arm: str = "ur5e",
    enable_retreat_recovery: bool = True,
    sim_steps_per_tick: int = 4,
    tick_sleep: float = 0.001,
    debug_first_ticks: int = 0,
    max_total_ticks: int = 120000,
    max_no_progress_ticks: int = 4000,
    anti_oscillation_trigger: int = 600,
    anti_oscillation_hold: int = 120,
    anti_oscillation_max_attempts: int = 4,
    disable_anti_oscillation: bool = False,
    retreat_cooldown_ticks: int = 8,
    allow_high_priority_retreat: bool = False,
    sample_dt: float = 0.002,
    max_cmd_step: float = 0.035,
    jump_step_threshold: float = 0.06,
    jump_velocity_threshold: float = 25.0,
):
    scheduler = PriorityDelayScheduler(
        env,
        priority_arm=priority_arm,
        enable_retreat_recovery=enable_retreat_recovery,
        retreat_cooldown_ticks=retreat_cooldown_ticks,
        allow_high_priority_retreat=allow_high_priority_retreat,
    )
    indices = {"ur5e": 0, "fr3": 0}
    trajectories = {"ur5e": ur5e_traj, "fr3": fr3_traj}
    wait_counter = {"ur5e": 0, "fr3": 0}
    mode_counter: Dict[str, int] = {}
    deadlock_counter = 0
    max_deadlock = 200
    tick = 0
    success = True
    best_progress = 0
    no_progress_counter = 0
    recovery_remaining = 0
    recovery_frozen_arm = ""
    recovery_attempts = 0
    base_priority = priority_arm
    prev_applied_cmd = {
        "ur5e": trajectories["ur5e"][0].copy(),
        "fr3": trajectories["fr3"][0].copy(),
    }
    jump_summary = {
        "raw": {
            "ur5e": {"max_step": 0.0, "max_eq_velocity": 0.0, "exceed_count": 0},
            "fr3": {"max_step": 0.0, "max_eq_velocity": 0.0, "exceed_count": 0},
        },
        "applied": {
            "ur5e": {"max_step": 0.0, "max_eq_velocity": 0.0, "exceed_count": 0},
            "fr3": {"max_step": 0.0, "max_eq_velocity": 0.0, "exceed_count": 0},
        },
        "thresholds": {
            "jump_step_threshold": float(jump_step_threshold),
            "jump_velocity_threshold": float(jump_velocity_threshold),
        },
    }

    def _update_jump_stats(kind: str, arm: str, delta: np.ndarray):
        max_step = float(np.max(np.abs(delta))) if delta.size else 0.0
        eq_vel = max_step / max(sample_dt, 1e-9)
        stats = jump_summary[kind][arm]
        stats["max_step"] = max(stats["max_step"], max_step)
        stats["max_eq_velocity"] = max(stats["max_eq_velocity"], eq_vel)
        if max_step > jump_step_threshold or eq_vel > jump_velocity_threshold:
            stats["exceed_count"] += 1

    while viewer.is_running():
        if tick > max_total_ticks:
            print("❌ 调度超时：超过最大 tick，已终止本次并行执行。")
            success = False
            mode_counter["max_total_ticks_exceeded"] = mode_counter.get("max_total_ticks_exceeded", 0) + 1
            break

        if recovery_remaining > 0 and recovery_frozen_arm in ["ur5e", "fr3"]:
            frozen = recovery_frozen_arm
            mover = "fr3" if frozen == "ur5e" else "ur5e"
            last_idx = {arm: len(trajectories[arm]) - 1 for arm in ["ur5e", "fr3"]}
            candidate = indices.copy()
            if candidate[mover] < last_idx[mover]:
                candidate[mover] += 1
                ur_q = trajectories["ur5e"][candidate["ur5e"]]
                fr_q = trajectories["fr3"][candidate["fr3"]]
                if not scheduler.will_conflict(ur_q, fr_q):
                    next_indices, mode = candidate, f"{frozen}_forced_wait"
                    wait_counter[frozen] += 1
                else:
                    next_indices, mode = indices, "forced_wait_blocked"
            else:
                next_indices, mode = indices, "forced_wait_blocked"
            recovery_remaining -= 1
            if recovery_remaining <= 0:
                scheduler.priority_arm = base_priority
                scheduler.enable_retreat_recovery = enable_retreat_recovery
        else:
            next_indices, mode = scheduler.choose_next_indices(indices, trajectories)
        mode_counter[mode] = mode_counter.get(mode, 0) + 1
        if mode == "done":
            break

        if mode == "ur5e_wait":
            wait_counter["ur5e"] += 1
        elif mode == "fr3_wait":
            wait_counter["fr3"] += 1
        elif mode == "deadlock_wait":
            deadlock_counter += 1
            if deadlock_counter > max_deadlock:
                print("❌ 调度器进入持续死锁等待，已终止本次并行执行。")
                success = False
                break
        else:
            deadlock_counter = 0

        indices = next_indices
        progress = indices["ur5e"] + indices["fr3"]
        if progress > best_progress:
            best_progress = progress
            no_progress_counter = 0
        else:
            no_progress_counter += 1
            trigger = max(1, min(anti_oscillation_trigger, max_no_progress_ticks))
            if (
                not disable_anti_oscillation
                and recovery_remaining <= 0
                and no_progress_counter > trigger
                and recovery_attempts < anti_oscillation_max_attempts
            ):
                recovery_attempts += 1
                recovery_remaining = max(1, anti_oscillation_hold)
                recovery_frozen_arm = "fr3" if scheduler.priority_arm == "ur5e" else "ur5e"
                scheduler.priority_arm = recovery_frozen_arm
                scheduler.enable_retreat_recovery = False
                mode_counter["anti_oscillation_triggered"] = mode_counter.get("anti_oscillation_triggered", 0) + 1
                print(
                    f"⚠️ 触发防振荡恢复: attempt={recovery_attempts}, "
                    f"frozen={recovery_frozen_arm}, hold_ticks={recovery_remaining}"
                )
                no_progress_counter = 0
            elif no_progress_counter > max_no_progress_ticks:
                print("❌ 调度无净进展：长时间回退/循环，已终止本次并行执行。")
                success = False
                mode_counter["no_progress_loop"] = mode_counter.get("no_progress_loop", 0) + 1
                break

        ur5e_target = trajectories["ur5e"][indices["ur5e"]]
        fr3_target = trajectories["fr3"][indices["fr3"]]

        raw_delta_ur5e = ur5e_target - prev_applied_cmd["ur5e"]
        raw_delta_fr3 = fr3_target - prev_applied_cmd["fr3"]
        _update_jump_stats("raw", "ur5e", raw_delta_ur5e)
        _update_jump_stats("raw", "fr3", raw_delta_fr3)

        if max_cmd_step > 0:
            ur5e_step = np.clip(raw_delta_ur5e, -max_cmd_step, max_cmd_step)
            fr3_step = np.clip(raw_delta_fr3, -max_cmd_step, max_cmd_step)
            ur5e_cmd = prev_applied_cmd["ur5e"] + ur5e_step
            fr3_cmd = prev_applied_cmd["fr3"] + fr3_step
        else:
            ur5e_cmd = ur5e_target
            fr3_cmd = fr3_target

        applied_delta_ur5e = ur5e_cmd - prev_applied_cmd["ur5e"]
        applied_delta_fr3 = fr3_cmd - prev_applied_cmd["fr3"]
        _update_jump_stats("applied", "ur5e", applied_delta_ur5e)
        _update_jump_stats("applied", "fr3", applied_delta_fr3)
        prev_applied_cmd["ur5e"] = ur5e_cmd.copy()
        prev_applied_cmd["fr3"] = fr3_cmd.copy()

        if tick < debug_first_ticks:
            print(
                f"[并行调试] tick={tick} mode={mode} "
                f"ur5e_idx={indices['ur5e']} fr3_idx={indices['fr3']} "
                f"ur5e_q0={ur5e_cmd[0]:.3f} fr3_q0={fr3_cmd[0]:.3f}"
            )

        for _ in range(sim_steps_per_tick):
            if not viewer.is_running():
                break
            env.step(ur5e_cmd, fr3_cmd)
            viewer.sync()
            time.sleep(tick_sleep)

        tick += 1
        if tick % 200 == 0:
            print(f"[调度进度] tick={tick}, ur5e_idx={indices['ur5e']}/{len(ur5e_traj)-1}, fr3_idx={indices['fr3']}/{len(fr3_traj)-1}, waits={wait_counter}")

    return {
        "success": success,
        "tick": tick,
        "wait_counter": wait_counter,
        "mode_counter": mode_counter,
        "jump_summary": jump_summary,
    }


def summarize_schedule_result(
    result: Dict[str, object],
    ur5e_traj: List[np.ndarray],
    fr3_traj: List[np.ndarray],
    sample_dt: float,
) -> Dict[str, object]:
    ur5e_ticks = max(0, len(ur5e_traj) - 1)
    fr3_ticks = max(0, len(fr3_traj) - 1)
    parallel_ticks = int(result.get("tick", 0))
    wait_counter = result.get("wait_counter", {"ur5e": 0, "fr3": 0})

    serial_makespan = (ur5e_ticks + fr3_ticks) * sample_dt
    ideal_parallel_makespan = max(ur5e_ticks, fr3_ticks) * sample_dt
    success = bool(result.get("success", False))
    parallel_makespan = (parallel_ticks * sample_dt) if success else float("inf")
    wait_time_ur5e = float(wait_counter.get("ur5e", 0)) * sample_dt
    wait_time_fr3 = float(wait_counter.get("fr3", 0)) * sample_dt
    total_wait_time = wait_time_ur5e + wait_time_fr3

    speedup_vs_serial = (serial_makespan / parallel_makespan) if (success and parallel_makespan > 1e-12) else 0.0
    overhead_vs_ideal = max(0.0, parallel_makespan - ideal_parallel_makespan) if success else float("inf")

    summary = {
        "success": success,
        "parallel_ticks": parallel_ticks,
        "parallel_makespan_s": parallel_makespan,
        "serial_makespan_s": serial_makespan,
        "ideal_parallel_makespan_s": ideal_parallel_makespan,
        "speedup_vs_serial": speedup_vs_serial,
        "overhead_vs_ideal_parallel_s": overhead_vs_ideal,
        "wait_ticks": {
            "ur5e": int(wait_counter.get("ur5e", 0)),
            "fr3": int(wait_counter.get("fr3", 0)),
        },
        "wait_time_s": {
            "ur5e": wait_time_ur5e,
            "fr3": wait_time_fr3,
            "total": total_wait_time,
        },
        "mode_counter": result.get("mode_counter", {}),
    }
    finish_tick = result.get("finish_tick")
    if isinstance(finish_tick, dict):
        ur_finish = int(finish_tick.get("ur5e", parallel_ticks))
        fr_finish = int(finish_tick.get("fr3", parallel_ticks))
        summary["finish_tick"] = {
            "ur5e": ur_finish,
            "fr3": fr_finish,
        }
        summary["finish_time_s"] = {
            "ur5e": ur_finish * sample_dt,
            "fr3": fr_finish * sample_dt,
        }
        summary["finish_gap_ticks"] = abs(ur_finish - fr_finish)
        summary["finish_gap_s"] = abs(ur_finish - fr_finish) * sample_dt
    if "jump_summary" in result:
        summary["jump_summary"] = result["jump_summary"]
    return summary


def print_stats(title: str, stats: Dict[str, object]):
    print("\n" + "-" * 72)
    print(title)
    print("-" * 72)
    print(f"完成状态: {'成功' if stats['success'] else '失败'}")
    parallel_text = f"{stats['parallel_makespan_s']:.3f} s" if np.isfinite(stats["parallel_makespan_s"]) else "不可达(调度失败)"
    overhead_text = f"{stats['overhead_vs_ideal_parallel_s']:.3f} s" if np.isfinite(stats["overhead_vs_ideal_parallel_s"]) else "不可达"
    print(f"并行 makespan: {parallel_text}")
    print(f"串行 makespan: {stats['serial_makespan_s']:.3f} s")
    print(f"理想并行下界: {stats['ideal_parallel_makespan_s']:.3f} s")
    print(f"相对串行加速比: {stats['speedup_vs_serial']:.3f}x")
    print(f"相对理想并行额外开销: {overhead_text}")
    print(
        f"等待时间: ur5e={stats['wait_time_s']['ur5e']:.3f} s, "
        f"fr3={stats['wait_time_s']['fr3']:.3f} s, total={stats['wait_time_s']['total']:.3f} s"
    )
    print(f"模式统计: {stats['mode_counter']}")
    jump = stats.get("jump_summary", {})
    if jump:
        raw_ur5e = jump.get("raw", {}).get("ur5e", {})
        app_ur5e = jump.get("applied", {}).get("ur5e", {})
        print(
            "指令跳变(UR5e): "
            f"raw_max_step={raw_ur5e.get('max_step', 0.0):.4f}, "
            f"applied_max_step={app_ur5e.get('max_step', 0.0):.4f}, "
            f"applied_exceed_count={int(app_ur5e.get('exceed_count', 0))}"
        )


def build_comparison_rows(final_stats: Dict[str, object], ablation_stats: Dict[str, Dict[str, object]]) -> List[Dict[str, object]]:
    rows = []

    def _add_row(method_name: str, stats: Dict[str, object]):
        rows.append({
            "method": method_name,
            "success": int(bool(stats["success"])),
            "parallel_makespan_s": stats["parallel_makespan_s"],
            "serial_makespan_s": stats["serial_makespan_s"],
            "ideal_parallel_makespan_s": stats["ideal_parallel_makespan_s"],
            "speedup_vs_serial": stats["speedup_vs_serial"],
            "overhead_vs_ideal_parallel_s": stats["overhead_vs_ideal_parallel_s"],
            "wait_time_total_s": stats["wait_time_s"]["total"],
            "wait_time_ur5e_s": stats["wait_time_s"]["ur5e"],
            "wait_time_fr3_s": stats["wait_time_s"]["fr3"],
            "wait_ticks_ur5e": stats["wait_ticks"]["ur5e"],
            "wait_ticks_fr3": stats["wait_ticks"]["fr3"],
            "both_move_count": int(stats["mode_counter"].get("both_move", 0)),
            "deadlock_wait_count": int(stats["mode_counter"].get("deadlock_wait", 0)),
        })

    if "auto_delay_off" in ablation_stats:
        _add_row("auto_delay_off_preview", ablation_stats["auto_delay_off"])
    if "auto_delay_on" in ablation_stats:
        _add_row("auto_delay_on_preview", ablation_stats["auto_delay_on"])
    _add_row("execution_actual", final_stats)
    return rows


def _fmt_value(v):
    if isinstance(v, float):
        if np.isfinite(v):
            return f"{v:.6f}"
        return "inf"
    return str(v)


def write_comparison_csv(rows: List[Dict[str, object]], output_path: Path):
    if not rows:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _fmt_value(v) for k, v in row.items()})


def write_comparison_markdown(rows: List[Dict[str, object]], output_path: Path):
    if not rows:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0].keys())
    header = "| " + " | ".join(columns) + " |"
    sep = "| " + " | ".join(["---"] * len(columns)) + " |"
    lines = [header, sep]
    for row in rows:
        lines.append("| " + " | ".join(_fmt_value(row[c]) for c in columns) + " |")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_comparison_table(rows: List[Dict[str, object]]):
    if not rows:
        return
    print("\n" + "=" * 72)
    print("方法对比表（核心评价指标）")
    print("=" * 72)
    columns = [
        "method",
        "success",
        "parallel_makespan_s",
        "serial_makespan_s",
        "speedup_vs_serial",
        "wait_time_total_s",
        "deadlock_wait_count",
    ]
    print(" | ".join(columns))
    for row in rows:
        print(" | ".join(_fmt_value(row[c]) for c in columns))


def parse_args():
    parser = argparse.ArgumentParser(description="基于 TOPP-RA 的冲突检测 + 优先级等待时间调度脚本")
    parser.add_argument("--solution", default="xml_pipeline_run/ga_best_solution.json", help="GA 最优解 JSON 路径")
    parser.add_argument("--priority-arm", choices=["ur5e", "fr3"], default="ur5e", help="冲突时的高优先级机械臂")
    parser.add_argument("--headless", action="store_true", help="无界面运行，用于快速验证")
    parser.add_argument("--sim-steps-per-tick", type=int, default=4, help="每个调度 tick 的物理步数")
    parser.add_argument("--tick-sleep", type=float, default=0.001, help="每个物理步的休眠秒数，批量实验可设为 0")
    parser.add_argument("--hold-steps", type=int, default=160, help="每个任务到达目标位后的名义保持步数")
    parser.add_argument("--velocity-scale", type=float, default=1.0, help="统一缩放关节速度上限，默认 1.0")
    parser.add_argument("--acceleration-scale", type=float, default=1.0, help="统一缩放关节加速度上限，默认 1.0")
    parser.add_argument("--disable-auto-delay", action="store_true", help="禁用自动起始延迟，便于放大等待策略差异")
    parser.add_argument("--disable-retreat-recovery", action="store_true", help="禁用死锁回退恢复策略（用于对照）")
    parser.add_argument("--max-start-delay", type=int, default=600, help="自动延迟搜索最大 tick")
    parser.add_argument("--delay-coarse-step", type=int, default=20, help="自动延迟粗搜索步长")
    parser.add_argument("--debug-first-ticks", type=int, default=0, help="打印前 N 个调度 tick 的双臂并行下发信息")
    parser.add_argument("--max-total-ticks", type=int, default=120000, help="调度执行最大 tick（防止长时间卡住）")
    parser.add_argument("--max-no-progress-ticks", type=int, default=4000, help="允许无净进展的最大 tick（防循环）")
    parser.add_argument("--disable-anti-oscillation", action="store_true", help="禁用防振荡恢复机制（用于对照）")
    parser.add_argument("--anti-oscillation-trigger", type=int, default=600, help="无净进展超过该值触发防振荡恢复")
    parser.add_argument("--anti-oscillation-hold", type=int, default=120, help="防振荡时强制单臂等待 tick")
    parser.add_argument("--anti-oscillation-max-attempts", type=int, default=4, help="单次调度最多触发防振荡次数")
    parser.add_argument("--retreat-cooldown-ticks", type=int, default=8, help="回退后冷却 tick，避免立刻反向抢进")
    parser.add_argument("--allow-high-priority-retreat", action="store_true", help="允许高优先级机械臂回退（默认仅低优先级回退）")
    parser.add_argument("--max-cmd-step", type=float, default=0.035, help="执行阶段每 tick 关节最大步长(rad)，<=0 表示不限制")
    parser.add_argument("--jump-step-threshold", type=float, default=0.06, help="Δq 超阈值统计(rad/tick)")
    parser.add_argument("--jump-velocity-threshold", type=float, default=25.0, help="等效速度超阈值统计(rad/s)")
    parser.add_argument("--sync-balance-weight", type=float, default=0.0, help="软同步权重 λ（>0 时在自动延迟搜索中惩罚双臂完工时间差）")
    parser.add_argument("--random-seed", type=int, default=-1, help="随机种子（>=0 时固定RRT随机性，便于复现实验）")
    parser.add_argument("--disable-auto-organize", action="store_true", help="禁用实验结束后的自动结果整理")
    parser.add_argument("--report-ablation", action="store_true", help="输出 auto-delay 开/关 的调度统计对比（不改算法）")
    parser.add_argument("--stats-json", default="", help="将统计结果写入 JSON 文件路径")
    parser.add_argument("--table-csv", default="", help="将方法对比表写入 CSV 文件路径")
    parser.add_argument("--table-md", default="", help="将方法对比表写入 Markdown 文件路径")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.random_seed >= 0:
        np.random.seed(args.random_seed)
        random.seed(args.random_seed)
        print(f"已固定随机种子: {args.random_seed}")

    def _auto_organize_outputs():
        if args.disable_auto_organize:
            return
        organizer = Path(__file__).resolve().parent / "organize_outputs.py"
        if not organizer.exists():
            return
        try:
            print("🧹 正在自动整理输出文件...")
            subprocess.run(
                [sys.executable, str(organizer), "--root", str(Path(__file__).resolve().parent), "--apply"],
                cwd=str(Path(__file__).resolve().parent),
                check=False,
            )
        except Exception as e:
            print(f"⚠️ 自动整理输出失败（不影响主流程）：{e}")
    solution = load_solution(Path(args.solution))
    ur5e_sequence = solution.get("ur5e_sequence", [])
    fr3_sequence = solution.get("fr3_sequence", [])

    env = DualArmEnvironment("dual_arm_scene.xml")
    env.reset(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
    sample_dt = env.model.opt.timestep * args.sim_steps_per_tick

    print("\n" + "=" * 72)
    print("正在构建双臂 TOPP-RA 时间最优轨迹...")
    print("=" * 72)
    ur5e_traj, ur5e_timing, ur5e_target_records = build_time_optimal_trajectory(
        env,
        "ur5e",
        ur5e_sequence,
        sample_dt=sample_dt,
        hold_steps=args.hold_steps,
        velocity_scale=args.velocity_scale,
        acceleration_scale=args.acceleration_scale,
    )
    fr3_traj, fr3_timing, fr3_target_records = build_time_optimal_trajectory(
        env,
        "fr3",
        fr3_sequence,
        sample_dt=sample_dt,
        hold_steps=args.hold_steps,
        velocity_scale=args.velocity_scale,
        acceleration_scale=args.acceleration_scale,
    )
    print(f"UR5e 轨迹点数: {len(ur5e_traj)}")
    print(f"FR3  轨迹点数: {len(fr3_traj)}")
    print(f"统一调度采样周期: {sample_dt:.4f} s")
    print(f"UR5e 运动时间: {ur5e_timing['motion_time']:.3f} s, 保持时间: {ur5e_timing['hold_time']:.3f} s")
    print(f"FR3  运动时间: {fr3_timing['motion_time']:.3f} s, 保持时间: {fr3_timing['hold_time']:.3f} s")
    ur5e_target_summary = summarize_target_records("ur5e", ur5e_target_records)
    fr3_target_summary = summarize_target_records("fr3", fr3_target_records)

    raw_ur5e_traj = [q.copy() for q in ur5e_traj]
    raw_fr3_traj = [q.copy() for q in fr3_traj]
    ablation_stats = {}
    enable_retreat_recovery = not args.disable_retreat_recovery

    if args.report_ablation:
        preview_off = dry_run_schedule(
            env,
            raw_ur5e_traj,
            raw_fr3_traj,
            priority_arm=args.priority_arm,
            enable_retreat_recovery=enable_retreat_recovery,
            max_total_ticks=args.max_total_ticks,
            max_no_progress_ticks=args.max_no_progress_ticks,
            anti_oscillation_trigger=args.anti_oscillation_trigger,
            anti_oscillation_hold=args.anti_oscillation_hold,
            anti_oscillation_max_attempts=args.anti_oscillation_max_attempts,
            disable_anti_oscillation=args.disable_anti_oscillation,
            retreat_cooldown_ticks=args.retreat_cooldown_ticks,
            allow_high_priority_retreat=args.allow_high_priority_retreat,
        )
        stats_off = summarize_schedule_result(preview_off, raw_ur5e_traj, raw_fr3_traj, sample_dt)
        ablation_stats["auto_delay_off"] = stats_off
        print_stats("[消融对比] auto-delay 关闭（预览）", stats_off)

    if args.disable_auto_delay:
        print("已禁用自动起始延迟：将直接使用原始双臂 TOPP-RA 轨迹并行执行。")
    else:
        ur5e_traj, fr3_traj, delay_info = auto_delay_low_priority_trajectory(
            env,
            ur5e_traj,
            fr3_traj,
            priority_arm=args.priority_arm,
            enable_retreat_recovery=enable_retreat_recovery,
            max_start_delay=args.max_start_delay,
            coarse_step=args.delay_coarse_step,
            max_total_ticks=args.max_total_ticks,
            max_no_progress_ticks=args.max_no_progress_ticks,
            anti_oscillation_trigger=args.anti_oscillation_trigger,
            anti_oscillation_hold=args.anti_oscillation_hold,
            anti_oscillation_max_attempts=args.anti_oscillation_max_attempts,
            disable_anti_oscillation=args.disable_anti_oscillation,
            retreat_cooldown_ticks=args.retreat_cooldown_ticks,
            allow_high_priority_retreat=args.allow_high_priority_retreat,
            sync_balance_weight=args.sync_balance_weight,
        )
        if delay_info.get("applied"):
            delay_time = delay_info["delay_steps"] * sample_dt
            print(
                f"已自动为低优先级机械臂 {delay_info['delayed_arm'].upper()} 增加起始延迟: "
                f"{delay_info['delay_steps']} tick ({delay_time:.3f} s)"
            )
            if args.sync_balance_weight > 0:
                finish_gap = delay_info.get("preview", {}).get("finish_gap_ticks", None)
                print(
                    f"同步化软目标已启用: λ={args.sync_balance_weight:.3f}, "
                    f"objective={delay_info.get('selection_objective', float('nan')):.3f}, "
                    f"finish_gap_ticks={finish_gap}"
                )
        elif args.sync_balance_weight > 0:
            finish_gap = delay_info.get("preview", {}).get("finish_gap_ticks", None)
            print(
                f"同步化软目标已启用: λ={args.sync_balance_weight:.3f}, "
                f"选择 delay=0, finish_gap_ticks={finish_gap}"
            )
        elif delay_info.get("warning"):
            print(f"⚠️ {delay_info['warning']}")

    if args.report_ablation:
        preview_on = dry_run_schedule(
            env,
            ur5e_traj,
            fr3_traj,
            priority_arm=args.priority_arm,
            enable_retreat_recovery=enable_retreat_recovery,
            max_total_ticks=args.max_total_ticks,
            max_no_progress_ticks=args.max_no_progress_ticks,
            anti_oscillation_trigger=args.anti_oscillation_trigger,
            anti_oscillation_hold=args.anti_oscillation_hold,
            anti_oscillation_max_attempts=args.anti_oscillation_max_attempts,
            disable_anti_oscillation=args.disable_anti_oscillation,
            retreat_cooldown_ticks=args.retreat_cooldown_ticks,
            allow_high_priority_retreat=args.allow_high_priority_retreat,
        )
        stats_on = summarize_schedule_result(preview_on, ur5e_traj, fr3_traj, sample_dt)
        ablation_stats["auto_delay_on"] = stats_on
        print_stats("[消融对比] auto-delay 开启（预览）", stats_on)

    env.reset(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
    viewer = _HeadlessViewer()
    viewer_manager = None
    if not args.headless:
        viewer_manager = mujoco.viewer.launch_passive(env.model, env.data)
        viewer = viewer_manager.__enter__()

    try:
        print("\n" + "=" * 72)
        print(f"开始执行 TOPP-RA 并行时间调度，优先级: {args.priority_arm.upper()}")
        print("并行驱动说明：每个物理步都同时调用 env.step(ur5e_cmd, fr3_cmd)。")
        print("=" * 72)
        run_result = simulate_synchronized_execution(
            env,
            viewer,
            ur5e_traj,
            fr3_traj,
            priority_arm=args.priority_arm,
            enable_retreat_recovery=enable_retreat_recovery,
            sim_steps_per_tick=args.sim_steps_per_tick,
            tick_sleep=args.tick_sleep,
            debug_first_ticks=args.debug_first_ticks,
            max_total_ticks=args.max_total_ticks,
            max_no_progress_ticks=args.max_no_progress_ticks,
            anti_oscillation_trigger=args.anti_oscillation_trigger,
            anti_oscillation_hold=args.anti_oscillation_hold,
            anti_oscillation_max_attempts=args.anti_oscillation_max_attempts,
            disable_anti_oscillation=args.disable_anti_oscillation,
            retreat_cooldown_ticks=args.retreat_cooldown_ticks,
            allow_high_priority_retreat=args.allow_high_priority_retreat,
            sample_dt=sample_dt,
            max_cmd_step=args.max_cmd_step,
            jump_step_threshold=args.jump_step_threshold,
            jump_velocity_threshold=args.jump_velocity_threshold,
        )

        print("\n✅ 第一版并行调度执行结束！")
        print(f"总调度 tick: {run_result['tick']}")
        print(f"等待统计: {run_result['wait_counter']}")
        final_stats = summarize_schedule_result(run_result, ur5e_traj, fr3_traj, sample_dt)
        print_stats("[本次实际执行统计]", final_stats)

        comparison_rows = build_comparison_rows(final_stats, ablation_stats)
        print_comparison_table(comparison_rows)

        if args.table_csv:
            table_csv_path = Path(args.table_csv)
            write_comparison_csv(comparison_rows, table_csv_path)
            print(f"🧾 已写出方法对比表 CSV: {table_csv_path}")
        if args.table_md:
            table_md_path = Path(args.table_md)
            write_comparison_markdown(comparison_rows, table_md_path)
            print(f"🧾 已写出方法对比表 Markdown: {table_md_path}")

        if args.stats_json:
            stats_payload = {
                "config": {
                    "solution": args.solution,
                    "priority_arm": args.priority_arm,
                    "headless": args.headless,
                    "sim_steps_per_tick": args.sim_steps_per_tick,
                    "hold_steps": args.hold_steps,
                    "velocity_scale": args.velocity_scale,
                    "acceleration_scale": args.acceleration_scale,
                    "disable_auto_delay": args.disable_auto_delay,
                    "disable_retreat_recovery": args.disable_retreat_recovery,
                    "max_start_delay": args.max_start_delay,
                    "delay_coarse_step": args.delay_coarse_step,
                    "sample_dt": sample_dt,
                    "max_total_ticks": args.max_total_ticks,
                    "max_no_progress_ticks": args.max_no_progress_ticks,
                    "disable_anti_oscillation": args.disable_anti_oscillation,
                    "anti_oscillation_trigger": args.anti_oscillation_trigger,
                    "anti_oscillation_hold": args.anti_oscillation_hold,
                    "anti_oscillation_max_attempts": args.anti_oscillation_max_attempts,
                    "retreat_cooldown_ticks": args.retreat_cooldown_ticks,
                    "allow_high_priority_retreat": args.allow_high_priority_retreat,
                    "max_cmd_step": args.max_cmd_step,
                    "jump_step_threshold": args.jump_step_threshold,
                    "jump_velocity_threshold": args.jump_velocity_threshold,
                    "sync_balance_weight": args.sync_balance_weight,
                    "random_seed": args.random_seed,
                },
                "trajectory_info": {
                    "ur5e_points": len(ur5e_traj),
                    "fr3_points": len(fr3_traj),
                    "ur5e_timing": ur5e_timing,
                    "fr3_timing": fr3_timing,
                    "target_summary": {
                        "ur5e": ur5e_target_summary,
                        "fr3": fr3_target_summary,
                    },
                },
                "execution_stats": final_stats,
                "ablation_preview": ablation_stats,
                "comparison_rows": comparison_rows,
            }
            stats_path = Path(args.stats_json)
            stats_path.parent.mkdir(parents=True, exist_ok=True)
            stats_path.write_text(json.dumps(stats_payload, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"📊 已写出统计结果: {stats_path}")

        while viewer.is_running() and not args.headless:
            env.step(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
            viewer.sync()
            time.sleep(0.01)
    finally:
        if viewer_manager is not None:
            viewer_manager.__exit__(None, None, None)
        _auto_organize_outputs()


if __name__ == "__main__":
    main()
