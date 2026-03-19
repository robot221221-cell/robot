"""
异构双臂系统：基于 Bi-RRT 与 DLS-IK 的多目标无碰撞轨迹规划框架
作者：硕士论文项目
"""

import argparse
import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import Tuple, List
from trajectory_smoothing import b_spline_smooth
from rrt_planner import BiRRTPlanner


ARM_HOME_Q = {
    'ur5e': np.array([0.0, -1.57, 0.5, -1.57, -1.57, 0.0]),
    'fr3': np.array([0.0, -1.2, 0.0, -2.4, 0.0, 1.57, 0.785]),
}

DEFAULT_TASK_SEQUENCE = {
    'ur5e': ['target_left_1', 'target_top_1', 'target_right_1'],
    'fr3': ['target_right_1', 'target_top_1', 'target_left_1'],
}

ALL_TARGETS = [
    'target_top_1', 'target_top_2', 'target_top_3', 'target_top_4',
    'target_front_1', 'target_front_2', 'target_front_3', 'target_front_4',
    'target_back_1', 'target_back_2', 'target_back_3', 'target_back_4',
    'target_left_1', 'target_left_2', 'target_right_1', 'target_right_2',
    'target_bottom_1', 'target_bottom_2'
]


class _HeadlessViewer:
    def is_running(self):
        return True

    def sync(self):
        pass


def _get_allowed_workpiece_parts(arm_name: str) -> list:
    return ['wrist_3', 'robotiq'] if arm_name == 'ur5e' else ['link7', 'hand', 'finger']


def _has_forbidden_arm_contact(model, data, arm_name: str, allow_workpiece_contact: bool = False) -> bool:
    arm_keywords = ['ur5e'] if arm_name == 'ur5e' else ['fr3', 'panda']
    allowed_parts = _get_allowed_workpiece_parts(arm_name)

    def _is_arm_body(name: str) -> bool:
        return any(keyword in name for keyword in arm_keywords)

    for c_idx in range(data.ncon):
        contact = data.contact[c_idx]
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""

        if ('table' in b1 and ('base' in b2 or 'link0' in b2)) or ('table' in b2 and ('base' in b1 or 'link0' in b1)):
            continue

        if 'table' in b1 or 'table' in b2:
            if _is_arm_body(b1) or _is_arm_body(b2):
                return True
            continue

        if 'workpiece' in b1 or 'workpiece' in b2:
            other_body = b1 if 'workpiece' in b2 else b2
            if not _is_arm_body(other_body):
                continue
            if allow_workpiece_contact and any(part in other_body for part in allowed_parts):
                continue
            return True

    return False

class DualArmEnvironment:
    def __init__(self, scene_path: str = "dual_arm_scene.xml"):
        print("=" * 60)
        print("初始化异构双臂仿真环境 (带 RRT 避障模块)")
        print("=" * 60)
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self._setup_robot_info()
        
    def _setup_robot_info(self):
        self.ur5e_joint_names = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint', 'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
        self.fr3_joint_names = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        self.ur5e_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.ur5e_joint_names]
        self.fr3_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.fr3_joint_names]
        self.ur5e_actuator_ids = list(range(6))
        self.fr3_actuator_ids = list(range(6, 13))
        
        # ====== 【补上这两行遗漏的代码】 ======
        self.ur5e_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ur5e_ee_site')
        self.fr3_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'fr3_ee_site')
        # ======================================
        
    def reset(self, ur5e_qpos: np.ndarray, fr3_qpos: np.ndarray):
        mujoco.mj_resetData(self.model, self.data)
        for i, joint_id in enumerate(self.ur5e_joint_ids): self.data.qpos[joint_id] = ur5e_qpos[i]
        for i, joint_id in enumerate(self.fr3_joint_ids): self.data.qpos[joint_id] = fr3_qpos[i]
        self.data.ctrl[self.ur5e_actuator_ids] = ur5e_qpos
        self.data.ctrl[self.fr3_actuator_ids] = fr3_qpos
        mujoco.mj_forward(self.model, self.data)
        
    def step(self, ur5e_ctrl: np.ndarray, fr3_ctrl: np.ndarray):
        self.data.ctrl[self.ur5e_actuator_ids] = ur5e_ctrl
        self.data.ctrl[self.fr3_actuator_ids] = fr3_ctrl
        mujoco.mj_step(self.model, self.data)
        
    def compute_ik(self, site_name: str, target_pos: np.ndarray, target_mat: np.ndarray = None,
                   max_iter: int = 800, tol: float = 1e-4, damping: float = 0.15,
                   max_attempts: int = 10, allow_workpiece_contact: bool = False) -> np.ndarray:
        """回归最稳健的 6-DOF 严谨求解器（带底座豁免）"""
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        joint_ids = self.ur5e_joint_ids if 'ur5e' in site_name else self.fr3_joint_ids
        ik_data = mujoco.MjData(self.model)
        dof_indices = [self.model.jnt_dofadr[jid] for jid in joint_ids]
        qpos_indices = [self.model.jnt_qposadr[jid] for jid in joint_ids]
        jnt_ranges = [self.model.jnt_range[jid] for jid in joint_ids]
        
        best_q = None
        best_err, best_pos_err, best_ori_err = float('inf'), float('inf'), float('inf')

        def _is_ik_safe():
            mujoco.mj_forward(self.model, ik_data)
            arm_name = 'ur5e' if 'ur5e' in site_name else 'fr3'
            return not _has_forbidden_arm_contact(
                self.model,
                ik_data,
                arm_name,
                allow_workpiece_contact=allow_workpiece_contact,
            )

        for attempt in range(max_attempts):
            mujoco.mj_copyData(ik_data, self.model, self.data)
            if attempt > 0:
                for i, q_idx in enumerate(qpos_indices):
                    if attempt < max_attempts // 2:
                        new_q = self.data.qpos[q_idx] + np.random.uniform(-0.5, 0.5) 
                    else:
                        new_q = np.random.uniform(jnt_ranges[i][0], jnt_ranges[i][1])
                    ik_data.qpos[q_idx] = np.clip(new_q, jnt_ranges[i][0], jnt_ranges[i][1])
                    
            for step in range(max_iter):
                mujoco.mj_kinematics(self.model, ik_data)
                mujoco.mj_comPos(self.model, ik_data)
                
                curr_pos = ik_data.site_xpos[site_id]
                curr_mat = ik_data.site_xmat[site_id].reshape(3, 3)
                err_pos = target_pos - curr_pos
                err_ori = np.zeros(3)
                if target_mat is not None:
                    err_ori = 0.5 * (np.cross(curr_mat[:, 0], target_mat[:, 0]) + 
                                     np.cross(curr_mat[:, 1], target_mat[:, 1]) + 
                                     np.cross(curr_mat[:, 2], target_mat[:, 2]))
                
                err_spatial = np.hstack((err_pos, err_ori))
                error_norm = np.linalg.norm(err_spatial)
                
                if error_norm < best_err:
                    best_err, best_pos_err, best_ori_err = error_norm, np.linalg.norm(err_pos), np.linalg.norm(err_ori)
                    best_q = np.array([ik_data.qpos[q_idx] for q_idx in qpos_indices])
                    
                if error_norm < tol:
                    if _is_ik_safe(): return best_q
                    break
                    
                jac_p = np.zeros((3, self.model.nv))
                jac_r = np.zeros((3, self.model.nv))
                mujoco.mj_jacSite(self.model, ik_data, jac_p, jac_r, site_id)
                jac_arm = np.vstack((jac_p, jac_r))[:, dof_indices]
                y = np.linalg.solve(jac_arm @ jac_arm.T + (damping**2) * np.eye(6), err_spatial)
                delta_q = jac_arm.T @ y
                
                for i, q_idx in enumerate(qpos_indices):
                    new_q = ik_data.qpos[q_idx] + delta_q[i]
                    ik_data.qpos[q_idx] = np.clip(new_q, jnt_ranges[i][0], jnt_ranges[i][1])
                    
        if best_pos_err > 0.02 or best_ori_err > 0.35: return None
        if best_q is not None:
            mujoco.mj_copyData(ik_data, self.model, self.data)
            for i, q_idx in enumerate(qpos_indices): ik_data.qpos[q_idx] = best_q[i]
            if not _is_ik_safe(): return None
        return best_q


def _is_target_pose_contact_safe(
    env: DualArmEnvironment,
    arm_name: str,
    q_target: np.ndarray,
) -> bool:
    sim_data = mujoco.MjData(env.model)
    mujoco.mj_copyData(sim_data, env.model, env.data)

    joint_ids = env.ur5e_joint_ids if arm_name == 'ur5e' else env.fr3_joint_ids
    for i, jid in enumerate(joint_ids):
        sim_data.qpos[jid] = q_target[i]
    mujoco.mj_forward(env.model, sim_data)
    return not _has_forbidden_arm_contact(env.model, sim_data, arm_name, allow_workpiece_contact=True)


def _get_current_arm_q(env: DualArmEnvironment, arm_name: str) -> np.ndarray:
    joint_ids = env.ur5e_joint_ids if arm_name == 'ur5e' else env.fr3_joint_ids
    return np.array([env.data.qpos[jid] for jid in joint_ids])


def _get_other_arm_name(arm_name: str) -> str:
    return 'fr3' if arm_name == 'ur5e' else 'ur5e'


def _get_ee_site_id(env: DualArmEnvironment, arm_name: str) -> int:
    return env.ur5e_ee_site_id if arm_name == 'ur5e' else env.fr3_ee_site_id


def _get_arm_joint_ids(env: DualArmEnvironment, arm_name: str) -> list:
    return env.ur5e_joint_ids if arm_name == 'ur5e' else env.fr3_joint_ids


def _env_step_for_arm(env: DualArmEnvironment, arm_name: str, arm_q: np.ndarray, other_arm_q: np.ndarray):
    if arm_name == 'ur5e':
        env.step(arm_q, other_arm_q)
    else:
        env.step(other_arm_q, arm_q)


def _servo_to_waypoint(
    env: DualArmEnvironment,
    viewer,
    arm_name: str,
    target_q: np.ndarray,
    other_arm_q: np.ndarray,
    joint_tol: float = 0.03,
    max_servo_steps: int = 20,
    base_sleep: float = 0.001,
) -> bool:
    for _ in range(max_servo_steps):
        if not viewer.is_running():
            return False

        _env_step_for_arm(env, arm_name, target_q, other_arm_q)

        viewer.sync()
        time.sleep(base_sleep)

        if np.linalg.norm(_get_current_arm_q(env, arm_name) - target_q) < joint_tol:
            return True

    return True

# def calculate_target_posture(env: DualArmEnvironment, arm_name: str, target_name: str, retract_dist: float = 0.15):
#     target_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, target_name)
#     target_pos = env.data.site_xpos[target_id].copy()
#     raw_target_mat = env.data.site_xmat[target_id].reshape(3, 3).copy()
#     normal_vector = raw_target_mat[:, 2]
    
#     workpiece_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, 'workpiece')
#     workpiece_center = env.data.xpos[workpiece_id].copy()
#     if np.dot(normal_vector, target_pos - workpiece_center) < 0: normal_vector = -normal_vector
        
#     target_z = -normal_vector
#     base_forward = np.array([1.0, 0.0, 0.0]) if arm_name == 'ur5e' else np.array([-1.0, 0.0, 0.0])
#     temp_x = base_forward if abs(np.dot(target_z, base_forward)) < 0.9 else np.array([0.0, 1.0, 0.0])
#     target_y = np.cross(target_z, temp_x)
#     target_y /= np.linalg.norm(target_y)
#     target_x = np.cross(target_y, target_z)
#     correct_target_mat = np.column_stack((target_x, target_y, target_z))
    
#     approach_pos = target_pos + retract_dist * normal_vector
#     site_name = f"{arm_name}_ee_site"
    
#     good_seed_ur5e = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
#     # 【已彻底修复】：换回最高高抬手姿态，彻底脱离桌子干涉！
#     good_seed_fr3 = np.array([0.0, -1.2, 0.0, -2.4, 0.0, 1.571, 0.785])
    
#     joint_ids = env.ur5e_joint_ids if arm_name == 'ur5e' else env.fr3_joint_ids
#     original_qpos = np.array([env.data.qpos[jid] for jid in joint_ids])
    
#     seed_q = good_seed_ur5e if arm_name == 'ur5e' else good_seed_fr3
#     for i, jid in enumerate(joint_ids): env.data.qpos[jid] = seed_q[i]
#     mujoco.mj_forward(env.model, env.data)
    
#     q_approach = env.compute_ik(site_name, approach_pos, correct_target_mat)
#     if q_approach is None:
#         for i, jid in enumerate(joint_ids): env.data.qpos[jid] = original_qpos[i]
#         mujoco.mj_forward(env.model, env.data)
#         return None, None
    
#     for i, jid in enumerate(joint_ids): env.data.qpos[jid] = q_approach[i]
#     mujoco.mj_forward(env.model, env.data)
    
#     q_target = env.compute_ik(site_name, target_pos, correct_target_mat)
#     for i, jid in enumerate(joint_ids): env.data.qpos[jid] = original_qpos[i]
#     mujoco.mj_forward(env.model, env.data)
    
#     if q_target is None: return None, None
#     return q_approach, q_target

def calculate_target_posture(env: DualArmEnvironment, arm_name: str, target_name: str, retract_dist: float = 0.15):
    target_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, target_name)
    target_pos = env.data.site_xpos[target_id].copy()
    raw_target_mat = env.data.site_xmat[target_id].reshape(3, 3).copy()
    normal_vector = raw_target_mat[:, 2]
    
    workpiece_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, 'workpiece')
    workpiece_center = env.data.xpos[workpiece_id].copy()
    if np.dot(normal_vector, target_pos - workpiece_center) < 0: normal_vector = -normal_vector
        
    target_z = -normal_vector
    base_forward = np.array([1.0, 0.0, 0.0]) if arm_name == 'ur5e' else np.array([-1.0, 0.0, 0.0])
    temp_x = base_forward if abs(np.dot(target_z, base_forward)) < 0.9 else np.array([0.0, 1.0, 0.0])
    target_y = np.cross(target_z, temp_x)
    target_y /= np.linalg.norm(target_y)
    target_x = np.cross(target_y, target_z)
    correct_target_mat = np.column_stack((target_x, target_y, target_z))
    
    approach_pos = target_pos + retract_dist * normal_vector
    site_name = f"{arm_name}_ee_site"
    
    good_seed_ur5e = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    good_seed_fr3 = np.array([0.0, -1.2, 0.0, -2.4, 0.0, 1.571, 0.785])
    
    joint_ids = env.ur5e_joint_ids if arm_name == 'ur5e' else env.fr3_joint_ids
    original_qpos = np.array([env.data.qpos[jid] for jid in joint_ids])

    def _set_arm_q(q: np.ndarray):
        for i, jid in enumerate(joint_ids):
            env.data.qpos[jid] = q[i]
        mujoco.mj_forward(env.model, env.data)

    def _solve_ik_with_seed(target_p: np.ndarray, target_R: np.ndarray, seed_q: np.ndarray, allow_workpiece_contact: bool = False):
        _set_arm_q(seed_q)
        return env.compute_ik(
            site_name,
            target_p,
            target_R,
            allow_workpiece_contact=allow_workpiece_contact,
        )

    def _pick_best_by_reference(candidates: List[np.ndarray], q_ref: np.ndarray):
        if not candidates:
            return None
        return min(candidates, key=lambda q: float(np.linalg.norm(np.asarray(q) - q_ref)))
    
    seed_q = good_seed_ur5e if arm_name == 'ur5e' else good_seed_fr3

    # 优先使用“当前关节姿态”作为 IK 初值，保证轨迹连续性；
    # 若失败再回退到经验 good seed，提升可解率。
    approach_candidates: List[np.ndarray] = []
    q_try = _solve_ik_with_seed(approach_pos, correct_target_mat, original_qpos, allow_workpiece_contact=False)
    if q_try is not None:
        approach_candidates.append(q_try)
    q_try = _solve_ik_with_seed(approach_pos, correct_target_mat, seed_q, allow_workpiece_contact=False)
    if q_try is not None:
        approach_candidates.append(q_try)

    q_approach = _pick_best_by_reference(approach_candidates, original_qpos)
    if q_approach is None:
        _set_arm_q(original_qpos)
        return None, None

    # 压入位姿优先从 approach 解继续求，避免关节翻转；失败再尝试 good seed。
    target_candidates: List[np.ndarray] = []
    q_try = _solve_ik_with_seed(target_pos, correct_target_mat, q_approach, allow_workpiece_contact=True)
    if q_try is not None:
        target_candidates.append(q_try)
    q_try = _solve_ik_with_seed(target_pos, correct_target_mat, seed_q, allow_workpiece_contact=True)
    if q_try is not None:
        target_candidates.append(q_try)

    q_target = _pick_best_by_reference(target_candidates, q_approach)
    _set_arm_q(original_qpos)
    
    if q_target is None:
        return None, None
    
    if not _is_target_pose_contact_safe(env, arm_name, q_target):
        return None, None
        
    return q_approach, q_target

def execute_trajectory(env, viewer, arm_name: str, smoothed_path: list, other_arm_q: np.ndarray, base_sleep: float = 0.001):
    if not smoothed_path: return True
    for current_q in smoothed_path:
        if not _servo_to_waypoint(env, viewer, arm_name, current_q, other_arm_q, base_sleep=base_sleep):
            return False
    return True

def run_thesis_experiment(arm_name: str = 'ur5e', task_sequence: list = None, use_viewer: bool = True):
    print("\n" + "★" * 60)
    print(f"开始执行论文最终实验：{arm_name.upper()} 基于 RRT, B-Spline 与 IK 的多位姿连续作业")
    print("★" * 60)

    if task_sequence is None:
        task_sequence = DEFAULT_TASK_SEQUENCE[arm_name]

    env = DualArmEnvironment("dual_arm_scene.xml")
    planner = BiRRTPlanner(env, arm_name=arm_name, step_size=0.15)

    ur5e_init = ARM_HOME_Q['ur5e'].copy()
    fr3_init = ARM_HOME_Q['fr3'].copy()
    active_init = ARM_HOME_Q[arm_name].copy()
    other_arm_name = _get_other_arm_name(arm_name)
    other_arm_q = ARM_HOME_Q[other_arm_name].copy()

    env.reset(ur5e_init, fr3_init)

    viewer_context = mujoco.viewer.launch_passive(env.model, env.data) if use_viewer else None

    if use_viewer:
        viewer_manager = viewer_context
    else:
        viewer_manager = None

    viewer = _HeadlessViewer()
    current_arm_q = active_init.copy()

    try:
        if use_viewer:
            viewer = viewer_manager.__enter__()

        for target in task_sequence:
            if not viewer.is_running():
                break
            print(f"\n[{target}] ---------------------------------")
            print(f"[{target}] 1. 正在求解逆运动学 (IK)...")
            
            q_approach, q_target = calculate_target_posture(env, arm_name, target)
            if q_approach is None or q_target is None:
                print(f"❌ IK 求解失败或姿态发生干涉，无法安全到达 {target}，已智能跳过此任务。")
                continue
            
            if not planner._is_edge_collision_free(q_approach, q_target, allow_workpiece_contact=True):
                print(f"❌ 接近位到目标位的压入段存在碰撞风险，{target} 已在执行前被拦截。")
                continue

            print(f"[{target}] 2. 正在进行 RRT 全局防碰撞路径规划...")
            rrt_path = planner.plan(current_arm_q, q_approach)
            if rrt_path is None:
                print(f"❌ RRT 规划失败，去往 {target} 的路径被障碍物彻底封死，已智能跳过。")
                continue
                
            print(f"[{target}] 3. 接近段与压入段预检通过，正在生成 B-Spline 平滑曲线...")
            # 加入 degree=1，强制严格沿着 RRT 的安全连线走，绝不允许曲线圆滑外抛穿模！
            smooth_rrt_path = b_spline_smooth(rrt_path, num_points=200, degree=1)
            if not execute_trajectory(env, viewer, arm_name, smooth_rrt_path, other_arm_q):
                break

            print(f"[{target}] 4. 垂直压入作业...")
            smooth_insert_path = b_spline_smooth([q_approach, q_target], num_points=50, degree=1)
            if not execute_trajectory(env, viewer, arm_name, smooth_insert_path, other_arm_q):
                break
            if not _servo_to_waypoint(env, viewer, arm_name, q_target, other_arm_q, joint_tol=0.02, max_servo_steps=80):
                break
            
            # ====== 【核心修复：极其敏锐的物理监工 (工业级防卡死)】 ======
            target_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, target)
            target_pos = env.data.site_xpos[target_id].copy()
            
            is_physically_failed = False
            fail_reason = ""

            # 模拟停留加工作业 (并在作业期间进行 500 次高频物理抓拍)
            for _ in range(500): 
                _env_step_for_arm(env, arm_name, q_target, other_arm_q)
                viewer.sync()
                time.sleep(0.002)
                
                # 抓拍 1：检查是否发生非法身体碰撞（手臂被挤压）
                for c_idx in range(env.data.ncon):
                    contact = env.data.contact[c_idx]
                    b1 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, env.model.geom_bodyid[contact.geom1]) or ""
                    b2 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, env.model.geom_bodyid[contact.geom2]) or ""
                    if 'workpiece' in b1 or 'workpiece' in b2:
                        other_body = b1 if 'workpiece' in b2 else b2
                        allowed_parts = ['wrist_3', 'link7']
                        if not any(part in other_body for part in allowed_parts):
                            is_physically_failed = True
                            fail_reason = f"手臂非允许部位({other_body})强行挤压了箱子"
                            break
                if is_physically_failed: break
                
            # 抓拍 2：检查最终的停留精度 (缩小到工业级 1.5 厘米！)
            actual_pos = env.data.site_xpos[_get_ee_site_id(env, arm_name)].copy()
            pos_error = np.linalg.norm(actual_pos - target_pos)
            if not is_physically_failed and pos_error > 0.015:
                is_physically_failed = True
                fail_reason = f"末端被物理卡住，位置误差高达 {pos_error*100:.1f} cm (允许范围 1.5 cm)"

            if is_physically_failed:
                print(f"\n❌ [物理引擎警报] 执行 {target} 时失败：{fail_reason}！")
                print(f"👉 判定为单臂物理极限，果断放弃该点位，正在执行安全撤离...")
                # 即使失败了，也要优雅地把手臂拔出来回到准备点，不能停在原地发抖
                smooth_retract_path = b_spline_smooth([_get_current_arm_q(env, arm_name), q_approach], num_points=50, degree=1)
                execute_trajectory(env, viewer, arm_name, smooth_retract_path, other_arm_q)
                current_arm_q = q_approach.copy()
                continue
            # ========================================================
            
            print(f"[{target}] 5. 作业完成，安全拔出...")
            smooth_retract_path = b_spline_smooth([q_target, q_approach], num_points=50, degree=1)
            if not execute_trajectory(env, viewer, arm_name, smooth_retract_path, other_arm_q):
                break
            
            current_arm_q = q_approach.copy()
            
        print("\n✅ 实验序列执行完毕！")
        while viewer.is_running():
            _env_step_for_arm(env, arm_name, current_arm_q, other_arm_q)
            viewer.sync()
            if not use_viewer:
                break
            time.sleep(0.01)

    finally:
        if use_viewer and viewer_manager is not None:
            viewer_manager.__exit__(None, None, None)


def parse_args():
    parser = argparse.ArgumentParser(description="按指定机械臂执行目标点仿真")
    parser.add_argument('--arm', choices=['ur5e', 'fr3'], default='ur5e', help='选择要运动的机械臂')
    parser.add_argument('--targets', nargs='*', default=None, help='目标点列表，例如 target_top_1 target_left_1')
    parser.add_argument('--headless', action='store_true', help='无界面运行，便于快速验证')
    parser.add_argument('--list-targets', action='store_true', help='仅输出全部可用目标点')
    return parser.parse_args()

if __name__ == "__main__":
    try:
        args = parse_args()
        if args.list_targets:
            print("可用目标点:")
            for target in ALL_TARGETS:
                print(f"- {target}")
        else:
            task_sequence = args.targets if args.targets else DEFAULT_TASK_SEQUENCE[args.arm]
            run_thesis_experiment(args.arm, task_sequence, use_viewer=not args.headless)
    except KeyboardInterrupt:
        print("\n程序被用户中断")