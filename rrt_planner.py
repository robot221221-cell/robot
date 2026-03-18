import numpy as np
import mujoco
import time

class Node:
    def __init__(self, q: np.ndarray):
        self.q = q
        self.parent = None

class BiRRTPlanner:
    def __init__(self, env, arm_name='ur5e', step_size=0.1, max_iter=3000):
        self.env = env
        self.model = env.model
        self.arm_name = arm_name
        self.step_size = step_size
        self.max_iter = max_iter
        self.sim_data = mujoco.MjData(self.model)
        
        if arm_name == 'ur5e':
            self.joint_ids = env.ur5e_joint_ids
        else:
            self.joint_ids = env.fr3_joint_ids
            
        self.jnt_ranges = [self.model.jnt_range[jid] for jid in self.joint_ids]

    def _is_collision_free(self, q: np.ndarray, allow_workpiece_contact: bool = False) -> bool:
        mujoco.mj_copyData(self.sim_data, self.model, self.env.data)
        for i, jid in enumerate(self.joint_ids):
            self.sim_data.qpos[jid] = q[i]
            
        mujoco.mj_forward(self.model, self.sim_data)
        
        # 【核心修复1：加入夹爪的名字，彻底扫除碰撞盲区！】
        arm_keywords = ['ur5e', 'robotiq'] if self.arm_name == 'ur5e' else ['fr3', 'panda', 'hand']
        allowed_parts = ['wrist_3', 'robotiq'] if self.arm_name == 'ur5e' else ['link7', 'hand', 'finger']
        def _is_arm(name): return any(k in name for k in arm_keywords)
        
        for i in range(self.sim_data.ncon):
            contact = self.sim_data.contact[i]
            b1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[contact.geom1]) or ""
            b2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[contact.geom2]) or ""
            
            # 【核心修复2：豁免底座！底座装在桌子上是合法的，不能算作碰撞】
            if ('table' in b1 and ('base' in b2 or 'link0' in b2)) or ('table' in b2 and ('base' in b1 or 'link0' in b1)):
                continue
                
            if 'workpiece' in b1 or 'workpiece' in b2:
                other_body = b1 if 'workpiece' in b2 else b2
                if _is_arm(other_body):
                    if allow_workpiece_contact and any(part in other_body for part in allowed_parts):
                        continue
                    return False

            is_arm_involved = _is_arm(b1) or _is_arm(b2)
            is_fatal_obstacle = ('table' in b1 or 'table' in b2 or
                                 ('ur5e' in b1 and 'fr3' in b2) or ('fr3' in b1 and 'ur5e' in b2))

            if is_arm_involved and is_fatal_obstacle:
                return False
        return True

    def _is_edge_collision_free(
        self,
        q1: np.ndarray,
        q2: np.ndarray,
        allow_workpiece_contact: bool = False,
        allow_goal_workpiece_contact: bool = False,
    ) -> bool:
        dist = np.linalg.norm(q2 - q1)
        steps = max(5, int(dist / 0.05)) 
        for i in range(1, steps + 1):
            q_interp = q1 + (q2 - q1) * (i / steps)
            is_last_step = i == steps
            allow_interp_contact = allow_workpiece_contact or (allow_goal_workpiece_contact and is_last_step)
            if not self._is_collision_free(q_interp, allow_workpiece_contact=allow_interp_contact): return False
        return True

    def _sample_random_node(self) -> Node:
        q_rand = np.zeros(len(self.joint_ids))
        for i, (low, high) in enumerate(self.jnt_ranges):
            q_rand[i] = np.random.uniform(low, high)
        return Node(q_rand)

    def _get_nearest_node(self, tree: list, random_node: Node) -> Node:
        distances = [np.linalg.norm(node.q - random_node.q) for node in tree]
        return tree[np.argmin(distances)]

    def _steer(self, from_node: Node, to_node: Node) -> Node:
        direction = to_node.q - from_node.q
        distance = np.linalg.norm(direction)
        if distance <= self.step_size: return Node(to_node.q)
        else: return Node(from_node.q + (direction / distance) * self.step_size)

    def plan(self, q_start: np.ndarray, q_goal: np.ndarray, allow_endpoint_workpiece_contact: bool = False) -> list:
        print("\n[RRT Planner] 开始进行高维空间防碰撞探路...")
        if not self._is_collision_free(q_start, allow_workpiece_contact=allow_endpoint_workpiece_contact):
            print("[RRT 失败] 起点本身发生碰撞！")
            return None
        if not self._is_collision_free(q_goal, allow_workpiece_contact=allow_endpoint_workpiece_contact):
            print("[RRT 失败] 终点本身发生碰撞 (IK 给了一个穿模姿态)！")
            return None
        if self._is_edge_collision_free(q_start, q_goal, allow_goal_workpiece_contact=allow_endpoint_workpiece_contact):
            print("[RRT 成功] 直线路径安全，无需复杂规划。")
            return [q_start, q_goal]

        tree_start, tree_goal = [Node(q_start)], [Node(q_goal)]
        start_time = time.time()

        for i in range(self.max_iter):
            rand_node = self._sample_random_node()
            nearest_start = self._get_nearest_node(tree_start, rand_node)
            new_start = self._steer(nearest_start, rand_node)
            
            if self._is_edge_collision_free(nearest_start.q, new_start.q):
                new_start.parent = nearest_start
                tree_start.append(new_start)
                
                nearest_goal = self._get_nearest_node(tree_goal, new_start)
                new_goal = self._steer(nearest_goal, new_start)
                
                if self._is_edge_collision_free(nearest_goal.q, new_goal.q):
                    new_goal.parent = nearest_goal
                    tree_goal.append(new_goal)
                    
                    if np.linalg.norm(new_start.q - new_goal.q) < 1e-4:
                        print(f"[RRT 成功] 汇合！迭代次数: {i}, 耗时: {time.time()-start_time:.2f}s")
                        return self._extract_path(new_start, new_goal)
            tree_start, tree_goal = tree_goal, tree_start
            
        print("[RRT 失败] 超过最大迭代次数，未找到安全路径。")
        return None

    def _extract_path(self, node_start_tree: Node, node_goal_tree: Node) -> list:
        path_start = []
        curr = node_start_tree
        while curr is not None:
            path_start.append(curr.q)
            curr = curr.parent
        path_start.reverse()
        
        path_goal = []
        curr = node_goal_tree.parent 
        while curr is not None:
            path_goal.append(curr.q)
            curr = curr.parent
            
        raw_path = path_start + path_goal
        if np.linalg.norm(raw_path[0] - node_start_tree.q) > 0.1: 
             if np.linalg.norm(raw_path[-1] - node_start_tree.q) > 0.1: pass 
             else: raw_path.reverse()
        return self._smooth_path(raw_path)

    def _smooth_path(self, path: list) -> list:
        if path is None or len(path) <= 2: return path
        smoothed_path = [path[0]]
        curr_idx = 0
        while curr_idx < len(path) - 1:
            furthest_safe_idx = curr_idx + 1
            for j in range(len(path) - 1, curr_idx + 1, -1):
                if self._is_edge_collision_free(path[curr_idx], path[j]):
                    furthest_safe_idx = j
                    break
            smoothed_path.append(path[furthest_safe_idx])
            curr_idx = furthest_safe_idx
        print(f"[路径优化] 剪枝完成：原节点数 {len(path)} -> 现节点数 {len(smoothed_path)}")
        return smoothed_path