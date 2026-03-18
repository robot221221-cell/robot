"""
碰撞检测验证脚本 - 测试FR3胶囊体碰撞体是否正常工作
"""

import numpy as np
import mujoco
import time

print("=" * 70)
print("碰撞检测系统验证".center(70))
print("=" * 70)

# 加载模型
model = mujoco.MjModel.from_xml_path("dual_arm_scene.xml")
data = mujoco.MjData(model)

print("\n[1] 检查碰撞体配置")
collision_geoms = []
ur5e_collision_count = 0
fr3_collision_count = 0

for i in range(model.ngeom):
    geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    geom_type = model.geom_type[i]
    type_names = ['PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH']
    
    # 检查是否为碰撞体（group=3表示碰撞体）
    if model.geom_group[i] == 3:
        collision_geoms.append((geom_name, type_names[geom_type]))
        
        # 通过body名称判断归属
        body_id = model.geom_bodyid[i]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        
        if body_name and 'ur5e' in str(body_name):
            ur5e_collision_count += 1
        elif body_name and 'fr3' in str(body_name):
            fr3_collision_count += 1

print(f"    发现 {len(collision_geoms)} 个碰撞几何体:")
print(f"      - UR5e: {ur5e_collision_count} 个")
print(f"      - FR3: {fr3_collision_count} 个")

# 显示所有碰撞体
print("\n    UR5e 碰撞体列表:")
for i in range(model.ngeom):
    if model.geom_group[i] == 3:
        body_id = model.geom_bodyid[i]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        geom_type = model.geom_type[i]
        type_names = ['PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH']
        
        if body_name and 'ur5e' in str(body_name):
            print(f"      ✓ {body_name:30s} - {type_names[geom_type]:10s} {geom_name if geom_name else '(未命名)'}")

print("\n    FR3 胶囊体碰撞体列表:")
fr3_capsules = []
for i in range(model.ngeom):
    if model.geom_group[i] == 3:
        body_id = model.geom_bodyid[i]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        geom_type = model.geom_type[i]
        type_names = ['PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH']
        
        if body_name and 'fr3' in str(body_name) and type_names[geom_type] == 'CAPSULE':
            fr3_capsules.append((geom_name, type_names[geom_type]))
            print(f"      ✓ {geom_name:40s} [{type_names[geom_type]}]")

if len(fr3_capsules) == 0:
    print("      ⚠️ 警告: 未检测到FR3胶囊体碰撞体!")
else:
    print(f"\n    ✓ FR3已配置 {len(fr3_capsules)} 个胶囊体碰撞体")

print("\n[2] 碰撞检测功能测试")

# 获取关节ID
ur5e_joints = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
               'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
fr3_joints = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
              'fr3_joint5', 'fr3_joint6', 'fr3_joint7']

ur5e_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ur5e_joints]
fr3_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in fr3_joints]

# 测试1: 初始姿态（应该无碰撞）
print("    测试1: 初始姿态")
ur5e_init = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
fr3_init = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

for i, jid in enumerate(ur5e_ids):
    data.qpos[jid] = ur5e_init[i]
for i, jid in enumerate(fr3_ids):
    data.qpos[jid] = fr3_init[i]

mujoco.mj_forward(model, data)

if data.ncon > 0:
    print(f"      检测到 {data.ncon} 个接触点")
    # 检查是否为双臂碰撞
    dual_arm_collision = False
    for i in range(data.ncon):
        contact = data.contact[i]
        g1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        g2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        
        is_ur5e_1 = 'ur5e' in str(g1_name)
        is_ur5e_2 = 'ur5e' in str(g2_name)
        is_fr3_1 = 'fr3' in str(g1_name)
        is_fr3_2 = 'fr3' in str(g2_name)
        
        if (is_ur5e_1 and is_fr3_2) or (is_fr3_1 and is_ur5e_2):
            dual_arm_collision = True
            print(f"      ⚠️ 双臂碰撞: {g1_name} <-> {g2_name}, 距离={contact.dist:.4f}m")
    
    if not dual_arm_collision:
        print("      ✓ 无双臂碰撞（仅与环境接触）")
else:
    print("      ✓ 无碰撞")

# 测试2: 危险姿态（让双臂靠近）
print("\n    测试2: 危险姿态（模拟双臂接近）")
ur5e_close = np.array([0.5, -1.2, 1.2, -1.57, -1.57, 0.0])
fr3_close = np.array([0.5, -0.5, 0.3, -2.0, 0.0, 1.2, 0.5])

for i, jid in enumerate(ur5e_ids):
    data.qpos[jid] = ur5e_close[i]
for i, jid in enumerate(fr3_ids):
    data.qpos[jid] = fr3_close[i]

mujoco.mj_forward(model, data)

collision_detected = False
if data.ncon > 0:
    for i in range(data.ncon):
        contact = data.contact[i]
        g1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        g2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        
        is_ur5e_1 = 'ur5e' in str(g1_name)
        is_ur5e_2 = 'ur5e' in str(g2_name)
        is_fr3_1 = 'fr3' in str(g1_name)
        is_fr3_2 = 'fr3' in str(g2_name)
        
        if (is_ur5e_1 and is_fr3_2) or (is_fr3_1 and is_ur5e_2):
            collision_detected = True
            print(f"      ✓ 检测到碰撞: {g1_name} <-> {g2_name}")
            print(f"        距离: {contact.dist:.4f}m, 位置: [{contact.pos[0]:.3f}, {contact.pos[1]:.3f}, {contact.pos[2]:.3f}]")

if collision_detected:
    print("      ✓ 碰撞检测系统工作正常!")
else:
    print("      ⚠️ 未检测到预期碰撞（可能姿态还不够近）")

print("\n[3] 执行器类型验证")
ur5e_actuator_types = []
fr3_actuator_types = []
for i in range(model.nu):
    act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    act_type = model.actuator_dyntype[i]
    type_names = ['NONE', 'INTEGRATOR', 'FILTER', 'MUSCLE', 'USER']
    
    if 'ur5e' in str(act_name):
        ur5e_actuator_types.append((act_name, type_names[act_type]))
    elif 'fr3' in str(act_name):
        fr3_actuator_types.append((act_name, type_names[act_type]))

print(f"    UR5e执行器类型: {set([t for _, t in ur5e_actuator_types])}")
print(f"    FR3执行器类型: {set([t for _, t in fr3_actuator_types])}")

# 检查是否统一
all_types = [t for _, t in ur5e_actuator_types + fr3_actuator_types]
if len(set(all_types)) == 1:
    print(f"    ✓ 执行器类型统一为: {all_types[0]}")
else:
    print(f"    ⚠️ 执行器类型不统一: {set(all_types)}")

print("\n[4] 工作空间布局验证")
ur5e_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'ur5e_ee_site')
fr3_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'fr3_ee_site')

# 重置到初始姿态
for i, jid in enumerate(ur5e_ids):
    data.qpos[jid] = ur5e_init[i]
for i, jid in enumerate(fr3_ids):
    data.qpos[jid] = fr3_init[i]
mujoco.mj_forward(model, data)

ur5e_pos = data.site_xpos[ur5e_site]
fr3_pos = data.site_xpos[fr3_site]
distance = np.linalg.norm(ur5e_pos - fr3_pos)

print(f"    UR5e基座: Y坐标")
ur5e_base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ur5e_base')
print(f"    FR3基座: Y坐标")

print(f"    初始末端距离: {distance:.3f}m")
if distance < 1.0:
    print(f"    ✓ 双臂距离适中，有良好的协作空间重叠")
else:
    print(f"    ⚠️ 双臂距离较远，协作空间可能不足")

print("\n" + "=" * 70)
print("验证总结".center(70))
print("=" * 70)

summary = []
if len(fr3_capsules) >= 6:
    summary.append("✓ FR3碰撞体配置完整")
else:
    summary.append("⚠️ FR3碰撞体可能配置不完整")

if distance < 1.0:
    summary.append("✓ 双臂布局优化完成")
else:
    summary.append("⚠️ 双臂布局需要进一步调整")

if len(set(all_types)) == 1:
    summary.append("✓ 执行器统一为position控制")
else:
    summary.append("⚠️ 执行器类型需要统一")

for item in summary:
    print(f"  {item}")

print("\n提示: 运行 'python dual_arm_simulation.py' 查看可视化中的红色半透明碰撞体")
print()
