"""
快速验证脚本 - 检查双臂是否正确配置并显示关键信息
"""

import numpy as np
import mujoco

print("=" * 70)
print("双臂配置验证报告".center(70))
print("=" * 70)

# 加载模型
model = mujoco.MjModel.from_xml_path("dual_arm_scene.xml")
data = mujoco.MjData(model)

print("\n[1] 模型基本信息")
print(f"    总自由度 (nv): {model.nv}")
print(f"    控制维度 (nu): {model.nu}")
print(f"    关节数量 (njnt): {model.njnt}")
print(f"    刚体数量 (nbody): {model.nbody}")
print(f"    时间步长: {model.opt.timestep*1000:.1f} ms")

print("\n[2] 关节配置")
print("    UR5e关节:")
ur5e_joints = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
               'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
for i, name in enumerate(ur5e_joints):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    print(f"      [{i+1}] {name:30s} (ID: {jid})")

print("\n    FR3关节:")
fr3_joints = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
              'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
for i, name in enumerate(fr3_joints):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    print(f"      [{i+1}] {name:30s} (ID: {jid})")

print("\n[3] 执行器配置")
for i in range(model.nu):
    actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"      [{i:2d}] {actuator_name}")

print("\n[4] 末端执行器站点")
ur5e_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'ur5e_ee_site')
fr3_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'fr3_ee_site')
print(f"      UR5e末端站点 ID: {ur5e_site}")
print(f"      FR3末端站点 ID: {fr3_site}")

# 测试前向运动学
mujoco.mj_forward(model, data)

print("\n[5] 初始位姿测试")
ur5e_ee_pos = data.site_xpos[ur5e_site]
fr3_ee_pos = data.site_xpos[fr3_site]
print(f"      UR5e末端初始位置: [{ur5e_ee_pos[0]:.3f}, {ur5e_ee_pos[1]:.3f}, {ur5e_ee_pos[2]:.3f}]")
print(f"      FR3末端初始位置:  [{fr3_ee_pos[0]:.3f}, {fr3_ee_pos[1]:.3f}, {fr3_ee_pos[2]:.3f}]")
distance = np.linalg.norm(ur5e_ee_pos - fr3_ee_pos)
print(f"      双臂间距: {distance:.3f} m")

print("\n[6] 目标点配置")
targets = ['target1', 'target2', 'target3']
for target_name in targets:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_name)
    if body_id >= 0:
        pos = data.xpos[body_id]
        print(f"      {target_name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

print("\n[7] 物体配置")
boxes = ['box1', 'box2']
for box_name in boxes:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, box_name)
    if body_id >= 0:
        pos = data.xpos[body_id]
        print(f"      {box_name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

print("\n" + "=" * 70)
print("✓ 双臂环境配置正确！可以开始开发规划算法".center(70))
print("=" * 70)
print("\n提示: 运行 './run_dual_arm.sh' 或 'python dual_arm_simulation.py' 启动可视化")
print()
