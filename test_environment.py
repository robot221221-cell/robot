#!/usr/bin/env python3
"""
快速测试脚本：验证双臂环境是否正确加载
"""

import sys
import numpy as np

try:
    import mujoco
    print("✓ MuJoCo 已安装")
except ImportError:
    print("✗ MuJoCo 未安装，请运行: pip install mujoco")
    sys.exit(1)

try:
    # 测试加载场景
    print("\n测试加载双臂场景...")
    model = mujoco.MjModel.from_xml_path("dual_arm_scene.xml")
    data = mujoco.MjData(model)
    
    print(f"✓ 场景加载成功!")
    print(f"  - 总自由度: {model.nv}")
    print(f"  - 控制输入维度: {model.nu}")
    print(f"  - 刚体数量: {model.nbody}")
    print(f"  - 关节数量: {model.njnt}")
    
    # 测试前向运动学
    mujoco.mj_forward(model, data)
    print(f"✓ 前向运动学计算成功!")
    
    # 检查关节名称
    print(f"\n关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            print(f"  [{i}] {joint_name}")
    
    print("\n✓ 所有测试通过! 可以运行主程序: python dual_arm_simulation.py")
    
except FileNotFoundError:
    print("✗ 找不到 dual_arm_scene.xml 文件")
    print("  请确保在 /home/jiangshuo/robot/ 目录下运行")
    sys.exit(1)
except Exception as e:
    print(f"✗ 错误: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
