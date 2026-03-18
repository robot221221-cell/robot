"""
轨迹平滑模块：将 RRT 的生硬折线转化为 B-样条 (B-Spline) 优美弧线
"""
import numpy as np
from scipy.interpolate import splprep, splev

def b_spline_smooth(path: list, num_points: int = 200, degree: int = 3) -> list:
    if path is None or len(path) < 2: return path
        
    if len(path) <= degree:
        degree = len(path) - 1
        if degree <= 0: return path

    path_array = np.array(path).T
    
    # ====== 【核心修复：s=0.0 锁死抄近道】 ======
    # s=0.0 强制要求生成的平滑曲线必须 100% 穿过 RRT 给出的每一个安全路点！
    # 绝对不允许为了圆滑而向内切弯导致穿入箱子内部！
    tck, u = splprep(path_array, s=0.0, k=degree)
    # ============================================
    
    u_new = np.linspace(0, 1.0, num_points)
    smoothed_path = np.array(splev(u_new, tck)).T.tolist()
    
    return [np.array(q) for q in smoothed_path]