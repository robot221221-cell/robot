import json
import matplotlib.pyplot as plt

# 读取你脚本跑出来的 JSON 文件
with open('ga_best_solution.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

history = data['history']
generations = [h['generation'] for h in history]
fitness = [h['best_fitness'] for h in history]

# 绘制高大上的收敛曲线
plt.figure(figsize=(10, 6))
plt.plot(generations, fitness, linewidth=2.5, color='#d62728', label='Best Fitness')
plt.title('Genetic Algorithm Convergence Curve (Multi-Robot Task Allocation)', fontsize=15, fontweight='bold')
plt.xlabel('Generation', fontsize=13)
plt.ylabel('Fitness Score (Total Cost + Penalty)', fontsize=13)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)
plt.tight_layout()
plt.savefig('ga_convergence_curve.png', dpi=300)
print("✅ 完美！论文配图 ga_convergence_curve.png 已生成！")