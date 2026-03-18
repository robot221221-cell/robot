"""
基于当前 XML 已定义目标点，自动跑通完整流程：
1. 生成单点代价矩阵
2. 分类目标点并筛出 GA 可用任务
3. 生成 pairwise transition cost matrix
4. 执行 GA 任务分配与排序优化
5. 生成 GA 收敛曲线
"""

import argparse
import json
import shutil
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from classify_targets_for_ga import build_classification, build_output_payload, load_cost_matrix
from ga_task_allocation import GATaskAllocator, load_matrix, save_solution
from generate_cost_matrix import generate_cost_matrix
from generate_pairwise_transition_cost_matrix import build_pairwise_matrix_for_arm
from dual_arm_simulation import DualArmEnvironment


def plot_history(history, output_path: Path):
    if plt is None:
        print("⚠️ 未安装 matplotlib，已跳过 GA 收敛曲线绘制")
        return False

    generations = [item["generation"] for item in history]
    fitness = [item["best_fitness"] for item in history]

    plt.figure(figsize=(10, 6))
    plt.plot(generations, fitness, linewidth=2.5, color="#d62728", label="Best Fitness")
    plt.title("Genetic Algorithm Convergence Curve (XML Task Set)", fontsize=15, fontweight="bold")
    plt.xlabel("Generation", fontsize=13)
    plt.ylabel("Fitness Score", fontsize=13)
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()
    return True


def parse_args():
    parser = argparse.ArgumentParser(description="基于当前 XML 目标点自动跑通完整 GA 流程")
    parser.add_argument("--output-dir", default=".", help="输出目录")
    parser.add_argument("--population", type=int, default=100, help="GA 种群规模")
    parser.add_argument("--generations", type=int, default=200, help="GA 进化代数")
    parser.add_argument("--elite-size", type=int, default=8, help="GA 精英数量")
    parser.add_argument("--balance-weight", type=float, default=0.15, help="负载均衡惩罚权重")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("Step 1/5: 生成单点代价矩阵")
    print("=" * 70)
    generate_cost_matrix()
    kinematic_src = Path("kinematic_cost_matrix.csv")
    kinematic_dst = output_dir / "kinematic_cost_matrix.csv"
    if kinematic_src.resolve() != kinematic_dst.resolve():
        shutil.copyfile(kinematic_src, kinematic_dst)

    print("\n" + "=" * 70)
    print("Step 2/5: 分类目标点并筛选 GA 任务集")
    print("=" * 70)
    cost_df = load_cost_matrix(kinematic_dst)
    classified = build_classification(cost_df)
    payload = build_output_payload(classified)
    ga_targets = payload["ga_task_targets"]
    if not ga_targets:
        raise ValueError("当前 XML 目标点中没有任何可用于 GA 的任务点")

    classified.to_csv(output_dir / "target_classification.csv", index=False)
    (output_dir / "ga_task_list.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "ga_task_targets.txt").write_text("\n".join(ga_targets) + "\n", encoding="utf-8")
    print(f"GA 任务点: {ga_targets}")

    print("\n" + "=" * 70)
    print("Step 3/5: 生成 pairwise transition cost matrix")
    print("=" * 70)
    env = DualArmEnvironment("dual_arm_scene.xml")
    build_pairwise_matrix_for_arm(env, "ur5e", ga_targets, output_dir)
    build_pairwise_matrix_for_arm(env, "fr3", ga_targets, output_dir)

    print("\n" + "=" * 70)
    print("Step 4/5: 执行 GA 任务分配与排序优化")
    print("=" * 70)
    ur5e_matrix = load_matrix(output_dir / "pairwise_transition_cost_ur5e.csv")
    fr3_matrix = load_matrix(output_dir / "pairwise_transition_cost_fr3.csv")
    allocator = GATaskAllocator(
        ur5e_matrix=ur5e_matrix,
        fr3_matrix=fr3_matrix,
        targets=ga_targets,
        balance_weight=args.balance_weight,
        random_seed=args.seed,
    )
    best_chromosome, best_eval, history = allocator.evolve(
        population_size=args.population,
        generations=args.generations,
        elite_size=args.elite_size,
    )

    solution_path = output_dir / "ga_best_solution.json"
    save_solution(solution_path, best_chromosome, best_eval, history)
    print(f"最优适应度: {best_eval.fitness:.3f}")
    print(f"UR5e 顺序: {best_eval.ur5e_sequence}")
    print(f"FR3 顺序: {best_eval.fr3_sequence}")

    print("\n" + "=" * 70)
    print("Step 5/5: 生成 GA 收敛曲线")
    print("=" * 70)
    plot_ok = plot_history(history, output_dir / "ga_convergence_curve.png")
    if plot_ok:
        print("✅ 已生成 ga_convergence_curve.png")

    print("\n" + "=" * 70)
    print("完整 XML 流程已跑通")
    print("=" * 70)
    print(f"输出目录: {output_dir}")
    print(f"- {kinematic_dst}")
    print(f"- {output_dir / 'target_classification.csv'}")
    print(f"- {output_dir / 'ga_task_list.json'}")
    print(f"- {output_dir / 'pairwise_transition_cost_ur5e.csv'}")
    print(f"- {output_dir / 'pairwise_transition_cost_fr3.csv'}")
    print(f"- {output_dir / 'ga_best_solution.json'}")
    if plot_ok:
        print(f"- {output_dir / 'ga_convergence_curve.png'}")


if __name__ == "__main__":
    main()
