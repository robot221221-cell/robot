"""
基于 pairwise transition cost matrix 的双臂任务分配与排序优化脚本。

输入：
- pairwise_transition_cost_ur5e.csv
- pairwise_transition_cost_fr3.csv

核心思想：
- 染色体 = 全局任务访问排列 + 每个任务分配给哪只手臂
- 对某只手臂的最终访问顺序：按全局排列中过滤出分配给该手臂的任务
- 代价：home -> ...targets... -> home 的转移代价之和
- 额外加入负载均衡惩罚与不可达大惩罚
"""

import argparse
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import pandas as pd

ARM_NAMES = ["ur5e", "fr3"]
INF_PENALTY = 1e6
DEFAULT_BALANCE_WEIGHT = 0.15
DEFAULT_RANDOM_SEED = 42


@dataclass
class Chromosome:
    permutation: List[str]
    assignment: Dict[str, str]


@dataclass
class PlanEvaluation:
    total_cost: float
    ur5e_cost: float
    fr3_cost: float
    balance_penalty: float
    infeasible_edges: int
    fitness: float
    ur5e_sequence: List[str]
    fr3_sequence: List[str]
    ur5e_edges: List[Tuple[str, str, float]]
    fr3_edges: List[Tuple[str, str, float]]


class GATaskAllocator:
    def __init__(
        self,
        ur5e_matrix: pd.DataFrame,
        fr3_matrix: pd.DataFrame,
        targets: Sequence[str],
        balance_weight: float = DEFAULT_BALANCE_WEIGHT,
        infeasible_penalty: float = INF_PENALTY,
        random_seed: int = DEFAULT_RANDOM_SEED,
    ):
        self.matrices = {
            "ur5e": ur5e_matrix,
            "fr3": fr3_matrix,
        }
        self.targets = list(targets)
        self.balance_weight = balance_weight
        self.infeasible_penalty = infeasible_penalty
        self.random = random.Random(random_seed)
        self._validate_inputs()
        self.feasible_arms = self._build_feasible_arm_map()

    def _validate_inputs(self):
        required_labels = {"home", *self.targets}
        for arm_name, matrix in self.matrices.items():
            row_labels = set(matrix.index.tolist())
            col_labels = set(matrix.columns.tolist())
            missing_rows = required_labels - row_labels
            missing_cols = required_labels - col_labels
            if missing_rows or missing_cols:
                raise ValueError(
                    f"{arm_name} 矩阵缺少标签: rows={sorted(missing_rows)}, cols={sorted(missing_cols)}"
                )

    def _build_feasible_arm_map(self) -> Dict[str, List[str]]:
        feasible = {}
        for target in self.targets:
            target_feasible_arms = []
            for arm_name in ARM_NAMES:
                matrix = self.matrices[arm_name]
                if np.isfinite(matrix.loc["home", target]) and np.isfinite(matrix.loc[target, "home"]):
                    target_feasible_arms.append(arm_name)
            if not target_feasible_arms:
                raise ValueError(f"目标 {target} 对两只手臂都不可达，无法参与 GA 优化")
            feasible[target] = target_feasible_arms
        return feasible

    def random_chromosome(self) -> Chromosome:
        permutation = self.targets.copy()
        self.random.shuffle(permutation)
        assignment = {
            target: self.random.choice(self.feasible_arms[target])
            for target in self.targets
        }
        return Chromosome(permutation=permutation, assignment=assignment)

    def greedy_seed(self) -> Chromosome:
        remaining = set(self.targets)
        permutation: List[str] = []
        assignment: Dict[str, str] = {}
        arm_state = {"ur5e": "home", "fr3": "home"}

        while remaining:
            best = None
            for target in remaining:
                for arm_name in self.feasible_arms[target]:
                    cost = self.matrices[arm_name].loc[arm_state[arm_name], target]
                    if not np.isfinite(cost):
                        continue
                    if best is None or cost < best[0]:
                        best = (float(cost), arm_name, target)

            if best is None:
                # 退化回随机，但尽量保持可行分配
                leftover = list(remaining)
                self.random.shuffle(leftover)
                for target in leftover:
                    permutation.append(target)
                    assignment[target] = self.feasible_arms[target][0]
                break

            _, arm_name, target = best
            permutation.append(target)
            assignment[target] = arm_name
            arm_state[arm_name] = target
            remaining.remove(target)

        return Chromosome(permutation=permutation, assignment=assignment)

    def decode_sequences(self, chromosome: Chromosome) -> Dict[str, List[str]]:
        return {
            arm_name: [target for target in chromosome.permutation if chromosome.assignment[target] == arm_name]
            for arm_name in ARM_NAMES
        }

    def _evaluate_arm_sequence(self, arm_name: str, sequence: Sequence[str]) -> Tuple[float, int, List[Tuple[str, str, float]]]:
        matrix = self.matrices[arm_name]
        if not sequence:
            return 0.0, 0, [("home", "home", 0.0)]

        total_cost = 0.0
        infeasible_edges = 0
        edges: List[Tuple[str, str, float]] = []
        prev = "home"
        for target in sequence:
            edge_cost = float(matrix.loc[prev, target])
            edges.append((prev, target, edge_cost))
            if np.isfinite(edge_cost):
                total_cost += edge_cost
            else:
                total_cost += self.infeasible_penalty
                infeasible_edges += 1
            prev = target

        back_home_cost = float(matrix.loc[prev, "home"])
        edges.append((prev, "home", back_home_cost))
        if np.isfinite(back_home_cost):
            total_cost += back_home_cost
        else:
            total_cost += self.infeasible_penalty
            infeasible_edges += 1

        return total_cost, infeasible_edges, edges

    def evaluate(self, chromosome: Chromosome) -> PlanEvaluation:
        sequences = self.decode_sequences(chromosome)
        ur5e_cost, ur5e_bad, ur5e_edges = self._evaluate_arm_sequence("ur5e", sequences["ur5e"])
        fr3_cost, fr3_bad, fr3_edges = self._evaluate_arm_sequence("fr3", sequences["fr3"])

        total_cost = ur5e_cost + fr3_cost
        balance_penalty = self.balance_weight * abs(ur5e_cost - fr3_cost)
        infeasible_edges = ur5e_bad + fr3_bad
        fitness = total_cost + balance_penalty + infeasible_edges * self.infeasible_penalty

        return PlanEvaluation(
            total_cost=total_cost,
            ur5e_cost=ur5e_cost,
            fr3_cost=fr3_cost,
            balance_penalty=balance_penalty,
            infeasible_edges=infeasible_edges,
            fitness=fitness,
            ur5e_sequence=sequences["ur5e"],
            fr3_sequence=sequences["fr3"],
            ur5e_edges=ur5e_edges,
            fr3_edges=fr3_edges,
        )

    def tournament_select(self, population: Sequence[Chromosome], evaluations: Sequence[PlanEvaluation], k: int = 3) -> Chromosome:
        indices = [self.random.randrange(len(population)) for _ in range(k)]
        winner_idx = min(indices, key=lambda idx: evaluations[idx].fitness)
        winner = population[winner_idx]
        return Chromosome(winner.permutation.copy(), winner.assignment.copy())

    def order_crossover(self, parent_a: Chromosome, parent_b: Chromosome) -> Tuple[Chromosome, Chromosome]:
        size = len(self.targets)
        left, right = sorted(self.random.sample(range(size), 2)) if size >= 2 else (0, 0)

        def ox(pa: List[str], pb: List[str]) -> List[str]:
            child = [None] * size
            child[left:right + 1] = pa[left:right + 1]
            insert_values = [gene for gene in pb if gene not in child]
            insert_idx = 0
            for i in range(size):
                if child[i] is None:
                    child[i] = insert_values[insert_idx]
                    insert_idx += 1
            return child

        child1_perm = ox(parent_a.permutation, parent_b.permutation)
        child2_perm = ox(parent_b.permutation, parent_a.permutation)

        child1_assign = {}
        child2_assign = {}
        for target in self.targets:
            child1_assign[target] = parent_a.assignment[target] if self.random.random() < 0.5 else parent_b.assignment[target]
            child2_assign[target] = parent_b.assignment[target] if self.random.random() < 0.5 else parent_a.assignment[target]

        child1 = Chromosome(child1_perm, self.repair_assignment(child1_assign))
        child2 = Chromosome(child2_perm, self.repair_assignment(child2_assign))
        return child1, child2

    def repair_assignment(self, assignment: Dict[str, str]) -> Dict[str, str]:
        repaired = assignment.copy()
        for target, arm_name in repaired.items():
            if arm_name not in self.feasible_arms[target]:
                repaired[target] = self.feasible_arms[target][0]
        return repaired

    def mutate(self, chromosome: Chromosome, permutation_rate: float = 0.25, assignment_rate: float = 0.15):
        if len(chromosome.permutation) >= 2 and self.random.random() < permutation_rate:
            i, j = self.random.sample(range(len(chromosome.permutation)), 2)
            chromosome.permutation[i], chromosome.permutation[j] = chromosome.permutation[j], chromosome.permutation[i]

        for target in self.targets:
            if self.random.random() < assignment_rate:
                candidates = self.feasible_arms[target]
                chromosome.assignment[target] = self.random.choice(candidates)

        chromosome.assignment = self.repair_assignment(chromosome.assignment)

    def evolve(
        self,
        population_size: int = 80,
        generations: int = 200,
        elite_size: int = 6,
        crossover_rate: float = 0.9,
        use_greedy_seed: bool = True,
    ) -> Tuple[Chromosome, PlanEvaluation, List[dict]]:
        if population_size < 4:
            raise ValueError("population_size 至少为 4")

        if use_greedy_seed:
            population = [self.greedy_seed()] + [self.random_chromosome() for _ in range(population_size - 1)]
        else:
            population = [self.random_chromosome() for _ in range(population_size)]
        history = []

        best_chromosome = None
        best_eval = None

        for gen in range(generations):
            evaluations = [self.evaluate(ch) for ch in population]
            ranked_indices = sorted(range(len(population)), key=lambda idx: evaluations[idx].fitness)
            population = [population[idx] for idx in ranked_indices]
            evaluations = [evaluations[idx] for idx in ranked_indices]

            if best_eval is None or evaluations[0].fitness < best_eval.fitness:
                best_chromosome = Chromosome(population[0].permutation.copy(), population[0].assignment.copy())
                best_eval = evaluations[0]

            history.append(
                {
                    "generation": gen,
                    "best_fitness": evaluations[0].fitness,
                    "best_total_cost": evaluations[0].total_cost,
                    "best_infeasible_edges": evaluations[0].infeasible_edges,
                }
            )

            next_population = [
                Chromosome(population[i].permutation.copy(), population[i].assignment.copy())
                for i in range(min(elite_size, len(population)))
            ]

            while len(next_population) < population_size:
                parent_a = self.tournament_select(population, evaluations)
                parent_b = self.tournament_select(population, evaluations)
                if self.random.random() < crossover_rate:
                    child_a, child_b = self.order_crossover(parent_a, parent_b)
                else:
                    child_a = parent_a
                    child_b = parent_b

                self.mutate(child_a)
                self.mutate(child_b)
                next_population.append(child_a)
                if len(next_population) < population_size:
                    next_population.append(child_b)

            population = next_population

        return best_chromosome, best_eval, history


def load_matrix(csv_path: Path) -> pd.DataFrame:
    matrix = pd.read_csv(csv_path, index_col=0)
    matrix = matrix.apply(pd.to_numeric, errors="coerce")
    return matrix


def infer_targets(ur5e_matrix: pd.DataFrame, fr3_matrix: pd.DataFrame) -> List[str]:
    shared = set(ur5e_matrix.index.tolist()) & set(fr3_matrix.index.tolist())
    shared.discard("home")
    return sorted(shared)


def save_solution(
    output_path: Path,
    chromosome: Chromosome,
    evaluation: PlanEvaluation,
    history: List[dict],
):
    payload = {
        "fitness": evaluation.fitness,
        "total_cost": evaluation.total_cost,
        "ur5e_cost": evaluation.ur5e_cost,
        "fr3_cost": evaluation.fr3_cost,
        "balance_penalty": evaluation.balance_penalty,
        "infeasible_edges": evaluation.infeasible_edges,
        "assignment": chromosome.assignment,
        "global_permutation": chromosome.permutation,
        "ur5e_sequence": evaluation.ur5e_sequence,
        "fr3_sequence": evaluation.fr3_sequence,
        "ur5e_edges": evaluation.ur5e_edges,
        "fr3_edges": evaluation.fr3_edges,
        "history": history,
    }
    output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def print_summary(evaluation: PlanEvaluation):
    print("\n" + "=" * 70)
    print("GA 优化完成")
    print("=" * 70)
    print(f"总成本: {evaluation.total_cost:.3f}")
    print(f"UR5e 成本: {evaluation.ur5e_cost:.3f}")
    print(f"FR3  成本: {evaluation.fr3_cost:.3f}")
    print(f"均衡惩罚: {evaluation.balance_penalty:.3f}")
    print(f"不可达边数: {evaluation.infeasible_edges}")
    print(f"适应度: {evaluation.fitness:.3f}")
    print(f"UR5e 顺序: {evaluation.ur5e_sequence}")
    print(f"FR3  顺序: {evaluation.fr3_sequence}")


def parse_args():
    parser = argparse.ArgumentParser(description="基于 pairwise 转移矩阵执行 GA 任务分配与排序优化")
    parser.add_argument("--ur5e-matrix", default="pairwise_transition_cost_ur5e.csv", help="UR5e pairwise 矩阵路径")
    parser.add_argument("--fr3-matrix", default="pairwise_transition_cost_fr3.csv", help="FR3 pairwise 矩阵路径")
    parser.add_argument("--targets", nargs="*", default=None, help="参与优化的目标点；默认从矩阵自动推断")
    parser.add_argument("--population", type=int, default=80, help="种群规模")
    parser.add_argument("--generations", type=int, default=200, help="进化代数")
    parser.add_argument("--elite-size", type=int, default=6, help="精英保留数量")
    parser.add_argument("--balance-weight", type=float, default=DEFAULT_BALANCE_WEIGHT, help="负载均衡惩罚权重")
    parser.add_argument("--seed", type=int, default=DEFAULT_RANDOM_SEED, help="随机种子")
    parser.add_argument("--disable-greedy-seed", action="store_true", help="禁用启发式种子注入（对照组：随机初始化）")
    parser.add_argument("--output", default="ga_best_solution.json", help="最优解输出 JSON 路径")
    return parser.parse_args()


def main():
    args = parse_args()
    ur5e_matrix = load_matrix(Path(args.ur5e_matrix))
    fr3_matrix = load_matrix(Path(args.fr3_matrix))
    targets = args.targets if args.targets else infer_targets(ur5e_matrix, fr3_matrix)
    if not targets:
        raise ValueError("未能从矩阵中推断出任何目标点，请通过 --targets 显式指定")

    allocator = GATaskAllocator(
        ur5e_matrix=ur5e_matrix,
        fr3_matrix=fr3_matrix,
        targets=targets,
        balance_weight=args.balance_weight,
        random_seed=args.seed,
    )
    best_chromosome, best_eval, history = allocator.evolve(
        population_size=args.population,
        generations=args.generations,
        elite_size=args.elite_size,
        use_greedy_seed=not args.disable_greedy_seed,
    )

    print_summary(best_eval)
    output_path = Path(args.output)
    save_solution(output_path, best_chromosome, best_eval, history)
    print(f"最优解已保存到: {output_path}")


if __name__ == "__main__":
    main()
