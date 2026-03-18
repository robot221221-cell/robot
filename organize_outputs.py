#!/usr/bin/env python3
"""整理项目中已生成的实验结果文件到分类目录。

默认行为：打印计划（dry-run）
加 --apply：执行移动
"""

from __future__ import annotations

import argparse
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import List


@dataclass
class MovePlan:
    src: Path
    dst: Path


def build_plans(root: Path) -> List[MovePlan]:
    plans: List[MovePlan] = []

    # 1) 主目录结果 -> artifacts/root_outputs
    root_csv_dir = root / "artifacts" / "root_outputs" / "csv"
    root_json_dir = root / "artifacts" / "root_outputs" / "json"

    root_csv_patterns = [
        "kinematic_cost_matrix.csv",
        "pairwise_transition_cost_*.csv",
        "pairwise_transition_edges_*.csv",
        "target_classification.csv",
    ]
    root_json_patterns = [
        "ga_task_list.json",
        "ga_best_solution_subset.json",
    ]

    for pattern in root_csv_patterns:
        for src in root.glob(pattern):
            if src.is_file():
                plans.append(MovePlan(src=src, dst=root_csv_dir / src.name))
    for pattern in root_json_patterns:
        for src in root.glob(pattern):
            if src.is_file():
                plans.append(MovePlan(src=src, dst=root_json_dir / src.name))

    # 2) xml_pipeline_run 根目录分流
    xroot = root / "xml_pipeline_run"
    if xroot.exists():
        cost_dir = xroot / "data" / "cost_matrices"
        class_dir = xroot / "data" / "classification"
        report_dir = xroot / "reports" / "schedule"

        for src in xroot.glob("pairwise_transition_*.csv"):
            if src.is_file():
                plans.append(MovePlan(src=src, dst=cost_dir / src.name))

        for name in ["kinematic_cost_matrix.csv"]:
            src = xroot / name
            if src.is_file():
                plans.append(MovePlan(src=src, dst=cost_dir / src.name))

        for name in ["target_classification.csv"]:
            src = xroot / name
            if src.is_file():
                plans.append(MovePlan(src=src, dst=class_dir / src.name))

        for name in [
            "schedule_ablation_stats.json",
            "schedule_comparison.csv",
            "schedule_comparison.md",
            "schedule_report.md",
            "schedule_report.png",
        ]:
            src = xroot / name
            if src.is_file():
                plans.append(MovePlan(src=src, dst=report_dir / src.name))

        # 3) difficulty_experiments_v6 顶层 stats -> stats/ 子目录（保留 solution_*.json 在原位）
        d6 = xroot / "difficulty_experiments_v6"
        if d6.exists():
            d6_stats_dir = d6 / "stats"
            for src in d6.glob("stats_*.json"):
                if src.is_file():
                    plans.append(MovePlan(src=src, dst=d6_stats_dir / src.name))

        # 4) difficulty_experiments_v5 顶层 stats -> stats/ 子目录
        d5 = xroot / "difficulty_experiments_v5"
        if d5.exists():
            d5_stats_dir = d5 / "stats"
            for src in d5.glob("stats_*.json"):
                if src.is_file():
                    plans.append(MovePlan(src=src, dst=d5_stats_dir / src.name))
            for src in d5.glob("debug_*.json"):
                if src.is_file():
                    plans.append(MovePlan(src=src, dst=d5_stats_dir / src.name))

    # 去重（src相同仅保留一个）
    unique = {}
    for p in plans:
        unique[str(p.src)] = p
    return list(unique.values())


def apply_plans(plans: List[MovePlan], do_apply: bool) -> None:
    if not plans:
        print("没有可整理的文件。")
        return

    print(f"待处理文件数: {len(plans)}")
    for p in plans:
        print(f"- {p.src} -> {p.dst}")

    if not do_apply:
        print("\n当前为 dry-run。加 --apply 执行移动。")
        return

    moved = 0
    skipped = 0
    for p in plans:
        if not p.src.exists():
            skipped += 1
            continue
        p.dst.parent.mkdir(parents=True, exist_ok=True)
        if p.dst.exists():
            # 已存在则覆盖
            if p.dst.is_file():
                p.dst.unlink()
        shutil.move(str(p.src), str(p.dst))
        moved += 1

    print(f"\n完成：moved={moved}, skipped={skipped}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="整理实验输出文件到分类目录")
    parser.add_argument("--root", default=".", help="项目根目录")
    parser.add_argument("--apply", action="store_true", help="执行移动（默认仅预览）")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(args.root).resolve()
    plans = build_plans(root)
    apply_plans(plans, do_apply=args.apply)


if __name__ == "__main__":
    main()
