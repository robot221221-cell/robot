"""
读取单点代价矩阵并自动分类目标点，输出可直接给 GA 使用的任务列表。

分类规则：
- dual_feasible: UR5e 和 FR3 都可达
- ur5e_only: 仅 UR5e 可达
- fr3_only: 仅 FR3 可达
- unreachable: 两者都不可达

输出文件：
- target_classification.csv
- ga_task_list.json
- ga_task_targets.txt
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd


def is_finite_cost(value) -> bool:
    try:
        return np.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def classify_row(ur5e_cost, fr3_cost) -> str:
    ur5e_ok = is_finite_cost(ur5e_cost)
    fr3_ok = is_finite_cost(fr3_cost)

    if ur5e_ok and fr3_ok:
        return "dual_feasible"
    if ur5e_ok:
        return "ur5e_only"
    if fr3_ok:
        return "fr3_only"
    return "unreachable"


def load_cost_matrix(cost_matrix_path: Path) -> pd.DataFrame:
    df = pd.read_csv(cost_matrix_path)
    required_columns = {"Target", "UR5e_Cost", "FR3_Cost"}
    missing = required_columns - set(df.columns)
    if missing:
        raise ValueError(f"代价矩阵缺少必要列: {sorted(missing)}")
    return df


def build_classification(df: pd.DataFrame) -> pd.DataFrame:
    classified = df.copy()
    classified["Category"] = classified.apply(
        lambda row: classify_row(row["UR5e_Cost"], row["FR3_Cost"]),
        axis=1,
    )
    classified["FeasibleForGA"] = classified["Category"] != "unreachable"
    return classified


def build_output_payload(classified: pd.DataFrame) -> Dict[str, List[str]]:
    result = {
        "dual_feasible": classified.loc[classified["Category"] == "dual_feasible", "Target"].tolist(),
        "ur5e_only": classified.loc[classified["Category"] == "ur5e_only", "Target"].tolist(),
        "fr3_only": classified.loc[classified["Category"] == "fr3_only", "Target"].tolist(),
        "unreachable": classified.loc[classified["Category"] == "unreachable", "Target"].tolist(),
    }
    result["ga_task_targets"] = classified.loc[classified["FeasibleForGA"], "Target"].tolist()
    return result


def parse_args():
    parser = argparse.ArgumentParser(description="读取单点代价矩阵并生成可直接给 GA 使用的任务列表")
    parser.add_argument(
        "--cost-matrix",
        default="kinematic_cost_matrix.csv",
        help="单点代价矩阵路径，默认 kinematic_cost_matrix.csv",
    )
    parser.add_argument(
        "--output-dir",
        default=".",
        help="输出目录，默认当前工作区根目录",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cost_matrix_path = Path(args.cost_matrix)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_cost_matrix(cost_matrix_path)
    classified = build_classification(df)
    payload = build_output_payload(classified)

    classification_csv = output_dir / "target_classification.csv"
    ga_task_json = output_dir / "ga_task_list.json"
    ga_task_txt = output_dir / "ga_task_targets.txt"

    classified.to_csv(classification_csv, index=False)
    ga_task_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    ga_task_txt.write_text("\n".join(payload["ga_task_targets"]) + ("\n" if payload["ga_task_targets"] else ""), encoding="utf-8")

    print("=" * 70)
    print("目标点分类完成")
    print("=" * 70)
    for key in ["dual_feasible", "ur5e_only", "fr3_only", "unreachable", "ga_task_targets"]:
        print(f"{key}: {payload[key]}")

    print(f"\n✅ 已保存分类表: {classification_csv.name}")
    print(f"✅ 已保存 GA 任务列表(JSON): {ga_task_json.name}")
    print(f"✅ 已保存 GA 任务列表(TXT): {ga_task_txt.name}")


if __name__ == "__main__":
    main()
