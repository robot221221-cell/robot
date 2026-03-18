"""
批量运行调度实验，自动汇总“方法差异”表与图。

设计目标：
- 不换算法，仅通过不同任务强度参数观察指标差异
- 自动重复多次，输出每次结果 + 分组统计结果
- 图表文字使用中文
"""

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from statistics import mean
from typing import Dict, List, Tuple


def _safe_float(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def _finite_mean(vals: List[float]) -> float:
    valid = [v for v in vals if math.isfinite(v)]
    if not valid:
        return float("nan")
    return mean(valid)


def _fmt(v, digits=4):
    x = _safe_float(v)
    if math.isnan(x):
        return "-"
    if math.isinf(x):
        return "inf"
    return f"{x:.{digits}f}"


def _method_cn(name: str) -> str:
    return {
        "auto_delay_off_preview": "AutoDelay关闭(预览)",
        "auto_delay_on_preview": "AutoDelay开启(预览)",
        "execution_actual": "实际执行",
    }.get(name, name)


def _method_cn_plot(name: str) -> str:
    return {
        "auto_delay_off_preview": "关闭",
        "auto_delay_on_preview": "开启",
        "execution_actual": "实际",
    }.get(name, name)


def run_one(
    project_root: Path,
    solution: Path,
    output_json: Path,
    hold_steps: int,
    sim_steps_per_tick: int,
    priority_arm: str,
    tick_sleep: float,
):
    cmd = [
        "python",
        str(project_root / "schedule_ga_with_priority_delay.py"),
        "--solution",
        str(solution),
        "--headless",
        "--report-ablation",
        "--stats-json",
        str(output_json),
        "--hold-steps",
        str(hold_steps),
        "--sim-steps-per-tick",
        str(sim_steps_per_tick),
        "--priority-arm",
        priority_arm,
        "--tick-sleep",
        str(tick_sleep),
    ]
    print("运行:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=str(project_root))


def aggregate_rows(all_rows: List[Dict]) -> List[Dict]:
    groups: Dict[Tuple[str, str], List[Dict]] = {}
    for row in all_rows:
        key = (row["scenario"], row["method"])
        groups.setdefault(key, []).append(row)

    summary = []
    for (scenario, method), rows in groups.items():
        success_vals = [int(r["success"]) for r in rows]
        parallel_vals = [_safe_float(r["parallel_makespan_s"]) for r in rows]
        speedup_vals = [_safe_float(r["speedup_vs_serial"]) for r in rows]
        wait_vals = [_safe_float(r["wait_time_total_s"]) for r in rows]
        deadlock_vals = [_safe_float(r["deadlock_wait_count"]) for r in rows]

        summary.append(
            {
                "scenario": scenario,
                "method": method,
                "method_cn": _method_cn(method),
                "runs": len(rows),
                "success_rate": sum(success_vals) / max(1, len(success_vals)),
                "parallel_makespan_mean_s": _finite_mean(parallel_vals),
                "speedup_mean": _finite_mean(speedup_vals),
                "wait_time_total_mean_s": _finite_mean(wait_vals),
                "deadlock_wait_mean": _finite_mean(deadlock_vals),
            }
        )

    summary.sort(key=lambda x: (x["scenario"], x["method"]))
    return summary


def write_csv(rows: List[Dict], path: Path):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown_summary(summary_rows: List[Dict], path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 批量调度实验汇总报告",
        "",
        "| 场景 | 方法 | 重复次数 | 成功率 | 并行时长均值(s) | 加速比均值 | 总等待时间均值(s) | 死锁等待均值 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in summary_rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(r["scenario"]),
                    str(r["method_cn"]),
                    str(r["runs"]),
                    _fmt(r["success_rate"], 3),
                    _fmt(r["parallel_makespan_mean_s"], 3),
                    _fmt(r["speedup_mean"], 3),
                    _fmt(r["wait_time_total_mean_s"], 3),
                    _fmt(r["deadlock_wait_mean"], 3),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## 建议固定评价指标",
            "",
            "1. 成功率 success_rate",
            "2. 并行总时长 parallel_makespan_mean_s",
            "3. 相对串行加速比 speedup_mean",
            "4. 总等待时间 wait_time_total_mean_s",
            "5. 死锁等待次数 deadlock_wait_mean",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_summary(summary_rows: List[Dict], output_png: Path):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.font_manager as fm
    except Exception:
        print("⚠️ 未安装 matplotlib，跳过汇总图导出")
        return

    available = {f.name for f in fm.fontManager.ttflist}
    for name in ["Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei"]:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False

    scenarios = sorted(set(r["scenario"] for r in summary_rows))
    methods = ["auto_delay_off_preview", "auto_delay_on_preview", "execution_actual"]

    def get_metric(scenario, method, key):
        for r in summary_rows:
            if r["scenario"] == scenario and r["method"] == method:
                return _safe_float(r[key])
        return float("nan")

    x = list(range(len(scenarios)))
    w = 0.22

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    for i, m in enumerate(methods):
        vals = [get_metric(s, m, "success_rate") for s in scenarios]
        axes[0].bar([xi + (i - 1) * w for xi in x], vals, width=w, label=_method_cn_plot(m))
    axes[0].set_title("成功率")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(scenarios, rotation=15)

    for i, m in enumerate(methods):
        vals = [get_metric(s, m, "speedup_mean") for s in scenarios]
        vals = [0.0 if (not math.isfinite(v)) else v for v in vals]
        axes[1].bar([xi + (i - 1) * w for xi in x], vals, width=w, label=_method_cn_plot(m))
    axes[1].set_title("相对串行加速比")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(scenarios, rotation=15)

    for i, m in enumerate(methods):
        vals = [get_metric(s, m, "wait_time_total_mean_s") for s in scenarios]
        vals = [0.0 if (not math.isfinite(v)) else v for v in vals]
        axes[2].bar([xi + (i - 1) * w for xi in x], vals, width=w, label=_method_cn_plot(m))
    axes[2].set_title("总等待时间均值(s)")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(scenarios, rotation=15)

    axes[2].legend(loc="upper right")
    plt.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_png, dpi=220)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="批量运行调度实验并输出汇总表/图")
    parser.add_argument("--solution", default="xml_pipeline_run/ga_best_solution.json", help="GA 解路径")
    parser.add_argument("--output-dir", default="xml_pipeline_run/batch_experiments", help="批量输出目录")
    parser.add_argument("--repeats", type=int, default=2, help="每个场景重复次数")
    parser.add_argument("--hold-steps-list", nargs="+", type=int, default=[20, 80, 160], help="任务保持步数列表")
    parser.add_argument("--sim-steps-list", nargs="+", type=int, default=[1], help="sim_steps_per_tick 列表")
    parser.add_argument("--priority-arms", nargs="+", default=["ur5e"], choices=["ur5e", "fr3"], help="优先级机械臂列表")
    parser.add_argument("--tick-sleep", type=float, default=0.0, help="每个物理步休眠，批量建议 0")
    parser.add_argument("--disable-auto-organize", action="store_true", help="禁用实验结束后的自动结果整理")
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parent
    solution = (root / args.solution).resolve() if not Path(args.solution).is_absolute() else Path(args.solution)
    out_dir = (root / args.output_dir).resolve() if not Path(args.output_dir).is_absolute() else Path(args.output_dir)
    runs_dir = out_dir / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)

    all_rows: List[Dict] = []

    scenario_idx = 0
    for hold in args.hold_steps_list:
        for sim_step in args.sim_steps_list:
            for priority in args.priority_arms:
                scenario_idx += 1
                scenario_name = f"S{scenario_idx}_hold{hold}_sim{sim_step}_prio{priority}"
                for r in range(1, args.repeats + 1):
                    json_path = runs_dir / f"{scenario_name}_run{r}.json"
                    run_one(
                        project_root=root,
                        solution=solution,
                        output_json=json_path,
                        hold_steps=hold,
                        sim_steps_per_tick=sim_step,
                        priority_arm=priority,
                        tick_sleep=args.tick_sleep,
                    )

                    data = json.loads(json_path.read_text(encoding="utf-8"))
                    for row in data.get("comparison_rows", []):
                        enriched = {
                            "scenario": scenario_name,
                            "run_id": r,
                            "hold_steps": hold,
                            "sim_steps_per_tick": sim_step,
                            "priority_arm": priority,
                        }
                        enriched.update(row)
                        all_rows.append(enriched)

    detailed_csv = out_dir / "batch_detailed_rows.csv"
    write_csv(all_rows, detailed_csv)

    summary_rows = aggregate_rows(all_rows)
    summary_csv = out_dir / "batch_summary.csv"
    summary_md = out_dir / "batch_summary.md"
    summary_png = out_dir / "batch_summary.png"

    write_csv(summary_rows, summary_csv)
    write_markdown_summary(summary_rows, summary_md)
    plot_summary(summary_rows, summary_png)

    print("\n✅ 批量实验完成")
    print(f"- 逐次明细: {detailed_csv}")
    print(f"- 汇总表 CSV: {summary_csv}")
    print(f"- 汇总报告 MD: {summary_md}")
    print(f"- 汇总图 PNG: {summary_png}")

    if not args.disable_auto_organize:
        organizer = root / "organize_outputs.py"
        if organizer.exists():
            print("\n🧹 自动整理输出文件...")
            subprocess.run([sys.executable, str(organizer), "--root", str(root), "--apply"], check=False)


if __name__ == "__main__":
    main()
