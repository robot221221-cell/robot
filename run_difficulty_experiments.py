"""
基于现有 GA 最优解自动构造 Easy/Medium/Hard 三档任务难度，
并运行调度对比实验，输出中文汇总报告与图表。
"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, List

from dual_arm_simulation import ALL_TARGETS, ARM_HOME_Q, DualArmEnvironment, calculate_target_posture


def load_solution(path: Path) -> Dict:
    return json.loads(path.read_text(encoding="utf-8"))


EXTRA_CONFLICT_TARGETS = {
    # 选择双臂均可执行的新目标点，确保“难度上升”而非“问题变成不可解”
    "ur5e": ["target_top_3", "target_top_4", "target_left_2"],
    "fr3": ["target_top_3", "target_top_4", "target_left_2"],
}


def expand_sequence(seq: List[str], level: str, arm_name: str) -> List[str]:
    base = list(seq)
    rev = list(reversed(seq))
    extras = list(EXTRA_CONFLICT_TARGETS.get(arm_name, []))

    # 去重但保序，避免和原序列重复点导致有效任务数虚增
    extras = [t for t in extras if t not in base]

    if level == "easy":
        return base
    if level == "medium":
        # 往返 + 中区冲突点，显著增加交汇区时空重叠
        return base + extras + rev
    if level == "hard":
        # 在 medium 基础上增加“有限强化段”：
        # - 仍包含共享冲突点（保持 hard 难度）
        # - 减少连续重复长度，避免早期长段顶牛（目标总任务约 30）
        medium = base + extras + rev
        hard_tail = {
            "ur5e": ["target_left_1", "target_top_3", "target_front_2"],
            "fr3": ["target_back_1", "target_top_4", "target_right_1"],
        }
        return medium + hard_tail.get(arm_name, [])
    raise ValueError(level)


def build_solution_variant(base_solution: Dict, level: str) -> Dict:
    out = dict(base_solution)
    ur_seq = base_solution.get("ur5e_sequence", [])
    fr_seq = base_solution.get("fr3_sequence", [])
    out["ur5e_sequence"] = expand_sequence(ur_seq, level, "ur5e")
    out["fr3_sequence"] = expand_sequence(fr_seq, level, "fr3")
    return out


def detect_reachable_targets(scene_path: Path) -> Dict[str, List[str]]:
    env = DualArmEnvironment(str(scene_path))
    reachable = {"ur5e": [], "fr3": []}
    for target in ALL_TARGETS:
        for arm_name in ["ur5e", "fr3"]:
            env.reset(ARM_HOME_Q["ur5e"], ARM_HOME_Q["fr3"])
            q_approach, q_target = calculate_target_posture(env, arm_name, target)
            if q_approach is not None and q_target is not None:
                reachable[arm_name].append(target)
    return reachable


def ensure_all_reachable_targets_covered(solution: Dict, reachable: Dict[str, List[str]]) -> Dict:
    out = dict(solution)
    ur_seq = list(solution.get("ur5e_sequence", []))
    fr_seq = list(solution.get("fr3_sequence", []))

    ur_reach = set(reachable.get("ur5e", []))
    fr_reach = set(reachable.get("fr3", []))

    visited = set(ur_seq) | set(fr_seq)
    all_targets = list(ALL_TARGETS)
    missing = [t for t in all_targets if t not in visited]

    unresolved = []
    for target in missing:
        feasible_arms = []
        if target in ur_reach:
            feasible_arms.append("ur5e")
        if target in fr_reach:
            feasible_arms.append("fr3")

        if not feasible_arms:
            unresolved.append(target)
            continue

        if len(feasible_arms) == 1:
            chosen = feasible_arms[0]
        else:
            chosen = "ur5e" if len(ur_seq) <= len(fr_seq) else "fr3"

        if chosen == "ur5e":
            ur_seq.append(target)
        else:
            fr_seq.append(target)

    out["ur5e_sequence"] = ur_seq
    out["fr3_sequence"] = fr_seq
    out["coverage_unreachable_targets"] = unresolved
    out["coverage_mode"] = "all_reachable_targets"
    return out


def run_schedule(
    project_root: Path,
    solution_path: Path,
    stats_json: Path,
    hold_steps: int,
    sim_steps: int,
    priority_arm: str,
    max_start_delay: int,
    delay_coarse_step: int,
):
    cmd = [
        "python",
        str(project_root / "schedule_ga_with_priority_delay.py"),
        "--solution",
        str(solution_path),
        "--headless",
        "--report-ablation",
        "--stats-json",
        str(stats_json),
        "--hold-steps",
        str(hold_steps),
        "--sim-steps-per-tick",
        str(sim_steps),
        "--priority-arm",
        priority_arm,
        "--tick-sleep",
        "0",
        "--max-start-delay",
        str(max_start_delay),
        "--delay-coarse-step",
        str(delay_coarse_step),
    ]
    print("运行:", " ".join(cmd))
    subprocess.run(cmd, cwd=str(project_root), check=True)


def _safe(v):
    try:
        return float(v)
    except Exception:
        return float("nan")


def _fmt(v, d=3):
    x = _safe(v)
    if math.isnan(x):
        return "-"
    if math.isinf(x):
        return "inf"
    return f"{x:.{d}f}"


def summarize_case(case_name: str, stats: Dict) -> Dict:
    rows = stats.get("comparison_rows", [])
    idx = {r.get("method"): r for r in rows}
    off = idx.get("auto_delay_off_preview", {})
    on = idx.get("auto_delay_on_preview", {})
    act = idx.get("execution_actual", {})

    return {
        "case": case_name,
        "off_success": int(off.get("success", 0)),
        "off_deadlock_wait": int(off.get("deadlock_wait_count", 0)),
        "on_success": int(on.get("success", 0)),
        "on_speedup": _safe(on.get("speedup_vs_serial", float("nan"))),
        "on_wait_time_s": _safe(on.get("wait_time_total_s", float("nan"))),
        "act_success": int(act.get("success", 0)),
        "act_parallel_makespan_s": _safe(act.get("parallel_makespan_s", float("nan"))),
        "act_speedup": _safe(act.get("speedup_vs_serial", float("nan"))),
    }


def write_markdown(summary_rows: List[Dict], out_md: Path):
    lines = [
        "# 难度分级实验报告（中文）",
        "",
        "| 难度 | AutoDelay关闭成功 | 关闭死锁等待次数 | AutoDelay开启成功 | 开启加速比 | 开启总等待时间(s) | 实际执行成功 | 实际并行时长(s) | 实际加速比 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in summary_rows:
        lines.append(
            "| " + " | ".join([
                r["case"],
                str(r["off_success"]),
                str(r["off_deadlock_wait"]),
                str(r["on_success"]),
                _fmt(r["on_speedup"]),
                _fmt(r["on_wait_time_s"]),
                str(r["act_success"]),
                _fmt(r["act_parallel_makespan_s"]),
                _fmt(r["act_speedup"]),
            ]) + " |"
        )

    lines.extend([
        "",
        "## 结论阅读建议",
        "",
        "1. 看 `AutoDelay关闭成功` 与 `关闭死锁等待次数`：衡量无保护策略是否稳定。",
        "2. 看 `开启加速比` 与 `开启总等待时间`：衡量当前策略收益与代价。",
        "3. 看 `实际并行时长`：最终执行端真实表现。",
        "",
    ])

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines), encoding="utf-8")


def plot_summary(summary_rows: List[Dict], out_png: Path):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.font_manager as fm
    except Exception:
        print("⚠️ 无法导出图表（缺少 matplotlib）")
        return

    # 中文字体
    available = {f.name for f in fm.fontManager.ttflist}
    for name in ["Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Micro Hei", "Microsoft YaHei", "SimHei"]:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False

    cases = [r["case"] for r in summary_rows]
    off_success = [r["off_success"] for r in summary_rows]
    on_success = [r["on_success"] for r in summary_rows]
    on_speedup = [0.0 if not math.isfinite(r["on_speedup"]) else r["on_speedup"] for r in summary_rows]
    on_wait = [0.0 if not math.isfinite(r["on_wait_time_s"]) else r["on_wait_time_s"] for r in summary_rows]

    x = list(range(len(cases)))
    w = 0.35

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    axes[0].bar([xi - w / 2 for xi in x], off_success, width=w, label="关闭")
    axes[0].bar([xi + w / 2 for xi in x], on_success, width=w, label="开启")
    axes[0].set_title("成功率对比")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(cases)
    axes[0].legend()

    axes[1].bar(x, on_speedup, width=0.5, color="#1f77b4")
    axes[1].set_title("AutoDelay开启：加速比")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(cases)

    axes[2].bar(x, on_wait, width=0.5, color="#ff7f0e")
    axes[2].set_title("AutoDelay开启：总等待时间(s)")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(cases)

    plt.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_png, dpi=220)
    plt.close(fig)


def parse_args():
    p = argparse.ArgumentParser(description="难度分级实验执行器")
    p.add_argument("--solution", default="xml_pipeline_run/ga_best_solution.json", help="基础 GA 方案")
    p.add_argument("--output-dir", default="xml_pipeline_run/difficulty_experiments", help="输出目录")
    p.add_argument("--hold-steps", type=int, default=80, help="保持步数")
    p.add_argument("--sim-steps-per-tick", type=int, default=1, help="调度步参数")
    p.add_argument("--priority-arm", choices=["ur5e", "fr3"], default="ur5e", help="高优先级机械臂")
    p.add_argument("--scene", default="dual_arm_scene.xml", help="场景 XML 路径（用于可达性检查）")
    p.add_argument(
        "--no-cover-all-reachable-targets",
        action="store_true",
        help="关闭默认的“覆盖全部可达目标点”行为",
    )
    p.add_argument("--max-start-delay", type=int, default=2000, help="自动延迟搜索上限（tick）")
    p.add_argument("--delay-coarse-step", type=int, default=10, help="自动延迟粗搜索步长")
    p.add_argument("--disable-auto-organize", action="store_true", help="禁用实验结束后的自动结果整理")
    return p.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parent
    solution_path = (root / args.solution).resolve() if not Path(args.solution).is_absolute() else Path(args.solution)
    out_dir = (root / args.output_dir).resolve() if not Path(args.output_dir).is_absolute() else Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    base = load_solution(solution_path)
    scene_path = (root / args.scene).resolve() if not Path(args.scene).is_absolute() else Path(args.scene)

    reachable = detect_reachable_targets(scene_path)
    reachable_union = set(reachable["ur5e"]) | set(reachable["fr3"])
    unreachable = [t for t in ALL_TARGETS if t not in reachable_union]
    print(f"可达目标统计: 可达 {len(reachable_union)} / 全部 {len(ALL_TARGETS)}")
    if unreachable:
        print(f"⚠️ 不可达目标（任一机械臂都不可达）: {unreachable}")

    levels = ["easy", "medium", "hard"]
    summary_rows = []

    for lv in levels:
        solution_variant = build_solution_variant(base, lv)
        if not args.no_cover_all_reachable_targets:
            solution_variant = ensure_all_reachable_targets_covered(solution_variant, reachable)
        case_solution = out_dir / f"solution_{lv}.json"
        case_stats = out_dir / f"stats_{lv}.json"
        case_solution.write_text(json.dumps(solution_variant, ensure_ascii=False, indent=2), encoding="utf-8")

        run_schedule(
            project_root=root,
            solution_path=case_solution,
            stats_json=case_stats,
            hold_steps=args.hold_steps,
            sim_steps=args.sim_steps_per_tick,
            priority_arm=args.priority_arm,
            max_start_delay=args.max_start_delay,
            delay_coarse_step=args.delay_coarse_step,
        )

        stats = json.loads(case_stats.read_text(encoding="utf-8"))
        summary_rows.append(summarize_case(lv, stats))

    report_md = out_dir / "difficulty_report.md"
    report_png = out_dir / "difficulty_report.png"
    report_json = out_dir / "difficulty_summary.json"

    write_markdown(summary_rows, report_md)
    plot_summary(summary_rows, report_png)
    report_json.write_text(json.dumps(summary_rows, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n✅ 难度分级实验完成")
    print(f"- 报告: {report_md}")
    print(f"- 图表: {report_png}")
    print(f"- 汇总JSON: {report_json}")
    print("\n可视化查看真实运行（建议先看 hard 案例）：")
    print(
        "conda run -n lerobot python schedule_ga_with_priority_delay.py "
        f"--solution {out_dir / 'solution_hard.json'} --hold-steps {args.hold_steps} "
        f"--sim-steps-per-tick {args.sim_steps_per_tick} --priority-arm {args.priority_arm}"
    )

    if not args.disable_auto_organize:
        organizer = root / "organize_outputs.py"
        if organizer.exists():
            print("\n🧹 自动整理输出文件...")
            subprocess.run([sys.executable, str(organizer), "--root", str(root), "--apply"], check=False)


if __name__ == "__main__":
    main()
