#!/usr/bin/env python3
"""
运行 Baseline 批量实验（冻结 solution + 固定 seeds），并自动汇总结果。
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List


DEFAULT_SOLUTIONS = {
    "easy": "xml_pipeline_run/difficulty_experiments_v6/solution_easy.json",
    "medium": "xml_pipeline_run/difficulty_experiments_v6/solution_medium.json",
    "hard": "xml_pipeline_run/difficulty_experiments_v6/solution_hard.json",
}


@dataclass
class RunRecord:
    difficulty: str
    seed: int
    returncode: int
    wall_time_s: float
    stats_json: str
    log_file: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="批量执行 baseline 实验并汇总")
    parser.add_argument("--batch-name", default="", help="批次名称，默认自动生成")
    parser.add_argument("--output-root", default="xml_pipeline_run/baseline_runs", help="批次输出根目录")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44, 45, 46, 47, 48, 49, 50, 51], help="随机种子列表")
    parser.add_argument("--difficulties", nargs="+", default=["easy", "medium", "hard"], choices=["easy", "medium", "hard"], help="难度列表")
    parser.add_argument("--solutions-json", default="", help="可选，覆盖默认 solution 映射的 JSON 文件")

    # 调度参数（默认使用当前稳定配置）
    parser.add_argument("--priority-arm", default="ur5e", choices=["ur5e", "fr3"])
    parser.add_argument("--hold-steps", type=int, default=80)
    parser.add_argument("--sim-steps-per-tick", type=int, default=1)
    parser.add_argument("--velocity-scale", type=float, default=0.8)
    parser.add_argument("--acceleration-scale", type=float, default=0.7)
    parser.add_argument("--max-start-delay", type=int, default=4000)
    parser.add_argument("--delay-coarse-step", type=int, default=20)
    parser.add_argument("--anti-oscillation-trigger", type=int, default=900)
    parser.add_argument("--anti-oscillation-hold", type=int, default=80)
    parser.add_argument("--anti-oscillation-max-attempts", type=int, default=3)
    parser.add_argument("--max-total-ticks", type=int, default=120000)
    parser.add_argument("--max-no-progress-ticks", type=int, default=5000)
    parser.add_argument("--retreat-cooldown-ticks", type=int, default=16)
    parser.add_argument("--max-cmd-step", type=float, default=0.02)
    parser.add_argument("--jump-step-threshold", type=float, default=0.06)
    parser.add_argument("--jump-velocity-threshold", type=float, default=25.0)
    parser.add_argument("--sync-balance-weight", type=float, default=0.3)
    parser.add_argument("--tick-sleep", type=float, default=0.0)
    parser.add_argument("--report-ablation", action="store_true", help="是否输出 auto-delay 开关预览")
    parser.add_argument("--python-exe", default=sys.executable, help="用于执行子进程的 Python")
    parser.add_argument("--stop-on-failure", action="store_true", help="单次失败是否立即停止")
    return parser.parse_args()


def load_solutions(project_root: Path, solutions_json: str) -> Dict[str, str]:
    mapping = dict(DEFAULT_SOLUTIONS)
    if solutions_json:
        custom = json.loads((project_root / solutions_json).read_text(encoding="utf-8"))
        mapping.update(custom)
    return mapping


def build_batch_dir(project_root: Path, output_root: str, batch_name: str) -> Path:
    if not batch_name:
        batch_name = f"b0_system_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    batch_dir = project_root / output_root / batch_name
    (batch_dir / "raw_stats").mkdir(parents=True, exist_ok=True)
    (batch_dir / "logs").mkdir(parents=True, exist_ok=True)
    (batch_dir / "tables").mkdir(parents=True, exist_ok=True)
    return batch_dir


def run_one(
    project_root: Path,
    args: argparse.Namespace,
    difficulty: str,
    seed: int,
    solution_path: Path,
    stats_path: Path,
    log_path: Path,
) -> RunRecord:
    cmd = [
        args.python_exe,
        str(project_root / "schedule_ga_with_priority_delay.py"),
        "--solution", str(solution_path),
        "--headless",
        "--disable-auto-organize",
        "--priority-arm", args.priority_arm,
        "--hold-steps", str(args.hold_steps),
        "--sim-steps-per-tick", str(args.sim_steps_per_tick),
        "--velocity-scale", str(args.velocity_scale),
        "--acceleration-scale", str(args.acceleration_scale),
        "--max-start-delay", str(args.max_start_delay),
        "--delay-coarse-step", str(args.delay_coarse_step),
        "--anti-oscillation-trigger", str(args.anti_oscillation_trigger),
        "--anti-oscillation-hold", str(args.anti_oscillation_hold),
        "--anti-oscillation-max-attempts", str(args.anti_oscillation_max_attempts),
        "--max-total-ticks", str(args.max_total_ticks),
        "--max-no-progress-ticks", str(args.max_no_progress_ticks),
        "--retreat-cooldown-ticks", str(args.retreat_cooldown_ticks),
        "--max-cmd-step", str(args.max_cmd_step),
        "--jump-step-threshold", str(args.jump_step_threshold),
        "--jump-velocity-threshold", str(args.jump_velocity_threshold),
        "--sync-balance-weight", str(args.sync_balance_weight),
        "--tick-sleep", str(args.tick_sleep),
        "--random-seed", str(seed),
        "--stats-json", str(stats_path),
    ]
    if args.report_ablation:
        cmd.append("--report-ablation")

    t0 = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as f:
        proc = subprocess.run(cmd, cwd=str(project_root), stdout=f, stderr=subprocess.STDOUT)
    wall = float(time.perf_counter() - t0)

    return RunRecord(
        difficulty=difficulty,
        seed=seed,
        returncode=int(proc.returncode),
        wall_time_s=wall,
        stats_json=str(stats_path.relative_to(project_root)),
        log_file=str(log_path.relative_to(project_root)),
    )


def _mean_std(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"mean": float("nan"), "sd": float("nan")}
    if len(values) == 1:
        return {"mean": float(values[0]), "sd": 0.0}
    return {"mean": float(statistics.mean(values)), "sd": float(statistics.stdev(values))}


def safe_get(d: Dict, *keys, default=float("nan")):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def summarize(project_root: Path, batch_dir: Path, records: List[RunRecord]) -> None:
    rows = []
    for rec in records:
        stats_path = project_root / rec.stats_json
        payload = {}
        if stats_path.exists():
            payload = json.loads(stats_path.read_text(encoding="utf-8"))

        e = payload.get("execution_stats", {})
        runtime = payload.get("runtime_profile", {})
        row = {
            "difficulty": rec.difficulty,
            "seed": rec.seed,
            "returncode": rec.returncode,
            "success": int(bool(safe_get(e, "success", default=False))),
            "parallel_makespan_s": safe_get(e, "parallel_makespan_s"),
            "speedup_vs_serial": safe_get(e, "speedup_vs_serial"),
            "deadlock_wait_count": safe_get(e, "mode_counter", "deadlock_wait", default=0),
            "wait_time_total_s": safe_get(e, "wait_time_s", "total"),
            "finish_gap_s": safe_get(e, "finish_gap_s"),
            "planning_wall_time_s": runtime.get("planning_wall_time_s", float("nan")),
            "scheduling_preview_wall_time_s": runtime.get("scheduling_preview_wall_time_s", float("nan")),
            "execution_wall_time_s": runtime.get("execution_wall_time_s", float("nan")),
            "total_wall_time_s": runtime.get("total_wall_time_s", rec.wall_time_s),
            "stats_json": rec.stats_json,
            "log_file": rec.log_file,
        }
        rows.append(row)

    csv_path = batch_dir / "tables" / "baseline_trials.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["difficulty", "seed"])
        writer.writeheader()
        writer.writerows(rows)

    summary_rows = []
    for diff in sorted({r["difficulty"] for r in rows}):
        group = [r for r in rows if r["difficulty"] == diff]
        n = len(group)
        succ = sum(int(r["success"]) for r in group)

        def vals(k):
            return [float(r[k]) for r in group if isinstance(r[k], (int, float))]

        makespan = _mean_std(vals("parallel_makespan_s"))
        speedup = _mean_std(vals("speedup_vs_serial"))
        deadlock = _mean_std(vals("deadlock_wait_count"))
        wait_t = _mean_std(vals("wait_time_total_s"))
        plan_t = _mean_std(vals("planning_wall_time_s"))
        preview_t = _mean_std(vals("scheduling_preview_wall_time_s"))
        exec_t = _mean_std(vals("execution_wall_time_s"))
        total_t = _mean_std(vals("total_wall_time_s"))

        summary_rows.append({
            "difficulty": diff,
            "n": n,
            "success_count": succ,
            "success_rate": (succ / n) if n else 0.0,
            "parallel_makespan_mean": makespan["mean"],
            "parallel_makespan_sd": makespan["sd"],
            "speedup_mean": speedup["mean"],
            "speedup_sd": speedup["sd"],
            "deadlock_wait_mean": deadlock["mean"],
            "deadlock_wait_sd": deadlock["sd"],
            "wait_time_mean": wait_t["mean"],
            "wait_time_sd": wait_t["sd"],
            "planning_wall_mean": plan_t["mean"],
            "planning_wall_sd": plan_t["sd"],
            "preview_wall_mean": preview_t["mean"],
            "preview_wall_sd": preview_t["sd"],
            "execution_wall_mean": exec_t["mean"],
            "execution_wall_sd": exec_t["sd"],
            "total_wall_mean": total_t["mean"],
            "total_wall_sd": total_t["sd"],
        })

    summary_csv = batch_dir / "tables" / "baseline_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()) if summary_rows else ["difficulty"])
        writer.writeheader()
        writer.writerows(summary_rows)

    summary_json = batch_dir / "baseline_summary.json"
    summary_json.write_text(json.dumps({"summary": summary_rows, "trials": rows}, ensure_ascii=False, indent=2), encoding="utf-8")

    md_lines = [
        "# Baseline 批次结果汇总",
        "",
        f"- 批次目录: `{batch_dir}`",
        f"- 试验数: {len(rows)}",
        "",
        "| 难度 | n | success_rate | makespan_mean±sd (s) | speedup_mean±sd | deadlock_wait_mean±sd | planning_wall_mean±sd (s) | preview_wall_mean±sd (s) | execution_wall_mean±sd (s) | total_wall_mean±sd (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in summary_rows:
        md_lines.append(
            "| " + " | ".join([
                r["difficulty"],
                str(r["n"]),
                f"{r['success_rate']:.3f}",
                f"{r['parallel_makespan_mean']:.3f} ± {r['parallel_makespan_sd']:.3f}",
                f"{r['speedup_mean']:.3f} ± {r['speedup_sd']:.3f}",
                f"{r['deadlock_wait_mean']:.3f} ± {r['deadlock_wait_sd']:.3f}",
                f"{r['planning_wall_mean']:.3f} ± {r['planning_wall_sd']:.3f}",
                f"{r['preview_wall_mean']:.3f} ± {r['preview_wall_sd']:.3f}",
                f"{r['execution_wall_mean']:.3f} ± {r['execution_wall_sd']:.3f}",
                f"{r['total_wall_mean']:.3f} ± {r['total_wall_sd']:.3f}",
            ]) + " |"
        )

    (batch_dir / "tables" / "baseline_summary.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    project_root = Path(__file__).resolve().parent
    solutions = load_solutions(project_root, args.solutions_json)

    batch_dir = build_batch_dir(project_root, args.output_root, args.batch_name)
    (batch_dir / "configs").mkdir(parents=True, exist_ok=True)

    config_snapshot = {
        "batch_name": batch_dir.name,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "solutions": {k: solutions[k] for k in args.difficulties},
        "seeds": args.seeds,
        "difficulties": args.difficulties,
        "scheduler_args": {
            "priority_arm": args.priority_arm,
            "hold_steps": args.hold_steps,
            "sim_steps_per_tick": args.sim_steps_per_tick,
            "velocity_scale": args.velocity_scale,
            "acceleration_scale": args.acceleration_scale,
            "max_start_delay": args.max_start_delay,
            "delay_coarse_step": args.delay_coarse_step,
            "anti_oscillation_trigger": args.anti_oscillation_trigger,
            "anti_oscillation_hold": args.anti_oscillation_hold,
            "anti_oscillation_max_attempts": args.anti_oscillation_max_attempts,
            "max_total_ticks": args.max_total_ticks,
            "max_no_progress_ticks": args.max_no_progress_ticks,
            "retreat_cooldown_ticks": args.retreat_cooldown_ticks,
            "max_cmd_step": args.max_cmd_step,
            "jump_step_threshold": args.jump_step_threshold,
            "jump_velocity_threshold": args.jump_velocity_threshold,
            "sync_balance_weight": args.sync_balance_weight,
            "tick_sleep": args.tick_sleep,
            "report_ablation": args.report_ablation,
        },
    }
    (batch_dir / "configs" / "baseline_config_snapshot.json").write_text(
        json.dumps(config_snapshot, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    records: List[RunRecord] = []
    for difficulty in args.difficulties:
        solution = project_root / solutions[difficulty]
        if not solution.exists():
            raise FileNotFoundError(f"缺少 solution 文件: {solution}")

        for seed in args.seeds:
            stats_path = batch_dir / "raw_stats" / f"stats_{difficulty}_seed_{seed}.json"
            log_path = batch_dir / "logs" / f"run_{difficulty}_seed_{seed}.log"
            print(f"[Baseline] 运行 {difficulty} seed={seed} ...")
            rec = run_one(project_root, args, difficulty, seed, solution, stats_path, log_path)
            records.append(rec)
            print(f"  -> returncode={rec.returncode}, wall={rec.wall_time_s:.2f}s")
            if rec.returncode != 0 and args.stop_on_failure:
                print("检测到失败，按 --stop-on-failure 停止。")
                summarize(project_root, batch_dir, records)
                sys.exit(rec.returncode)

    summarize(project_root, batch_dir, records)
    print(f"\n✅ Baseline 批量实验完成。输出目录: {batch_dir}")


if __name__ == "__main__":
    main()
