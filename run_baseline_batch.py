#!/usr/bin/env python3
"""
运行 Baseline 批量实验（冻结 solution + 固定 seeds），并自动汇总结果。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
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
    parser.add_argument("--sequence-mode", choices=["ga", "heuristic", "random", "no_evo"], default="ga", help="序列来源：ga / heuristic / random / no_evo（显式GA-off对照）")
    parser.add_argument("--ur5e-matrix", default="xml_pipeline_run/data/cost_matrices/pairwise_transition_cost_ur5e.csv", help="no_evo 模式使用的 UR5e pairwise 矩阵")
    parser.add_argument("--fr3-matrix", default="xml_pipeline_run/data/cost_matrices/pairwise_transition_cost_fr3.csv", help="no_evo 模式使用的 FR3 pairwise 矩阵")

    # 调度参数（默认使用当前稳定配置）
    parser.add_argument("--priority-arm", default="ur5e", choices=["ur5e", "fr3"])
    parser.add_argument("--hold-steps", type=int, default=80)
    parser.add_argument("--sim-steps-per-tick", type=int, default=1)
    parser.add_argument("--velocity-scale", type=float, default=0.8)
    parser.add_argument("--acceleration-scale", type=float, default=0.7)
    parser.add_argument("--disable-auto-delay", action="store_true", help="禁用自动起始延迟（用于对照组）")
    parser.add_argument("--force-serial", action="store_true", help="强制串行基线（先高优先级臂，再低优先级臂）")
    parser.add_argument("--disable-retreat-recovery", action="store_true", help="禁用回退恢复（用于对照组）")
    parser.add_argument("--disable-anti-oscillation", action="store_true", help="禁用防振荡恢复（用于对照组）")
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
    parser.add_argument("--enable-distance-lod", action="store_true", help="启用 Stage-1B 距离感知 LOD")
    parser.add_argument("--enable-priority-lod", action="store_true", help="启用 Stage-1C Priority-Aware LOD")
    parser.add_argument("--lod-distance-in", type=float, default=0.28, help="Distance-LOD 进入精细检测阈值 D_in（m）")
    parser.add_argument("--lod-distance-out", type=float, default=0.35, help="Distance-LOD 退出精细检测阈值 D_out（m）")
    parser.add_argument("--lod-distance-out-both-move", type=float, default=-1.0, help="可选：both_move 专用 D_out（m）；<=0 表示复用 --lod-distance-out")
    parser.add_argument("--lod-check-interval", type=int, default=1, help="Distance-LOD 距离重算间隔（每 N 次冲突检查重算一次，>=1）")
    parser.add_argument("--priority-lod-static-step-threshold", type=float, default=0.0, help="1C 优化：both_move 下低优先级 quasi-static 判定阈值(rad)，<=0 关闭")
    parser.add_argument("--enable-horizon-lod", action="store_true", help="启用 Stage-1D Horizon-Aware LOD")
    parser.add_argument("--horizon-near-steps", type=int, default=10, help="Stage-1D 近视野阈值")
    parser.add_argument("--horizon-far-steps", type=int, default=50, help="Stage-1D 远视野阈值")
    parser.add_argument("--horizon-near-check-interval", type=int, default=1, help="Stage-1D 近视野距离重算间隔")
    parser.add_argument("--horizon-far-check-interval", type=int, default=8, help="Stage-1D 远视野距离重算间隔")
    parser.add_argument("--enable-priority-lazy-validation", action="store_true", help="启用创新1最小版：低优先级臂RRT惰性验证使用高优先级胶囊包络")
    parser.add_argument("--tick-sleep", type=float, default=0.0)
    parser.add_argument("--report-ablation", action="store_true", help="是否输出 auto-delay 开关预览")
    parser.add_argument("--python-exe", default=sys.executable, help="用于执行子进程的 Python")
    parser.add_argument("--heartbeat-seconds", type=float, default=30.0, help="子进程无输出时的心跳打印间隔（秒）")
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
        "--sequence-mode", str(args.sequence_mode),
        "--ur5e-matrix", str(project_root / args.ur5e_matrix),
        "--fr3-matrix", str(project_root / args.fr3_matrix),
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
        "--lod-distance-in", str(args.lod_distance_in),
        "--lod-distance-out", str(args.lod_distance_out),
        "--lod-distance-out-both-move", str(args.lod_distance_out_both_move),
        "--lod-check-interval", str(args.lod_check_interval),
        "--priority-lod-static-step-threshold", str(args.priority_lod_static_step_threshold),
        "--horizon-near-steps", str(args.horizon_near_steps),
        "--horizon-far-steps", str(args.horizon_far_steps),
        "--horizon-near-check-interval", str(args.horizon_near_check_interval),
        "--horizon-far-check-interval", str(args.horizon_far_check_interval),
        "--tick-sleep", str(args.tick_sleep),
        "--random-seed", str(seed),
        "--stats-json", str(stats_path),
    ]
    if args.enable_distance_lod:
        cmd.append("--enable-distance-lod")
    if args.enable_priority_lod:
        cmd.append("--enable-priority-lod")
    if args.enable_horizon_lod:
        cmd.append("--enable-horizon-lod")
    if args.enable_priority_lazy_validation:
        cmd.append("--enable-priority-lazy-validation")
    if args.disable_auto_delay:
        cmd.append("--disable-auto-delay")
    if args.force_serial:
        cmd.append("--force-serial")
    if args.disable_retreat_recovery:
        cmd.append("--disable-retreat-recovery")
    if args.disable_anti_oscillation:
        cmd.append("--disable-anti-oscillation")
    if args.report_ablation:
        cmd.append("--report-ablation")

    t0 = time.perf_counter()
    heartbeat_interval = max(float(args.heartbeat_seconds), 1.0)
    next_heartbeat = t0 + heartbeat_interval

    with log_path.open("w", encoding="utf-8") as f:
        proc = subprocess.Popen(
            cmd,
            cwd=str(project_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        assert proc.stdout is not None
        while True:
            ready, _, _ = select.select([proc.stdout], [], [], 1.0)
            if ready:
                line = proc.stdout.readline()
                if line:
                    f.write(line)
                    f.flush()

            if proc.poll() is not None:
                # 读掉剩余缓冲
                tail = proc.stdout.read()
                if tail:
                    f.write(tail)
                    f.flush()
                break

            now = time.perf_counter()
            if now >= next_heartbeat:
                elapsed = now - t0
                print(
                    f"    [heartbeat] {difficulty} seed={seed} 运行中，"
                    f"elapsed={elapsed:.1f}s, log={os.path.getsize(log_path)} bytes",
                    flush=True,
                )
                next_heartbeat = now + heartbeat_interval

        returncode = int(proc.returncode)

    wall = float(time.perf_counter() - t0)

    return RunRecord(
        difficulty=difficulty,
        seed=seed,
        returncode=returncode,
        wall_time_s=wall,
        stats_json=str(stats_path.relative_to(project_root)),
        log_file=str(log_path.relative_to(project_root)),
    )


def _mean_std(values: List[float]) -> Dict[str, float]:
    finite_vals = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(float(v))]
    if not finite_vals:
        return {"mean": float("nan"), "sd": float("nan")}
    if len(finite_vals) == 1:
        return {"mean": float(finite_vals[0]), "sd": 0.0}
    return {"mean": float(statistics.mean(finite_vals)), "sd": float(statistics.stdev(finite_vals))}


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
        traj = payload.get("trajectory_info", {})
        ur_timing = traj.get("ur5e_timing", {})
        fr_timing = traj.get("fr3_timing", {})
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
            "rrt_nodes_mean_ur5e": ur_timing.get("rrt_nodes_mean", float("nan")),
            "rrt_nodes_mean_fr3": fr_timing.get("rrt_nodes_mean", float("nan")),
            "rrt_iterations_mean_ur5e": ur_timing.get("rrt_iterations_mean", float("nan")),
            "rrt_iterations_mean_fr3": fr_timing.get("rrt_iterations_mean", float("nan")),
            "rrt_nodes_max_ur5e": ur_timing.get("rrt_nodes_max", float("nan")),
            "rrt_nodes_max_fr3": fr_timing.get("rrt_nodes_max", float("nan")),
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
        rrt_nodes_ur = _mean_std(vals("rrt_nodes_mean_ur5e"))
        rrt_nodes_fr = _mean_std(vals("rrt_nodes_mean_fr3"))
        rrt_iter_ur = _mean_std(vals("rrt_iterations_mean_ur5e"))
        rrt_iter_fr = _mean_std(vals("rrt_iterations_mean_fr3"))

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
            "rrt_nodes_mean_ur5e": rrt_nodes_ur["mean"],
            "rrt_nodes_sd_ur5e": rrt_nodes_ur["sd"],
            "rrt_nodes_mean_fr3": rrt_nodes_fr["mean"],
            "rrt_nodes_sd_fr3": rrt_nodes_fr["sd"],
            "rrt_iter_mean_ur5e": rrt_iter_ur["mean"],
            "rrt_iter_sd_ur5e": rrt_iter_ur["sd"],
            "rrt_iter_mean_fr3": rrt_iter_fr["mean"],
            "rrt_iter_sd_fr3": rrt_iter_fr["sd"],
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
        "| 难度 | n | success_rate | makespan_mean±sd (s) | speedup_mean±sd | deadlock_wait_mean±sd | planning_wall_mean±sd (s) | preview_wall_mean±sd (s) | execution_wall_mean±sd (s) | total_wall_mean±sd (s) | rrt_nodes_mean_ur5e±sd | rrt_iter_mean_ur5e±sd |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
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
                f"{r['rrt_nodes_mean_ur5e']:.3f} ± {r['rrt_nodes_sd_ur5e']:.3f}",
                f"{r['rrt_iter_mean_ur5e']:.3f} ± {r['rrt_iter_sd_ur5e']:.3f}",
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
            "disable_auto_delay": args.disable_auto_delay,
            "force_serial": args.force_serial,
            "disable_retreat_recovery": args.disable_retreat_recovery,
            "disable_anti_oscillation": args.disable_anti_oscillation,
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
            "enable_distance_lod": args.enable_distance_lod,
            "enable_priority_lod": args.enable_priority_lod,
            "lod_distance_in": args.lod_distance_in,
            "lod_distance_out": args.lod_distance_out,
            "lod_check_interval": args.lod_check_interval,
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
            print(f"[Baseline] 运行 {difficulty} seed={seed} ...", flush=True)
            rec = run_one(project_root, args, difficulty, seed, solution, stats_path, log_path)
            records.append(rec)
            print(f"  -> returncode={rec.returncode}, wall={rec.wall_time_s:.2f}s", flush=True)
            if rec.returncode != 0 and args.stop_on_failure:
                print("检测到失败，按 --stop-on-failure 停止。", flush=True)
                summarize(project_root, batch_dir, records)
                sys.exit(rec.returncode)

    summarize(project_root, batch_dir, records)
    print(f"\n✅ Baseline 批量实验完成。输出目录: {batch_dir}", flush=True)


if __name__ == "__main__":
    main()
