#!/usr/bin/env python3
"""
固定 lambda 后，搜索其余调度参数（success-first）。
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple


def _run(cmd: List[str], tag: str) -> float:
    print(f"\n[开始] {tag}", flush=True)
    print("[命令]", " ".join(cmd), flush=True)
    t0 = time.perf_counter()
    subprocess.run(cmd, check=True)
    dt = time.perf_counter() - t0
    print(f"[完成] {tag}, 耗时 {dt:.1f}s", flush=True)
    return dt


def _load(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _extract_metrics(payload: Dict[str, object]) -> Dict[str, float]:
    stats = payload.get("execution_stats", {})
    comp_rows = payload.get("comparison_rows", [])
    if isinstance(comp_rows, list) and comp_rows:
        deadlock_wait_count = int(comp_rows[0].get("deadlock_wait_count", 0))
    else:
        deadlock_wait_count = int(stats.get("mode_counter", {}).get("deadlock_wait", 0))

    success = 1 if bool(stats.get("success", False)) else 0
    makespan = float(stats.get("parallel_makespan_s", float("inf"))) if success else float("inf")
    speedup = float(stats.get("speedup_vs_serial", 0.0)) if success else 0.0

    return {
        "success": float(success),
        "parallel_makespan_s": makespan,
        "speedup_vs_serial": speedup,
        "deadlock_wait_count": float(deadlock_wait_count),
    }


def _aggregate(config: Dict[str, float], rows: List[Dict[str, float]]) -> Dict[str, object]:
    success_rate = statistics.mean([r["success"] for r in rows]) if rows else 0.0
    succ = [r for r in rows if int(r["success"]) == 1]
    avg_makespan = statistics.mean([r["parallel_makespan_s"] for r in succ]) if succ else float("inf")
    avg_speedup = statistics.mean([r["speedup_vs_serial"] for r in succ]) if succ else 0.0
    avg_deadlock = statistics.mean([r["deadlock_wait_count"] for r in rows]) if rows else 0.0

    out: Dict[str, object] = {
        **config,
        "trials": len(rows),
        "success_rate": success_rate,
        "avg_parallel_makespan_s_on_success": avg_makespan,
        "avg_speedup_on_success": avg_speedup,
        "avg_deadlock_wait_count": avg_deadlock,
    }
    return out


def _rank_key(row: Dict[str, object]) -> Tuple[float, float, float, float]:
    return (
        -float(row["success_rate"]),
        float(row["avg_parallel_makespan_s_on_success"]),
        -float(row["avg_speedup_on_success"]),
        float(row["avg_deadlock_wait_count"]),
    )


def _write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="固定 lambda 的调度参数搜索")
    p.add_argument("--solution", required=True)
    p.add_argument("--output-dir", default="xml_pipeline_run/difficulty_experiments_v6/fixed_lambda_param_search")
    p.add_argument("--lambda-sync", type=float, default=0.3)
    p.add_argument("--seeds", nargs="+", type=int, default=[45, 46])

    p.add_argument("--anti-trigger-list", nargs="+", type=int, default=[700, 900])
    p.add_argument("--anti-hold-list", nargs="+", type=int, default=[80])
    p.add_argument("--anti-attempts-list", nargs="+", type=int, default=[3, 4])
    p.add_argument("--max-no-progress-list", nargs="+", type=int, default=[4000, 5000])
    p.add_argument("--max-cmd-step-list", nargs="+", type=float, default=[0.02])

    p.add_argument("--priority-arm", choices=["ur5e", "fr3"], default="ur5e")
    p.add_argument("--hold-steps", type=int, default=80)
    p.add_argument("--sim-steps-per-tick", type=int, default=1)
    p.add_argument("--velocity-scale", type=float, default=0.8)
    p.add_argument("--acceleration-scale", type=float, default=0.7)
    p.add_argument("--max-start-delay", type=int, default=4000)
    p.add_argument("--delay-coarse-step", type=int, default=20)
    p.add_argument("--anti-oscillation-hold", type=int, default=80)
    p.add_argument("--max-total-ticks", type=int, default=120000)
    p.add_argument("--retreat-cooldown-ticks", type=int, default=16)
    p.add_argument("--jump-step-threshold", type=float, default=0.06)
    p.add_argument("--jump-velocity-threshold", type=float, default=25.0)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.output_dir)
    stats_dir = out_dir / "stats"
    table_dir = out_dir / "tables"
    stats_dir.mkdir(parents=True, exist_ok=True)
    table_dir.mkdir(parents=True, exist_ok=True)

    schedule_py = Path(__file__).with_name("schedule_ga_with_priority_delay.py")

    base = [
        sys.executable,
        str(schedule_py),
        "--solution", args.solution,
        "--headless",
        "--disable-auto-organize",
        "--priority-arm", args.priority_arm,
        "--hold-steps", str(args.hold_steps),
        "--sim-steps-per-tick", str(args.sim_steps_per_tick),
        "--velocity-scale", str(args.velocity_scale),
        "--acceleration-scale", str(args.acceleration_scale),
        "--max-start-delay", str(args.max_start_delay),
        "--delay-coarse-step", str(args.delay_coarse_step),
        "--max-total-ticks", str(args.max_total_ticks),
        "--retreat-cooldown-ticks", str(args.retreat_cooldown_ticks),
        "--jump-step-threshold", str(args.jump_step_threshold),
        "--jump-velocity-threshold", str(args.jump_velocity_threshold),
        "--sync-balance-weight", str(args.lambda_sync),
    ]

    grid = list(
        itertools.product(
            args.anti_trigger_list,
            args.anti_hold_list,
            args.anti_attempts_list,
            args.max_no_progress_list,
            args.max_cmd_step_list,
        )
    )

    trial_rows: List[Dict[str, object]] = []
    agg_rows: List[Dict[str, object]] = []

    for anti_trigger, anti_hold, anti_attempts, max_no_progress, max_cmd_step in grid:
        config = {
            "lambda_sync": args.lambda_sync,
            "anti_trigger": anti_trigger,
            "anti_hold": anti_hold,
            "anti_attempts": anti_attempts,
            "max_no_progress_ticks": max_no_progress,
            "max_cmd_step": max_cmd_step,
        }
        runs: List[Dict[str, float]] = []
        for seed in args.seeds:
            tag = (
                f"cfg(trigger={anti_trigger},hold={anti_hold},attempts={anti_attempts},"
                f"no_progress={max_no_progress},cmd_step={max_cmd_step}), seed={seed}"
            )
            name = (
                f"t{anti_trigger}_h{anti_hold}_a{anti_attempts}_n{max_no_progress}_"
                f"c{str(max_cmd_step).replace('.', '_')}_s{seed}.json"
            )
            stats_path = stats_dir / name
            cmd = base + [
                "--anti-oscillation-trigger", str(anti_trigger),
                "--anti-oscillation-hold", str(anti_hold),
                "--anti-oscillation-max-attempts", str(anti_attempts),
                "--max-no-progress-ticks", str(max_no_progress),
                "--max-cmd-step", str(max_cmd_step),
                "--random-seed", str(seed),
                "--stats-json", str(stats_path),
            ]
            _run(cmd, tag)
            row = _extract_metrics(_load(stats_path))
            runs.append(row)
            trial_rows.append({**config, "seed": seed, **row})

        agg_rows.append(_aggregate(config, runs))

    agg_rows = sorted(agg_rows, key=_rank_key)
    best = agg_rows[0] if agg_rows else None

    _write_csv(table_dir / "fixed_lambda_param_search_summary.csv", agg_rows)
    (table_dir / "fixed_lambda_param_search_trials.json").write_text(
        json.dumps(trial_rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print("\n✅ 固定lambda参数搜索完成")
    print(f"- summary: {table_dir / 'fixed_lambda_param_search_summary.csv'}")
    print(f"- trials : {table_dir / 'fixed_lambda_param_search_trials.json'}")
    if best:
        print("\n🏁 推荐配置（success-first）")
        print(best)


if __name__ == "__main__":
    main()
