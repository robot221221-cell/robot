#!/usr/bin/env python3
"""
成功率优先的 hard 场景最终参数搜索（当前重点：sync_balance_weight）。

做法：
- 对多个 lambda × 多个随机种子进行 headless 调度运行
- 汇总 success rate / makespan / speedup / deadlock 指标
- 给出 success-first 推荐参数
"""

from __future__ import annotations

import argparse
import csv
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


def _safe_float(v: object, default: float = 0.0) -> float:
    try:
        x = float(v)
        if math.isnan(x):
            return default
        return x
    except Exception:
        return default


def _extract_metrics(payload: Dict[str, object]) -> Dict[str, float]:
    stats = payload.get("execution_stats", {})
    comp_rows = payload.get("comparison_rows", [])
    deadlock_wait_count = 0
    if isinstance(comp_rows, list) and comp_rows:
        try:
            deadlock_wait_count = int(comp_rows[0].get("deadlock_wait_count", 0))
        except Exception:
            deadlock_wait_count = 0
    else:
        deadlock_wait_count = int(stats.get("mode_counter", {}).get("deadlock_wait", 0))

    success = 1 if bool(stats.get("success", False)) else 0
    makespan = _safe_float(stats.get("parallel_makespan_s", float("inf")), float("inf"))
    speedup = _safe_float(stats.get("speedup_vs_serial", 0.0), 0.0)

    if success == 0:
        makespan = float("inf")
        speedup = 0.0

    return {
        "success": float(success),
        "parallel_makespan_s": makespan,
        "speedup_vs_serial": speedup,
        "deadlock_wait_count": float(deadlock_wait_count),
        "wait_time_total_s": _safe_float(stats.get("wait_time_s", {}).get("total", 0.0), 0.0),
        "finish_gap_s": _safe_float(stats.get("finish_gap_s", 0.0), 0.0),
    }


def _aggregate(lambda_value: float, rows: List[Dict[str, float]]) -> Dict[str, object]:
    n = len(rows)
    success_list = [int(r["success"]) for r in rows]
    success_rate = sum(success_list) / n if n > 0 else 0.0

    success_rows = [r for r in rows if int(r["success"]) == 1]
    if success_rows:
        makespans = [r["parallel_makespan_s"] for r in success_rows]
        speedups = [r["speedup_vs_serial"] for r in success_rows]
        p95_idx = max(0, math.ceil(0.95 * len(makespans)) - 1)
        makespans_sorted = sorted(makespans)
        makespan_p95 = makespans_sorted[p95_idx]
        avg_makespan = statistics.mean(makespans)
        avg_speedup = statistics.mean(speedups)
    else:
        makespan_p95 = float("inf")
        avg_makespan = float("inf")
        avg_speedup = 0.0

    avg_deadlock = statistics.mean([r["deadlock_wait_count"] for r in rows]) if rows else 0.0

    return {
        "lambda": lambda_value,
        "trials": n,
        "success_rate": success_rate,
        "avg_parallel_makespan_s_on_success": avg_makespan,
        "p95_parallel_makespan_s_on_success": makespan_p95,
        "avg_speedup_on_success": avg_speedup,
        "avg_deadlock_wait_count": avg_deadlock,
    }


def _rank_key(row: Dict[str, object]) -> Tuple[float, float, float, float]:
    return (
        -float(row["success_rate"]),
        float(row["avg_parallel_makespan_s_on_success"]),
        -float(row["avg_speedup_on_success"]),
        float(row["avg_deadlock_wait_count"]),
    )


def _write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "lambda",
        "trials",
        "success_rate",
        "avg_parallel_makespan_s_on_success",
        "p95_parallel_makespan_s_on_success",
        "avg_speedup_on_success",
        "avg_deadlock_wait_count",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def _write_md(path: Path, rows: List[Dict[str, object]], seeds: List[int], solution: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    headers = [
        "lambda",
        "trials",
        "success_rate",
        "avg_parallel_makespan_s_on_success",
        "p95_parallel_makespan_s_on_success",
        "avg_speedup_on_success",
        "avg_deadlock_wait_count",
    ]
    lines = [
        "# hard 场景最终参数搜索（成功率优先）",
        "",
        f"- solution: {solution}",
        f"- seeds: {seeds}",
        "- 排序规则：success_rate 降序 -> avg_parallel_makespan 升序 -> avg_speedup 降序 -> avg_deadlock_wait_count 升序",
        "",
        "| " + " | ".join(headers) + " |",
        "|" + "|".join(["---"] * len(headers)) + "|",
    ]
    for r in rows:
        lines.append("| " + " | ".join(str(r[h]) for h in headers) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="hard 场景 success-first 参数搜索")
    p.add_argument("--solution", required=True)
    p.add_argument("--output-dir", default="xml_pipeline_run/difficulty_experiments_v6/final_param_search")
    p.add_argument("--lambdas", nargs="+", type=float, default=[0.0, 0.3, 0.5, 0.7])
    p.add_argument("--seeds", nargs="+", type=int, default=[42, 43, 44, 45, 46])

    # 与当前 hard 稳定配置一致
    p.add_argument("--priority-arm", choices=["ur5e", "fr3"], default="ur5e")
    p.add_argument("--hold-steps", type=int, default=80)
    p.add_argument("--sim-steps-per-tick", type=int, default=1)
    p.add_argument("--velocity-scale", type=float, default=0.8)
    p.add_argument("--acceleration-scale", type=float, default=0.7)
    p.add_argument("--max-start-delay", type=int, default=4000)
    p.add_argument("--delay-coarse-step", type=int, default=20)
    p.add_argument("--anti-oscillation-trigger", type=int, default=900)
    p.add_argument("--anti-oscillation-hold", type=int, default=80)
    p.add_argument("--anti-oscillation-max-attempts", type=int, default=3)
    p.add_argument("--max-total-ticks", type=int, default=120000)
    p.add_argument("--max-no-progress-ticks", type=int, default=5000)
    p.add_argument("--retreat-cooldown-ticks", type=int, default=16)
    p.add_argument("--max-cmd-step", type=float, default=0.02)
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
    common = [
        sys.executable,
        str(schedule_py),
        "--solution",
        args.solution,
        "--headless",
        "--disable-auto-organize",
        "--hold-steps",
        str(args.hold_steps),
        "--sim-steps-per-tick",
        str(args.sim_steps_per_tick),
        "--priority-arm",
        args.priority_arm,
        "--velocity-scale",
        str(args.velocity_scale),
        "--acceleration-scale",
        str(args.acceleration_scale),
        "--max-start-delay",
        str(args.max_start_delay),
        "--delay-coarse-step",
        str(args.delay_coarse_step),
        "--anti-oscillation-trigger",
        str(args.anti_oscillation_trigger),
        "--anti-oscillation-hold",
        str(args.anti_oscillation_hold),
        "--anti-oscillation-max-attempts",
        str(args.anti_oscillation_max_attempts),
        "--max-total-ticks",
        str(args.max_total_ticks),
        "--max-no-progress-ticks",
        str(args.max_no_progress_ticks),
        "--retreat-cooldown-ticks",
        str(args.retreat_cooldown_ticks),
        "--max-cmd-step",
        str(args.max_cmd_step),
        "--jump-step-threshold",
        str(args.jump_step_threshold),
        "--jump-velocity-threshold",
        str(args.jump_velocity_threshold),
    ]

    trial_rows: List[Dict[str, object]] = []
    agg_rows: List[Dict[str, object]] = []

    for lam in args.lambdas:
        lam_results: List[Dict[str, float]] = []
        for seed in args.seeds:
            tag = f"lambda={lam}, seed={seed}"
            stats_path = stats_dir / f"stats_lambda_{str(lam).replace('.', '_')}_seed_{seed}.json"
            cmd = common + [
                "--sync-balance-weight",
                str(lam),
                "--random-seed",
                str(seed),
                "--stats-json",
                str(stats_path),
            ]
            _run(cmd, tag)
            payload = _load(stats_path)
            m = _extract_metrics(payload)
            lam_results.append(m)
            trial_rows.append(
                {
                    "lambda": lam,
                    "seed": seed,
                    **m,
                }
            )

        agg_rows.append(_aggregate(lam, lam_results))

    agg_rows = sorted(agg_rows, key=_rank_key)
    best = agg_rows[0] if agg_rows else None

    _write_csv(table_dir / "final_param_search_summary.csv", agg_rows)
    _write_md(table_dir / "final_param_search_summary.md", agg_rows, args.seeds, args.solution)
    (table_dir / "final_param_search_trials.json").write_text(
        json.dumps(trial_rows, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("\n✅ 参数搜索完成")
    print(f"- summary csv: {table_dir / 'final_param_search_summary.csv'}")
    print(f"- summary md : {table_dir / 'final_param_search_summary.md'}")
    print(f"- trials json: {table_dir / 'final_param_search_trials.json'}")
    if best is not None:
        print("\n🏁 推荐参数（success-first）")
        print(f"- sync_balance_weight(lambda): {best['lambda']}")
        print(f"- success_rate: {best['success_rate']}")
        print(f"- avg_parallel_makespan_s_on_success: {best['avg_parallel_makespan_s_on_success']}")
        print(f"- avg_speedup_on_success: {best['avg_speedup_on_success']}")
        print(f"- avg_deadlock_wait_count: {best['avg_deadlock_wait_count']}")


if __name__ == "__main__":
    main()
