#!/usr/bin/env python3
"""
同步化(软目标) vs 非同步化(效率优先) 对比实验。

功能：
- 自动运行两组 headless 调度
- 汇总关键指标
- 输出 CSV/Markdown
- 生成对比图（PNG）
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt


def _run(cmd: List[str], tag: str) -> float:
    print(f"\n[开始] {tag}", flush=True)
    print("[命令]", " ".join(cmd), flush=True)
    t0 = time.perf_counter()
    subprocess.run(cmd, check=True)
    dt = time.perf_counter() - t0
    print(f"[完成] {tag}，耗时 {dt:.1f}s", flush=True)
    return dt


def _load_stats(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _extract_row(label: str, payload: Dict[str, object]) -> Dict[str, object]:
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
    return {
        "method": label,
        "success": int(bool(stats.get("success", False))),
        "parallel_makespan_s": float(stats.get("parallel_makespan_s", 0.0)),
        "speedup_vs_serial": float(stats.get("speedup_vs_serial", 0.0)),
        "deadlock_wait_count": deadlock_wait_count,
        "wait_time_total_s": float(stats.get("wait_time_s", {}).get("total", 0.0)),
        "finish_gap_s": float(stats.get("finish_gap_s", 0.0)),
        "finish_gap_ticks": int(stats.get("finish_gap_ticks", 0)),
        "overhead_vs_ideal_parallel_s": float(stats.get("overhead_vs_ideal_parallel_s", 0.0)),
    }


def _write_csv(rows: List[Dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = [
        "method",
        "success",
        "parallel_makespan_s",
        "speedup_vs_serial",
        "deadlock_wait_count",
        "wait_time_total_s",
        "finish_gap_s",
        "finish_gap_ticks",
        "overhead_vs_ideal_parallel_s",
    ]
    lines = [",".join(keys)]
    for r in rows:
        lines.append(",".join(str(r[k]) for k in keys))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_md(rows: List[Dict[str, object]], path: Path, lambda_sync: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    headers = [
        "method",
        "success",
        "parallel_makespan_s",
        "speedup_vs_serial",
        "deadlock_wait_count",
        "wait_time_total_s",
        "finish_gap_s",
        "overhead_vs_ideal_parallel_s",
    ]
    lines = [
        "# 同步化 vs 非同步化 对比结果",
        "",
        f"- 同步化软目标权重 λ = {lambda_sync}",
        "- 说明：同步化方法在自动延迟搜索时最小化 `makespan_ticks + λ * finish_gap_ticks`",
        "- 判定规则（成功率优先）：先看 `success`，再看 `parallel_makespan_s`，再看 `speedup_vs_serial`，再看 `deadlock_wait_count`；`finish_gap_s`仅作参考",
        "",
        "| " + " | ".join(headers) + " |",
        "|" + "|".join(["---"] * len(headers)) + "|",
    ]
    for r in rows:
        lines.append("| " + " | ".join(str(r[h]) for h in headers) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot(rows: List[Dict[str, object]], path: Path) -> None:
    labels = [str(r["method"]) for r in rows]
    makespan = [float(r["parallel_makespan_s"]) for r in rows]
    finish_gap = [float(r["finish_gap_s"]) for r in rows]
    speedup = [float(r["speedup_vs_serial"]) for r in rows]
    wait_total = [float(r["wait_time_total_s"]) for r in rows]
    deadlock_wait = [float(r["deadlock_wait_count"]) for r in rows]

    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    fig.suptitle("Sync-soft vs Non-sync (hard)")

    axes[0, 0].bar(labels, makespan)
    axes[0, 0].set_title("parallel makespan (s)")

    axes[0, 1].bar(labels, finish_gap)
    axes[0, 1].set_title("finish gap (s)")

    axes[1, 0].bar(labels, speedup)
    axes[1, 0].set_title("speedup vs serial")

    axes[1, 1].bar(labels, wait_total)
    axes[1, 1].set_title("total wait (s)")

    axes[1, 2].bar(labels, deadlock_wait)
    axes[1, 2].set_title("deadlock wait count")

    axes[0, 2].axis("off")

    for ax in axes.flat:
        ax.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="运行同步化/非同步化对比实验并输出图表")
    p.add_argument("--solution", required=True, help="hard 场景 solution JSON")
    p.add_argument("--output-dir", default="xml_pipeline_run/difficulty_experiments_v6/sync_comparison", help="输出目录")
    p.add_argument("--lambda-sync", type=float, default=0.5, help="同步化软目标权重 λ")
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
    p.add_argument("--random-seed", type=int, default=42, help="起始随机种子（两组实验共用同一seed）")
    p.add_argument("--max-retries", type=int, default=2, help="若某组失败则更换seed重试次数")
    p.add_argument("--allow-partial", action="store_true", help="允许在未同时成功时也输出对比文件（默认失败退出）")
    p.add_argument("--disable-auto-organize", action="store_true", help="禁用实验结束后的自动结果整理")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.output_dir)
    stats_dir = out_dir / "stats"
    tables_dir = out_dir / "tables"
    figures_dir = out_dir / "figures"
    stats_dir.mkdir(parents=True, exist_ok=True)
    tables_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)

    schedule_py = Path(__file__).with_name("schedule_ga_with_priority_delay.py")
    nonsync_json = stats_dir / "stats_hard_nonsync.json"
    sync_json = stats_dir / "stats_hard_sync_soft.json"

    common = [
        sys.executable,
        str(schedule_py),
        "--solution", args.solution,
        "--headless",
        "--hold-steps", str(args.hold_steps),
        "--sim-steps-per-tick", str(args.sim_steps_per_tick),
        "--priority-arm", args.priority_arm,
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
    ]

    nonsync_payload: Dict[str, object] = {}
    sync_payload: Dict[str, object] = {}
    used_seed = args.random_seed
    got_both_success = False
    for offset in range(max(1, args.max_retries)):
        used_seed = args.random_seed + offset
        print(f"\n================ 试验 seed={used_seed} ================", flush=True)
        _run(
            common + ["--random-seed", str(used_seed), "--sync-balance-weight", "0", "--stats-json", str(nonsync_json)],
            tag="非同步化(效率优先)",
        )
        _run(
            common + ["--random-seed", str(used_seed), "--sync-balance-weight", str(args.lambda_sync), "--stats-json", str(sync_json)],
            tag=f"同步化软目标(lambda={args.lambda_sync})",
        )
        nonsync_payload = _load_stats(nonsync_json)
        sync_payload = _load_stats(sync_json)
        nonsync_ok = bool(nonsync_payload.get("execution_stats", {}).get("success", False))
        sync_ok = bool(sync_payload.get("execution_stats", {}).get("success", False))
        print(f"[结果] seed={used_seed} -> nonsync_success={nonsync_ok}, sync_success={sync_ok}", flush=True)
        if nonsync_ok and sync_ok:
            got_both_success = True
            break
        print(f"⚠️ seed={used_seed} 对比未同时成功，尝试下一个seed...")

    rows = [
        _extract_row("nonsync_efficiency", nonsync_payload),
        _extract_row(f"sync_soft_lambda_{args.lambda_sync}", sync_payload),
    ]

    def _rank_key(row: Dict[str, object]):
        success = int(row["success"])
        makespan = float(row["parallel_makespan_s"])
        speedup = float(row["speedup_vs_serial"])
        deadlock = int(row["deadlock_wait_count"])
        if success == 0:
            makespan = float("inf")
            speedup = -1.0
        return (-success, makespan, -speedup, deadlock)

    best_row = sorted(rows, key=_rank_key)[0]

    _write_csv(rows, tables_dir / "sync_vs_nonsync.csv")
    _write_md(rows, tables_dir / "sync_vs_nonsync.md", args.lambda_sync)
    _plot(rows, figures_dir / "sync_vs_nonsync.png")

    if (not got_both_success) and (not args.allow_partial):
        print("\n❌ 在最大重试次数内未找到‘两组都成功’的seed。")
        print("   你可以：1) 提高 --max-retries；2) 增大 --max-total-ticks；3) 临时加 --allow-partial 导出当前结果。")
        raise SystemExit(2)

    if not args.disable_auto_organize:
        organizer = Path(__file__).with_name("organize_outputs.py")
        if organizer.exists():
            print("\n🧹 自动整理输出文件...")
            subprocess.run([sys.executable, str(organizer), "--root", str(Path(__file__).resolve().parent), "--apply"], check=False)

    print("\n✅ 对比实验完成")
    print(f"- 使用随机种子: {used_seed}")
    print(f"- 统计JSON: {stats_dir}")
    print(f"- 对比CSV: {tables_dir / 'sync_vs_nonsync.csv'}")
    print(f"- 对比报告: {tables_dir / 'sync_vs_nonsync.md'}")
    print(f"- 对比图表: {figures_dir / 'sync_vs_nonsync.png'}")
    print(f"- 推荐配置(成功率优先): {best_row['method']}")


if __name__ == "__main__":
    main()
