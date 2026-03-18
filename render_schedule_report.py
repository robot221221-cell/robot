"""
将 schedule_ga_with_priority_delay.py 导出的 stats JSON 渲染为更直观的实验报告。

输入：
- schedule_ablation_stats.json

输出：
- Markdown 报告（默认：xml_pipeline_run/schedule_report.md）
- 可选 PNG 图（默认：xml_pipeline_run/schedule_report.png）
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List


def _safe_num(v):
    try:
        x = float(v)
        return x
    except Exception:
        return float("nan")


def _fmt(v, digits: int = 3):
    x = _safe_num(v)
    if math.isnan(x):
        return "-"
    if math.isinf(x):
        return "inf"
    return f"{x:.{digits}f}"


def _method_alias(name: str) -> str:
    mapping = {
        "auto_delay_off_preview": "AutoDelay关闭(预览)",
        "auto_delay_on_preview": "AutoDelay开启(预览)",
        "execution_actual": "实际执行",
    }
    return mapping.get(name, name)


def _method_alias_plot(name: str) -> str:
    mapping = {
        "auto_delay_off_preview": "AutoDelay关闭",
        "auto_delay_on_preview": "AutoDelay开启",
        "execution_actual": "实际执行",
    }
    return mapping.get(name, name)


def _configure_chinese_font(plt_module):
    """尽量选择可用中文字体，避免图表中文显示为方框。"""
    try:
        import matplotlib.font_manager as fm
        available = {f.name for f in fm.fontManager.ttflist}
    except Exception:
        available = set()

    candidates = [
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "Microsoft YaHei",
        "SimHei",
        "PingFang SC",
        "Heiti SC",
        "Arial Unicode MS",
    ]
    chosen = None
    for c in candidates:
        if c in available:
            chosen = c
            break

    if chosen is not None:
        plt_module.rcParams["font.sans-serif"] = [chosen, "DejaVu Sans"]
    else:
        # 保底：即便没有中文字体，也尽量不影响绘图流程
        plt_module.rcParams["font.sans-serif"] = ["DejaVu Sans"]
        print("⚠️ 未找到可用中文字体，图中中文可能显示不完整。")

    plt_module.rcParams["axes.unicode_minus"] = False


def build_markdown(data: Dict) -> str:
    cfg = data.get("config", {})
    rows: List[Dict] = data.get("comparison_rows", [])

    lines: List[str] = []
    lines.append("# 调度实验报告（可读版）")
    lines.append("")
    lines.append("## 实验配置")
    lines.append("")
    lines.append(f"- 优先级机械臂: {cfg.get('priority_arm', '-')}")
    lines.append(f"- 调度步长参数 sim_steps_per_tick: {cfg.get('sim_steps_per_tick', '-')}")
    lines.append(f"- 每任务保持步数 hold_steps: {cfg.get('hold_steps', '-')}")
    lines.append(f"- 采样周期 sample_dt: {_fmt(cfg.get('sample_dt', float('nan')), 4)} s")
    lines.append("")

    lines.append("## 核心指标对比")
    lines.append("")
    lines.append("| 方法 | 成功 | 并行makespan(s) | 串行makespan(s) | 加速比 | 总等待时间(s) | 死锁等待次数 |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: |")

    for row in rows:
        lines.append(
            "| "
            + " | ".join([
                _method_alias(str(row.get("method", "-"))),
                "✅" if int(row.get("success", 0)) == 1 else "❌",
                _fmt(row.get("parallel_makespan_s")),
                _fmt(row.get("serial_makespan_s")),
                _fmt(row.get("speedup_vs_serial")),
                _fmt(row.get("wait_time_total_s")),
                str(int(row.get("deadlock_wait_count", 0))),
            ])
            + " |"
        )

    lines.append("")
    lines.append("## 结论摘要")
    lines.append("")

    row_off = next((r for r in rows if r.get("method") == "auto_delay_off_preview"), None)
    row_on = next((r for r in rows if r.get("method") == "auto_delay_on_preview"), None)
    row_exec = next((r for r in rows if r.get("method") == "execution_actual"), None)

    if row_off and row_on:
        off_ok = int(row_off.get("success", 0)) == 1
        on_ok = int(row_on.get("success", 0)) == 1
        lines.append(f"- AutoDelay关闭预览: {'成功' if off_ok else '失败'}")
        lines.append(f"- AutoDelay开启预览: {'成功' if on_ok else '失败'}")

        if (not off_ok) and on_ok:
            lines.append("- ✅ 关键改进：AutoDelay 将调度从不可达（死锁）提升为可达。")

        on_speedup = _safe_num(row_on.get("speedup_vs_serial", float("nan")))
        if math.isfinite(on_speedup):
            lines.append(f"- ✅ 开启后相对串行加速比: {_fmt(on_speedup)}x")

        on_wait = _safe_num(row_on.get("wait_time_total_s", float("nan")))
        if math.isfinite(on_wait):
            lines.append(f"- 开启后总等待时间: {_fmt(on_wait)} s")

    if row_on and row_exec:
        on_m = _safe_num(row_on.get("parallel_makespan_s", float("nan")))
        ex_m = _safe_num(row_exec.get("parallel_makespan_s", float("nan")))
        if math.isfinite(on_m) and math.isfinite(ex_m):
            gap = abs(on_m - ex_m)
            lines.append(f"- 预览与实际执行并行时长差: {_fmt(gap, 4)} s（可用于验证预测一致性）")

    lines.append("")
    lines.append("## 评价指标说明（建议论文固定使用）")
    lines.append("")
    lines.append("1. `success`：是否完成调度（可达性）")
    lines.append("2. `parallel_makespan_s`：并行总工时（越小越好）")
    lines.append("3. `speedup_vs_serial`：相对串行加速比（越大越好）")
    lines.append("4. `wait_time_total_s`：等待代价（越小越好）")
    lines.append("5. `deadlock_wait_count`：死锁风险强度（越小越好）")
    lines.append("")

    return "\n".join(lines) + "\n"


def build_plot(data: Dict, output_png: Path):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("⚠️ 未检测到 matplotlib，跳过图像导出")
        return False

    _configure_chinese_font(plt)

    rows = data.get("comparison_rows", [])
    labels = [_method_alias_plot(str(r.get("method", "-"))) for r in rows]
    speedup = [_safe_num(r.get("speedup_vs_serial", float("nan"))) for r in rows]
    wait_t = [_safe_num(r.get("wait_time_total_s", float("nan"))) for r in rows]
    deadlock = [int(r.get("deadlock_wait_count", 0)) for r in rows]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))

    axes[0].bar(labels, [0 if not math.isfinite(v) else v for v in speedup], color=["#d62728", "#2ca02c", "#1f77b4"])
    axes[0].set_title("相对串行加速比")
    axes[0].tick_params(axis="x", rotation=20)

    axes[1].bar(labels, [0 if not math.isfinite(v) else v for v in wait_t], color=["#d62728", "#2ca02c", "#1f77b4"])
    axes[1].set_title("总等待时间 (s)")
    axes[1].tick_params(axis="x", rotation=20)

    axes[2].bar(labels, deadlock, color=["#d62728", "#2ca02c", "#1f77b4"])
    axes[2].set_title("死锁等待次数")
    axes[2].tick_params(axis="x", rotation=20)

    plt.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_png, dpi=200)
    plt.close(fig)
    return True


def parse_args():
    parser = argparse.ArgumentParser(description="将调度 JSON 渲染为更直观的 Markdown/图表报告")
    parser.add_argument("--input", default="xml_pipeline_run/schedule_ablation_stats.json", help="输入 JSON 路径")
    parser.add_argument("--output-md", default="xml_pipeline_run/schedule_report.md", help="输出 Markdown 路径")
    parser.add_argument("--output-png", default="xml_pipeline_run/schedule_report.png", help="输出图表 PNG 路径")
    return parser.parse_args()


def main():
    args = parse_args()
    input_path = Path(args.input)
    output_md = Path(args.output_md)
    output_png = Path(args.output_png)

    data = json.loads(input_path.read_text(encoding="utf-8"))
    md = build_markdown(data)
    output_md.parent.mkdir(parents=True, exist_ok=True)
    output_md.write_text(md, encoding="utf-8")
    print(f"✅ 已生成可读报告: {output_md}")

    if build_plot(data, output_png):
        print(f"✅ 已生成可视化图表: {output_png}")


if __name__ == "__main__":
    main()
