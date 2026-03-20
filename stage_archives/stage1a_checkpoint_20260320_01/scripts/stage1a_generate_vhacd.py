#!/usr/bin/env python3
"""
Stage-1A: 为末端复杂网格生成 V-HACD 资产（保持局部坐标系）。

默认仅处理：
- UR5e: wrist3.obj
- FR3: link7.obj
- 若存在：hand.obj / finger.obj

输出：同目录 *_vhacd.obj
汇总：artifacts/stage1a/vhacd_report.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List

import pybullet as p


@dataclass
class VhacdResult:
    source: str
    output: str
    log: str
    exists_source: bool
    success: bool
    resolution: int
    concavity: float
    max_num_vertices_per_ch: int
    min_volume_per_ch: float


def run_vhacd(
    src: Path,
    resolution: int,
    concavity: float,
    max_num_vertices_per_ch: int,
    min_volume_per_ch: float,
) -> VhacdResult:
    out_path = src.with_name(f"{src.stem}_vhacd.obj")
    log_path = src.with_name(f"{src.stem}_vhacd.log")

    if not src.exists():
        return VhacdResult(
            source=str(src),
            output=str(out_path),
            log=str(log_path),
            exists_source=False,
            success=False,
            resolution=resolution,
            concavity=concavity,
            max_num_vertices_per_ch=max_num_vertices_per_ch,
            min_volume_per_ch=min_volume_per_ch,
        )

    p.vhacd(
        str(src),
        str(out_path),
        str(log_path),
        resolution=resolution,
        concavity=concavity,
        maxNumVerticesPerCH=max_num_vertices_per_ch,
        minVolumePerCH=min_volume_per_ch,
    )

    ok = out_path.exists() and out_path.stat().st_size > 0

    return VhacdResult(
        source=str(src),
        output=str(out_path),
        log=str(log_path),
        exists_source=True,
        success=bool(ok),
        resolution=resolution,
        concavity=concavity,
        max_num_vertices_per_ch=max_num_vertices_per_ch,
        min_volume_per_ch=min_volume_per_ch,
    )


def collect_targets(workspace: Path) -> List[Path]:
    targets = [
        workspace / "universal_robots_ur5e" / "assets" / "wrist3.obj",
        workspace / "franka_fr3" / "assets" / "link7.obj",
        workspace / "franka_fr3" / "assets" / "hand.obj",
        workspace / "franka_fr3" / "assets" / "finger.obj",
    ]
    return targets


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--resolution", type=int, default=1_000_000)
    parser.add_argument("--concavity", type=float, default=0.0025)
    parser.add_argument("--max-num-vertices-per-ch", type=int, default=64)
    parser.add_argument("--min-volume-per-ch", type=float, default=1e-5)
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    out_dir = workspace / "artifacts" / "stage1a"
    out_dir.mkdir(parents=True, exist_ok=True)

    p.connect(p.DIRECT)
    try:
        results = [
            run_vhacd(
                src=src,
                resolution=args.resolution,
                concavity=args.concavity,
                max_num_vertices_per_ch=args.max_num_vertices_per_ch,
                min_volume_per_ch=args.min_volume_per_ch,
            )
            for src in collect_targets(workspace)
        ]
    finally:
        p.disconnect()

    report = {
        "stage": "Stage-1A",
        "method": "pybullet.vhacd",
        "workspace": str(workspace),
        "results": [asdict(r) for r in results],
    }

    report_path = out_dir / "vhacd_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    ok_count = sum(1 for r in results if r.success)
    miss_count = sum(1 for r in results if not r.exists_source)
    fail_count = len(results) - ok_count - miss_count

    print(f"[Stage-1A] VHACD done. success={ok_count}, missing={miss_count}, failed={fail_count}")
    print(f"report: {report_path}")


if __name__ == "__main__":
    main()
