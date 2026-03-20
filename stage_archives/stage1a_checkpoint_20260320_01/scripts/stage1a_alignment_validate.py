#!/usr/bin/env python3
"""
Stage-1A 对齐验证（数值记录）：
比较原始 mesh 与 V-HACD mesh 的 `mesh_pos` / `mesh_quat` 偏差。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np


@dataclass
class MeshAlignment:
    pair: str
    orig_mesh: str
    vhacd_mesh: str
    pos_diff_norm: float
    quat_angle_rad: float
    correction_geom_pos: list[float]
    correction_geom_quat: list[float]
    corrected_pos_diff_norm: float
    corrected_quat_angle_rad: float
    pass_threshold: bool


def quat_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = float(np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0))
    return 2.0 * float(np.arccos(dot))


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def check_pair(model: mujoco.MjModel, pair_name: str, orig_name: str, vhacd_name: str) -> MeshAlignment:
    oid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, orig_name)
    vid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, vhacd_name)

    opos = model.mesh_pos[oid].copy()
    vpos = model.mesh_pos[vid].copy()
    oq = model.mesh_quat[oid].copy()
    vq = model.mesh_quat[vid].copy()

    pos_diff = float(np.linalg.norm(opos - vpos))
    q_ang = quat_angle(oq, vq)

    # 计算使 V-HACD 与原始 mesh 对齐所需的 geom 补偿变换：G = M_orig * inv(M_vhacd)
    qg = quat_mul(oq, quat_conj(vq))
    qg = qg / np.linalg.norm(qg)
    pg = opos - quat_rotate(qg, vpos)

    # 验证补偿后残差（应接近 0）
    corrected_vpos = quat_rotate(qg, vpos) + pg
    corrected_vquat = quat_mul(qg, vq)
    corrected_vquat = corrected_vquat / np.linalg.norm(corrected_vquat)
    corrected_pos_diff = float(np.linalg.norm(opos - corrected_vpos))
    corrected_q_ang = quat_angle(oq, corrected_vquat)

    # Stage-1A 对齐阈值（近似零偏移）
    pos_th = 1e-5
    quat_th = 1e-5

    return MeshAlignment(
        pair=pair_name,
        orig_mesh=orig_name,
        vhacd_mesh=vhacd_name,
        pos_diff_norm=pos_diff,
        quat_angle_rad=q_ang,
        correction_geom_pos=[float(x) for x in pg],
        correction_geom_quat=[float(x) for x in qg],
        corrected_pos_diff_norm=corrected_pos_diff,
        corrected_quat_angle_rad=corrected_q_ang,
        pass_threshold=(corrected_pos_diff <= pos_th and corrected_q_ang <= quat_th),
    )


def main() -> None:
    root = Path(__file__).resolve().parent
    xml_path = root / "stage1a_alignment_view.xml"
    out_dir = root / "artifacts" / "stage1a"
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(xml_path))

    checks = [
        check_pair(model, "UR5e wrist3", "ur5e_wrist3_orig", "ur5e_wrist3_vhacd"),
        check_pair(model, "FR3 link7", "fr3_link7_orig", "fr3_link7_vhacd"),
    ]

    report = {
        "stage": "Stage-1A",
        "type": "alignment_numeric_validation",
        "xml": str(xml_path),
        "checks": [asdict(c) for c in checks],
        "all_pass": all(c.pass_threshold for c in checks),
        "threshold": {
            "pos_diff_norm_max": 1e-5,
            "quat_angle_rad_max": 1e-5,
        },
    }

    out_json = out_dir / "alignment_metrics.json"
    out_json.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[Stage-1A] alignment metrics written: {out_json}")
    print(f"all_pass={report['all_pass']}")


if __name__ == "__main__":
    main()
