# Stage-1A Checkpoint Archive Manifest

- Archive: `stage1a_checkpoint_20260320_01`
- Date: 2026-03-20
- Scope: Stage-1A first action (`V-HACD` asset generation + alignment verification)

## Commit Base
- See `base_commit.txt`

## Commands
1. 生成 V-HACD：
   - `conda run -n lerobot python stage1a_generate_vhacd.py`
2. 对齐验证：
   - `conda run -n lerobot python stage1a_alignment_validate.py`
3. 场景加载验证：
   - `conda run -n lerobot python -c "import mujoco; m=mujoco.MjModel.from_xml_path('dual_arm_scene.xml'); print('ok', m.ngeom, m.nmesh)"`

## Parameters
- V-HACD:
  - resolution=`1000000`
  - concavity=`0.0025`
  - maxNumVerticesPerCH=`64`
  - minVolumePerCH=`1e-5`

## Results
- `results/vhacd_report.json`
- `results/alignment_metrics.json`

## Notes
- FR3 `hand.obj` / `finger.obj` are not present in current repository; skipped and logged.
- Due to mesh-frame mismatch after decomposition, compensation transform is applied in `dual_arm_scene.xml` for `wrist3_vhacd` and `link7_vhacd`.
