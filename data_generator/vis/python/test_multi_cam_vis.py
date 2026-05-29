#!/usr/bin/env python3
"""Smoke test: DataGenerator + landmark union coloring for num_cam in {1, 2, 4}."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "build_standalone" / "data_generator_vis"))

import vins_sim_data as s  # noqa: E402


def _write_config(tmp: Path, num_cam: int) -> Path:
    src = REPO / "config/simulation/simulation_config.yaml"
    text = src.read_text(encoding="utf-8")
    import re

    text = re.sub(r"num_of_cam:\s*\d+", f"num_of_cam: {num_cam}", text, count=1)
    ids = ", ".join(str(i) for i in range(num_cam))
    text = re.sub(r"camera_ids:\s*\[[^\]]*\]", f"camera_ids: [{ids}]", text, count=1)
    out = tmp / f"sim_{num_cam}cam.yaml"
    out.write_text(text, encoding="utf-8")
    return out


def _check_num_cam(num_cam: int) -> None:
    with tempfile.TemporaryDirectory() as td:
        cfg = _write_config(Path(td), num_cam)
        opts = s.load_options(str(cfg))
        gen = s.DataGenerator(opts, False)
        assert gen.num_cameras() == num_cam

        for _ in range(opts.imu_per_img):
            gen.update()
        gen.get_image()

        by_cam = gen.get_observed_landmark_ids()
        assert len(by_cam) == num_cam
        union_pts = gen.get_observed_points()
        union_ids: set[int] = set()
        for ids in by_cam:
            union_ids.update(int(i) for i in ids)
        assert len(union_pts) == len(union_ids) > 0

        for k in range(num_cam):
            assert len(by_cam[k]) > 0, f"cam slot {k} has no observed landmarks"

        print(f"OK num_cam={num_cam}: union={len(union_pts)}, per_slot={[len(x) for x in by_cam]}")


def main() -> int:
    if not (REPO / "build_standalone/data_generator_vis").exists():
        print("Build vins_sim_data first", file=sys.stderr)
        return 1
    for n in (1, 2, 4):
        _check_num_cam(n)
    print("All camera counts passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
