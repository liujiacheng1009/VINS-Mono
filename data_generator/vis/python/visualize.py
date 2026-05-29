#!/usr/bin/env python3
"""Visualize DataGenerator dump: 3D trajectory, landmarks, observed features, IMU signals."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers '3d' projection


def load_dump(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def camera_pose_world(position: np.ndarray, quat: np.ndarray, ric: np.ndarray, tic: np.ndarray):
    r_wi = quat_wxyz_to_rot(quat)
    r_wc = r_wi @ ric
    t_wc = r_wi @ tic + position
    return r_wc, t_wc


def _camera_colors(num_cam: int) -> np.ndarray:
    return plt.cm.tab10(np.linspace(0, 1, max(num_cam, 1)))


def _parse_extrinsics(extrinsics: list) -> list[dict]:
    return [
        {
            "slot": int(ex.get("slot", i)),
            "camera_id": int(ex.get("camera_id", ex.get("slot", i))),
            "ric": np.asarray(ex["ric"], dtype=float),
            "tic": np.asarray(ex["tic"], dtype=float),
        }
        for i, ex in enumerate(extrinsics)
    ]


def _plot_frustum(ax, r_wc: np.ndarray, t_wc: np.ndarray, color, scale: float = 1.2) -> None:
    s = scale
    corners_cam = np.array(
        [
            [0.0, 0.0, 0.0],
            [-0.4 * s, -0.3 * s, 1.0 * s],
            [0.4 * s, -0.3 * s, 1.0 * s],
            [0.4 * s, 0.3 * s, 1.0 * s],
            [-0.4 * s, 0.3 * s, 1.0 * s],
        ],
        dtype=float,
    )
    corners_w = (r_wc @ corners_cam.T).T + t_wc.reshape(1, 3)
    edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
    for i, j in edges:
        seg = corners_w[[i, j]]
        ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], color=color, alpha=0.9, linewidth=1.2)


def _obs_uv(ray_cam: np.ndarray) -> np.ndarray | None:
    z = float(ray_cam[2])
    if z <= 1e-9:
        return None
    return np.array([ray_cam[0] / z, ray_cam[1] / z], dtype=float)


def _plot_fov_panel(ax, observations: list, color, camera_id: int, title: str) -> None:
    uv = []
    for ob in observations:
        if int(ob["camera_id"]) != camera_id:
            continue
        pt = _obs_uv(np.asarray(ob["ray_cam"], dtype=float))
        if pt is not None:
            uv.append(pt)
    if uv:
        uv_arr = np.stack(uv, axis=0)
        ax.scatter(uv_arr[:, 0], uv_arr[:, 1], c=[color], s=14, alpha=0.9)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("x / z", fontsize=7)
    ax.set_ylabel("y / z", fontsize=7)
    ax.grid(True, alpha=0.25)
    ax.tick_params(labelsize=6)
    ax.set_aspect("equal", adjustable="box")


def _plot_3d_on_ax(
    ax,
    data: dict,
    frame_stride: int,
    ray_length: float,
    max_rays_per_frame: int,
    colors: np.ndarray,
    cams: list[dict],
) -> None:
    landmarks = np.asarray(data["landmarks"], dtype=float)
    frames = data["image_frames"][::frame_stride]
    num_cam = int(data.get("num_cam", len(cams)))

    ax.scatter(
        landmarks[:, 0],
        landmarks[:, 1],
        landmarks[:, 2],
        c="lightgray",
        s=4,
        alpha=0.35,
        label="landmarks",
    )

    traj = np.array([f["position"] for f in data["image_frames"]], dtype=float)
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], "b-", linewidth=1.5, label="IMU path")

    if frames:
        last = frames[-1]
        pos = np.asarray(last["position"], dtype=float)
        quat = np.asarray(last["quaternion_wxyz"], dtype=float)
        observations = last.get("observations", [])

        for k, cam in enumerate(cams):
            cid = cam["camera_id"]
            color = colors[k % len(colors)]
            r_wc, t_wc = camera_pose_world(pos, quat, cam["ric"], cam["tic"])
            _plot_frustum(ax, r_wc, t_wc, color)
            ax.plot([t_wc[0]], [t_wc[1]], [t_wc[2]], "o", color=color, markersize=7, label=f"cam {cid}")

        lm_ids = last.get("observed_landmark_ids")
        if lm_ids is not None:
            valid_ids = [int(i) for i in lm_ids if 0 <= int(i) < len(landmarks)]
        else:
            valid_ids = []
        if valid_ids:
            pts = landmarks[valid_ids]
            ax.scatter(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                c="limegreen",
                s=16,
                alpha=0.9,
                label="observed feat",
            )

    title = "GT: trajectory, landmarks"
    if num_cam >= 2:
        title += f" ({num_cam} cams)"
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper right", fontsize=7)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect([1, 1, 1])


def _fov_grid_shape(n_cams: int) -> tuple[int, int]:
    if n_cams >= 4:
        return 2, 2
    if n_cams == 3:
        return 3, 1
    return max(n_cams, 1), 1


def _plot_fov_panels(fig, gs_fov, data: dict, frame_stride: int, colors: np.ndarray, cams: list[dict]) -> None:
    frames = data["image_frames"][::frame_stride]
    if not frames:
        return
    observations = frames[-1].get("observations", [])
    rows, cols = _fov_grid_shape(len(cams))
    fov_spec = gs_fov.subgridspec(rows, cols, hspace=0.45, wspace=0.35)
    for k, cam in enumerate(cams):
        if cols == 1:
            ax = fig.add_subplot(fov_spec[k, 0])
        else:
            ax = fig.add_subplot(fov_spec[k // cols, k % cols])
        _plot_fov_panel(ax, observations, colors[k % len(colors)], cam["camera_id"], f"cam {cam['camera_id']} FOV")


def _plot_imu_on_axes(axes, data: dict) -> None:
    imu = data["imu"]
    t = np.asarray(imu["t"], dtype=float)
    acc = np.asarray(imu["acc"], dtype=float)
    gyr = np.asarray(imu["gyr"], dtype=float)
    pos = np.asarray(imu["position"], dtype=float)
    vel_body = np.asarray(imu["velocity_body"], dtype=float)

    labels = ["x", "y", "z"]
    for i in range(3):
        axes[0].plot(t, acc[:, i], label=labels[i])
        axes[1].plot(t, gyr[:, i], label=labels[i])
        axes[2].plot(t, pos[:, i], label=f"p_{labels[i]}")
        axes[2].plot(t, vel_body[:, i], "--", label=f"v_body_{labels[i]}")

    axes[0].set_ylabel("acc [m/s^2]")
    axes[1].set_ylabel("gyr [rad/s]")
    axes[2].set_ylabel("pos / v_body")
    axes[2].set_xlabel("time [s]")
    titles = ("acceleration (IMU frame)", "angular rate (IMU frame)", "position & body velocity")
    for ax, title in zip(axes, titles):
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)


def build_figure(
    data: dict,
    frame_stride: int,
    ray_length: float,
    max_rays: int,
    show_3d: bool,
    show_imu: bool,
) -> plt.Figure:
    num_cam = int(data.get("num_cam", 1))
    cams = _parse_extrinsics(data["extrinsics"])
    colors = _camera_colors(len(cams) if cams else num_cam)
    multi_cam = num_cam >= 2 and len(cams) >= 2

    if show_3d and show_imu:
        fig_w = 15 if num_cam >= 4 else (14 if multi_cam else 11)
        fig = plt.figure(figsize=(fig_w, 12), constrained_layout=True)
        if multi_cam:
            right_w = 1.4 if num_cam >= 4 else 1.0
            gs = gridspec.GridSpec(
                4, 2, width_ratios=[3.0, right_w], height_ratios=[2.8, 1.0, 1.0, 1.0], figure=fig
            )
            ax3d = fig.add_subplot(gs[0, 0], projection="3d")
            _plot_fov_panels(fig, gs[0, 1], data, frame_stride, colors, cams)
            imu_axes = [fig.add_subplot(gs[i, :]) for i in (1, 2, 3)]
        else:
            gs = gridspec.GridSpec(4, 1, height_ratios=[2.8, 1.0, 1.0, 1.0], figure=fig)
            ax3d = fig.add_subplot(gs[0], projection="3d")
            imu_axes = [fig.add_subplot(gs[i]) for i in (1, 2, 3)]
        _plot_3d_on_ax(ax3d, data, frame_stride, ray_length, max_rays, colors, cams)
        _plot_imu_on_axes(imu_axes, data)
    elif show_3d:
        if multi_cam:
            fig_w = 14 if num_cam >= 4 else 13
            right_w = 1.4 if num_cam >= 4 else 1.0
            fig = plt.figure(figsize=(fig_w, 8), constrained_layout=True)
            gs = gridspec.GridSpec(1, 2, width_ratios=[3.0, right_w], figure=fig)
            ax3d = fig.add_subplot(gs[0, 0], projection="3d")
            _plot_fov_panels(fig, gs[0, 1], data, frame_stride, colors, cams)
        else:
            fig = plt.figure(figsize=(10, 8))
            ax3d = fig.add_subplot(111, projection="3d")
        _plot_3d_on_ax(ax3d, data, frame_stride, ray_length, max_rays, colors, cams)
    elif show_imu:
        fig, imu_axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        _plot_imu_on_axes(list(imu_axes), data)
    else:
        raise ValueError("At least one of 3D or IMU view must be enabled")
    if not fig.get_constrained_layout():
        fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Visualize sim_generator_dump JSON output.")
    parser.add_argument("dump", type=Path, nargs="?", default=Path("sim_dump.json"))
    parser.add_argument("--frame-stride", type=int, default=5, help="Plot every N-th image frame in 3D")
    parser.add_argument("--ray-length", type=float, default=3.0, help="Observation ray length [m] in 3D")
    parser.add_argument("--max-rays", type=int, default=80, help="Max rays per camera in 3D view")
    parser.add_argument("--no-3d", action="store_true")
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--save", type=Path, default=None, help="Save figure (headless)")
    args = parser.parse_args()

    data = load_dump(args.dump)
    num_cam = int(data.get("num_cam", 1))
    print(
        f"Loaded {args.dump}: imu={len(data['imu']['t'])}, "
        f"frames={len(data['image_frames'])}, cam={num_cam}"
    )
    if num_cam >= 2 and data["image_frames"]:
        from collections import Counter

        obs = data["image_frames"][-1].get("observations", [])
        by_cam = Counter(int(o["camera_id"]) for o in obs)
        print(f"  last frame obs by camera_id: {dict(sorted(by_cam.items()))}")

    fig = build_figure(
        data,
        args.frame_stride,
        args.ray_length,
        args.max_rays,
        show_3d=not args.no_3d,
        show_imu=not args.no_imu,
    )

    if args.save is not None:
        fig.savefig(args.save, dpi=120)
        print(f"Saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
