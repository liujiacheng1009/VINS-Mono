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


def _plot_3d_on_ax(
    ax,
    data: dict,
    frame_stride: int,
    _ray_length: float,
    _max_rays_per_frame: int,
) -> None:
    landmarks = np.asarray(data["landmarks"], dtype=float)
    extrinsics = data["extrinsics"]
    frames = data["image_frames"][::frame_stride]
    ric0 = np.asarray(extrinsics[0]["ric"], dtype=float)
    tic0 = np.asarray(extrinsics[0]["tic"], dtype=float)

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

    # Current observed feature landmarks (last shown frame).
    if frames:
        last_obs = frames[-1].get("observations", [])
        feat_ids = sorted({int(ob.get("feature_id", -1)) for ob in last_obs})
        valid_ids = [fid for fid in feat_ids if 0 <= fid < len(landmarks)]
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

    # Show one camera frustum only (last shown frame).
    if frames:
        pos = np.asarray(frames[-1]["position"], dtype=float)
        quat = np.asarray(frames[-1]["quaternion_wxyz"], dtype=float)
        r_wc, t_wc = camera_pose_world(pos, quat, ric0, tic0)
        s = 1.2
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
            ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], color="orangered", alpha=0.9, linewidth=1.2)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("GT: trajectory, landmarks, observed feat")
    ax.legend(loc="upper right", fontsize=8)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect([1, 1, 1])


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
    if show_3d and show_imu:
        fig = plt.figure(figsize=(11, 12), constrained_layout=True)
        gs = gridspec.GridSpec(4, 1, height_ratios=[2.8, 1.0, 1.0, 1.0], figure=fig)
        ax3d = fig.add_subplot(gs[0], projection="3d")
        ax_acc = fig.add_subplot(gs[1])
        ax_gyr = fig.add_subplot(gs[2], sharex=ax_acc)
        ax_kin = fig.add_subplot(gs[3], sharex=ax_acc)
        _plot_3d_on_ax(ax3d, data, frame_stride, ray_length, max_rays)
        _plot_imu_on_axes([ax_acc, ax_gyr, ax_kin], data)
    elif show_3d:
        fig = plt.figure(figsize=(10, 8))
        ax3d = fig.add_subplot(111, projection="3d")
        _plot_3d_on_ax(ax3d, data, frame_stride, ray_length, max_rays)
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
    parser.add_argument("--ray-length", type=float, default=3.0, help="Observation ray length [m]")
    parser.add_argument("--max-rays", type=int, default=80, help="Max rays per displayed frame")
    parser.add_argument("--no-3d", action="store_true")
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--save", type=Path, default=None, help="Save figure (headless)")
    args = parser.parse_args()

    data = load_dump(args.dump)
    print(
        f"Loaded {args.dump}: imu={len(data['imu']['t'])}, "
        f"frames={len(data['image_frames'])}, cam={data['num_cam']}"
    )

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
