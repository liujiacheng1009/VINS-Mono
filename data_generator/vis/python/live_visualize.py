#!/usr/bin/env python3
"""Online visualization of DataGenerator via pybind11 module vins_sim_data."""

from __future__ import annotations

import argparse
import sys

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sim = None


def _import_sim():
    global sim
    if sim is not None:
        return sim
    try:
        import vins_sim_data as _sim
    except ImportError as exc:
        sys.exit(
            "Cannot import vins_sim_data. Install pybind11, build the module, set PYTHONPATH:\n"
            "  sudo apt install pybind11-dev   # or: pip3 install pybind11\n"
            "  cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON\n"
            "  cmake --build build_standalone --target vins_sim_data -j\n"
            "  export PYTHONPATH=$PWD/build_standalone/data_generator_vis:$PYTHONPATH\n"
            f"Original error: {exc}"
        )
    sim = _sim
    return sim


def camera_pose_world(position: np.ndarray, r_wi: np.ndarray, ric: np.ndarray, tic: np.ndarray):
    r_wc = r_wi @ ric
    t_wc = r_wi @ tic + position
    return r_wc, t_wc


def unpack_observations(raw_features, num_cam: int):
    obs = []
    for packed_id, ray in raw_features:
        fid = packed_id // num_cam
        cid = packed_id % num_cam
        obs.append((fid, cid, np.asarray(ray, dtype=float).reshape(3)))
    return obs


class LiveSimViewer:
    def __init__(
        self,
        duration_scale: float,
        ray_length: float,
        max_rays: int,
        show_imu: bool,
        imu_samples: int,
    ):
        self.duration_scale = duration_scale
        self.ray_length = ray_length
        self.max_rays = max_rays
        self.show_imu = show_imu
        self.imu_samples = imu_samples
        s = _import_sim()
        self.t_end = duration_scale * s.MAX_TIME

        self.gen = s.DataGenerator(False)
        self.gen.set_quiet(True)
        self.num_cam = self.gen.num_cameras()
        self.ric0 = np.asarray(self.gen.get_ric(0), dtype=float)
        self.tic0 = np.asarray(self.gen.get_tic(0), dtype=float)
        self.landmarks = np.asarray(self.gen.get_cloud(), dtype=float)

        self.publish_count = 0
        self._anim = None
        self.traj: list[np.ndarray] = []
        self.imu_t: list[float] = []
        self.imu_acc: list[np.ndarray] = []
        self.imu_gyr: list[np.ndarray] = []

        self._build_figure()

    def _build_figure(self) -> None:
        if self.show_imu:
            self.fig = plt.figure(figsize=(11, 10), constrained_layout=True)
            gs = gridspec.GridSpec(3, 1, height_ratios=[2.6, 1.0, 1.0], figure=self.fig)
            self.ax3d = self.fig.add_subplot(gs[0], projection="3d")
            self.ax_acc = self.fig.add_subplot(gs[1])
            self.ax_gyr = self.fig.add_subplot(gs[2], sharex=self.ax_acc)
            self.acc_lines = self._init_imu_axes(self.ax_acc, "acceleration (IMU frame)")
            self.gyr_lines = self._init_imu_axes(self.ax_gyr, "angular rate (IMU frame)")
            self.ax_gyr.set_xlabel("time [s]")
        else:
            self.fig = plt.figure(figsize=(10, 8))
            self.ax3d = self.fig.add_subplot(111, projection="3d")
            self.ax_acc = self.ax_gyr = None
            self.acc_lines = self.gyr_lines = []

        self.ax3d.scatter(
            self.landmarks[:, 0],
            self.landmarks[:, 1],
            self.landmarks[:, 2],
            c="lightgray",
            s=4,
            alpha=0.35,
            label="landmarks",
        )
        (self.traj_line,) = self.ax3d.plot([], [], [], "b-", linewidth=1.5, label="IMU path")
        (self.cam_point,) = self.ax3d.plot([], [], [], "ro", markersize=6, label="camera")
        self.ray_lines: list = []
        self.ax3d.set_xlabel("X [m]")
        self.ax3d.set_ylabel("Y [m]")
        self.ax3d.set_zlabel("Z [m]")
        self.ax3d.legend(loc="upper right", fontsize=8)
        if hasattr(self.ax3d, "set_box_aspect"):
            self.ax3d.set_box_aspect([1, 1, 1])
        self._set_axis_limits()

    @staticmethod
    def _init_imu_axes(ax, title: str):
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
        lines = []
        for label, style in zip("xyz", ["-", "--", ":"]):
            (ln,) = ax.plot([], [], style, label=label)
            lines.append(ln)
        ax.legend(loc="upper right", fontsize=7)
        return lines

    def _set_axis_limits(self):
        mn = self.landmarks.min(axis=0) - 1.0
        mx = self.landmarks.max(axis=0) + 1.0
        self.ax3d.set_xlim(mn[0], mx[0])
        self.ax3d.set_ylim(mn[1], mx[1])
        self.ax3d.set_zlim(mn[2], mx[2])

    def _clear_rays(self):
        for ln in self.ray_lines:
            ln.remove()
        self.ray_lines.clear()

    def _draw_rays(self, position: np.ndarray, r_wi: np.ndarray, observations):
        self._clear_rays()
        r_wc, t_wc = camera_pose_world(position, r_wi, self.ric0, self.tic0)
        for _, _, ray_cam in observations[: self.max_rays]:
            if ray_cam[2] <= 1e-9:
                continue
            d_world = r_wc @ (ray_cam / np.linalg.norm(ray_cam))
            end = t_wc + self.ray_length * d_world
            (ln,) = self.ax3d.plot(
                [t_wc[0], end[0]],
                [t_wc[1], end[1]],
                [t_wc[2], end[2]],
                color="C1",
                alpha=0.25,
                linewidth=0.5,
            )
            self.ray_lines.append(ln)

    def _advance_one_frame(self) -> bool:
        observations = []
        s = _import_sim()
        for _ in range(s.IMU_PER_IMG):
            if self.gen.get_time() > self.t_end:
                return False
            pos = np.asarray(self.gen.get_position(), dtype=float)
            self.imu_t.append(self.gen.get_time())
            self.imu_acc.append(np.asarray(self.gen.get_linear_acceleration(), dtype=float))
            self.imu_gyr.append(np.asarray(self.gen.get_angular_velocity(), dtype=float))

            if self.publish_count % s.IMU_PER_IMG == 0:
                raw = self.gen.get_image()
                observations = unpack_observations(raw, self.num_cam)
                r_wi = np.asarray(self.gen.get_rotation(), dtype=float)
                self.traj.append(pos.copy())
                self._draw_rays(pos, r_wi, observations)
                r_wc, t_wc = camera_pose_world(pos, r_wi, self.ric0, self.tic0)
                self.cam_point.set_data([t_wc[0]], [t_wc[1]])
                self.cam_point.set_3d_properties([t_wc[2]])

            self.gen.update()
            self.publish_count += 1

        if self.traj:
            traj = np.stack(self.traj, axis=0)
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
            self.traj_line.set_3d_properties(traj[:, 2])

        t_show = self.imu_t[-1] if self.imu_t else 0.0
        self.ax3d.set_title(f"live sim  t={t_show:.2f}s  obs={len(observations)}")

        if self.show_imu and self.imu_t:
            w = min(len(self.imu_t), self.imu_samples)
            ts = np.asarray(self.imu_t[-w:])
            acc = np.stack(self.imu_acc[-w:], axis=0)
            gyr = np.stack(self.imu_gyr[-w:], axis=0)
            for i, ln in enumerate(self.acc_lines):
                ln.set_data(ts, acc[:, i])
            for i, ln in enumerate(self.gyr_lines):
                ln.set_data(ts, gyr[:, i])
            for ax in (self.ax_acc, self.ax_gyr):
                ax.relim()
                ax.autoscale_view()
        return True

    def animate(self, _frame):
        if self.gen.get_time() > self.t_end:
            if self._anim is not None:
                self._anim.event_source.stop()
            return []
        self._advance_one_frame()
        return []

    def run(self, interval_ms: int, realtime: bool):
        self._anim = None
        s = _import_sim()
        frame_ms = 1000.0 * s.IMU_PER_IMG / s.FREQ
        if realtime:
            interval_ms = int(frame_ms)
        print(
            f"Live playback: image ~{s.FREQ / s.IMU_PER_IMG:.1f} Hz, "
            f"interval {interval_ms} ms, duration {self.t_end:.1f} s"
        )
        self._anim = FuncAnimation(
            self.fig,
            self.animate,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Live view of DataGenerator trajectory and observations",
        allow_abbrev=False,
    )
    parser.add_argument("--duration-scale", type=float, default=3.0, help="Simulation length multiplier (x MAX_TIME)")
    parser.add_argument("--ray-length", type=float, default=3.0)
    parser.add_argument("--max-rays", type=int, default=80)
    parser.add_argument("--interval-ms", type=int, default=80, help="Animation interval [ms]")
    parser.add_argument("--realtime", action="store_true", help="Match simulated image frame rate")
    parser.add_argument("--no-imu", action="store_true", help="Show 3D view only (default: 3D + IMU stacked)")
    parser.add_argument("--imu-samples", type=int, default=500, help="IMU rolling window length [samples]")
    args = parser.parse_args()

    show_imu = not args.no_imu
    _import_sim()
    viewer = LiveSimViewer(
        args.duration_scale,
        args.ray_length,
        args.max_rays,
        show_imu,
        args.imu_samples,
    )
    viewer.run(args.interval_ms, args.realtime)


if __name__ == "__main__":
    main()
