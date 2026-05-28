#!/usr/bin/env python3
"""Online visualization of DataGenerator via pybind11 module vins_sim_data."""

from __future__ import annotations

import argparse
import sys

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
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
        self.fov_deg = float(getattr(s, "FOV", 90.0))
        self._tan_half_fov = np.tan(np.deg2rad(self.fov_deg * 0.5))

        self.gen = s.DataGenerator(False)
        self.gen.set_quiet(True)
        self.num_cam = self.gen.num_cameras()
        self.ric0 = np.asarray(self.gen.get_ric(0), dtype=float)
        self.tic0 = np.asarray(self.gen.get_tic(0), dtype=float)
        self.landmarks = np.asarray(self.gen.get_cloud(), dtype=float)

        self.publish_count = 0
        self._anim = None
        self._base_interval_ms = 80.0
        self.playback_speed = 1.0
        self._speed_phase = 0.0
        self.traj: list[np.ndarray] = []
        self.imu_t: list[float] = []
        self.imu_acc: list[np.ndarray] = []
        self.imu_gyr: list[np.ndarray] = []
        self.prev_obs_ids: set[int] = set()

        self._build_figure()

    def _build_figure(self) -> None:
        if self.show_imu:
            self.fig = plt.figure(figsize=(13, 9), constrained_layout=False)
            gs = gridspec.GridSpec(3, 1, height_ratios=[2.6, 1.0, 1.0], figure=self.fig)
            self.ax3d = self.fig.add_subplot(gs[0], projection="3d")
            self.ax_acc = self.fig.add_subplot(gs[1])
            self.ax_gyr = self.fig.add_subplot(gs[2], sharex=self.ax_acc)
            self.ax3d.set_position([0.06, 0.34, 0.68, 0.60])
            self.ax_acc.set_position([0.06, 0.18, 0.68, 0.12])
            self.ax_gyr.set_position([0.06, 0.05, 0.68, 0.12])
            self.acc_lines = self._init_imu_axes(self.ax_acc, "acceleration (IMU frame)")
            self.gyr_lines = self._init_imu_axes(self.ax_gyr, "angular rate (IMU frame)")
            self.ax_gyr.set_xlabel("time [s]")
        else:
            self.fig = plt.figure(figsize=(13, 8), constrained_layout=False)
            self.ax3d = self.fig.add_subplot(111, projection="3d")
            self.ax3d.set_position([0.06, 0.08, 0.68, 0.86])
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
        self.obs_scatter = self.ax3d.scatter([], [], [], c="limegreen", s=16, alpha=0.9, label="observed feat")
        self.new_obs_scatter = self.ax3d.scatter([], [], [], c="orange", s=24, alpha=0.95, label="new observed")
        self.frustum_lines = [
            self.ax3d.plot([], [], [], color="orangered", linewidth=1.2, alpha=0.9)[0] for _ in range(8)
        ]
        self._frustum_scale = 1.2
        self.ax3d.set_xlabel("X [m]")
        self.ax3d.set_ylabel("Y [m]")
        self.ax3d.set_zlabel("Z [m]")
        self.ax3d.legend(loc="upper right", fontsize=8)
        if hasattr(self.ax3d, "set_box_aspect"):
            self.ax3d.set_box_aspect([1, 1, 1])
        self._set_axis_limits()
        self._create_fov_panel()
        self._create_controls()

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

    def _create_fov_panel(self):
        # 2D distribution panel in normalized image plane (x/z, y/z).
        self.ax_fov = self.fig.add_axes([0.78, 0.58, 0.20, 0.30])
        lim = float(self._tan_half_fov)
        self.ax_fov.set_xlim(-lim, lim)
        self.ax_fov.set_ylim(-lim, lim)
        self.ax_fov.grid(True, alpha=0.25)
        self.ax_fov.set_title("Current FOV distribution", fontsize=9)
        self.ax_fov.set_xlabel("x / z", fontsize=8)
        self.ax_fov.set_ylabel("y / z", fontsize=8)
        self.ax_fov.tick_params(labelsize=7)
        self.fov_all_scatter = self.ax_fov.scatter([], [], c="lightgray", s=8, alpha=0.6, label="in FOV")
        self.fov_obs_scatter = self.ax_fov.scatter([], [], c="limegreen", s=14, alpha=0.9, label="observed")
        self.fov_new_scatter = self.ax_fov.scatter([], [], c="orange", s=18, alpha=0.95, label="new observed")
        self.ax_fov.legend(loc="upper right", fontsize=7)
        self.obs_count_text = self.ax_fov.text(
            0.02,
            0.98,
            "observed now: 0",
            transform=self.ax_fov.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            color="black",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75),
        )
        self.speed_text = self.ax_fov.text(
            0.02,
            0.90,
            "speed: 1.0x",
            transform=self.ax_fov.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            color="black",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75),
        )

    def _create_controls(self):
        ctrl_ax = self.fig.add_axes([0.78, 0.47, 0.20, 0.03])
        self.speed_slider = Slider(ctrl_ax, "speed x", 0.2, 5.0, valinit=self.playback_speed, valstep=0.1)

        def _on_speed_change(val):
            self.playback_speed = float(val)
            self.speed_text.set_text(f"speed: {self.playback_speed:.1f}x")
            if self._anim is not None:
                self._anim.event_source.interval = max(1, int(self._base_interval_ms / self.playback_speed))

        self.speed_slider.on_changed(_on_speed_change)

    def _compute_fov_distribution(self, r_wc: np.ndarray, t_wc: np.ndarray):
        p_c = (r_wc.T @ (self.landmarks - t_wc.reshape(1, 3)).T).T
        z = p_c[:, 2]
        valid = z > 1e-9
        if not np.any(valid):
            return np.empty((0, 2)), 0
        uv = p_c[valid, :2] / z[valid].reshape(-1, 1)
        inside = (np.abs(uv[:, 0]) <= self._tan_half_fov) & (np.abs(uv[:, 1]) <= self._tan_half_fov)
        uv_fov = uv[inside]
        return uv_fov, int(len(uv_fov))

    def _obs_uv_from_rays(self, observations, new_obs_ids: set[int]):
        uv_obs = []
        uv_new = []
        for fid, _, ray in observations:
            z = float(ray[2])
            if z <= 1e-9:
                continue
            uv = [float(ray[0] / z), float(ray[1] / z)]
            uv_obs.append(uv)
            if fid in new_obs_ids:
                uv_new.append(uv)
        return np.asarray(uv_obs, dtype=float), np.asarray(uv_new, dtype=float)

    def _update_fov_panel(self, uv_fov: np.ndarray, uv_obs: np.ndarray, uv_new: np.ndarray):
        if len(uv_fov) == 0:
            self.fov_all_scatter.set_offsets(np.empty((0, 2)))
        else:
            self.fov_all_scatter.set_offsets(uv_fov)
        if len(uv_obs) == 0:
            self.fov_obs_scatter.set_offsets(np.empty((0, 2)))
        else:
            self.fov_obs_scatter.set_offsets(uv_obs)
        if len(uv_new) == 0:
            self.fov_new_scatter.set_offsets(np.empty((0, 2)))
        else:
            self.fov_new_scatter.set_offsets(uv_new)

    def _set_obs_points(self, observations, new_obs_ids: set[int], observed_world: np.ndarray):
        if not observations or observed_world.size == 0:
            self.obs_scatter._offsets3d = ([], [], [])
            self.new_obs_scatter._offsets3d = ([], [], [])
            return
        n = min(len(observations), observed_world.shape[0])
        pts = observed_world[:n]
        self.obs_scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
        new_pts = [pts[i] for i, (fid, _, _) in enumerate(observations[:n]) if fid in new_obs_ids]
        if new_pts:
            new_pts_arr = np.asarray(new_pts, dtype=float)
            self.new_obs_scatter._offsets3d = (new_pts_arr[:, 0], new_pts_arr[:, 1], new_pts_arr[:, 2])
        else:
            self.new_obs_scatter._offsets3d = ([], [], [])

    def _update_frustum(self, position: np.ndarray, r_wi: np.ndarray):
        r_wc, t_wc = camera_pose_world(position, r_wi, self.ric0, self.tic0)
        s = self._frustum_scale
        # Pinhole frustum in camera frame (+Z forward), then transform to world frame.
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
        for ln, (i, j) in zip(self.frustum_lines, edges):
            seg = corners_w[[i, j]]
            ln.set_data(seg[:, 0], seg[:, 1])
            ln.set_3d_properties(seg[:, 2])
        return r_wc, t_wc

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
                observed_world = np.asarray(self.gen.get_observed_points(), dtype=float)
                r_wi = np.asarray(self.gen.get_rotation(), dtype=float)
                self.traj.append(pos.copy())
                obs_ids = {fid for fid, _, _ in observations}
                # Non-continuous new observation: visible now but absent in previous frame.
                new_obs_ids = obs_ids - self.prev_obs_ids
                self.prev_obs_ids = obs_ids
                r_wc, t_wc = self._update_frustum(pos, r_wi)
                self._set_obs_points(observations, new_obs_ids, observed_world)
                uv_fov, fov_total = self._compute_fov_distribution(r_wc, t_wc)
                uv_obs, uv_new = self._obs_uv_from_rays(observations, new_obs_ids)
                self._update_fov_panel(uv_fov, uv_obs, uv_new)
                obs_now = len(obs_ids)
                self._last_obs_stats = (obs_now, fov_total)
                self.obs_count_text.set_text(f"observed now: {obs_now}")

            self.gen.update()
            self.publish_count += 1

        if self.traj:
            traj = np.stack(self.traj, axis=0)
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
            self.traj_line.set_3d_properties(traj[:, 2])

        t_show = self.imu_t[-1] if self.imu_t else 0.0
        obs_now, _ = getattr(self, "_last_obs_stats", (0, 0))
        self.ax3d.set_title(f"live sim t={t_show:.2f}s  observed={obs_now}")

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
        # Advance simulation according to speed factor, not only timer interval.
        self._speed_phase += self.playback_speed
        n_steps = int(self._speed_phase)
        self._speed_phase -= n_steps
        if n_steps <= 0:
            return []
        for _ in range(n_steps):
            if not self._advance_one_frame():
                if self._anim is not None:
                    self._anim.event_source.stop()
                break
        return []

    def run(self, interval_ms: int, realtime: bool):
        self._anim = None
        s = _import_sim()
        frame_ms = 1000.0 * s.IMU_PER_IMG / s.FREQ
        if realtime:
            interval_ms = int(frame_ms)
        self._base_interval_ms = float(max(1, interval_ms))
        print(
            f"Live playback: image ~{s.FREQ / s.IMU_PER_IMG:.1f} Hz, "
            f"interval {int(self._base_interval_ms)} ms, duration {self.t_end:.1f} s"
        )
        self._anim = FuncAnimation(
            self.fig,
            self.animate,
            interval=max(1, int(self._base_interval_ms / self.playback_speed)),
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
