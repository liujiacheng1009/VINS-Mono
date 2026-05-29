#!/usr/bin/env python3
"""Online visualization of DataGenerator via pybind11 module vins_sim_data."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sim = None


def _candidate_module_dirs() -> list[Path]:
    """Search for vins_sim_data.so under common build output locations."""
    dirs: list[Path] = []
    seen: set[Path] = set()

    def add_dir(p: Path) -> None:
        p = p.resolve()
        if p in seen or not p.is_dir():
            return
        if any(p.glob("vins_sim_data*.so")):
            seen.add(p)
            dirs.append(p)

    script_root = Path(__file__).resolve().parents[3]  # repo root
    for base in (Path.cwd(), script_root):
        add_dir(base / "build_standalone" / "data_generator_vis")
        add_dir(base / "build" / "data_generator_vis")
    return dirs


def _import_sim():
    global sim
    if sim is not None:
        return sim
    for mod_dir in _candidate_module_dirs():
        s = str(mod_dir)
        if s not in sys.path:
            sys.path.insert(0, s)
    try:
        import vins_sim_data as _sim
    except ImportError as exc:
        sys.exit(
            "Cannot import vins_sim_data. Build the pybind module first:\n"
            "  cmake -S standalone -B build_standalone -DBUILD_SIM_PYTHON=ON\n"
            "  cmake --build build_standalone --target vins_sim_data -j\n"
            "Then run from repo root, or:\n"
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
        sim_config_path: str | None,
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

        if sim_config_path:
            opts = s.load_options(sim_config_path)
            self.gen = s.DataGenerator(opts, False)
        else:
            self.gen = s.DataGenerator(False)
        self.gen.set_quiet(True)
        self.num_cam = self.gen.num_cameras()
        self.cam_colors = plt.cm.tab10(np.linspace(0, 1, max(self.num_cam, 1)))
        self.cam_extrinsics = [
            {
                "slot": k,
                "cam_id": self.gen.camera_id(k),
                "ric": np.asarray(self.gen.get_ric(self.gen.camera_id(k)), dtype=float),
                "tic": np.asarray(self.gen.get_tic(self.gen.camera_id(k)), dtype=float),
                "color": self.cam_colors[k],
            }
            for k in range(self.num_cam)
        ]
        self.fov_deg = float(self.gen.fov_deg())
        self._tan_half_fov = np.tan(np.deg2rad(self.fov_deg * 0.5))
        self.landmarks = np.asarray(self.gen.get_cloud(), dtype=float)
        self.imu_per_img = self.gen.imu_per_image()

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
        self.obs_scatter = self.ax3d.scatter(
            [], [], [], c="limegreen", s=16, alpha=0.9, label="observed feat"
        )
        self.new_obs_scatter = self.ax3d.scatter(
            [], [], [], c="orange", s=22, alpha=0.95, label="new observed"
        )
        self.cam_points = []
        self.frustum_lines = []
        self._frustum_scale = 1.2
        for cam in self.cam_extrinsics:
            cid = cam["cam_id"]
            color = cam["color"]
            (pt,) = self.ax3d.plot([], [], [], "o", markersize=7, color=color, label=f"cam {cid}")
            self.cam_points.append(pt)
            edges = [
                self.ax3d.plot([], [], [], color=color, linewidth=1.2, alpha=0.9)[0] for _ in range(8)
            ]
            self.frustum_lines.append(edges)
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

    def _fov_panel_rect(self, k: int) -> list[float]:
        """Right-side FOV subplot [left, bottom, width, height] for camera index k."""
        if self.num_cam >= 4:
            col_w, row_h = 0.105, 0.19
            left0, bottom0 = 0.755, 0.42
            col = k % 2
            row = 1 - k // 2
            return [left0 + col * (col_w + 0.015), bottom0 + row * (row_h + 0.04), col_w, row_h]
        panel_h = 0.28 / max(self.num_cam, 1)
        bottom = 0.88 - (k + 1) * panel_h
        return [0.78, bottom, 0.20, panel_h - 0.01]

    def _create_fov_panel(self):
        lim = float(self._tan_half_fov)
        top = 0.88
        self.fov_axes = []
        self.fov_all_scatter = []
        self.fov_obs_scatter = []
        self.fov_new_scatter = []

        for k, cam in enumerate(self.cam_extrinsics):
            ax = self.fig.add_axes(self._fov_panel_rect(k))
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.grid(True, alpha=0.25)
            ax.set_title(f"cam {cam['cam_id']} FOV", fontsize=9)
            ax.set_xlabel("x / z", fontsize=7)
            ax.set_ylabel("y / z", fontsize=7)
            ax.tick_params(labelsize=6)
            self.fov_axes.append(ax)
            self.fov_all_scatter.append(
                ax.scatter([], [], c="lightgray", s=6, alpha=0.6, label="in FOV")
            )
            self.fov_obs_scatter.append(
                ax.scatter([], [], c=[cam["color"]], s=12, alpha=0.9, label="observed")
            )
            self.fov_new_scatter.append(
                ax.scatter([], [], c="orange", s=14, alpha=0.95, label="new")
            )
            if k == 0:
                ax.legend(loc="upper right", fontsize=6)

        if self.num_cam >= 4:
            stats_bottom = 0.36
        else:
            panel_h = 0.28 / max(self.num_cam, 1)
            stats_bottom = top - self.num_cam * panel_h - 0.02
        self.obs_count_text = self.fig.text(
            0.79,
            stats_bottom,
            "observed now: 0",
            fontsize=8,
            color="black",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75),
        )
        self.speed_text = self.fig.text(
            0.79,
            stats_bottom - 0.04,
            "speed: 1.0x",
            fontsize=8,
            color="black",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75),
        )
        self._controls_bottom = max(0.05, stats_bottom - 0.08)

    def _create_controls(self):
        ctrl_ax = self.fig.add_axes([0.78, self._controls_bottom, 0.20, 0.03])
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

    def _update_fov_panel(self, cam_idx: int, uv_fov: np.ndarray, uv_obs: np.ndarray, uv_new: np.ndarray):
        if len(uv_fov) == 0:
            self.fov_all_scatter[cam_idx].set_offsets(np.empty((0, 2)))
        else:
            self.fov_all_scatter[cam_idx].set_offsets(uv_fov)
        if len(uv_obs) == 0:
            self.fov_obs_scatter[cam_idx].set_offsets(np.empty((0, 2)))
        else:
            self.fov_obs_scatter[cam_idx].set_offsets(uv_obs)
        if len(uv_new) == 0:
            self.fov_new_scatter[cam_idx].set_offsets(np.empty((0, 2)))
        else:
            self.fov_new_scatter[cam_idx].set_offsets(uv_new)

    def _set_obs_points(self) -> None:
        """Highlight observed landmarks (union of all cameras), single color in 3D."""
        empty = ([], [], [])
        union_pts = np.asarray(self.gen.get_observed_points(), dtype=float)
        if len(union_pts) > 0:
            self.obs_scatter._offsets3d = (union_pts[:, 0], union_pts[:, 1], union_pts[:, 2])
        else:
            self.obs_scatter._offsets3d = empty

        new_ids: set[int] = set()
        for ids in self.gen.get_new_observed_landmark_ids():
            new_ids.update(int(i) for i in ids)
        if new_ids:
            pts = self.landmarks[np.asarray(sorted(new_ids), dtype=int)]
            self.new_obs_scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
        else:
            self.new_obs_scatter._offsets3d = empty

    def _update_frustums(self, position: np.ndarray, r_wi: np.ndarray):
        s = self._frustum_scale
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
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
        poses = []
        for k, cam in enumerate(self.cam_extrinsics):
            r_wc, t_wc = camera_pose_world(position, r_wi, cam["ric"], cam["tic"])
            corners_w = (r_wc @ corners_cam.T).T + t_wc.reshape(1, 3)
            for ln, (i, j) in zip(self.frustum_lines[k], edges):
                seg = corners_w[[i, j]]
                ln.set_data(seg[:, 0], seg[:, 1])
                ln.set_3d_properties(seg[:, 2])
            self.cam_points[k].set_data([t_wc[0]], [t_wc[1]])
            self.cam_points[k].set_3d_properties([t_wc[2]])
            poses.append((r_wc, t_wc))
        return poses

    def _advance_one_frame(self) -> bool:
        observations = []
        s = _import_sim()
        for _ in range(self.imu_per_img):
            if self.gen.get_time() > self.t_end:
                return False
            pos = np.asarray(self.gen.get_position(), dtype=float)
            self.imu_t.append(self.gen.get_time())
            self.imu_acc.append(np.asarray(self.gen.get_linear_acceleration(), dtype=float))
            self.imu_gyr.append(np.asarray(self.gen.get_angular_velocity(), dtype=float))

            if self.publish_count % self.imu_per_img == 0:
                raw = self.gen.get_image()
                observations = unpack_observations(raw, self.num_cam)
                r_wi = np.asarray(self.gen.get_rotation(), dtype=float)
                self.traj.append(pos.copy())
                obs_ids = {fid for fid, _, _ in observations}
                new_obs_ids = obs_ids - self.prev_obs_ids
                self.prev_obs_ids = obs_ids
                poses = self._update_frustums(pos, r_wi)
                self._set_obs_points()

                obs_by_slot = [0] * self.num_cam
                for _, slot, _ in observations:
                    if 0 <= slot < self.num_cam:
                        obs_by_slot[slot] += 1

                for k, (r_wc, t_wc) in enumerate(poses):
                    slot_obs = [(fid, slot, ray) for fid, slot, ray in observations if slot == k]
                    uv_fov, _ = self._compute_fov_distribution(r_wc, t_wc)
                    uv_obs, uv_new = self._obs_uv_from_rays(slot_obs, new_obs_ids)
                    self._update_fov_panel(k, uv_fov, uv_obs, uv_new)

                obs_now = len(obs_ids)
                self._last_obs_stats = (obs_now, obs_by_slot)
                parts = " ".join(
                    f"cam{self.cam_extrinsics[k]['cam_id']}={obs_by_slot[k]}"
                    for k in range(self.num_cam)
                )
                self.obs_count_text.set_text(f"observed now: {obs_now} ({parts})")

            self.gen.update()
            self.publish_count += 1

        if self.traj:
            traj = np.stack(self.traj, axis=0)
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
            self.traj_line.set_3d_properties(traj[:, 2])

        t_show = self.imu_t[-1] if self.imu_t else 0.0
        obs_now, obs_by_slot = getattr(self, "_last_obs_stats", (0, []))
        cam_summary = ",".join(
            f"{self.cam_extrinsics[k]['cam_id']}:{obs_by_slot[k] if k < len(obs_by_slot) else 0}"
            for k in range(self.num_cam)
        )
        self.ax3d.set_title(f"live sim t={t_show:.2f}s  cams=[{cam_summary}]  obs={obs_now}")

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
        frame_ms = 1000.0 * self.imu_per_img / s.FREQ
        if realtime:
            interval_ms = int(frame_ms)
        self._base_interval_ms = float(max(1, interval_ms))
        print(
            f"Live playback: {self.num_cam} cam(s), image ~{s.FREQ / self.imu_per_img:.1f} Hz, "
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
    parser.add_argument(
        "--config",
        type=str,
        default="config/simulation/simulation_config.yaml",
        help="simulation_config.yaml (num_of_cam, cam_chain extrinsics)",
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
        args.config,
        args.duration_scale,
        args.ray_length,
        args.max_rays,
        show_imu,
        args.imu_samples,
    )
    viewer.run(args.interval_ms, args.realtime)


if __name__ == "__main__":
    main()
