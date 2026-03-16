#!/usr/bin/env python3
"""Animate sampled TOWR hopper results saved in hopper_result.json."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation


STANCE_COLORS = {0: "#d9534f", 1: "#2a9d8f"}


def load_result(result_path: Path) -> dict:
    with result_path.open("r", encoding="utf-8") as input_file:
        return json.load(input_file)


def to_array(values):
    return np.asarray(values, dtype=float)


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y @ rot_x


def set_axes_equal(ax, x_limits, y_limits, z_limits):
    ranges = np.array([
        x_limits[1] - x_limits[0],
        y_limits[1] - y_limits[0],
        z_limits[1] - z_limits[0],
    ])
    centers = np.array([
        np.mean(x_limits),
        np.mean(y_limits),
        np.mean(z_limits),
    ])
    radius = 0.5 * max(ranges)

    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(max(0.0, centers[2] - radius), centers[2] + radius)


def build_body_geometry(base_linear: np.ndarray, foot_motion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    initial_base = base_linear[0]
    foot_offset = foot_motion[0] - initial_base

    body_half_length = max(0.12, 0.35 * abs(foot_offset[0]))
    body_half_width = max(0.06, 0.12 + 0.5 * abs(foot_offset[1]))

    hip_local = np.array([0.0, 0.0, 0.0], dtype=float)
    body_outline_local = np.array(
        [
            [body_half_length, body_half_width, 0.0],
            [body_half_length, -body_half_width, 0.0],
            [-body_half_length, -body_half_width, 0.0],
            [-body_half_length, body_half_width, 0.0],
            [body_half_length, body_half_width, 0.0],
        ],
        dtype=float,
    )
    return hip_local, body_outline_local


def main():
    parser = argparse.ArgumentParser(description="Animate TOWR hopper solve results.")
    parser.add_argument(
        "result",
        nargs="?",
        default="hopper_result.json",
        help="Path to the saved hopper result JSON file.",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Optional animation output path, for example hopper.gif or hopper.mp4.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=12,
        help="Frames per second when saving the animation.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Animate every Nth sample to speed up playback.",
    )
    parser.add_argument(
        "--force-scale",
        type=float,
        default=0.002,
        help="Scale factor applied to the visualized ground reaction force vector.",
    )
    args = parser.parse_args()

    result_path = Path(args.result)
    result = load_result(result_path)
    sampled = result["sampled"]

    times = to_array(sampled["times"])
    base_linear = to_array(sampled["base_linear"])
    base_angular = to_array(sampled["base_angular"])
    ee_motion = to_array(sampled["ee_motion"])
    ee_force = to_array(sampled["ee_force"])
    ee_contact = np.asarray(sampled["ee_contact"], dtype=int)

    foot_motion = ee_motion[:, 0, :]
    foot_force = ee_force[:, 0, :]
    foot_contact = ee_contact[:, 0]

    stride = max(1, args.stride)
    frame_indices = np.arange(0, len(times), stride)
    if frame_indices[-1] != len(times) - 1:
        frame_indices = np.append(frame_indices, len(times) - 1)

    hip_local, body_outline_local = build_body_geometry(base_linear, foot_motion)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("TOWR Hopper Animation")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=20, azim=-60)

    x_limits = (min(base_linear[:, 0].min(), foot_motion[:, 0].min()) - 0.2,
                max(base_linear[:, 0].max(), foot_motion[:, 0].max()) + 0.2)
    y_limits = (min(base_linear[:, 1].min(), foot_motion[:, 1].min()) - 0.2,
                max(base_linear[:, 1].max(), foot_motion[:, 1].max()) + 0.2)
    z_limits = (0.0, max(base_linear[:, 2].max(), foot_motion[:, 2].max()) + 0.2)
    set_axes_equal(ax, x_limits, y_limits, z_limits)

    xx, yy = np.meshgrid(np.linspace(*x_limits, 2), np.linspace(*y_limits, 2))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.12, color="#8ecae6", shade=False)

    body_line, = ax.plot([], [], [], color="#1d3557", linewidth=2.5)
    leg_line, = ax.plot([], [], [], color="#264653", linewidth=2.5)
    base_point, = ax.plot([], [], [], marker="o", markersize=8, color="#1d3557")
    hip_point, = ax.plot([], [], [], marker="o", markersize=5, color="#1d3557")
    foot_point, = ax.plot([], [], [], marker="o", markersize=8)
    force_line, = ax.plot([], [], [], color="#f4a261", linewidth=2)
    time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

    def update(frame_id: int):
        sample_id = frame_indices[frame_id]
        base_pos = base_linear[sample_id]
        roll, pitch, yaw = base_angular[sample_id]
        world_from_base = rotation_matrix(roll, pitch, yaw)

        hip_world = (world_from_base @ hip_local) + base_pos
        body_world = (world_from_base @ body_outline_local.T).T + base_pos
        foot_world = foot_motion[sample_id]
        contact = int(foot_contact[sample_id])
        force_tip = foot_world + args.force_scale * foot_force[sample_id]

        body_line.set_data(body_world[:, 0], body_world[:, 1])
        body_line.set_3d_properties(body_world[:, 2])

        leg_points = np.vstack([hip_world, foot_world])
        leg_line.set_data(leg_points[:, 0], leg_points[:, 1])
        leg_line.set_3d_properties(leg_points[:, 2])
        leg_line.set_color(STANCE_COLORS[contact])

        base_point.set_data([base_pos[0]], [base_pos[1]])
        base_point.set_3d_properties([base_pos[2]])

        hip_point.set_data([hip_world[0]], [hip_world[1]])
        hip_point.set_3d_properties([hip_world[2]])

        foot_point.set_data([foot_world[0]], [foot_world[1]])
        foot_point.set_3d_properties([foot_world[2]])
        foot_point.set_color(STANCE_COLORS[contact])

        force_points = np.vstack([foot_world, force_tip])
        force_line.set_data(force_points[:, 0], force_points[:, 1])
        force_line.set_3d_properties(force_points[:, 2])
        force_line.set_alpha(1.0 if contact else 0.25)

        state_label = "stance" if contact else "swing"
        time_text.set_text(
            f"t = {times[sample_id]:.2f} s\nfoot: {state_label}\nforce z = {foot_force[sample_id, 2]:.1f} N"
        )

        return [body_line, leg_line, base_point, hip_point, foot_point, force_line, time_text]

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=1000 / max(1, args.fps),
        blit=False,
        repeat=True,
    )

    if args.save:
        output_path = Path(args.save)
        suffix = output_path.suffix.lower()
        save_kwargs = {"fps": args.fps}

        if suffix == ".gif":
            writer = animation.PillowWriter(fps=args.fps)
            ani.save(output_path, writer=writer)
        else:
            ani.save(output_path, **save_kwargs)
        print(f"Saved animation: {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()