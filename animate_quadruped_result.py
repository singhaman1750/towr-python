#!/usr/bin/env python3
"""Animate sampled TOWR quadruped results saved in quadruped_result.json."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation


LEG_NAMES = ["LF", "RF", "LH", "RH"]
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


def build_body_geometry(base_linear: np.ndarray, ee_motion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    initial_base = base_linear[0]
    local_foot_offsets = ee_motion[0] - initial_base

    hip_xy = local_foot_offsets[:, :2].copy()
    hip_xy[:, 0] = np.sign(hip_xy[:, 0]) * np.max(np.abs(hip_xy[:, 0]))
    hip_xy[:, 1] = np.sign(hip_xy[:, 1]) * np.max(np.abs(hip_xy[:, 1]))
    hip_points = np.column_stack([hip_xy, np.zeros(len(hip_xy))])

    corners_local = np.array(
        [
            [hip_xy[0, 0], hip_xy[0, 1], 0.0],
            [hip_xy[1, 0], hip_xy[1, 1], 0.0],
            [hip_xy[3, 0], hip_xy[3, 1], 0.0],
            [hip_xy[2, 0], hip_xy[2, 1], 0.0],
            [hip_xy[0, 0], hip_xy[0, 1], 0.0],
        ],
        dtype=float,
    )
    return hip_points, corners_local


def main():
    parser = argparse.ArgumentParser(description="Animate TOWR quadruped solve results.")
    parser.add_argument(
        "result",
        nargs="?",
        default="quadruped_result.json",
        help="Path to the saved quadruped result JSON file.",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Optional animation output path, for example quadruped.gif or quadruped.mp4.",
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
    args = parser.parse_args()

    result_path = Path(args.result)
    result = load_result(result_path)
    sampled = result["sampled"]

    times = to_array(sampled["times"])
    base_linear = to_array(sampled["base_linear"])
    base_angular = to_array(sampled["base_angular"])
    ee_motion = to_array(sampled["ee_motion"])
    ee_contact = np.asarray(sampled["ee_contact"], dtype=int)

    stride = max(1, args.stride)
    frame_indices = np.arange(0, len(times), stride)
    if frame_indices[-1] != len(times) - 1:
        frame_indices = np.append(frame_indices, len(times) - 1)

    hip_points_local, body_corners_local = build_body_geometry(base_linear, ee_motion)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("TOWR Quadruped Animation")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=20, azim=-60)

    x_limits = (min(base_linear[:, 0].min(), ee_motion[:, :, 0].min()) - 0.2,
                max(base_linear[:, 0].max(), ee_motion[:, :, 0].max()) + 0.2)
    y_limits = (min(base_linear[:, 1].min(), ee_motion[:, :, 1].min()) - 0.3,
                max(base_linear[:, 1].max(), ee_motion[:, :, 1].max()) + 0.3)
    z_limits = (0.0, max(base_linear[:, 2].max(), ee_motion[:, :, 2].max()) + 0.2)
    set_axes_equal(ax, x_limits, y_limits, z_limits)

    xx, yy = np.meshgrid(np.linspace(*x_limits, 2), np.linspace(*y_limits, 2))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.12, color="#8ecae6", shade=False)

    body_line, = ax.plot([], [], [], color="#1d3557", linewidth=2.5)
    base_point, = ax.plot([], [], [], marker="o", markersize=7, color="#1d3557")
    foot_scatter = ax.scatter([], [], [], s=40)
    leg_lines = [ax.plot([], [], [], linewidth=2)[0] for _ in LEG_NAMES]
    hip_scatter = ax.scatter([], [], [], s=20, color="#1d3557")
    time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

    def update(frame_id: int):
        sample_id = frame_indices[frame_id]
        base_pos = base_linear[sample_id]
        roll, pitch, yaw = base_angular[sample_id]
        world_from_base = rotation_matrix(roll, pitch, yaw)

        hip_world = (world_from_base @ hip_points_local.T).T + base_pos
        body_world = (world_from_base @ body_corners_local.T).T + base_pos
        foot_world = ee_motion[sample_id]
        contact_world = ee_contact[sample_id]

        body_line.set_data(body_world[:, 0], body_world[:, 1])
        body_line.set_3d_properties(body_world[:, 2])

        base_point.set_data([base_pos[0]], [base_pos[1]])
        base_point.set_3d_properties([base_pos[2]])

        hip_scatter._offsets3d = (hip_world[:, 0], hip_world[:, 1], hip_world[:, 2])
        foot_scatter._offsets3d = (foot_world[:, 0], foot_world[:, 1], foot_world[:, 2])
        foot_scatter.set_color([STANCE_COLORS[int(value)] for value in contact_world])

        for leg_index, line in enumerate(leg_lines):
            points = np.vstack([hip_world[leg_index], foot_world[leg_index]])
            line.set_data(points[:, 0], points[:, 1])
            line.set_3d_properties(points[:, 2])
            line.set_color(STANCE_COLORS[int(contact_world[leg_index])])

        contact_labels = "  ".join(
            f"{LEG_NAMES[index]}:{'stance' if contact_world[index] else 'swing'}"
            for index in range(len(LEG_NAMES))
        )
        time_text.set_text(f"t = {times[sample_id]:.2f} s\n{contact_labels}")

        return [body_line, base_point, foot_scatter, hip_scatter, time_text, *leg_lines]

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