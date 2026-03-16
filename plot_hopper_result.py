#!/usr/bin/env python3
"""Plot sampled TOWR hopper results saved in hopper_result.json."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_result(result_path: Path) -> dict:
    with result_path.open("r", encoding="utf-8") as input_file:
        return json.load(input_file)


def to_array(values):
    return np.asarray(values, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="Plot TOWR hopper solve results.")
    parser.add_argument(
        "result",
        nargs="?",
        default="hopper_result.json",
        help="Path to the saved hopper result JSON file.",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Optional output image path. If omitted, the figure is shown interactively.",
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

    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    fig.suptitle("TOWR Hopper Result", fontsize=14)

    axes[0].plot(times, base_linear[:, 0], label="x")
    axes[0].plot(times, base_linear[:, 1], label="y")
    axes[0].plot(times, base_linear[:, 2], label="z")
    axes[0].set_ylabel("Base Pos [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, base_angular[:, 0], label="roll")
    axes[1].plot(times, base_angular[:, 1], label="pitch")
    axes[1].plot(times, base_angular[:, 2], label="yaw")
    axes[1].set_ylabel("Base Ang [rad]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, ee_motion[:, 0, 0], label="foot x")
    axes[2].plot(times, ee_motion[:, 0, 1], label="foot y")
    axes[2].plot(times, ee_motion[:, 0, 2], label="foot z")
    axes[2].plot(times, ee_force[:, 0, 0], "--", label="force x")
    axes[2].plot(times, ee_force[:, 0, 1], "--", label="force y")
    axes[2].plot(times, ee_force[:, 0, 2], "--", label="force z")
    axes[2].set_ylabel("Foot / Force")
    axes[2].legend(loc="best", ncol=2)
    axes[2].grid(True, alpha=0.3)

    axes[3].step(times, ee_contact[:, 0], where="post", label="foot contact")
    axes[3].set_ylabel("Contact")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_yticks([0, 1])
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()

    if args.save:
        output_path = Path(args.save)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot: {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()