#!/usr/bin/env python3
"""
Create a GIF showing ego motion and the per-frame generated reference line.

Usage:
  python3 make_reference_line_gif.py
  python3 make_reference_line_gif.py --max-frames 120 --fps 12
  python3 make_reference_line_gif.py --follow-reference
  python3 make_reference_line_gif.py --full-route
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def read_global_path(path: Path):
    points = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))
    return points


def read_reference_motion(path: Path):
    frames = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = int(row["frame_id"])
            frames.setdefault(frame_id, []).append(
                {
                    "frame_id": frame_id,
                    "sim_time": float(row["sim_time"]),
                    "ego_x": float(row["ego_x"]),
                    "ego_y": float(row["ego_y"]),
                    "global_match_idx": int(row["global_match_idx"]),
                    "projection_x": float(row["projection_x"]),
                    "projection_y": float(row["projection_y"]),
                    "projection_hdg": float(row["projection_hdg"]),
                    "target_idx": int(row["target_idx"]),
                    "point_idx": int(row["point_idx"]),
                    "ref_s": float(row["ref_s"]),
                    "ref_x": float(row["ref_x"]),
                    "ref_y": float(row["ref_y"]),
                    "target_x": float(row["target_x"]),
                    "target_y": float(row["target_y"]),
                }
            )

    ordered = []
    for frame_id in sorted(frames):
        rows = sorted(frames[frame_id], key=lambda r: r["point_idx"])
        if rows:
            ordered.append(rows)
    return ordered


def sample_frames(frames, max_frames: int):
    if max_frames <= 0 or len(frames) <= max_frames:
        return frames

    last = len(frames) - 1
    result = []
    seen = set()
    for i in range(max_frames):
        index = round(last * i / (max_frames - 1))
        if index not in seen:
            result.append(frames[index])
            seen.add(index)
    return result


def expand_limits(xs, ys, aspect: float, padding: float):
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = max(max_x - min_x + 2.0 * padding, 20.0)
    height = max(max_y - min_y + 2.0 * padding, 12.0)

    if width / height > aspect:
        height = width / aspect
    else:
        width = height * aspect

    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    return (
        center_x - 0.5 * width,
        center_x + 0.5 * width,
        center_y - 0.5 * height,
        center_y + 0.5 * height,
    )


def all_xy(points):
    return [p[0] for p in points], [p[1] for p in points]


def frame_xy(rows):
    xs = [row["ref_x"] for row in rows]
    ys = [row["ref_y"] for row in rows]
    first = rows[0]
    xs.extend([first["ego_x"], first["projection_x"], first["target_x"]])
    ys.extend([first["ego_y"], first["projection_y"], first["target_y"]])
    return xs, ys


def make_gif(global_csv: Path,
             motion_csv: Path,
             output_gif: Path,
             max_frames: int,
             fps: int,
             dpi: int,
             trail: int,
             full_route: bool):
    if not global_csv.is_file():
        raise SystemExit(f"ERROR: global path CSV not found: {global_csv}")
    if not motion_csv.is_file():
        raise SystemExit(f"ERROR: reference-line motion CSV not found: {motion_csv}")

    global_points = read_global_path(global_csv)
    frames = read_reference_motion(motion_csv)
    if len(global_points) < 2:
        raise SystemExit(f"ERROR: not enough global path points: {global_csv}")
    if not frames:
        raise SystemExit(f"ERROR: no reference-line frames: {motion_csv}")

    sampled = sample_frames(frames, max_frames)
    output_gif.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10.5, 6.2))
    aspect = 10.5 / 6.2
    global_xs, global_ys = all_xy(global_points)
    route_limits = expand_limits(global_xs, global_ys, aspect, padding=8.0)

    ego_history = [(rows[0]["ego_x"], rows[0]["ego_y"]) for rows in sampled]

    def draw(frame_index: int):
        rows = sampled[frame_index]
        first = rows[0]
        ref_xs = [row["ref_x"] for row in rows]
        ref_ys = [row["ref_y"] for row in rows]

        ax.clear()
        ax.plot(global_xs, global_ys, color="#b8bec7", linewidth=1.4,
                label="global path", zorder=1)
        ax.plot(ref_xs, ref_ys, color="#188038", linewidth=2.6,
                label="current reference line", zorder=3)

        trail_begin = max(0, frame_index - max(0, trail))
        trail_points = ego_history[trail_begin:frame_index + 1]
        if len(trail_points) >= 2:
            txs, tys = all_xy(trail_points)
            ax.plot(txs, tys, color="#f29900", linewidth=2.0,
                    alpha=0.85, label="ego trail", zorder=4)

        ax.scatter([first["projection_x"]], [first["projection_y"]],
                   s=48, facecolors="white", edgecolors="#202124",
                   linewidths=1.4, label="projection", zorder=5)
        ax.scatter([first["target_x"]], [first["target_y"]],
                   s=58, color="#d93025", label="target", zorder=6)
        ax.scatter([first["ego_x"]], [first["ego_y"]],
                   s=70, color="#202124", label="ego", zorder=7)

        heading = first["projection_hdg"]
        ax.arrow(first["ego_x"], first["ego_y"],
                 math.cos(heading) * 3.0,
                 math.sin(heading) * 3.0,
                 width=0.16, head_width=0.9, head_length=1.2,
                 length_includes_head=True, color="#202124", zorder=8)

        if full_route:
            limits = route_limits
        else:
            xs, ys = frame_xy(rows)
            for x, y in trail_points:
                xs.append(x)
                ys.append(y)
            limits = expand_limits(xs, ys, aspect, padding=8.0)

        ax.set_xlim(limits[0], limits[1])
        ax.set_ylim(limits[2], limits[3])
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#e6e8eb", linewidth=0.8)
        ax.set_xlabel("world x (m)")
        ax.set_ylabel("world y (m)")
        ax.set_title(
            "Reference line generation "
            f"frame={first['frame_id']} t={first['sim_time']:.2f}s "
            f"globalMatch={first['global_match_idx']} "
            f"points={len(rows)} targetIdx={first['target_idx']} "
            f"view={'global route' if full_route else 'reference follow'}"
        )
        ax.legend(loc="upper right", fontsize=8, framealpha=0.92)
        return []

    animation = FuncAnimation(fig,
                              draw,
                              frames=len(sampled),
                              interval=1000.0 / max(1, fps),
                              blit=False)
    writer = PillowWriter(fps=max(1, fps))
    animation.save(output_gif, writer=writer, dpi=dpi)
    plt.close(fig)

    print(f"[gif] global points: {len(global_points)}")
    print(f"[gif] reference frames: {len(frames)} sampled: {len(sampled)}")
    print(f"[gif] output: {output_gif}")


def parse_args():
    here = Path(__file__).resolve().parent
    csv_dir = here / "output" / "csv"
    parser = argparse.ArgumentParser(
        description="Create a GIF from reference_line_motion.csv."
    )
    parser.add_argument("--global-csv",
                        type=Path,
                        default=csv_dir / "global_path_world_points.csv")
    parser.add_argument("--motion-csv",
                        type=Path,
                        default=csv_dir / "reference_line_motion.csv")
    parser.add_argument("--output",
                        type=Path,
                        default=csv_dir / "reference_line_motion.gif")
    parser.add_argument("--max-frames",
                        type=int,
                        default=160,
                        help="Uniformly sample at most this many frames; <=0 uses all frames.")
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument("--trail",
                        type=int,
                        default=24,
                        help="Number of sampled ego positions to keep as a trail.")
    view_group = parser.add_mutually_exclusive_group()
    view_group.add_argument("--follow-reference",
                            action="store_true",
                            help="Follow the current reference line instead of using the fixed global-path view.")
    view_group.add_argument("--full-route",
                            action="store_true",
                            help="Use the fixed global-path view. This is the default.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    make_gif(args.global_csv,
             args.motion_csv,
             args.output,
             args.max_frames,
             args.fps,
             args.dpi,
             args.trail,
             not args.follow_reference)
