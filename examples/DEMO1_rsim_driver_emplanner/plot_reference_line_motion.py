#!/usr/bin/env python3
"""
Plot per-frame reference lines generated while the plugin moves ego.

Usage:
  python3 plot_reference_line_motion.py [global_csv] [motion_csv] [svg_path] [max_frames]
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


def read_global_path(path: Path):
    points = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))
    return points


def read_motion(path: Path):
    frames = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = row["frame_id"]
            frames.setdefault(frame_id, []).append(
                {
                    "sim_time": float(row["sim_time"]),
                    "ego_x": float(row["ego_x"]),
                    "ego_y": float(row["ego_y"]),
                    "global_match_idx": int(row["global_match_idx"]),
                    "match_x": float(row.get("match_x", "nan")),
                    "match_y": float(row.get("match_y", "nan")),
                    "match_hdg": float(row.get("match_hdg", "nan")),
                    "projection_x": float(row["projection_x"]),
                    "projection_y": float(row["projection_y"]),
                    "projection_hdg": float(row["projection_hdg"]),
                    "target_idx": int(row["target_idx"]),
                    "point_idx": int(row["point_idx"]),
                    "ref_s": float(row["ref_s"]),
                    "ref_x": float(row["ref_x"]),
                    "ref_y": float(row["ref_y"]),
                    "ref_hdg": float(row["ref_hdg"]),
                    "target_x": float(row["target_x"]),
                    "target_y": float(row["target_y"]),
                }
            )
    return frames


def sample_keys(keys, max_count):
    if len(keys) <= max_count:
        return keys
    if max_count <= 1:
        return [keys[0]]
    result = []
    last = len(keys) - 1
    for i in range(max_count):
        result.append(keys[round(last * i / (max_count - 1))])
    deduped = []
    seen = set()
    for key in result:
        if key not in seen:
            deduped.append(key)
            seen.add(key)
    return deduped


def chord_length(points):
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0],
                            points[i][1] - points[i - 1][1])
    return total


def svg_polyline(points, tx, ty):
    return " ".join(f"{tx(x):.2f},{ty(y):.2f}" for x, y in points)


def plot(global_csv: Path, motion_csv: Path, svg_path: Path, max_frames: int = 24):
    if not global_csv.is_file():
        raise SystemExit(f"ERROR: global CSV not found: {global_csv}")
    if not motion_csv.is_file():
        raise SystemExit(f"ERROR: reference-line CSV not found: {motion_csv}")

    global_points = read_global_path(global_csv)
    frames = read_motion(motion_csv)
    if len(global_points) < 2:
        raise SystemExit(f"ERROR: not enough global path points: {global_csv}")
    if not frames:
        raise SystemExit(f"ERROR: no reference-line frames: {motion_csv}")

    frame_keys = list(frames.keys())
    sampled_keys = sample_keys(frame_keys, max(1, max_frames))

    global_xs = [p[0] for p in global_points]
    global_ys = [p[1] for p in global_points]
    global_h = max(max(global_ys) - min(global_ys), 1.0)
    row_gap = global_h + 8.0

    stacked_frames = []
    all_points = list(global_points)
    for rank, key in enumerate(sampled_keys, start=1):
        rows = sorted(frames[key], key=lambda r: r["point_idx"])
        offset_y = -rank * row_gap
        ref_points = [(row["ref_x"], row["ref_y"] + offset_y) for row in rows]
        target = (rows[0]["target_x"], rows[0]["target_y"] + offset_y)
        ego = (rows[0]["ego_x"], rows[0]["ego_y"] + offset_y)
        stacked_frames.append((key, rows, ref_points, target, ego, offset_y))
        all_points.extend(ref_points)
        all_points.append(target)
        all_points.append(ego)

    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    data_w = max(max_x - min_x, 1e-6)
    data_h = max(max_y - min_y, 1e-6)

    width, height = 1300, 900
    margin_left, margin_right = 90, 70
    margin_top, margin_bottom = 72, 70
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    scale = min(plot_w / data_w, plot_h / data_h)

    view_w = plot_w / scale
    view_h = plot_h / scale
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    view_min_x = center_x - view_w * 0.5
    view_min_y = center_y - view_h * 0.5

    def tx(x):
        return margin_left + (x - view_min_x) * scale

    def ty(y):
        return margin_top + plot_h - (y - view_min_y) * scale

    colors = [
        "#188038", "#d93025", "#f9ab00", "#9334e6", "#00acc1", "#e8710a",
        "#5f6368", "#1a73e8", "#c5221f", "#0b8043", "#8e24aa", "#00897b",
    ]

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{margin_left}" y="34" font-family="Arial" font-size="22" '
        f'font-weight="700" fill="#202124">Reference Lines During Plugin Motion</text>',
        f'<text x="{margin_left}" y="56" font-family="Arial" font-size="13" '
        f'fill="#5f6368">global={global_csv.name}, motion={motion_csv.name}, '
        f'frames={len(frames)}, sampled={len(sampled_keys)}</text>',
        f'<rect x="{margin_left:.2f}" y="{margin_top:.2f}" width="{plot_w:.2f}" '
        f'height="{plot_h:.2f}" fill="#fbfbfb" stroke="#c9cdd2" stroke-width="1"/>',
    ]

    for i in range(6):
        gx = margin_left + plot_w * i / 5
        gy = margin_top + plot_h * i / 5
        lines.append(
            f'<line x1="{gx:.2f}" y1="{margin_top:.2f}" x2="{gx:.2f}" '
            f'y2="{margin_top + plot_h:.2f}" stroke="#e6e8eb" stroke-width="1"/>'
        )
        lines.append(
            f'<line x1="{margin_left:.2f}" y1="{gy:.2f}" '
            f'x2="{margin_left + plot_w:.2f}" y2="{gy:.2f}" '
            f'stroke="#e6e8eb" stroke-width="1"/>'
        )

    lines.append(
        f'<polyline points="{svg_polyline(global_points, tx, ty)}" fill="none" '
        f'stroke="#1967d2" stroke-width="3" stroke-opacity="0.9"><title>global path</title></polyline>'
    )

    for rank, (key, rows, ref_points, target, ego, _offset_y) in enumerate(stacked_frames):
        color = colors[rank % len(colors)]
        first = rows[0]
        lines.append(
            f'<polyline points="{svg_polyline(ref_points, tx, ty)}" fill="none" '
            f'stroke="{color}" stroke-width="1.8" stroke-opacity="0.78">'
            f'<title>frame {key} t={first["sim_time"]:.2f}s</title></polyline>'
        )
        lines.append(
            f'<circle cx="{tx(ego[0]):.2f}" cy="{ty(ego[1]):.2f}" r="3.5" '
            f'fill="#202124"><title>ego frame {key}</title></circle>'
        )
        projection = (first["projection_x"], first["projection_y"] + _offset_y)
        lines.append(
            f'<circle cx="{tx(projection[0]):.2f}" cy="{ty(projection[1]):.2f}" r="4.0" '
            f'fill="#ffffff" stroke="#202124" stroke-width="1.4">'
            f'<title>projection frame {key}</title></circle>'
        )
        lines.append(
            f'<circle cx="{tx(target[0]):.2f}" cy="{ty(target[1]):.2f}" r="4.5" '
            f'fill="{color}" stroke="#ffffff" stroke-width="1"><title>target frame {key}</title></circle>'
        )
        label_x, label_y = ref_points[0]
        lines.append(
            f'<text x="{tx(label_x):.2f}" y="{ty(label_y) - 5:.2f}" '
            f'font-family="Arial" font-size="10" fill="{color}">'
            f'f={key} t={first["sim_time"]:.2f} global={first["global_match_idx"]} target={first["target_idx"]}</text>'
        )

    lines.append(
        f'<text x="{margin_left}" y="{height - 20}" font-family="Arial" '
        f'font-size="12" fill="#5f6368">global points={len(global_points)}, '
        f'chord={chord_length(global_points):.1f}m, sampled frames={len(sampled_keys)}</text>'
    )
    lines.append('</svg>')

    svg_path.parent.mkdir(parents=True, exist_ok=True)
    svg_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"[plot] global points={len(global_points)} chord={chord_length(global_points):.1f}m")
    print(f"[plot] reference frames={len(frames)} sampled={len(sampled_keys)}")
    for key in sampled_keys:
        rows = frames[key]
        first = rows[0]
        print(
            f"[plot] frame={key} t={first['sim_time']:.2f}s points={len(rows)} "
            f"global_match={first['global_match_idx']} target={first['target_idx']}"
        )
    print(f"[plot] SVG written: {svg_path}")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    out_csv = here / "output" / "csv"
    default_global = out_csv / "global_path_world_points.csv"
    default_motion = out_csv / "reference_line_motion.csv"
    default_svg = out_csv / "reference_line_motion_stacked.svg"

    global_arg = Path(sys.argv[1]) if len(sys.argv) > 1 else default_global
    motion_arg = Path(sys.argv[2]) if len(sys.argv) > 2 else default_motion
    svg_arg = Path(sys.argv[3]) if len(sys.argv) > 3 else default_svg
    max_frames_arg = int(sys.argv[4]) if len(sys.argv) > 4 else 24

    plot(global_arg, motion_arg, svg_arg, max_frames_arg)
