#!/usr/bin/env python3
"""
plot_global_path.py — 读取 global_path_world_points.csv 生成 SVG 覆盖图

用法: python3 plot_global_path.py [csv_path] [svg_path]
  csv_path  默认: output/global_path_world_points.csv
  svg_path  默认: output/global_path_overlay.svg
"""

import colorsys
import csv
import math
import sys
from pathlib import Path


def plot(csv_path, svg_path):
    if not csv_path.is_file():
        print(f"ERROR: CSV not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    points = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))

    if len(points) < 2:
        print(f"ERROR: not enough points in {csv_path} ({len(points)})", file=sys.stderr)
        sys.exit(1)

    # ---- 布局计算 ----
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    data_w = max(max_x - min_x, 1e-6)
    data_h = max(max_y - min_y, 1e-6)

    width, height = 1200, 850
    margin_left, margin_right = 80, 80
    margin_top, margin_bottom = 70, 75
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    scale = min(plot_w / data_w, plot_h / data_h)

    view_w = plot_w / scale
    view_h = plot_h / scale
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    view_min_x = center_x - view_w * 0.5
    view_max_x = center_x + view_w * 0.5
    view_min_y = center_y - view_h * 0.5
    view_max_y = center_y + view_h * 0.5

    def tx(x):
        return margin_left + (x - view_min_x) * scale

    def ty(y):
        return margin_top + plot_h - (y - view_min_y) * scale

    def polyline(pts):
        return " ".join(f"{tx(x):.2f},{ty(y):.2f}" for x, y in pts)

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{margin_left}" y="34" font-family="Arial" font-size="22" '
        f'font-weight="700" fill="#202124">Global Path — World Coordinates</text>',
        f'<text x="{margin_left}" y="56" font-family="Arial" font-size="13" '
        f'fill="#5f6368">source: {csv_path.name}, points={len(points)}, '
        f'chord≈{_chord_length(points):.1f}m</text>',
        f'<rect x="{margin_left:.2f}" y="{margin_top:.2f}" width="{plot_w:.2f}" '
        f'height="{plot_h:.2f}" fill="#fbfbfb" stroke="#c9cdd2" stroke-width="1"/>',
    ]

    # ---- 网格 ----
    for i in range(6):
        gx = margin_left + plot_w * i / 5
        gy = margin_top + plot_h * i / 5
        x_label = view_min_x + view_w * i / 5
        y_label = view_max_y - view_h * i / 5
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
            f'<text x="{gx - 16:.2f}" y="{margin_top + plot_h + 18:.2f}" '
            f'font-family="Arial" font-size="11" fill="#5f6368">{x_label:.0f}</text>'
        )
        lines.append(
            f'<text x="{margin_left - 52:.2f}" y="{gy + 4:.2f}" '
            f'font-family="Arial" font-size="11" fill="#5f6368">{y_label:.0f}</text>'
        )

    # ---- 全局路径 polyline ----
    lines.append(
        f'<polyline points="{polyline(points)}" fill="none" '
        f'stroke="#1967d2" stroke-width="2.5" stroke-opacity="0.85">'
    )
    lines.append('<title>global path</title>')
    lines.append('</polyline>')

    # ---- 起点 (绿圈) ----
    sx, sy = points[0]
    lines.append(
        f'<circle cx="{tx(sx):.2f}" cy="{ty(sy):.2f}" r="5" fill="#12a150">'
    )
    lines.append('<title>start</title>')
    lines.append('</circle>')

    # ---- 终点 (红叉) ----
    ex, ey = points[-1]
    ex_tx, ey_ty = tx(ex), ty(ey)
    lines.append(
        f'<line x1="{ex_tx - 6:.2f}" y1="{ey_ty - 6:.2f}" '
        f'x2="{ex_tx + 6:.2f}" y2="{ey_ty + 6:.2f}" '
        f'stroke="#d93025" stroke-width="2"/>'
    )
    lines.append(
        f'<line x1="{ex_tx - 6:.2f}" y1="{ey_ty + 6:.2f}" '
        f'x2="{ex_tx + 6:.2f}" y2="{ey_ty - 6:.2f}" '
        f'stroke="#d93025" stroke-width="2"><title>end</title></line>'
    )

    # ---- 坐标轴标签 ----
    lines.append(
        f'<text x="{margin_left + plot_w * 0.5 - 18:.2f}" '
        f'y="{height - 24}" font-family="Arial" font-size="13" '
        f'fill="#3c4043">x (m)</text>'
    )
    lines.append(
        f'<text x="22" y="{margin_top + plot_h * 0.5:.2f}" '
        f'font-family="Arial" font-size="13" fill="#3c4043" '
        f'transform="rotate(-90 22 {margin_top + plot_h * 0.5:.2f})">y (m)</text>'
    )
    lines.append(
        f'<text x="{margin_left:.2f}" y="{height - 8}" font-family="Arial" '
        f'font-size="12" fill="#5f6368">global extent x [{min_x:.2f}, {max_x:.2f}], '
        f'y [{min_y:.2f}, {max_y:.2f}]</text>'
    )
    lines.append('</svg>')

    svg_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[plot] SVG written: {svg_path}")


def _chord_length(points):
    total = 0.0
    for i in range(1, len(points)):
        total += math.sqrt(
            (points[i][0] - points[i - 1][0]) ** 2
            + (points[i][1] - points[i - 1][1]) ** 2
        )
    return total


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    out_dir = here / "output"
    CSV_DIR= out_dir / "csv"
    default_csv = out_dir /CSV_DIR/"global_path_world_points.csv"
    default_svg = out_dir / CSV_DIR/"global_path_overlay.svg"

    csv_arg = Path(sys.argv[1]) if len(sys.argv) > 1 else default_csv
    svg_arg = Path(sys.argv[2]) if len(sys.argv) > 2 else default_svg

    plot(csv_arg, svg_arg)
