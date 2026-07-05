#!/usr/bin/env python3
"""
Plot ego speed and acceleration from DEMO1 plugin CSV output.

Default input:
  output/<scenario>/csv/planning_start_sl.csv

Default output:
  output/<scenario>/csv/ego_speed_accel.png
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path


DEFAULT_SCENARIO = "staticobstacles"
SCENARIOS = ("resource", "staticobstacles")


def default_csv_dir(scenario: str) -> Path:
    return Path(__file__).resolve().parent / "output" / scenario / "csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot ego speed and acceleration from planning_start_sl.csv."
    )
    parser.add_argument(
        "--scenario",
        choices=SCENARIOS,
        default=DEFAULT_SCENARIO,
        help="Scenario output directory to use for default paths.",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Input planning_start_sl.csv path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=140,
        help="Output image DPI.",
    )
    return parser.parse_args()


def parse_float(row: dict[str, str], name: str, line_number: int) -> float:
    value = row.get(name)
    if value in (None, ""):
        raise ValueError(f"missing {name} at line {line_number}")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite {name} at line {line_number}")
    return parsed


def read_ego_series(path: Path) -> list[tuple[float, float, float]]:
    if not path.is_file():
        raise SystemExit(f"ERROR: input CSV not found: {path}")

    samples: list[tuple[float, float, float]] = []
    skipped = 0
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"sim_time", "ego_speed", "ego_accel"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            fields = ", ".join(sorted(missing))
            raise SystemExit(f"ERROR: input CSV missing required columns: {fields}")

        for line_number, row in enumerate(reader, start=2):
            try:
                sim_time = parse_float(row, "sim_time", line_number)
                ego_speed = parse_float(row, "ego_speed", line_number)
                ego_accel = parse_float(row, "ego_accel", line_number)
            except ValueError:
                skipped += 1
                continue
            samples.append((sim_time, ego_speed, ego_accel))

    if not samples:
        raise SystemExit(f"ERROR: no valid ego speed/accel rows in: {path}")

    samples.sort(key=lambda item: item[0])
    if skipped:
        print(f"[plot] skipped invalid rows: {skipped}")
    return samples


def plot_ego_series(
    samples: list[tuple[float, float, float]],
    output_path: Path,
    dpi: int,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(output_path.parent / ".matplotlib"))

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [item[0] for item in samples]
    speeds = [item[1] for item in samples]
    accels = [item[2] for item in samples]

    fig, speed_ax = plt.subplots(figsize=(11.0, 5.5))
    speed_color = "#1f77b4"
    accel_color = "#d62728"

    speed_line = speed_ax.plot(
        times,
        speeds,
        color=speed_color,
        linewidth=1.8,
        label="ego speed",
    )
    speed_ax.set_xlabel("time (s)")
    speed_ax.set_ylabel("speed (m/s)", color=speed_color)
    speed_ax.tick_params(axis="y", labelcolor=speed_color)
    speed_ax.grid(True, color="#d9d9d9", linewidth=0.8, alpha=0.75)

    accel_ax = speed_ax.twinx()
    accel_line = accel_ax.plot(
        times,
        accels,
        color=accel_color,
        linewidth=1.5,
        label="ego acceleration",
    )
    accel_ax.set_ylabel("accel (m/s^2)", color=accel_color)
    accel_ax.tick_params(axis="y", labelcolor=accel_color)

    lines = speed_line + accel_line
    labels = [line.get_label() for line in lines]
    speed_ax.legend(lines, labels, loc="upper right")

    fig.suptitle("Ego speed and acceleration")
    fig.tight_layout()
    fig.savefig(output_path, dpi=max(1, dpi))
    plt.close(fig)


def main() -> None:
    args = parse_args()
    csv_dir = default_csv_dir(args.scenario)
    input_path = args.input or csv_dir / "planning_start_sl.csv"
    output_path = args.output or csv_dir / "ego_speed_accel.png"

    samples = read_ego_series(input_path)
    plot_ego_series(samples, output_path, args.dpi)
    print(f"[plot] input: {input_path}")
    print(f"[plot] rows: {len(samples)}")
    print(f"[plot] output: {output_path}")


if __name__ == "__main__":
    main()
