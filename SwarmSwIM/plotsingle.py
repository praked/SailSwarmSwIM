#!/usr/bin/env python3
"""
Plot a single flocking run CSV (time, area, polarisation_sd, avg_wind_speed)
for use in a scientific paper.

Usage:
    python plot_single_run.py run0-1.csv
    # or, if you omit the argument, it defaults to 'run0-1.csv'
"""

import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt

# ------------------- Config -------------------

DEFAULT_FILE = "run0-1.csv"

FIG_WIDTH = 6.0   # inches
FIG_HEIGHT = 7.5  # inches
DPI = 300

FONT_SIZE_BASE = 10
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": FONT_SIZE_BASE,
    "axes.labelsize": FONT_SIZE_BASE + 1,
    "axes.titlesize": FONT_SIZE_BASE + 1,
    "xtick.labelsize": FONT_SIZE_BASE,
    "ytick.labelsize": FONT_SIZE_BASE,
    "legend.fontsize": FONT_SIZE_BASE,
    "figure.dpi": DPI,
})

# colorblind-friendly palette
COLOR_AREA = "#0072B2"       # blue
COLOR_POL = "#D55E00"        # reddish
COLOR_WIND = "#009E73"       # green


def main():
    # --------------- Input ---------------
    csv_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(DEFAULT_FILE)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # sanity check on columns
    expected_cols = {"time", "area", "polarisation_sd", "avg_wind_speed"}
    missing = expected_cols - set(df.columns)
    if missing:
        raise ValueError(f"CSV missing columns: {missing}")

    time = df["time"].values
    area = df["area"].values
    pol = df["polarisation_sd"].values
    wind = df["avg_wind_speed"].values

    # --------------- Plot ---------------
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(FIG_WIDTH, FIG_HEIGHT),
        constrained_layout=True
    )

    # 1) Area
    ax = axes[0]
    ax.plot(time, area, color=COLOR_AREA, linewidth=1.2)
    ax.set_ylabel("Flock area [arb. units]")
    ax.set_title("Single run metrics")
    ax.grid(True, alpha=0.3)

    # 2) Polarisation SD
    ax = axes[1]
    ax.plot(time, pol, color=COLOR_POL, linewidth=1.2)
    ax.set_ylabel("Heading spread SD [rad]")
    ax.grid(True, alpha=0.3)

    # 3) Average wind speed
    ax = axes[2]
    ax.plot(time, wind, color=COLOR_WIND, linewidth=1.2)
    ax.set_ylabel("Avg. wind speed [arb. units]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # --------------- Save ---------------
    stem = csv_path.stem  # e.g. "run0-1"
    out_png = csv_path.with_name(f"{stem}_metrics.png")
    out_pdf = csv_path.with_name(f"{stem}_metrics.pdf")

    fig.savefig(out_png, dpi=DPI)
    fig.savefig(out_pdf)
    print(f"Saved {out_png} and {out_pdf}")


if __name__ == "__main__":
    main()