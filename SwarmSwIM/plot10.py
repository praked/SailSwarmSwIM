#!/usr/bin/env python3
"""
Combine multiple flocking run CSVs and plot mean ± spread for
area, polarisation_sd, avg_wind_speed.

Usage:
    # simplest: pick up all run*.csv in the folder
    python plot_multi_runs.py

    # or specify pattern explicitly
    python plot_multi_runs.py "run*.csv"
"""

import sys
import glob
import pathlib

# Define base directory (repo root, one level above this script)
BASE_DIR = pathlib.Path(__file__).resolve().parent.parent

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from functools import reduce

# ------------------- Config -------------------

FILE_PATTERN_DEFAULT = str(BASE_DIR / "experiment_day2_5" / "run??.csv")

FIG_WIDTH = 6.0
FIG_HEIGHT = 7.5
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

COLOR_AREA = "#0072B2"
COLOR_POL = "#D55E00"
COLOR_WIND = "#009E73"

BAND_ALPHA = 0.2  # transparency for std band


def load_and_merge(pattern: str) -> pd.DataFrame:
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files found matching pattern: {pattern}")

    print(f"Combining {len(files)} files:")
    for f in files:
        print("  ", f)

    dfs = []
    for i, f in enumerate(files, start=1):
        df = pd.read_csv(f)
        # rename columns to attach run index
        df = df.rename(columns={
            "area": f"area_{i}",
            "polarisation_sd": f"pol_{i}",
            "avg_wind_speed": f"wind_{i}",
        })
        dfs.append(df)

    # inner join on 'time' across all runs
    merged = reduce(lambda left, right: pd.merge(left, right, on="time", how="inner"), dfs)
    return merged, files


def compute_stats(merged: pd.DataFrame):
    time = merged["time"].values

    area_cols = [c for c in merged.columns if c.startswith("area_")]
    pol_cols = [c for c in merged.columns if c.startswith("pol_")]
    wind_cols = [c for c in merged.columns if c.startswith("wind_")]

    area_vals = merged[area_cols].values
    pol_vals = merged[pol_cols].values
    wind_vals = merged[wind_cols].values

    stats = {
        "time": time,
        "area_mean": np.mean(area_vals, axis=1),
        "area_std": np.std(area_vals, axis=1, ddof=1),
        "pol_mean": np.mean(pol_vals, axis=1),
        "pol_std": np.std(pol_vals, axis=1, ddof=1),
        "wind_mean": np.mean(wind_vals, axis=1),
        "wind_std": np.std(wind_vals, axis=1, ddof=1),
    }
    return stats


def plot_stats(stats: dict, n_runs: int, out_prefix: str):
    time = stats["time"]

    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(FIG_WIDTH, FIG_HEIGHT),
        constrained_layout=True,
    )
    # Set x-axis major ticks every 15 time units (shared across all subplots)
    axes[-2].yaxis.set_major_locator(MultipleLocator(0.25))
    # 1) Area
    ax = axes[0]
    ax.plot(time, stats["area_mean"], color=COLOR_AREA, linewidth=1.4, label="Mean")
    ax.fill_between(
        time,
        stats["area_mean"] - stats["area_std"],
        stats["area_mean"] + stats["area_std"],
        color=COLOR_AREA,
        alpha=BAND_ALPHA,
        label="±1 SD",
    )
    ax.set_ylabel("Flock area [arb. units]")
    ax.set_title(f"Flocking metrics ({n_runs} runs) with ZOO = 20.0")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", frameon=False)

    # 2) Polarisation SD
    ax = axes[1]
    ax.plot(time, stats["pol_mean"], color=COLOR_POL, linewidth=1.4, label="Mean")
    ax.fill_between(
        time,
        stats["pol_mean"] - stats["pol_std"],
        stats["pol_mean"] + stats["pol_std"],
        color=COLOR_POL,
        alpha=BAND_ALPHA,
        label="±1 SD",
    )
    ax.set_ylabel("Heading spread SD [rad]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", frameon=False)

    # 3) Avg wind speed
    ax = axes[2]
    ax.plot(time, stats["wind_mean"], color=COLOR_WIND, linewidth=1.4, label="Mean")
    ax.fill_between(
        time,
        stats["wind_mean"] - stats["wind_std"],
        stats["wind_mean"] + stats["wind_std"],
        color=COLOR_WIND,
        alpha=BAND_ALPHA,
        label="±1 SD",
    )
    ax.set_ylabel("Avg. wind speed [arb. units]", labelpad=20)
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", frameon=False)

    # Save
    out_png = f"{out_prefix}_multi_run_metrics.png"
    out_pdf = f"{out_prefix}_multi_run_metrics.pdf"
    fig.savefig(out_png, dpi=DPI)
    fig.savefig(out_pdf)
    print(f"Saved {out_png} and {out_pdf}")


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else FILE_PATTERN_DEFAULT

    merged, files = load_and_merge(pattern)
    stats = compute_stats(merged)

    # Use a simple prefix based on pattern or first file
    prefix = pathlib.Path(files[0]).stem.split("_")[0]  # e.g. "run0" -> "run0"
    plot_stats(stats, n_runs=len(files), out_prefix=prefix)


if __name__ == "__main__":
    main()