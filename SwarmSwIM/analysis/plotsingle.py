#!/usr/bin/env python3
"""
Plot the average metrics over multiple flocking runs (time, area, polarisation, avg_wind_speed)
from a given experiment directory, for use in a scientific paper.

Default: use directory '40experiment1' and files run101.csv..run110.csv.

Usage:
    python plotsingle.py 40experiment1
    # or, if you omit the argument, it defaults to '40experiment1'
"""

import sys
import pathlib
import pandas as pd
import matplotlib.pyplot as plt

# ------------------- Config -------------------

DEFAULT_DIR = "40experiment1"

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
    # --------------- Input: experiment directory and run files ---------------
    base_dir = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(DEFAULT_DIR)
    if not base_dir.exists():
        raise FileNotFoundError(f"Experiment directory not found: {base_dir}")

    # Look for run101.csv .. run110.csv (or run1??.csv in general)
    run_files = sorted(base_dir.glob("run1??.csv"))
    if not run_files:
        raise FileNotFoundError(f"No run1??.csv files found in {base_dir}")
    print("Found run files:")
    for f in run_files:
        print("  ", f)

    # Load and stack metrics across runs
    expected_cols = {"time", "area", "polarisation", "avg_wind_speed"}
    time = None
    areas = []
    pols = []
    winds = []

    for idx, path in enumerate(run_files, start=1):
        df = pd.read_csv(path)
        missing = expected_cols - set(df.columns)
        if missing:
            raise ValueError(f"CSV {path} missing columns: {missing}")

        if time is None:
            time = df["time"].values
        else:
            if len(df["time"].values) != len(time):
                raise ValueError(f"Time length mismatch in {path}")
        areas.append(df["area"].values)
        pols.append(df["polarisation"].values)
        winds.append(df["avg_wind_speed"].values)

    import numpy as np
    areas = np.vstack(areas)   # shape (n_runs, T)
    pols = np.vstack(pols)
    winds = np.vstack(winds)

    # Compute mean over runs
    area = areas.mean(axis=0)
    pol = pols.mean(axis=0)
    wind = winds.mean(axis=0)

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
    ax.set_title("Mean metrics over 10 runs, N=40")
    ax.grid(True, alpha=0.3)

    # 2) Polarisation
    ax = axes[1]
    ax.plot(time, pol, color=COLOR_POL, linewidth=1.2)
    ax.set_ylabel("Polarisation (0–1)")
    ax.grid(True, alpha=0.3)

    # 3) Average wind speed
    ax = axes[2]
    ax.plot(time, wind, color=COLOR_WIND, linewidth=1.2)
    ax.set_ylabel("Avg. wind speed [arb. units]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # --------------- Save ---------------
    stem = base_dir.name  # e.g. "40experiment1"
    out_png = base_dir / f"{stem}_mean_metrics.png"
    out_pdf = base_dir / f"{stem}_mean_metrics.pdf"

    fig.savefig(out_png, dpi=DPI)
    fig.savefig(out_pdf)
    print(f"Saved {out_png} and {out_pdf}")


if __name__ == "__main__":
    main()