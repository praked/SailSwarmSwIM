#!/usr/bin/env python3
"""
Combine multiple flocking run CSVs and plot mean ± spread for
area and avg_wind_speed.

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
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from functools import reduce
import csv

# ------------------- Config -------------------

# Root directory that contains experiment folders like:
#   experiment_day2_*, experimentday3_*
ANALYSIS_DIR = BASE_DIR / "analysis"
ANALYSIS_DIR.mkdir(exist_ok=True)

FIG_WIDTH = 6.0
FIG_HEIGHT = 6.0
DPI = 300

FONT_SIZE_BASE = 12
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
COLOR_WIND = "#009E73"

BAND_ALPHA = 0.2  # transparency for std band


def load_and_merge(pattern: str) -> pd.DataFrame:
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files found matching pattern: {pattern}")

    print(f"Combining {len(files)} files for pattern: {pattern}")
    for f in files:
        print("  ", f)

    dfs = []
    for i, f in enumerate(files, start=1):
        df = pd.read_csv(f)
        # rename columns to attach run index
        df = df.rename(columns={
            "area": f"area_{i}",
            "avg_wind_speed": f"wind_{i}",
        })
        dfs.append(df)

    # inner join on 'time' across all runs
    merged = reduce(lambda left, right: pd.merge(left, right, on="time", how="inner"), dfs)
    return merged, files


def compute_stats(merged: pd.DataFrame):
    time = merged["time"].values

    area_cols = [c for c in merged.columns if c.startswith("area_")]
    wind_cols = [c for c in merged.columns if c.startswith("wind_")]

    area_vals = merged[area_cols].values
    wind_vals = merged[wind_cols].values

    stats = {
        "time": time,
        "area_mean": np.mean(area_vals, axis=1),
        "area_std": np.std(area_vals, axis=1, ddof=1),
        "wind_mean": np.mean(wind_vals, axis=1),
        "wind_std": np.std(wind_vals, axis=1, ddof=1),
    }
    return stats

# ---- Parse experiment parameter info from params.csv ----
def parse_params_from_csv(csv_path: pathlib.Path) -> str | None:
    """
    Parse a short parameter string from a params.csv file in an experiment folder.

    Expected lines (from RunBoidsFlocking.sh):
    - SWARMSWIM_DOMAIN (half-size):             VALUE
    - SWARM_SEP_FAC (weight for separation)     VALUE
    - SWARM_ALG_FAC (weight for alignment)      VALUE
    - SWARM_COH_FAC (weight for cohesion)       VALUE
    """
    if not csv_path.is_file():
        return None

    try:
        with csv_path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            row = next(reader, None)

            if row is None:
                return None

            parts = []

            if "SWARMSWIM_DOMAIN" in row and row["SWARMSWIM_DOMAIN"]:
                parts.append(f"DOMAIN SIZE={row['SWARMSWIM_DOMAIN']}")

            if "SWARM_SEP_FAC" in row and row["SWARM_SEP_FAC"]:
                parts.append(f"SEPARATION={row['SWARM_SEP_FAC']}")

            if "SWARM_ALG_FAC" in row and row["SWARM_ALG_FAC"]:
                parts.append(f"ALIGNMENT={row['SWARM_ALG_FAC']}")

            if "SWARM_COH_FAC" in row and row["SWARM_COH_FAC"]:
                parts.append(f"COHERENCE={row['SWARM_COH_FAC']}")

            if "WIND_GUST" in row and row["WIND_GUST"]:
                parts.append(f"WIND GUST={row['WIND_GUST']}m/s")

            return ", ".join(parts) if parts else None
    
    except Exception:
        return None


def plot_stats(stats, n_runs, out_prefix, exp_name=None, param_info=None):
    time = stats["time"]

    fig, axes = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
        figsize=(FIG_WIDTH, FIG_HEIGHT),
        constrained_layout=True,
    )
    # Set x-axis major ticks every 15 time units (shared across all subplots)
    axes[-1].yaxis.set_major_locator(MultipleLocator(0.25))
    # Build a figure-level title using experiment name and parameter info
    fig.suptitle(f"Flocking metrics ({n_runs} runs)", fontsize=FONT_SIZE_BASE)

    fig.set_constrained_layout_pads(
        h_pad=0.15,
        w_pad=0.02,
        hspace=0.02,
        wspace=0.05
    )
    if param_info:
        fig.text(0.5, 0.94, param_info, ha="center", va="top", fontsize=FONT_SIZE_BASE - 2.7)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


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
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", frameon=False)

    # 2) Avg wind speed
    ax = axes[1]
    ax.plot(time, stats["wind_mean"], color=COLOR_WIND, linewidth=1.4, label="Mean")
    ax.fill_between(
        time,
        stats["wind_mean"] - stats["wind_std"],
        stats["wind_mean"] + stats["wind_std"],
        color=COLOR_WIND,
        alpha=BAND_ALPHA,
        label="±1 SD",
    )
    ax.set_ylabel("Avg. wind speed [arb. units]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", frameon=False)

    ax.yaxis.set_major_locator(MultipleLocator(2.0))

    y_min, y_max = ax.get_ylim()
    padding = 0.25 * (y_max - y_min)
    ax.set_ylim(y_min, y_max + padding)


    # Make numbers on axes monospaced to align y-axis descriptions
    for ax in axes:
        #ax.tick_params(labelsize=12)
        #ax.xaxis.label.set_size(12)
        #ax.yaxis.label.set_size(12)
        ax.yaxis.set_major_formatter(FormatStrFormatter('%8.2f'))
        for label in ax.get_yticklabels():
            label.set_fontfamily("monospace")

    # Save
    out_png = f"{out_prefix}_multi_run_metrics.png"
    out_pdf = f"{out_prefix}_multi_run_metrics.pdf"
    fig.savefig(out_png, dpi=DPI)
    fig.savefig(out_pdf)
    print(f"Saved {out_png} and {out_pdf}")


def main():
    # Collect experiment folders:
    #   - BOIDS_FLOCKING_experiment*
    exp_dirs = []

    exp_dirs.extend(sorted(BASE_DIR.glob("BOIDS_FLOCKING_experiment4")))


    if not exp_dirs:
        raise FileNotFoundError("No experiment directories found matching 'experiment_day2_*' or 'experiment_day3_*'")

    print("Found experiment directories:")
    for d in exp_dirs:
        print("  ", d)

    for exp_dir in exp_dirs:
        if not exp_dir.is_dir():
            continue

        print(f"\nProcessing experiment folder: {exp_dir}")

        # Expect run01.csv .. run10.csv (or run??.csv in general)
        pattern = str(exp_dir / "run1??.csv")

        try:
            merged, files = load_and_merge(pattern)
        except FileNotFoundError as e:
            print(f"  Skipping {exp_dir}: {e}")
            continue

        stats = compute_stats(merged)

        # Read parameter details from params.csv, if present
        csv_path = exp_dir / "params.csv"
        param_info = parse_params_from_csv(csv_path)
        exp_name = exp_dir.name

        # Local output prefix: inside the experiment folder
        local_prefix = str(exp_dir / exp_dir.name)

        # Common analysis prefix: inside ANALYSIS_DIR, named after the experiment folder
        analysis_prefix = str(ANALYSIS_DIR / exp_dir.name)

        # Plot into experiment folder
        plot_stats(
            stats,
            n_runs=len(files),
            out_prefix=local_prefix,
            exp_name=exp_name,
            param_info=param_info,
        )

        # Plot into analysis folder
        plot_stats(
            stats,
            n_runs=len(files),
            out_prefix=analysis_prefix,
            exp_name=exp_name,
            param_info=param_info,
        )


if __name__ == "__main__":
    main()