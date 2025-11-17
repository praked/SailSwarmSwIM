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

# Root directory that contains experiment folders like:
#   experiment_day2_*, experimentday3_*
ANALYSIS_DIR = BASE_DIR / "analysis"
ANALYSIS_DIR.mkdir(exist_ok=True)

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

    print(f"Combining {len(files)} files for pattern: {pattern}")
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


# ---- New helper: parse experiment parameter info from README.md ----
def parse_params_from_readme(readme_path: pathlib.Path) -> str | None:
    """
    Parse a short parameter string from a README.md file in an experiment folder.

    Expected lines (from ScriptFlocking.sh):
    - SWARM_ZOR (repulsion radius):   VALUE
    - SWARM_ZOO (orientation radius): VALUE
    - SWARM_ZOA (attraction radius):  VALUE
    - SWARMSWIM_DOMAIN (half-size):   VALUE
    - SWARM_T_MAX (duration):         VALUE s
    """
    if not readme_path.is_file():
        return None

    params = {}
    try:
        with readme_path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("- SWARM_ZOR"):
                    params["ZOR"] = line.split(":", 1)[1].strip()
                elif line.startswith("- SWARM_ZOO"):
                    params["ZOO"] = line.split(":", 1)[1].strip()
                elif line.startswith("- SWARM_ZOA"):
                    params["ZOA"] = line.split(":", 1)[1].strip()
                elif line.startswith("- SWARMSWIM_DOMAIN"):
                    params["DOMAIN"] = line.split(":", 1)[1].strip()
                elif line.startswith("- SWARM_T_MAX"):
                    val = line.split(":", 1)[1].strip()
                    if val.endswith(" s"):
                        val = val[:-2].strip()
                    params["T_MAX"] = val
    except Exception:
        return None

    if not params:
        return None

    # Build a compact one-line description
    parts = []
    if "ZOR" in params:
        parts.append(f"ZOR={params['ZOR']}")
    if "ZOO" in params:
        parts.append(f"ZOO={params['ZOO']}")
    if "ZOA" in params:
        parts.append(f"ZOA={params['ZOA']}")
    if "DOMAIN" in params:
        parts.append(f"DOMAIN SIZE={params['DOMAIN']}")
    if "T_MAX" in params:
        parts.append(f"DURATION={params['T_MAX']}s")

    return ", ".join(parts) if parts else None


def plot_stats(stats, n_runs, out_prefix, exp_name=None, param_info=None):
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
    # Build a figure-level title using experiment name and parameter info
    title_lines = []
    title_lines.append(f"Flocking metrics ({n_runs} runs)")
    if param_info:
        title_lines.append(param_info)
    fig.suptitle("\n".join(title_lines), fontsize=FONT_SIZE_BASE + 1)
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
    # Collect experiment folders:
    #   - experiment_day2_*
    #   - experiment_day3_*
    exp_dirs = []

    exp_dirs.extend(sorted(BASE_DIR.glob("experiment_day2_*")))
    exp_dirs.extend(sorted(BASE_DIR.glob("experiment_day3_*")))

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
        pattern = str(exp_dir / "run??.csv")

        try:
            merged, files = load_and_merge(pattern)
        except FileNotFoundError as e:
            print(f"  Skipping {exp_dir}: {e}")
            continue

        stats = compute_stats(merged)

        # Read parameter details from README, if present
        readme_path = exp_dir / "README.md"
        param_info = parse_params_from_readme(readme_path)
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