#!/usr/bin/env python3
"""
Plot mean aggregation metrics over multiple runs from an experiment directory.

Metrics per run CSV:
    time, c_max, a_hull_norm, w_mean

Each experiment directory is expected to contain:
    run101.csv .. run110.csv   (or generally run1??.csv)

Default base directory: '40aggregate1'.

Usage:
    python -m SwarmSwIM.plotaggregate            # uses 40aggregate1
    python -m SwarmSwIM.plotaggregate 40aggregate1
"""

import sys
import pathlib

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ------------------- Config -------------------

DEFAULT_DIR = "10aggregate1"

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

COLOR_CMAX = "#0072B2"   # blue
COLOR_FRAC = "#D55E00"   # orange/red
COLOR_AREA = "#009E73"   # green
COLOR_WIND = "#CC79A7"   # purple/magenta for mean wind speed


def main():
    # --------------- Input: experiment directory and run files ---------------
    base_dir = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(DEFAULT_DIR)
    if not base_dir.exists():
        raise FileNotFoundError(f"Experiment directory not found: {base_dir}")

    # Expect run101.csv .. run110.csv (or run1??.csv in general)
    run_files = sorted(base_dir.glob("run1??.csv"))
    if not run_files:
        raise FileNotFoundError(f"No run1??.csv files found in {base_dir}")

    print("Found aggregation run files:")
    for f in run_files:
        print("  ", f)

    expected_cols = {"time", "c_max", "a_hull_norm", "w_mean"}

    time = None
    c_list = []
    a_list = []
    w_list = []

    for path in run_files:
        df = pd.read_csv(path)
        missing = expected_cols - set(df.columns)
        if missing:
            raise ValueError(f"CSV {path} missing columns: {missing}")

        if time is None:
            time = df["time"].values
        else:
            if len(df["time"].values) != len(time):
                raise ValueError(f"Time length mismatch in {path}")

        c_list.append(df["c_max"].values.astype(float))
        a_list.append(df["a_hull_norm"].values.astype(float))
        w_list.append(df["w_mean"].values.astype(float))


    c_arr = np.vstack(c_list)
    a_arr = np.vstack(a_list)
    w_arr = np.vstack(w_list)

    c_mean = c_arr.mean(axis=0)
    a_mean = a_arr.mean(axis=0)
    w_mean = w_arr.mean(axis=0)

    # Estimate N as the maximum cluster size observed in any run/time
    n_est = float(c_arr.max()) if c_arr.size > 0 else 1.0
    frac_mean = c_mean / n_est if n_est > 0 else np.zeros_like(c_mean)

    # Optional: T90 (median over runs, printed to console)
    t90_list = []
    t90_median = None
    # for run_idx in range(c_arr.shape[0]):
    #     c_run = c_arr[run_idx, :]
    #     # first time where C_max >= 0.9N
    #     thr = 0.9 * n_est
    #     idx = np.where(c_run >= thr)[0]
    #     if idx.size > 0:
    #         t90_list.append(time[idx[0]])
    # if t90_list:
    #     t90_median = float(np.median(t90_list))
    #     print(f"T90 median over runs (C_max >= 0.9 N): {t90_median:.2f} s")
    # else:
    #     t90_median = None
    #     print("T90 not reached in any run (C_max < 0.9 N for all t).")

    # --------------- Plot ---------------
    fig, axes = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
        figsize=(FIG_WIDTH, FIG_HEIGHT),
        constrained_layout=True,
    )

    # 1) Largest cluster size C_max(t)
    ax = axes[0]
    ax.plot(time, c_mean, color=COLOR_CMAX, linewidth=1.2)
    ax.set_ylabel("$C_{\\max}(t)$")
    title = "Mean aggregation metrics over 10 runs, N = 10"
    if t90_median is not None:
        title += f" (median $T_{{90}} \\approx {t90_median:.0f}$ s)"
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    # # 2) Fraction C_max/N
    # ax = axes[1]
    # ax.plot(time, frac_mean, color=COLOR_FRAC, linewidth=1.2)
    # ax.set_ylabel(" $C_{\\max}/N$")
    # ax.set_ylim(0.0, 1.05)
    # ax.grid(True, alpha=0.3)

    # 3) Normalized hull area A_hull / arena
    ax = axes[1]
    ax.plot(time, a_mean, color=COLOR_AREA, linewidth=1.2)
    ax.set_ylabel("$A_{\\text{hull}}/(2L_h)^2$")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # 4) Mean wind speed
    ax = axes[2]
    ax.plot(time, w_mean, color=COLOR_WIND, linewidth=1.2)
    ax.set_ylabel("$\\overline{W}(t)$ [m/s]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)

    # --------------- Save ---------------
    stem = base_dir.name  # e.g. "40aggregate1"
    out_png = base_dir / f"{stem}_agg_mean_metrics.png"
    out_pdf = base_dir / f"{stem}_agg_mean_metrics.pdf"

    fig.savefig(out_png, dpi=DPI)
    fig.savefig(out_pdf)
    print(f"Saved {out_png} and {out_pdf}")


if __name__ == "__main__":
    main()