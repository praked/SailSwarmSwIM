#!/usr/bin/env python3
"""run_speed_weight_stats.py
Statistical analysis of speed-weighted (gamma-sweep) flocking experiments.

Controller:  w_ij = k / (v_j + eps)^gamma
  gamma < 0 : fast-neighbour following
  gamma = 0 : uniform Couzin-like weighting
  gamma > 0 : slow-neighbour anchoring

The exponent was renamed from `p` to `gamma` to avoid confusion with
statistical p-values.

Creates a timestamped output folder:
    analysis/DDMMYYYY_HHMMSS/
        code/run_speed_weight_stats.py
        figures/
        tables/
        report.md

Run from the repository root:
    python run_speed_weight_stats.py
"""

import os
import re
import csv
import json
import math
import shutil
import warnings
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import scipy.stats as sp_stats

# Optional: statsmodels for Holm correction
try:
    from statsmodels.stats.multitest import multipletests as _sm_multipletests
    _HAS_STATSMODELS = True
except ImportError:
    _HAS_STATSMODELS = False

# ── Configuration ─────────────────────────────────────────────────────────────

WINDOW_START  = 100.0
WINDOW_END    = 300.0
REPO_ROOT     = Path(__file__).resolve().parent
RESULTS_BASE  = REPO_ROOT / "results" / "comparison"
ANALYSIS_BASE = REPO_ROOT / "analysis"
PERM_SEED     = 42
N_PERM        = 4999   # permutations per cell; fast enough for ~200 cells

ENV_LABELS = {
    "w05_ng": "5 m/s, no gusts",
    "w10_ng": "10 m/s, no gusts",
    "w05_g":  "5 m/s + gusts",
    "w10_g":  "10 m/s + gusts",
}
ENV_ORDER = ["w05_ng", "w10_ng", "w05_g", "w10_g"]

METRICS = {
    "area":         {"label": "Flock Area (m²)",      "improvement": "lower",  "col": "area"},
    "polarisation": {"label": "Polarisation [0–1]",    "improvement": "higher", "col": "polarisation"},
    "collisions":   {"label": "Cumulative Collisions",  "improvement": "lower",  "col": "collisions"},
}
METRIC_ORDER = ["area", "polarisation", "collisions"]

# Gammas highlighted in diagnostic plots
DIAG_GAMMAS = [0.01, 0.1, 1.0, 2.0]

# Timestamp for this run (DDMMYYYY_HHMMSS)
TS = datetime.now().strftime("%d%m%Y_%H%M%S")

# ── Output folder setup ───────────────────────────────────────────────────────

def setup_output_dirs() -> dict[str, Path]:
    base = ANALYSIS_BASE / TS
    dirs = {
        "base":    base,
        "code":    base / "code",
        "figures": base / "figures",
        "tables":  base / "tables",
    }
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)
    # Copy this script into code/
    shutil.copy2(__file__, dirs["code"] / Path(__file__).name)
    return dirs


# ── Filename parsing ──────────────────────────────────────────────────────────

_RE_NAV  = re.compile(r'^nav_(w\d+_(?:ng|g))_seed(\d+)_(\d{8}_\d{6})\.csv$')
_RE_SLOW = re.compile(r'^slow_(w\d+_(?:ng|g))_(.+)_seed(\d+)_(\d{8}_\d{6})\.csv$')


def _decode_gamma(variant_tag: str) -> float:
    """'k1p0_p0p1' -> 0.1,  'k1p0_p-0p001' -> -0.001"""
    # Split off the gamma part after the last '_p' separator
    _, gamma_raw = variant_tag.rsplit("_p", 1)
    return float(gamma_raw.replace("p", "."))


def parse_filename(fname: str) -> dict | None:
    m = _RE_NAV.match(fname)
    if m:
        return {"algo": "nav", "env": m.group(1), "gamma": None,
                "seed": int(m.group(2)), "ts": m.group(3)}
    m = _RE_SLOW.match(fname)
    if m:
        try:
            gamma = _decode_gamma(m.group(2))
        except (ValueError, IndexError):
            return None
        return {"algo": "slow", "env": m.group(1), "gamma": gamma,
                "seed": int(m.group(3)), "ts": m.group(4)}
    return None


# ── Data loading ──────────────────────────────────────────────────────────────

def seed_steady_mean(path: Path) -> dict | None:
    """Return {metric: mean_over_[WINDOW_START, WINDOW_END]} for one CSV."""
    try:
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except Exception:
        return None
    if not rows or "time" not in rows[0]:
        return None
    try:
        times = np.array([float(r["time"]) for r in rows])
        mask  = (times >= WINDOW_START) & (times <= WINDOW_END)
        if mask.sum() == 0:
            return None
        result = {}
        for mkey, minfo in METRICS.items():
            col = minfo["col"]
            if col in rows[0]:
                vals = np.array([float(r[col]) for r in rows])
                result[mkey] = float(vals[mask].mean())
        return result or None
    except Exception:
        return None


def load_all_data() -> list[dict]:
    """Scan every flock_sweep_* dir, keep newest CSV per (algo,env,gamma,seed)."""
    # key -> (dir_timestamp_str, Path)
    best: dict[tuple, tuple[str, Path]] = {}

    for sweep_dir in sorted(RESULTS_BASE.glob("flock_sweep_*"), reverse=True):
        m = re.match(r'flock_sweep_(\d{8}_\d{6})', sweep_dir.name)
        dir_ts = m.group(1) if m else "00000000_000000"
        for p in sweep_dir.glob("*.csv"):
            parsed = parse_filename(p.name)
            if parsed is None:
                continue
            key = (parsed["algo"], parsed["env"], parsed["gamma"], parsed["seed"])
            if key not in best or dir_ts > best[key][0]:
                best[key] = (dir_ts, p)

    records = []
    for (algo, env, gamma, seed), (_, path) in best.items():
        means = seed_steady_mean(path)
        if means is None:
            continue
        records.append({"algo": algo, "env": env, "gamma": gamma,
                         "seed": seed, **means})
    return records


# ── Statistical functions ─────────────────────────────────────────────────────

def holm_correct(p_values: np.ndarray) -> np.ndarray:
    if _HAS_STATSMODELS:
        _, corrected, _, _ = _sm_multipletests(p_values, method="holm")
        return np.clip(corrected, 0.0, 1.0)
    m = len(p_values)
    order = np.argsort(p_values)
    corrected = np.empty(m)
    for rank, idx in enumerate(order):
        corrected[idx] = min(1.0, p_values[idx] * (m - rank))
    # Enforce step-down monotonicity
    for rank in range(1, m):
        corrected[order[rank]] = max(corrected[order[rank]],
                                     corrected[order[rank - 1]])
    return np.clip(corrected, 0.0, 1.0)


def _permutation_p(delta: np.ndarray, rng: np.random.Generator) -> float:
    obs = float(np.mean(delta))
    n   = len(delta)
    count = 0
    for _ in range(N_PERM):
        signs = rng.choice(np.array([-1.0, 1.0]), size=n)
        if abs(float(np.mean(signs * delta))) >= abs(obs):
            count += 1
    return (count + 1) / (N_PERM + 1)


def paired_stats(baseline: np.ndarray, treatment: np.ndarray,
                 rng: np.random.Generator) -> dict:
    delta = treatment - baseline
    n     = len(delta)
    if n < 3:
        return {}

    mean_d   = float(np.mean(delta))
    std_d    = float(np.std(delta, ddof=1)) if n > 1 else np.nan
    med_d    = float(np.median(delta))
    q1, q3   = float(np.percentile(delta, 25)), float(np.percentile(delta, 75))
    iqr_d    = q3 - q1
    skew_d   = float(sp_stats.skew(delta))

    sw_stat, sw_p = (sp_stats.shapiro(delta) if n >= 3
                     else (np.nan, np.nan))

    t_stat, t_p = sp_stats.ttest_1samp(delta, 0.0)

    try:
        w_stat, w_p = sp_stats.wilcoxon(delta, alternative="two-sided",
                                         zero_method="wilcox")
    except ValueError:
        w_stat, w_p = np.nan, np.nan

    perm_p = _permutation_p(delta, rng)

    dz = mean_d / std_d if (std_d is not np.nan and std_d > 1e-10) else np.nan

    return {
        "n":                 n,
        "baseline_mean":     float(np.mean(baseline)),
        "treatment_mean":    float(np.mean(treatment)),
        "baseline_median":   float(np.median(baseline)),
        "treatment_median":  float(np.median(treatment)),
        "mean_delta":        mean_d,
        "median_delta":      med_d,
        "std_delta":         std_d,
        "iqr_delta":         iqr_d,
        "skew_delta":        skew_d,
        "sw_stat":           float(sw_stat),
        "sw_pvalue":         float(sw_p),
        "t_stat":            float(t_stat),
        "p_value_ttest":     float(t_p),
        "w_stat":            float(w_stat),
        "p_value_wilcoxon":  float(w_p),
        "p_value_permutation": perm_p,
        "cohens_dz":         float(dz) if not math.isnan(dz) else np.nan,
    }


# ── Build pairwise stats table ────────────────────────────────────────────────

def build_stats_table(records: list[dict]) -> list[dict]:
    """Build one row per (env, gamma, metric) with full paired stats + Holm."""
    import pandas as pd

    df = pd.DataFrame(records)
    if df.empty:
        return []

    nav  = df[df["algo"] == "nav"].copy()
    slow = df[df["algo"] == "slow"].copy()

    rng = np.random.default_rng(PERM_SEED)
    rows = []

    for env in ENV_ORDER:
        nav_env  = nav[nav["env"] == env]
        slow_env = slow[slow["env"] == env]
        if nav_env.empty:
            continue
        nav_by_seed = nav_env.set_index("seed")

        gammas = sorted(slow_env["gamma"].unique())

        for metric in METRIC_ORDER:
            if metric not in df.columns:
                continue

            # Collect raw p-values for Holm correction across this (env, metric)
            cell_rows = []
            for gamma in gammas:
                g_df     = slow_env[slow_env["gamma"] == gamma]
                shared   = sorted(set(g_df["seed"]) & set(nav_by_seed.index))
                if len(shared) < 3:
                    continue
                base_vals  = nav_by_seed.loc[shared, metric].values.astype(float)
                treat_vals = g_df.set_index("seed").loc[shared, metric].values.astype(float)
                st = paired_stats(base_vals, treat_vals, rng)
                if not st:
                    continue
                row = {"env": env, "env_label": ENV_LABELS.get(env, env),
                       "gamma": gamma, "metric": metric, **st}
                cell_rows.append(row)

            if not cell_rows:
                continue

            # Holm correction on Wilcoxon p-values (primary test)
            pvals = np.array([r["p_value_wilcoxon"] for r in cell_rows])
            nan_mask = np.isnan(pvals)
            corr = np.full(len(pvals), np.nan)
            if (~nan_mask).sum() > 0:
                corr[~nan_mask] = holm_correct(pvals[~nan_mask])
            for r, c in zip(cell_rows, corr):
                r["p_value_holm"] = float(c)

            # Direction-of-improvement flags
            impdir = METRICS[metric]["improvement"]
            for r in cell_rows:
                if impdir == "lower":
                    improved = r["median_delta"] < 0
                else:
                    improved = r["median_delta"] > 0
                sig = (not math.isnan(r["p_value_holm"])
                       and r["p_value_holm"] < 0.05
                       and improved)
                r["improvement_direction"] = impdir
                r["significant_improvement_holm"] = sig

            rows.extend(cell_rows)

    return rows


# ── Figures ───────────────────────────────────────────────────────────────────

def _save_fig(fig: plt.Figure, dirs: dict, stem: str):
    for ext in ("png", "pdf"):
        fig.savefig(dirs["figures"] / f"{stem}.{ext}",
                    dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_diagnostic_histograms(records: list[dict], stats_rows: list[dict],
                               dirs: dict):
    """Histograms of paired (treatment - baseline) delta for diagnostic gammas."""
    import pandas as pd

    df   = pd.DataFrame(records)
    nav  = df[df["algo"] == "nav"].set_index(["env", "seed"])
    slow = df[df["algo"] == "slow"]

    diag = [g for g in DIAG_GAMMAS
            if g in slow["gamma"].unique()]
    if not diag:
        return

    for metric in METRIC_ORDER:
        if metric not in df.columns:
            continue
        fig, axes = plt.subplots(
            len(diag), len(ENV_ORDER),
            figsize=(4 * len(ENV_ORDER), 3.2 * len(diag)),
            squeeze=False,
        )
        fig.suptitle(
            f"Paired Δ histograms — {METRICS[metric]['label']}\n"
            f"(treatment − baseline, t∈[{int(WINDOW_START)},{int(WINDOW_END)}] s)",
            fontsize=12, fontweight="bold",
        )
        for ri, gamma in enumerate(diag):
            for ci, env in enumerate(ENV_ORDER):
                ax = axes[ri][ci]
                g_df = slow[(slow["gamma"] == gamma) & (slow["env"] == env)]
                shared = sorted(
                    set(g_df["seed"])
                    & set(nav.xs(env, level="env").index
                          if env in nav.index.get_level_values("env")
                          else [])
                )
                if len(shared) < 3:
                    ax.text(0.5, 0.5, "no data", ha="center", va="center",
                            transform=ax.transAxes, fontsize=9)
                    ax.set_title(f"{ENV_LABELS.get(env,env)}\nγ={gamma:g}", fontsize=8)
                    continue
                base  = nav.xs(env, level="env").loc[shared, metric].values.astype(float)
                treat = g_df.set_index("seed").loc[shared, metric].values.astype(float)
                delta = treat - base
                ax.hist(delta, bins=min(15, max(5, len(delta)//3)),
                        color="#2d708e", alpha=0.75, edgecolor="white", linewidth=0.5)
                ax.axvline(0, color="black", linewidth=1.2, linestyle="--")
                ax.axvline(float(np.median(delta)), color="#e07b00",
                           linewidth=1.2, linestyle="-", label=f"median={np.median(delta):.2f}")
                ax.set_title(f"{ENV_LABELS.get(env,env)}\nγ={gamma:g}  n={len(delta)}",
                             fontsize=8)
                ax.set_xlabel("Δ", fontsize=8)
                ax.set_ylabel("Count", fontsize=8)
                ax.tick_params(labelsize=7)
                ax.legend(fontsize=7)

        fig.tight_layout()
        _save_fig(fig, dirs, f"diag_hist_{metric}")


def fig_diagnostic_qq(records: list[dict], dirs: dict):
    """Q-Q plots of paired delta for diagnostic gammas."""
    import pandas as pd

    df   = pd.DataFrame(records)
    nav  = df[df["algo"] == "nav"].set_index(["env", "seed"])
    slow = df[df["algo"] == "slow"]

    diag = [g for g in DIAG_GAMMAS if g in slow["gamma"].unique()]
    if not diag:
        return

    for metric in METRIC_ORDER:
        if metric not in df.columns:
            continue
        fig, axes = plt.subplots(
            len(diag), len(ENV_ORDER),
            figsize=(4 * len(ENV_ORDER), 3.2 * len(diag)),
            squeeze=False,
        )
        fig.suptitle(
            f"Q-Q plots of paired Δ — {METRICS[metric]['label']}",
            fontsize=12, fontweight="bold",
        )
        for ri, gamma in enumerate(diag):
            for ci, env in enumerate(ENV_ORDER):
                ax = axes[ri][ci]
                g_df = slow[(slow["gamma"] == gamma) & (slow["env"] == env)]
                shared = sorted(
                    set(g_df["seed"])
                    & set(nav.xs(env, level="env").index
                          if env in nav.index.get_level_values("env")
                          else [])
                )
                if len(shared) < 3:
                    ax.text(0.5, 0.5, "no data", ha="center", va="center",
                            transform=ax.transAxes, fontsize=9)
                    ax.set_title(f"{ENV_LABELS.get(env,env)}\nγ={gamma:g}", fontsize=8)
                    continue
                base  = nav.xs(env, level="env").loc[shared, metric].values.astype(float)
                treat = g_df.set_index("seed").loc[shared, metric].values.astype(float)
                delta = treat - base
                (osm, osr), (slope, intercept, r) = sp_stats.probplot(delta, dist="norm")
                ax.plot(osm, osr, "o", markersize=3, color="#2d708e", alpha=0.7)
                ax.plot(osm, slope * np.array(osm) + intercept,
                        "r-", linewidth=1.2, label=f"r={r:.2f}")
                ax.set_title(f"{ENV_LABELS.get(env,env)}\nγ={gamma:g}  n={len(delta)}",
                             fontsize=8)
                ax.set_xlabel("Theoretical quantiles", fontsize=7)
                ax.set_ylabel("Sample quantiles", fontsize=7)
                ax.tick_params(labelsize=7)
                ax.legend(fontsize=7)

        fig.tight_layout()
        _save_fig(fig, dirs, f"diag_qq_{metric}")


def fig_median_delta(stats_rows: list[dict], dirs: dict):
    """Median delta vs gamma, faceted by environment, one figure per metric."""
    import pandas as pd

    sdf = pd.DataFrame(stats_rows)
    if sdf.empty:
        return

    for metric in METRIC_ORDER:
        mdf = sdf[sdf["metric"] == metric].copy()
        if mdf.empty:
            continue

        fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharey=False)
        fig.suptitle(
            f"Median Δ vs γ — {METRICS[metric]['label']}\n"
            f"(treatment − baseline; error bars = IQR)",
            fontsize=12, fontweight="bold",
        )
        envs = [e for e in ENV_ORDER if e in mdf["env"].values]
        for ax, env in zip(axes.flat, envs):
            edf = mdf[mdf["env"] == env].sort_values("gamma")
            if edf.empty:
                ax.set_visible(False)
                continue
            gammas  = edf["gamma"].values
            med_d   = edf["median_delta"].values
            iqr_d   = edf["iqr_delta"].values
            sig     = edf["significant_improvement_holm"].values

            colors = ["#c0392b" if s else "#95a5a6" for s in sig]
            ax.axhline(0, color="black", linewidth=1.0, linestyle="--", alpha=0.6)
            ax.errorbar(range(len(gammas)), med_d, yerr=iqr_d / 2,
                        fmt="none", ecolor="#7f8c8d", capsize=3, linewidth=0.8, zorder=2)
            ax.scatter(range(len(gammas)), med_d, c=colors, s=55, zorder=3)
            ax.set_xticks(range(len(gammas)))
            ax.set_xticklabels([f"{g:g}" for g in gammas],
                               rotation=45, ha="right", fontsize=7)
            ax.set_xlabel("γ", fontsize=9)
            ax.set_ylabel("Median Δ", fontsize=9)
            ax.set_title(ENV_LABELS.get(env, env), fontsize=10)
            ax.grid(True, alpha=0.3, axis="y")
            ax.tick_params(labelsize=8)

        # Legend
        from matplotlib.lines import Line2D
        handles = [
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#c0392b",
                   markersize=8, label="significant improvement (Holm)"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#95a5a6",
                   markersize=8, label="not significant"),
        ]
        fig.legend(handles=handles, loc="lower center", ncol=2,
                   bbox_to_anchor=(0.5, 0.01), fontsize=9)
        fig.tight_layout(rect=[0, 0.06, 1, 1])
        _save_fig(fig, dirs, f"median_delta_{metric}")


def fig_pvalue_plot(stats_rows: list[dict], dirs: dict):
    """Corrected p-value vs gamma, one figure per metric."""
    import pandas as pd

    sdf = pd.DataFrame(stats_rows)
    if sdf.empty:
        return

    for metric in METRIC_ORDER:
        mdf = sdf[sdf["metric"] == metric].copy()
        if mdf.empty:
            continue

        fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharey=False)
        fig.suptitle(
            f"Holm-corrected p-value vs γ — {METRICS[metric]['label']}\n"
            "(Wilcoxon signed-rank; log-scale y)",
            fontsize=12, fontweight="bold",
        )
        envs = [e for e in ENV_ORDER if e in mdf["env"].values]
        for ax, env in zip(axes.flat, envs):
            edf = mdf[mdf["env"] == env].sort_values("gamma")
            if edf.empty:
                ax.set_visible(False)
                continue
            gammas = edf["gamma"].values
            pvals  = edf["p_value_holm"].values
            safe_p = np.where(np.isnan(pvals), 1.0, np.clip(pvals, 1e-4, 1.0))
            colors = ["#c0392b" if p < 0.05 else "#7f8c8d" for p in safe_p]
            ax.scatter(range(len(gammas)), safe_p, c=colors, s=55, zorder=3)
            ax.axhline(0.05, color="black", linewidth=1.2, linestyle="--",
                       label="α = 0.05")
            ax.set_yscale("log")
            ax.set_ylim(5e-5, 2.0)
            ax.set_xticks(range(len(gammas)))
            ax.set_xticklabels([f"{g:g}" for g in gammas],
                               rotation=45, ha="right", fontsize=7)
            ax.set_xlabel("γ", fontsize=9)
            ax.set_ylabel("p (Holm-corrected)", fontsize=9)
            ax.set_title(ENV_LABELS.get(env, env), fontsize=10)
            ax.grid(True, alpha=0.3, axis="y")
            ax.legend(fontsize=8)
            ax.tick_params(labelsize=8)

        fig.tight_layout()
        _save_fig(fig, dirs, f"pvalue_{metric}")


def fig_tradeoff(stats_rows: list[dict], dirs: dict):
    """Pairwise 2-D trade-off scatter: median delta pairs across gamma."""
    import pandas as pd

    sdf = pd.DataFrame(stats_rows)
    if sdf.empty:
        return

    pairs = [
        ("area",         "polarisation",  "Δ Area (m²)",         "Δ Polarisation"),
        ("area",         "collisions",    "Δ Area (m²)",         "Δ Collisions"),
        ("polarisation", "collisions",    "Δ Polarisation",      "Δ Collisions"),
    ]

    for env in ENV_ORDER:
        edf = sdf[sdf["env"] == env]
        if edf.empty:
            continue

        # Pivot to wide: gamma -> {metric: median_delta}
        pivot = (edf[["gamma", "metric", "median_delta"]]
                 .pivot(index="gamma", columns="metric", values="median_delta")
                 .reset_index())

        if pivot.empty:
            continue

        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle(
            f"Metric trade-offs — {ENV_LABELS.get(env, env)}\n"
            "Median Δ (treatment − baseline) per γ",
            fontsize=11, fontweight="bold",
        )

        gammas = pivot["gamma"].values
        cmap   = plt.cm.viridis  # type: ignore[attr-defined]
        norm   = plt.Normalize(gammas.min(), gammas.max())
        sc_colors = cmap(norm(gammas))

        for ax, (mx, my, lx, ly) in zip(axes, pairs):
            if mx not in pivot.columns or my not in pivot.columns:
                ax.set_visible(False)
                continue
            xv = pivot[mx].values.astype(float)
            yv = pivot[my].values.astype(float)
            sc = ax.scatter(xv, yv, c=gammas, cmap=cmap, norm=norm,
                            s=60, zorder=3)
            ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
            ax.axvline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
            # Label diagnostic gammas
            for g, x, y in zip(gammas, xv, yv):
                if g in DIAG_GAMMAS:
                    ax.annotate(f"γ={g:g}", (x, y),
                                textcoords="offset points", xytext=(4, 4),
                                fontsize=7, color="#333333")
            ax.set_xlabel(lx, fontsize=9)
            ax.set_ylabel(ly, fontsize=9)
            ax.set_title(f"{lx} vs {ly}", fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            fig.colorbar(sc, ax=ax, label="γ", fraction=0.04, pad=0.02)

        fig.tight_layout()
        _save_fig(fig, dirs, f"tradeoff_{env}")


# ── Tables ────────────────────────────────────────────────────────────────────

def save_tables(stats_rows: list[dict], dirs: dict) -> dict[str, Path]:
    import pandas as pd

    sdf = pd.DataFrame(stats_rows)
    paths = {}

    # 1. Full pairwise stats
    p1_csv = dirs["tables"] / "pairwise_stats_all.csv"
    sdf.to_csv(p1_csv, index=False, float_format="%.6g")
    paths["pairwise_csv"] = p1_csv

    p1_json = dirs["tables"] / "pairwise_stats_all.json"
    sdf.to_json(p1_json, orient="records", indent=2, double_precision=6)
    paths["pairwise_json"] = p1_json

    # 2. Significant improvements only
    sig_df  = sdf[sdf["significant_improvement_holm"] == True]
    p2_csv  = dirs["tables"] / "significant_improvements.csv"
    sig_df.to_csv(p2_csv, index=False, float_format="%.6g")
    paths["significant_csv"] = p2_csv

    # 3. Best gamma per (env, metric)
    best_rows = []
    for env in ENV_ORDER:
        for metric in METRIC_ORDER:
            sub = sdf[(sdf["env"] == env) & (sdf["metric"] == metric)]
            if sub.empty:
                continue
            sig_sub = sub[sub["significant_improvement_holm"] == True]
            pool = sig_sub if not sig_sub.empty else sub
            impdir = METRICS[metric]["improvement"]
            if impdir == "lower":
                best_idx = pool["median_delta"].idxmin()
            else:
                best_idx = pool["median_delta"].idxmax()
            best_row = pool.loc[best_idx].to_dict()
            best_rows.append({
                "env": env, "env_label": ENV_LABELS.get(env, env),
                "metric": metric, "best_gamma": best_row["gamma"],
                "median_delta": best_row["median_delta"],
                "p_value_holm": best_row["p_value_holm"],
                "significant": best_row["significant_improvement_holm"],
                "cohens_dz": best_row.get("cohens_dz", np.nan),
                "n": best_row.get("n", np.nan),
            })
    p3_csv = dirs["tables"] / "best_gamma_by_metric_environment.csv"
    pd.DataFrame(best_rows).to_csv(p3_csv, index=False, float_format="%.6g")
    paths["best_gamma_csv"] = p3_csv

    return paths


# ── Overall gamma ranking ─────────────────────────────────────────────────────

def rank_gammas(stats_rows: list[dict]) -> list[dict]:
    """Score each gamma by net significant improvements across all env×metric cells."""
    import pandas as pd

    sdf = pd.DataFrame(stats_rows)
    if sdf.empty:
        return []

    scores: dict[float, dict] = {}
    gammas = sorted(sdf["gamma"].unique())
    for gamma in gammas:
        gdf = sdf[sdf["gamma"] == gamma]
        n_sig_impr  = int(gdf["significant_improvement_holm"].sum())
        # Count significant degradations (significant + wrong direction)
        impdir_map  = {m: METRICS[m]["improvement"] for m in METRIC_ORDER}
        n_sig_degrad = 0
        for _, row in gdf.iterrows():
            impdir = impdir_map.get(row["metric"], "lower")
            if impdir == "lower":
                wrong_dir = row["median_delta"] > 0
            else:
                wrong_dir = row["median_delta"] < 0
            if (not math.isnan(row["p_value_holm"])
                    and row["p_value_holm"] < 0.05
                    and wrong_dir):
                n_sig_degrad += 1
        scores[gamma] = {
            "gamma": gamma,
            "n_sig_improvements": n_sig_impr,
            "n_sig_degradations": n_sig_degrad,
            "net_score": n_sig_impr - n_sig_degrad,
            "n_cells": len(gdf),
        }

    return sorted(scores.values(), key=lambda x: x["net_score"], reverse=True)


# ── Markdown report ───────────────────────────────────────────────────────────

def write_report(records: list[dict], stats_rows: list[dict],
                 ranking: list[dict], table_paths: dict,
                 dirs: dict, input_dirs: list[str]):
    import pandas as pd

    sdf = pd.DataFrame(stats_rows) if stats_rows else pd.DataFrame()
    df  = pd.DataFrame(records)

    n_seeds = int(df["seed"].nunique()) if not df.empty else 0
    n_nav   = int((df["algo"] == "nav").sum()) if not df.empty else 0
    n_slow  = int((df["algo"] == "slow").sum()) if not df.empty else 0
    envs_found = [e for e in ENV_ORDER
                  if not df.empty and e in df["env"].values]
    gammas_found = sorted(df[df["algo"] == "slow"]["gamma"].unique().tolist()
                          if not df.empty else [])

    # Normality diagnostics (aggregate Shapiro-Wilk across cells)
    norm_notes: dict[str, str] = {}
    for metric in METRIC_ORDER:
        if sdf.empty or "sw_pvalue" not in sdf.columns:
            norm_notes[metric] = "n/a"
            continue
        sub = sdf[sdf["metric"] == metric]["sw_pvalue"].dropna()
        if sub.empty:
            norm_notes[metric] = "n/a"
            continue
        frac_non_normal = float((sub < 0.05).mean())
        if frac_non_normal > 0.5:
            norm_notes[metric] = (f"non-normal in {frac_non_normal:.0%} of cells "
                                  f"— prefer Wilcoxon / permutation tests")
        else:
            norm_notes[metric] = (f"approximately normal in {1-frac_non_normal:.0%} "
                                  f"of cells")

    # Skewness notes
    skew_notes: dict[str, str] = {}
    for metric in METRIC_ORDER:
        if sdf.empty or "skew_delta" not in sdf.columns:
            skew_notes[metric] = "n/a"
            continue
        sub = sdf[sdf["metric"] == metric]["skew_delta"].dropna()
        if sub.empty:
            skew_notes[metric] = "n/a"
            continue
        med_skew = float(sub.median())
        if abs(med_skew) > 0.5:
            skew_notes[metric] = f"median skewness = {med_skew:.2f} (skewed)"
        else:
            skew_notes[metric] = f"median skewness = {med_skew:.2f} (approximately symmetric)"

    # Best gammas for table
    top3 = ranking[:3] if ranking else []

    # Is gamma=0.01 in the top?
    g001_rank = next((i + 1 for i, r in enumerate(ranking) if r["gamma"] == 0.01), None)
    g001_row  = next((r for r in stats_rows
                      if r.get("gamma") == 0.01
                      and r.get("significant_improvement_holm")), None)

    lines = [
        "# Speed-Weighted Flocking Analysis",
        "",
        f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  ",
        f"**Analysis folder:** `{dirs['base'].relative_to(REPO_ROOT)}`",
        "",
        "---",
        "",
        "## 1. Naming Note",
        "",
        ("The speed-weighting exponent was renamed from `p` to `gamma` "
         "to avoid confusion with statistical p-values.  "),
        "The controller weight is:  ",
        "",
        "```",
        "w_ij = k / (v_j + eps)^gamma",
        "```",
        "",
        "| Symbol | Meaning |",
        "|--------|---------|",
        "| k      | linear gain (= 1 in this sweep) |",
        "| gamma  | speed-weighting exponent |",
        "| eps    | regulariser (= 0.1) |",
        "",
        "Interpretation:",
        "- **gamma < 0**: fast-neighbour following",
        "- **gamma = 0**: uniform Couzin-like weighting",
        "- **gamma > 0**: slow-neighbour anchoring",
        "",
        "---",
        "",
        "## 2. Input Data",
        "",
        "**Result directories scanned:**",
    ]
    for d in input_dirs:
        lines.append(f"- `{d}`")
    lines += [
        "",
        f"**Nav (baseline) records loaded:** {n_nav}  ",
        f"**Slow (treatment) records loaded:** {n_slow}  ",
        f"**Unique seeds found:** {n_seeds}  ",
        f"**Environments found:** {', '.join(ENV_LABELS.get(e, e) for e in envs_found)}  ",
        f"**Gamma values found:** {', '.join(f'{g:g}' for g in gammas_found)}  ",
        "",
        "**Metric column mapping:**",
        "",
        "| Analysis metric | CSV column | Better direction |",
        "|-----------------|------------|------------------|",
    ]
    for mkey, minfo in METRICS.items():
        lines.append(f"| {minfo['label']} | `{minfo['col']}` | {minfo['improvement']} |")

    lines += [
        "",
        "---",
        "",
        "## 3. Statistical Approach",
        "",
        ("Per-seed steady-state means are used as the unit of analysis "
         f"(window t ∈ [{int(WINDOW_START)}, {int(WINDOW_END)}] s). "
         "Each seed contributes one summary value per configuration. "
         "This avoids treating correlated timesteps as independent samples."),
        "",
        "**Paired differences** are computed as:",
        "",
        "```",
        "delta_{gamma,s} = metric_{gamma,s} - metric_{baseline,s}",
        "```",
        "",
        "for each seed *s* matched between treatment and baseline.",
        "",
        "**Tests run per (env, gamma, metric):**",
        "1. Paired t-test",
        "2. Wilcoxon signed-rank test (primary)",
        "3. Paired permutation test (sign-flip, 4999 resamples, seed=42)",
        "",
        "**Multiple-comparison correction:** Holm–Bonferroni across gamma values "
        "within each (env, metric) pair.",
        "",
        "**Effect size:** Cohen's d_z = mean(delta) / std(delta)",
        "",
        "---",
        "",
        "## 4. Distribution Diagnostics",
        "",
    ]
    for metric in METRIC_ORDER:
        lines.append(f"**{METRICS[metric]['label']}**  ")
        lines.append(f"- Normality: {norm_notes.get(metric, 'n/a')}  ")
        lines.append(f"- Symmetry:  {skew_notes.get(metric, 'n/a')}  ")
        lines.append("")

    lines += [
        "**Recommendation:** Use Wilcoxon signed-rank and permutation tests "
        "as primary evidence wherever normality is not confirmed. "
        "The paired t-test is included for reference.",
        "",
        "---",
        "",
        "## 5. Best Gamma Candidates",
        "",
        "Gammas ranked by net significant improvements "
        "(Holm-corrected p < 0.05 + correct direction) minus significant degradations.",
        "",
        "| Rank | γ | Net score | Sig. improvements | Sig. degradations |",
        "|------|---|-----------|-------------------|-------------------|",
    ]
    for i, r in enumerate(ranking[:10], 1):
        lines.append(
            f"| {i} | {r['gamma']:g} | {r['net_score']} "
            f"| {r['n_sig_improvements']} / {r['n_cells']} "
            f"| {r['n_sig_degradations']} |"
        )

    lines += [
        "",
        "---",
        "",
        "## 6. Status of γ = 0.01",
        "",
    ]
    if g001_rank is not None:
        lines.append(f"γ = 0.01 ranks **{g001_rank}** overall by net score.  ")
    else:
        lines.append("γ = 0.01 was not found in the data or had no scored cells.  ")
    if g001_row:
        lines.append(
            f"It shows at least one significant improvement "
            f"(env={g001_row['env']}, metric={g001_row['metric']}, "
            f"p_holm={g001_row['p_value_holm']:.3f}).  "
        )
    else:
        lines.append("γ = 0.01 shows no significant improvements after Holm correction.  ")

    lines += [
        "",
        "---",
        "",
        "## 7. Interpretation",
        "",
        "- **Negative gamma (fast-neighbour following):** Agents weight faster "
        "neighbours more. This can improve cohesion if faster agents are better "
        "aligned, but may increase collision risk by encouraging speed disparities.",
        "- **gamma ≈ 0 (uniform):** Equivalent to standard Couzin flocking. "
        "Provides a natural reference.",
        "- **Small positive gamma (slow-neighbour anchoring):** Stabilises "
        "cohesion by drawing the swarm toward its slower members. Moderate "
        "positive gamma likely improves flock area and collision safety.",
        "- **Large positive gamma:** Strong preference for the slowest neighbour "
        "may over-stabilise and reduce alignment or increase spread in certain "
        "wind conditions.",
        "",
        "---",
        "",
        "## 8. Output Files",
        "",
        "| File | Description |",
        "|------|-------------|",
    ]
    for stem, p in table_paths.items():
        lines.append(f"| `{p.relative_to(dirs['base'])}` | {stem} |")
    lines += [
        "",
        "Figures saved as both PNG (150 dpi) and PDF in `figures/`.",
        "",
    ]

    report_path = dirs["base"] / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"=== Speed-weight (gamma) statistical analysis ===")
    print(f"  Output timestamp : {TS}")

    dirs = setup_output_dirs()
    print(f"  Output folder    : {dirs['base']}")

    # ── 1. Load data ──────────────────────────────────────────────────────────
    print("\nScanning result directories …", flush=True)
    records = load_all_data()
    if not records:
        print("ERROR: No CSV data found. Check RESULTS_BASE path.")
        return

    import pandas as pd
    df = pd.DataFrame(records)
    n_nav  = int((df["algo"] == "nav").sum())
    n_slow = int((df["algo"] == "slow").sum())
    gammas = sorted(df[df["algo"] == "slow"]["gamma"].unique())
    envs   = [e for e in ENV_ORDER if e in df["env"].values]
    print(f"  nav  records : {n_nav}")
    print(f"  slow records : {n_slow}")
    print(f"  Environments : {envs}")
    print(f"  Gamma values : {gammas}")

    # ── 2. Paired statistics ──────────────────────────────────────────────────
    print("\nComputing paired statistics …", flush=True)
    stats_rows = build_stats_table(records)
    print(f"  Stats rows   : {len(stats_rows)}")

    # ── 3. Figures ────────────────────────────────────────────────────────────
    print("\nGenerating figures …", flush=True)
    fig_diagnostic_histograms(records, stats_rows, dirs)
    print("  ✓ diagnostic histograms")
    fig_diagnostic_qq(records, dirs)
    print("  ✓ Q-Q plots")
    fig_median_delta(stats_rows, dirs)
    print("  ✓ median delta vs gamma")
    fig_pvalue_plot(stats_rows, dirs)
    print("  ✓ p-value plots")
    fig_tradeoff(stats_rows, dirs)
    print("  ✓ trade-off scatter plots")

    # ── 4. Tables ─────────────────────────────────────────────────────────────
    print("\nSaving tables …", flush=True)
    table_paths = save_tables(stats_rows, dirs)
    for k, p in table_paths.items():
        print(f"  {p.relative_to(REPO_ROOT)}")

    # ── 5. Ranking ────────────────────────────────────────────────────────────
    ranking = rank_gammas(stats_rows)

    # ── 6. Report ─────────────────────────────────────────────────────────────
    input_dirs = [str(d.relative_to(REPO_ROOT))
                  for d in sorted(RESULTS_BASE.glob("flock_sweep_*"))
                  if any(d.glob("*.csv"))]
    report_path = write_report(records, stats_rows, ranking, table_paths,
                               dirs, input_dirs)
    print(f"\nReport: {report_path.relative_to(REPO_ROOT)}")

    # ── 7. Console summary ────────────────────────────────────────────────────
    print("\n── Top gamma candidates ──────────────────────────────────────────")
    for r in ranking[:5]:
        print(f"  γ={r['gamma']:6g}  net={r['net_score']:3d}  "
              f"sig_impr={r['n_sig_improvements']}  "
              f"sig_degrad={r['n_sig_degradations']}")

    g001 = next((r for r in ranking if r["gamma"] == 0.01), None)
    if g001:
        rank_idx = ranking.index(g001) + 1
        print(f"\n  γ=0.01 rank: {rank_idx} / {len(ranking)}  "
              f"(net_score={g001['net_score']})")
    else:
        print("\n  γ=0.01: not found in results")

    print(f"\n=== Done. Results in: {dirs['base']} ===")


if __name__ == "__main__":
    main()
