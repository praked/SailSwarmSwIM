"""
compare_flocking_vs_ang_slow.py
================================
Environment × inverse-speed-weight p-sweep, parallelised. (k held at 1.)

  - 4 wind environments  (5 m/s & 10 m/s, with and without gusts)
  - 1 flocking_nav baseline + 14 flocking_ang_slow variants
        SWARM_SPD_WT_K = 1.0 (fixed)
        SWARM_SPD_WT_P ∈ {-0.1, -0.001, 0.001, 0.005, 0.01, 0.05, 0.1,
                          0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0}
        Negative p flips the weighting toward FAST neighbours — testing
        the "fast = locally-validated heading" hypothesis.
  - N_RUNS=50 seeds per (env × config)
  - Total: 4 × (1 + 14) × 50 = 3 000 simulations
  - Reuses CSVs from prior 20260509_* and 20260510_* folders for matching
    cells. Only p ∈ {-0.1, -0.001} are new → ~400 sims actually run.

Steady-state stats use the window  t ∈ [WINDOW_START_S, WINDOW_END_S]
(default 100 → T_MAX). The per-env time-series plots also clip their x-axis
to that window — only the steady-state portion is shown.

Collision counter uses SWARM_NEAR_RADIUS = 1.0 m (tighter than the default
2.0 m = SWARM_CA_SAFE_RADIUS / 2).

Outputs go into  results/comparison/flock_sweep_<ts>/  with:
  env_<env_tag>_<ts>.png             — 4 per-env figures (7 algos overlaid, 4 metrics)
  env_sweep_boxplot_<ts>.png         — tick-pooled cross-env box plot summary
  env_sweep_boxplot_perseed_<ts>.png — per-seed-mean cross-env box plot summary
  env_wind_sanity_<ts>.png           — wind plumbing verification
  _xmls_<ts>/                        — generated environment XMLs
  *.csv                              — raw run-level metrics

Usage:
    python3 compare_flocking_vs_ang_slow.py

Env vars:
    N_RUNS              — replicates per (env × config) (default 5)
    SWARM_T_MAX         — sim duration s (default 300)
    BASE_SEED           — first seed (default 42)
    WINDOW_START_S      — drop transient before this time (default 100)
    WINDOW_END_S        — end of stats window (default = T_MAX)
    MAX_WORKERS         — parallel sims (default min(10, cpu_count - 2))
"""

import os
import re
import sys
import csv
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Config ───────────────────────────────────────────────────────────────────

N_RUNS         = int(os.environ.get("N_RUNS",         "50"))
T_MAX          = os.environ.get("SWARM_T_MAX",        "300")
T_MAX_F        = float(T_MAX)
BASE_SEED      = int(os.environ.get("BASE_SEED",      "42"))
SEEDS          = [BASE_SEED + i for i in range(N_RUNS)]
WINDOW_START_S = float(os.environ.get("WINDOW_START_S", "100.0"))
WINDOW_END_S   = float(os.environ.get("WINDOW_END_S",   str(T_MAX_F)))
# Cap workers at 10 — diminishing returns beyond that for this workload.
MAX_WORKERS    = int(os.environ.get(
    "MAX_WORKERS",
    str(min(10, max(1, (os.cpu_count() or 4) - 2))),
))

# Paths
REPO_ROOT      = Path(__file__).resolve().parent
TEMPLATE_XML   = REPO_ROOT / "SwarmSwIM" / "sail_extension" / "regatta.xml"
SAIL_DIR       = TEMPLATE_XML.parent  # generated XMLs must live here so the relative
                                      # <description>sail_agent_default.xml</description>
                                      # path resolves at parse time.
COMPARISON_DIR = REPO_ROOT / "results" / "comparison"
TIMESTAMP      = datetime.now().strftime("%Y%m%d_%H%M%S")
RESULTS_DIR    = COMPARISON_DIR / f"flock_sweep_{TIMESTAMP}"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
XML_OUT_DIR    = RESULTS_DIR / f"_xmls_{TIMESTAMP}"
XML_OUT_DIR.mkdir(parents=True, exist_ok=True)

# Reuse CSVs from prior 20260509_* and 20260510_* sweeps for cells that already exist.
# RESULTS_DIR is checked first, then prior folders in reverse-chronological order.
PRIOR_SWEEP_DIRS = [
    COMPARISON_DIR / "flock_sweep_20260510_115535",  # 12-value p sweep incl. 0.001, 0.005
    COMPARISON_DIR / "flock_sweep_20260510_111416",
    COMPARISON_DIR / "flock_sweep_20260510_110934",
    COMPARISON_DIR / "flock_sweep_20260510_100255",
    COMPARISON_DIR / "flock_sweep_20260509_205846",  # k=1, 14-value p sweep, 50 seeds
    COMPARISON_DIR / "flock_sweep_20260509_170453",  # k=1, p∈{0.5,1,1.5,2,2.5,3}, 50 seeds
    COMPARISON_DIR / "flock_sweep_20260509_164345",  # k=1, p∈{0.5..3}, 5 seeds (subset of above)
    COMPARISON_DIR / "flock_sweep_20260509_112633",  # k×p extension, 50 seeds for p∈{1,2}
    COMPARISON_DIR / "flock_sweep_20260509_031323",  # original 7×2 grid, 30 seeds
]
SEARCH_DIRS: list[Path] = [RESULTS_DIR] + [d for d in PRIOR_SWEEP_DIRS if d.exists()]

# Shared params (same for both behaviors, all envs, all variants)
SHARED_PARAMS: dict[str, str] = {
    "SWARMSWIM_DOMAIN":      "30.0",
    "SWARM_COMM_RADIUS":     "20.0",
    "SWARM_TOROIDAL":        "0",
    "SWARM_ZOR":             "4.0",
    "SWARM_ZOO":             "10.0",
    "SWARM_ZOA":             "18.0",
    "SWARM_W_ORI":           "1.0",
    "SWARM_W_ATT":           "0.6",
    "SWARM_FLOCK_NOISE":     "3.0",
    "SWARM_CA_SAFE_RADIUS":  "4.0",
    "SWARM_NEAR_RADIUS":     "1.0",   # collision-counter threshold (tighter than the 2.0 default)
    "SWARM_CA_HORIZON":      "12.0",
    "SWARM_CA_BIAS_MAX":     "90.0",
    "SWARM_CA_TIE_MODE":     "colregs",
    "SWARM_CA_DWELL":        "3.0",
    "SWARM_BOUND_GAIN":      "1.2",
    "SWARM_BOUND_MARGIN":    "5.0",
    "SWARM_METRICS_PERIOD":  "1.0",
}

# 10 ang_slow variants: k=1 fixed, p sweep extended into the very-low-p tail
# (0.01, 0.05) to characterise the near-uniform-weighting regime.
K_VALUES = (1.0,)
P_VALUES = (-0.1, -0.001, 0.001, 0.005, 0.01, 0.05, 0.1, 0.3, 0.5, 0.7,
            1.0, 1.5, 2.0, 3.0)
SWEEP_CONFIGS: list[dict] = [
    {
        "label": f"k={k:g}, p={p:g}",
        "tag":   f"k{str(k).replace('.', 'p')}_p{str(p).replace('.', 'p')}",
        "env":   {"SWARM_SPD_WT_K": f"{k}", "SWARM_SPD_WT_P": f"{p}"},
        "k":     k,
        "p":     p,
    }
    for p in P_VALUES         # outer p → keeps each p block contiguous in box plots
    for k in K_VALUES
]

# 4 wind environments (skip 15 m/s for this iteration)
ENVIRONMENTS = [
    {"label": "5 m/s, no gusts",  "speed": 5.0,  "gusts": False, "tag": "w05_ng"},
    {"label": "10 m/s, no gusts", "speed": 10.0, "gusts": False, "tag": "w10_ng"},
    {"label": "5 m/s + gusts",    "speed": 5.0,  "gusts": True,  "tag": "w05_g"},
    {"label": "10 m/s + gusts",   "speed": 10.0, "gusts": True,  "tag": "w10_g"},
]


# ── XML generation ───────────────────────────────────────────────────────────

def make_env_xml(speed: float, gusts: bool, tag: str) -> Path:
    """Create a regatta.xml variant in SAIL_DIR (so sibling agent XML resolves).
    Also copy to XML_OUT_DIR for archival inspection. Return path used at runtime.
    """
    src = TEMPLATE_XML.read_text()
    src = re.sub(
        r"<wind_field>\s*[\d.]+\s+(\S+)\s+(\S+)\s+(\S+)\s*</wind_field>",
        f"<wind_field> {speed} \\1 \\2 \\3 </wind_field>",
        src,
    )
    if gusts:
        src = src.replace("<!-- <wind_gusts>", "<wind_gusts>")
        src = src.replace("</wind_gusts> -->", "</wind_gusts>")

    runtime_path = SAIL_DIR / f"_tmp_regatta_{TIMESTAMP}_{tag}.xml"
    runtime_path.write_text(src)
    shutil.copy2(runtime_path, XML_OUT_DIR / f"regatta_{tag}.xml")
    return runtime_path


def cleanup_runtime_xmls(paths: list[Path]):
    for p in paths:
        try:
            p.unlink()
        except OSError:
            pass


# ── Run helpers ──────────────────────────────────────────────────────────────

def run_behavior(module: str, metrics_file: str, seed: int,
                 extra_env: dict | None = None) -> bool:
    """Spawn a single subprocess. Top-level → pickle-safe for ProcessPoolExecutor."""
    env = {
        **os.environ,
        **SHARED_PARAMS,
        "SWARM_HEADLESS":     "1",
        "SWARM_METRICS_FILE": str(metrics_file),
        "SWARMSWIM_SEED":     str(seed),
        "SWARM_T_MAX":        T_MAX,
    }
    if extra_env:
        env.update(extra_env)
    result = subprocess.run(
        [sys.executable, "-m", module],
        env=env, capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"  [WARN] {module} seed={seed} exited {result.returncode}: "
              f"{result.stderr[-300:].strip()}", flush=True)
    return result.returncode == 0


def load_csv(path: Path) -> dict:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    cols: dict[str, np.ndarray] = {}
    for key in rows[0]:
        try:
            cols[key] = np.array([float(r[key]) for r in rows])
        except ValueError:
            cols[key] = np.array([r[key] for r in rows])
    return cols


def collate_runs(csv_paths: list[Path]) -> dict:
    """Load N CSVs and stack onto a common time grid → column → (n_runs, T) matrix."""
    runs = []
    for p in csv_paths:
        if p.exists():
            d = load_csv(p)
            if d:
                runs.append(d)
    if not runs:
        return {}
    t_min = min(len(d["time"]) for d in runs if "time" in d)
    t_ref = runs[0]["time"][:t_min]
    out: dict[str, np.ndarray] = {"time": t_ref}
    for col in ("area", "polarisation", "avg_wind_speed", "collisions"):
        rows = [np.interp(t_ref, d["time"], d[col]) for d in runs if col in d]
        if rows:
            out[col] = np.array(rows)
    return out


def _key_prefix(key: tuple) -> str:
    """CSV filename prefix for a result key (matches build_jobs naming)."""
    if key[0] == "nav":
        return f"nav_{key[1]}_seed"
    return f"slow_{key[1]}_{key[2]}_seed"


def gather_csvs_for_key(key: tuple, search_dirs: list[Path]) -> list[Path]:
    """Find every CSV matching (algo, env, variant) across all search dirs,
    de-duplicated by seed (newer dir wins over older)."""
    prefix = _key_prefix(key)
    by_seed: dict[int, Path] = {}
    for d in search_dirs:
        if not d.exists():
            continue
        for p in d.glob(f"{prefix}*.csv"):
            after_prefix = p.stem[len(prefix):]
            seed_str = after_prefix.split("_", 1)[0]
            try:
                seed = int(seed_str)
            except ValueError:
                continue
            # First search dir takes priority (RESULTS_DIR before EXISTING_SWEEP_DIR)
            by_seed.setdefault(seed, p)
    return [by_seed[s] for s in sorted(by_seed)]


def _csv_prefix(csv_filename: str) -> str:
    """Return everything before the trailing '_YYYYMMDD_HHMMSS.csv' suffix.

    Filenames look like 'slow_w05_ng_k1p0_p1p0_seed42_20260509_205846.csv'.
    The timestamp itself contains an underscore, so a single rsplit('_') is
    not enough — we strip the last TWO underscore-separated segments.
    """
    stem = csv_filename[:-4] if csv_filename.endswith(".csv") else csv_filename
    parts = stem.rsplit("_", 2)
    return parts[0] if len(parts) >= 3 else stem


def precompute_existing_prefixes(search_dirs: list[Path]) -> set:
    """One-shot scan of every CSV in every search dir, indexed by their
    (algo, env, variant, seed) prefix. Used for O(1) skip checks at job-build
    time."""
    prefixes: set[str] = set()
    for d in search_dirs:
        if not d.exists():
            continue
        for p in d.glob("*.csv"):
            prefixes.add(_csv_prefix(p.name))
    return prefixes


def csv_exists_anywhere(csv_filename: str, search_dirs: list[Path]) -> bool:
    """Slow fallback used by worker subprocesses if the precomputed set isn't
    available."""
    prefix = _csv_prefix(csv_filename)
    target = prefix + "_"
    for d in search_dirs:
        if not d.exists():
            continue
        for _ in d.glob(f"{target}*.csv"):
            return True
    return False


# ── Plotting helpers ─────────────────────────────────────────────────────────

# Single-k ramp (only k=1 in this sweep) — kept for compatibility with helpers.
SWEEP_COLORS_K = ["#999999"]
# 14-colour ramp keyed by p value (negative p = fast-bias regime in red,
# positive p = slow-bias regime in viridis cool→warm).
SWEEP_COLORS_P = [
    "#7f0000",  # p=-0.1   (fast-bias, strong)
    "#d94801",  # p=-0.001 (fast-bias, weak)
    "#440154",  # p=0.001
    "#471365",  # p=0.005
    "#482576",  # p=0.01
    "#433d84",  # p=0.05
    "#38598c",  # p=0.1
    "#2d708e",  # p=0.3
    "#25858e",  # p=0.5
    "#1f9a8a",  # p=0.7
    "#2eb37c",  # p=1.0
    "#6dcd59",  # p=1.5
    "#b4dd2c",  # p=2.0
    "#fde725",  # p=3.0
]
BASELINE_COLOR = "black"

PRETTY_PARAMS = {
    "SWARMSWIM_DOMAIN":     "domain ½-side (m)",
    "SWARM_COMM_RADIUS":    "comm radius (m)",
    "SWARM_TOROIDAL":       "toroidal",
    "SWARM_ZOR":            "ZOR repulsion (m)",
    "SWARM_ZOO":            "ZOO orientation (m)",
    "SWARM_ZOA":            "ZOA attraction (m)",
    "SWARM_W_ORI":          "w_ori",
    "SWARM_W_ATT":          "w_att",
    "SWARM_FLOCK_NOISE":    "noise σ (deg)",
    "SWARM_CA_SAFE_RADIUS": "CA safe radius (m)",
    "SWARM_NEAR_RADIUS":    "collision radius (m)",
    "SWARM_CA_HORIZON":     "CA horizon (s)",
    "SWARM_CA_BIAS_MAX":    "CA bias max (deg)",
    "SWARM_CA_TIE_MODE":    "CA tie mode",
    "SWARM_CA_DWELL":       "CA dwell (s)",
    "SWARM_BOUND_GAIN":     "boundary gain",
    "SWARM_BOUND_MARGIN":   "boundary margin (m)",
    "SWARM_METRICS_PERIOD": "metrics period (s)",
}

PANELS = [
    ("area",           "Flock Area (m²)",        "Cohesion — lower is tighter"),
    ("polarisation",   "Polarisation [0–1]",      "Alignment — higher is better"),
    ("avg_wind_speed", "Avg Wind Speed (m/s)",    "Environmental context"),
    ("collisions",     "Cumulative Collisions",   "Safety — lower is safer"),
]


def _window_mask(t: np.ndarray) -> np.ndarray:
    return (t >= WINDOW_START_S) & (t <= WINDOW_END_S)


def windowed_samples(data: dict, col: str) -> np.ndarray:
    """Flat array of all samples within [WINDOW_START_S, WINDOW_END_S]."""
    if col not in data or "time" not in data or len(data["time"]) == 0:
        return np.array([])
    t    = data["time"]
    mask = _window_mask(t)
    return data[col][:, mask].ravel()


def per_seed_means(data: dict, col: str) -> np.ndarray:
    """One number per seed: that seed's mean over the steady-state window."""
    if col not in data or "time" not in data or len(data["time"]) == 0:
        return np.array([])
    t    = data["time"]
    mask = _window_mask(t)
    sub  = data[col][:, mask]
    if sub.size == 0:
        return np.array([])
    return sub.mean(axis=1)


def steady_stat(data: dict, col: str) -> tuple[float, float]:
    """(mean, std) of windowed samples — for tables and console summary."""
    s = windowed_samples(data, col)
    if s.size == 0:
        return float("nan"), float("nan")
    return float(s.mean()), float(s.std())


# Variant colour / alpha given a SWEEP_CONFIGS entry.
# When K is fixed (length 1) and P varies, colour by p; otherwise colour by k.
def _variant_style(cfg: dict) -> tuple[str, float]:
    if len(K_VALUES) == 1 and len(P_VALUES) > 1:
        color = SWEEP_COLORS_P[P_VALUES.index(cfg["p"])]
        alpha = 0.85
    else:
        color = SWEEP_COLORS_K[K_VALUES.index(cfg["k"])]
        alpha = 0.45 if cfg["p"] == 1.0 else 0.85
    return color, alpha


def plot_per_environment(env: dict, baseline: dict,
                         variants: list[tuple[dict, dict]], out: Path):
    """Per-env time-series, x-axis clipped to [WINDOW_START_S, WINDOW_END_S]."""
    fig = plt.figure(figsize=(17, 10.5))
    fig.suptitle(
        f"Environment: {env['label']}    "
        f"(flocking_nav vs {len(SWEEP_CONFIGS)} ang_slow k×p variants)\n"
        f"{N_RUNS} runs/config · seeds {BASE_SEED}–{BASE_SEED+N_RUNS-1} · T={T_MAX}s · "
        f"window [{int(WINDOW_START_S)}, {int(WINDOW_END_S)}]s",
        fontsize=13, fontweight="bold",
    )
    gs = fig.add_gridspec(2, 2, left=0.05, right=0.66, top=0.90, bottom=0.22,
                          hspace=0.42, wspace=0.30)
    axes = np.array([[fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])],
                     [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]])

    for ax, (col, ylabel, subtitle) in zip(axes.flat, PANELS):
        ax.set_title(subtitle, fontsize=9, color="gray")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(WINDOW_START_S, WINDOW_END_S)

        if col in baseline and len(baseline.get("time", [])) > 0:
            t  = baseline["time"]
            mu = baseline[col].mean(axis=0)
            ax.plot(t, mu, color=BASELINE_COLOR, linewidth=2.4,
                    label="flocking_nav (baseline)", zorder=10)

        for cfg, data in variants:
            if col not in data or len(data.get("time", [])) == 0:
                continue
            t  = data["time"]
            mu = data[col].mean(axis=0)
            color, alpha = _variant_style(cfg)
            ax.plot(t, mu, color=color, linewidth=1.4, alpha=alpha,
                    linestyle="-", label=cfg["label"])

        ax.legend(fontsize=6, loc="best", framealpha=0.9, ncol=2)

    # Steady-state table (uses [WINDOW_START_S, WINDOW_END_S])
    rows = [["flocking_nav"] +
            [f"{steady_stat(baseline, col)[0]:.2f}±{steady_stat(baseline, col)[1]:.2f}"
             for col, *_ in PANELS]]
    for cfg, data in variants:
        rows.append([cfg["label"]] +
                    [f"{steady_stat(data, col)[0]:.2f}±{steady_stat(data, col)[1]:.2f}"
                     for col, *_ in PANELS])

    tbl_ax = fig.add_axes([0.05, 0.01, 0.61, 0.18])
    tbl_ax.axis("off")
    tbl = tbl_ax.table(
        cellText=rows,
        colLabels=["config (mean±std, t∈["
                   f"{int(WINDOW_START_S)},{int(WINDOW_END_S)}]s)"]
                  + [p[1] for p in PANELS],
        loc="center", cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(6.5)
    tbl.scale(1, 1.15)

    # Right-side parameter panel
    lines = ["── Environment ──",
             f"  label  : {env['label']}",
             f"  speed  : {env['speed']} m/s",
             f"  gusts  : {'on (amp 2.5, freq 3, dur 5, λ 50)' if env['gusts'] else 'off'}",
             "",
             "── Run setup ──",
             f"  N runs/config : {N_RUNS}",
             f"  Seeds         : {BASE_SEED}…{BASE_SEED+N_RUNS-1}",
             f"  Sim duration  : {T_MAX} s",
             f"  Stats window  : [{int(WINDOW_START_S)}, {int(WINDOW_END_S)}] s",
             "",
             "── Shared params (all configs) ──"]
    for k, v in SHARED_PARAMS.items():
        lines.append(f"  {PRETTY_PARAMS.get(k, k):<22}: {v}")
    lines += ["",
              "── ang_slow knobs (defaults) ──",
              "  SPD_EPS   : 0.1",
              "  TRIM_KP   : 0.4",
              "  TRIM_RATE : 0.05",
              "",
              f"── {len(SWEEP_CONFIGS)} ang_slow variants ──",
              f"  k ∈ {{{', '.join(f'{k:g}' for k in K_VALUES)}}}",
              f"  p ∈ {{{', '.join(f'{p:g}' for p in P_VALUES)}}}"]
    pax = fig.add_axes([0.68, 0.02, 0.30, 0.88])
    pax.axis("off")
    pax.text(0.0, 1.0, "\n".join(lines),
             family="monospace", fontsize=8,
             verticalalignment="top", horizontalalignment="left",
             bbox=dict(facecolor="#f7f7f7", edgecolor="#aaaaaa",
                       boxstyle="round,pad=0.6"))

    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved → {out.name}", flush=True)


def _draw_box_plot(all_results: dict, out: Path,
                   sample_fn, title_suffix: str):
    """Shared box-plot drawing. sample_fn(data, col) returns the array of samples
    for one (algo, env, metric) cell — pooled ticks or per-seed means."""
    fig, axes = plt.subplots(2, 2, figsize=(17, 11))
    fig.suptitle(
        f"Cross-environment box plot — {title_suffix}\n"
        f"window t∈[{int(WINDOW_START_S)}, {int(WINDOW_END_S)}] s · "
        f"{N_RUNS} runs/config · seeds {BASE_SEED}–{BASE_SEED+N_RUNS-1}",
        fontsize=13, fontweight="bold",
    )

    # Layout: 1 nav box + len(SWEEP_CONFIGS) variants per env, evenly spaced.
    # Variants are drawn in SWEEP_CONFIGS order; colour comes from _variant_style.
    n_total   = 1 + len(SWEEP_CONFIGS)
    span      = 0.85
    slot      = span / n_total
    box_width = slot * 0.85
    half      = span / 2.0
    positions = -half + slot / 2.0 + np.arange(n_total) * slot   # positions[0] = nav

    for ax, (col, ylabel, subtitle) in zip(axes.flat, PANELS):
        ax.set_title(subtitle, fontsize=9, color="gray")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3, axis="y")

        for ei, env in enumerate(ENVIRONMENTS):
            data = all_results.get(("nav", env["tag"]), {})
            samples = sample_fn(data, col)
            if samples.size > 0:
                _draw_one_box(ax, samples, ei + positions[0], box_width,
                              BASELINE_COLOR, alpha=0.65)

            for ci, cfg in enumerate(SWEEP_CONFIGS):
                data = all_results.get(("slow", env["tag"], cfg["tag"]), {})
                samples = sample_fn(data, col)
                if samples.size == 0:
                    continue
                color, alpha = _variant_style(cfg)
                _draw_one_box(ax, samples, ei + positions[1 + ci], box_width,
                              color, alpha=alpha)

        ax.set_xticks(np.arange(len(ENVIRONMENTS)))
        ax.set_xticklabels([e["label"] for e in ENVIRONMENTS],
                           rotation=15, fontsize=8, ha="right")
        ax.set_xlim(-0.5, len(ENVIRONMENTS) - 0.5)

    # Combined legend at the top
    from matplotlib.patches import Patch
    handles = [Patch(facecolor=BASELINE_COLOR, edgecolor=BASELINE_COLOR,
                     alpha=0.65, label="flocking_nav")]
    for cfg in SWEEP_CONFIGS:
        color, alpha = _variant_style(cfg)
        handles.append(Patch(facecolor=color, edgecolor=color, alpha=alpha,
                             label=cfg["label"]))
    fig.legend(handles=handles, loc="upper center",
               ncol=min(len(handles), 8),
               bbox_to_anchor=(0.5, 0.945), fontsize=8, frameon=False)

    fig.subplots_adjust(top=0.86, bottom=0.10, hspace=0.40, wspace=0.22)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved → {out.name}", flush=True)


def _draw_one_box(ax, samples, position, width, color, alpha=0.65):
    bp = ax.boxplot(
        samples,
        positions=[position],
        widths=width,
        patch_artist=True,
        showfliers=True,
        flierprops=dict(marker=".", markersize=1.5,
                        markerfacecolor=color, markeredgecolor=color,
                        alpha=0.35),
        medianprops=dict(color="black", linewidth=0.8),
        whiskerprops=dict(color=color, linewidth=0.6),
        capprops=dict(color=color, linewidth=0.6),
        boxprops=dict(linewidth=0.6),
    )
    for box in bp["boxes"]:
        box.set_facecolor(color)
        box.set_edgecolor(color)
        box.set_alpha(alpha)


def plot_summary_box_pooled(all_results: dict, out: Path):
    _draw_box_plot(all_results, out, windowed_samples,
                   "tick-pooled (every seed × every metric tick)")


def plot_summary_box_perseed(all_results: dict, out: Path):
    _draw_box_plot(all_results, out, per_seed_means,
                   "per-seed steady-state means (one number per seed)")


def plot_wind_sanity(all_results: dict, out: Path):
    """One panel: avg_wind_speed time-series, all envs overlaid."""
    fig, ax = plt.subplots(1, 1, figsize=(11, 5.5))
    ax.set_title("Wind plumbing sanity — avg wind speed per environment",
                 fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Avg Wind Speed (m/s)")
    ax.grid(True, alpha=0.3)

    palette = ["#4292c6", "#41ab5d", "#08306b", "#67000d"]

    for env, color in zip(ENVIRONMENTS, palette):
        keys = [("nav", env["tag"])] + \
               [("slow", env["tag"], cfg["tag"]) for cfg in SWEEP_CONFIGS]
        traces = []
        t_ref  = None
        for k in keys:
            d = all_results.get(k, {})
            if "avg_wind_speed" in d and len(d.get("time", [])) > 0:
                traces.append(d["avg_wind_speed"].mean(axis=0))
                if t_ref is None:
                    t_ref = d["time"]
        if not traces:
            continue
        L = min(len(tr) for tr in traces)
        traces = np.array([tr[:L] for tr in traces])
        ax.plot(t_ref[:L], traces.mean(axis=0), color=color, linewidth=1.6,
                label=env["label"])

    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved → {out.name}", flush=True)


# ── Job dispatch ─────────────────────────────────────────────────────────────

def build_jobs(env_xml_paths: dict[str, Path]) -> list[dict]:
    jobs = []
    for env in ENVIRONMENTS:
        xml_path = env_xml_paths[env["tag"]]
        common_extra = {"SWARM_SIM_XML": str(xml_path)}

        for seed in SEEDS:
            csv_path = RESULTS_DIR / f"nav_{env['tag']}_seed{seed}_{TIMESTAMP}.csv"
            jobs.append({
                "module":      "SwarmSwIM.behaviors.flocking_nav",
                "csv":         str(csv_path),
                "seed":        seed,
                "extra_env":   dict(common_extra),
                "key":         ("nav", env["tag"]),
                "desc":        f"nav  env={env['tag']}  seed={seed}",
                "search_dirs": SEARCH_DIRS,
            })

        for cfg in SWEEP_CONFIGS:
            for seed in SEEDS:
                csv_path = RESULTS_DIR / (
                    f"slow_{env['tag']}_{cfg['tag']}_seed{seed}_{TIMESTAMP}.csv"
                )
                jobs.append({
                    "module":      "SwarmSwIM.behaviors.flocking_ang_slow",
                    "csv":         str(csv_path),
                    "seed":        seed,
                    "extra_env":   {**common_extra, **cfg["env"]},
                    "key":         ("slow", env["tag"], cfg["tag"]),
                    "desc":        f"slow {env['tag']}  {cfg['label']}  seed={seed}",
                    "search_dirs": SEARCH_DIRS,
                })
    return jobs


def _job_runner(job: dict) -> tuple[dict, bool, bool]:
    """Run one sim, or skip if a matching CSV already exists in any search dir.
    Returns (job, ok, skipped)."""
    csv_filename = Path(job["csv"]).name
    if csv_exists_anywhere(csv_filename, job["search_dirs"]):
        return job, True, True
    ok = run_behavior(job["module"], job["csv"], job["seed"], job["extra_env"])
    return job, ok, False


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=== Environment × ang_slow k×p sweep (parallelised) ===")
    print(f"  Environments  : {len(ENVIRONMENTS)}")
    print(f"  ang_slow vars : {len(SWEEP_CONFIGS)}  "
          f"(k ∈ {{{', '.join(f'{k:g}' for k in K_VALUES)}}}, "
          f"p ∈ {{{', '.join(f'{p:g}' for p in P_VALUES)}}})")
    print(f"  N_RUNS        : {N_RUNS}  (seeds {SEEDS[0]}…{SEEDS[-1]})")
    print(f"  T_MAX         : {T_MAX} s   "
          f"(stats window [{WINDOW_START_S}, {WINDOW_END_S}])")
    print(f"  Workers       : {MAX_WORKERS}")
    total = len(ENVIRONMENTS) * (1 + len(SWEEP_CONFIGS)) * N_RUNS
    print(f"  Total sims    : {total}")
    print(f"  Output dir    : {RESULTS_DIR}")
    print(f"  XMLs in       : {XML_OUT_DIR}")
    print()

    # ── 1. Pre-generate all environment XMLs (single-threaded) ───────────────
    env_xml_paths: dict[str, Path] = {}
    runtime_xml_paths: list[Path] = []
    for env in ENVIRONMENTS:
        xml_path = make_env_xml(env["speed"], env["gusts"], env["tag"])
        env_xml_paths[env["tag"]] = xml_path
        runtime_xml_paths.append(xml_path)
    print(f"Generated {len(env_xml_paths)} env XMLs in {SAIL_DIR}\n")

    try:
        # ── 2. Build flat job list, then partition into to_run vs to_skip ────
        jobs = build_jobs(env_xml_paths)
        assert len(jobs) == total, (len(jobs), total)

        existing_prefixes = precompute_existing_prefixes(SEARCH_DIRS)
        to_run, to_skip = [], []
        for j in jobs:
            if _csv_prefix(Path(j["csv"]).name) in existing_prefixes:
                to_skip.append(j)
            else:
                to_run.append(j)
        print(f"── Partition: {len(to_skip)} reusable · {len(to_run)} to run ──")

        # ── 3. Dispatch only the to_run jobs in parallel ─────────────────────
        print(f"── Dispatching {len(to_run)} sims to {MAX_WORKERS} workers ──")
        new_count = 0
        completed = 0
        with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
            futures = {ex.submit(_job_runner, j): j for j in to_run}
            for fut in as_completed(futures):
                result = fut.result()
                job, ok = result[0], result[1]
                completed += 1
                tag = "[ok]  " if ok else "[fail]"
                if ok:
                    new_count += 1
                print(f"  {tag} ({completed:4d}/{len(to_run)})  {job['desc']}",
                      flush=True)
        print(f"\n  ran {new_count} new sims · "
              f"reused {len(to_skip)} pre-existing\n",
              flush=True)

        # ── 4. Collate per-key CSVs (across BOTH RESULTS_DIR + EXISTING dir) ─
        all_results: dict[tuple, dict] = {}
        for env in ENVIRONMENTS:
            key = ("nav", env["tag"])
            all_results[key] = collate_runs(gather_csvs_for_key(key, SEARCH_DIRS))
            for cfg in SWEEP_CONFIGS:
                key = ("slow", env["tag"], cfg["tag"])
                all_results[key] = collate_runs(gather_csvs_for_key(key, SEARCH_DIRS))

        # ── 5. Plot ─────────────────────────────────────────────────────────
        print("\n── Generating plots ─────────────────────────────────────────")
        for env in ENVIRONMENTS:
            baseline = all_results.get(("nav", env["tag"]), {})
            variants = [(cfg, all_results.get(("slow", env["tag"], cfg["tag"]), {}))
                        for cfg in SWEEP_CONFIGS]
            plot_per_environment(
                env, baseline, variants,
                RESULTS_DIR / f"env_{env['tag']}_{TIMESTAMP}.png",
            )

        plot_summary_box_pooled(
            all_results,
            RESULTS_DIR / f"env_sweep_boxplot_{TIMESTAMP}.png",
        )
        plot_summary_box_perseed(
            all_results,
            RESULTS_DIR / f"env_sweep_boxplot_perseed_{TIMESTAMP}.png",
        )
        plot_wind_sanity(
            all_results,
            RESULTS_DIR / f"env_wind_sanity_{TIMESTAMP}.png",
        )

        # ── 6. Console summary ───────────────────────────────────────────────
        print("\n── Steady-state summary  (mean ± std, "
              f"t∈[{int(WINDOW_START_S)},{int(WINDOW_END_S)}]s) ──")
        for env in ENVIRONMENTS:
            print(f"\n  {env['label']}:")
            rows = [(("nav", env["tag"]), "nav             ")] + [
                (("slow", env["tag"], cfg["tag"]), f"slow {cfg['label']:<14}")
                for cfg in SWEEP_CONFIGS
            ]
            for key, label_kind in rows:
                d = all_results.get(key, {})
                cells = "  ".join(
                    f"{lbl}={steady_stat(d, col)[0]:.2f}±{steady_stat(d, col)[1]:.2f}"
                    for col, lbl in [("area", "Area"), ("polarisation", "Pol"),
                                      ("avg_wind_speed", "Wind"),
                                      ("collisions", "Coll")]
                )
                print(f"    {label_kind}: {cells}")

    finally:
        cleanup_runtime_xmls(runtime_xml_paths)
        print(f"\nGenerated XMLs archived in {XML_OUT_DIR}")


if __name__ == "__main__":
    main()
