# Flocking Results

Outputs from `SwarmSwIM/flocking_nav.py` — Couzin-zone flocking behavior.

## Behavior

Agents follow three priority zones:
- **ZOR** (zone of repulsion): agents steer away from neighbors that are too close
- **ZOO** (zone of orientation): agents align heading with neighbors in range
- **ZOA** (zone of attraction): agents move toward distant neighbors

## Metrics (per-run CSV columns)

| Column | Description |
|--------|-------------|
| `time` | Simulation time [s] |
| `area` | Convex hull area of the swarm [m²] |
| `polarization` | Mean heading alignment [0–1, 1 = fully aligned] |
| `speed_spread` | Std dev of agent speeds |
| `avg_wind_speed` | Mean wind speed across agents [m/s] |

## Folder naming

| Folder | Agents | Config |
|--------|--------|--------|
| `experiment1/` – `experiment5/` | baseline | original parameter set |
| `10experiment1/` | 10 | ZOR=2.0, ZOO=10.0, domain=35 |
| `20experiment1/` | 20 | same parameters |
| `30experiment1/` | 30 | same parameters |
| `40experiment1/` | 40 | same parameters |

Each folder contains `run101.csv` … `run110.csv` (10 seeds) and corresponding PNG/PDF plots.

## Reproduce

```bash
bash scripts/RunFlocking.sh    # 10-agent, 10 seeds → results/flocking/10experiment1/
bash scripts/ScriptFlocking.sh # full parameter sweep
```
