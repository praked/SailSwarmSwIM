# Aggregation Results

Outputs from `SwarmSwIM/aggregation_nav.py` — standard aggregation behavior.

## Behavior

Agents aggregate toward a shared goal location using connectivity-based clustering. Agents within a connectivity distance `DC` share pose information and vote on a common goal. The swarm converges when the largest cluster reaches a threshold fraction of the total fleet.

## Metrics (per-run CSV columns)

| Column | Description |
|--------|-------------|
| `time` | Simulation time [s] |
| `largest_cluster` | Size of the largest connected cluster [# agents] |
| `hull_area` | Normalised convex hull area of the swarm |
| `avg_wind_speed` | Mean wind speed across agents [m/s] |

## Folder naming

| Folder | Agents | Notes |
|--------|--------|-------|
| `10aggregate1/` | 10 | DC=20, domain=35, T_max=300 |
| `20aggregate1/` | 20 | same parameters |
| `40aggregate1/` | 40 | same parameters |
| `Random/` | varies | random-seed exploratory runs |

`aggregation_metrics_seed_<seed>_<timestamp>.csv` — loose CSV files from direct script runs.

## Reproduce

```bash
bash scripts/RunAggregation.sh   # → results/aggregation/Random/
```
