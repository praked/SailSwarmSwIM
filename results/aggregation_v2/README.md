# Aggregation V2 Results

Outputs from `SwarmSwIM/agg_nav_v2.py` — behavioral force-field aggregation.

## Behavior

Three concurrent force components combine into a virtual waypoint fed to the tacking controller:

| Force | Role |
|-------|------|
| **F_sep** | Separation — repulsion from neighbors closer than a safety radius |
| **F_coh** | Cohesion — attraction toward the centroid of visible neighbors |
| **F_agg** | Goal attraction — pull toward the consensus goal position |

Goal consensus uses version-based voting: agents broadcast their current goal version and adopt the highest-version goal they hear. No dedicated `CollisionAvoidance` object is used; separation is handled inline by F_sep.

## Metrics (per-run CSV columns)

Same schema as `aggregation/`:

| Column | Description |
|--------|-------------|
| `time` | Simulation time [s] |
| `largest_cluster` | Size of the largest connected cluster [# agents] |
| `hull_area` | Normalised convex hull area |
| `avg_wind_speed` | Mean wind speed [m/s] |

## File naming

`agg_v2_metrics_seed_<seed>_<timestamp>.csv` — one file per run.

## Reproduce

```bash
python -m SwarmSwIM.behaviors.agg_nav_v2
```
