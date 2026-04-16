# Flocking Slow Results

Outputs from `SwarmSwIM/flocking_ang_slow.py` — angular-slow flocking variant.

## Behavior

Same Couzin-zone logic as `flocking_nav.py` but heading changes use slower angular dynamics. Agents turn more gradually, producing smoother trajectories at the cost of slower convergence to the group heading.

Useful for comparing convergence speed and group coherence against the standard flocking behavior.

## Metrics (per-run CSV columns)

Same schema as `flocking/`:

| Column | Description |
|--------|-------------|
| `time` | Simulation time [s] |
| `area` | Convex hull area of the swarm [m²] |
| `polarization` | Mean heading alignment [0–1] |
| `speed_spread` | Std dev of agent speeds |
| `avg_wind_speed` | Mean wind speed [m/s] |

## File naming

`flock_slow_metrics_seed_<seed>_<timestamp>.csv` — one file per run, named by seed and wall-clock timestamp.

## Reproduce

```bash
python -m SwarmSwIM.behaviors.flocking_ang_slow
```
