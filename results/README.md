# Results

Experiment outputs for SwarmSwIM. All folders here are gitignored — data is generated locally by running the scripts in `scripts/`.

| Subfolder | Behavior | Nav script | Run script |
|-----------|----------|-----------|------------|
| `flocking/` | Couzin-zone flocking (repulsion / orientation / attraction) | `SwarmSwIM/flocking_nav.py` | `scripts/RunFlocking.sh` |
| `flocking_slow/` | Angular-slow flocking variant (smoother heading dynamics) | `SwarmSwIM/flocking_ang_slow.py` | run directly |
| `aggregation/` | Standard aggregation toward a shared goal | `SwarmSwIM/aggregation_nav.py` | `scripts/RunAggregation.sh` |
| `aggregation_v2/` | Force-field aggregation (F_sep + F_coh + F_agg) | `SwarmSwIM/agg_nav_v2.py` | run directly |
| `misc/` | One-off tests, archived external data | — | — |

## Running experiments

All scripts must be run from the **repo root**:

```bash
bash scripts/RunFlocking.sh       # → results/flocking/
bash scripts/RunAggregation.sh    # → results/aggregation/
bash scripts/ScriptFlocking.sh    # → results/flocking/ (parameter sweep)
bash scripts/120thread.sh         # → results/flocking/ (high-parallelism sweep)
```
