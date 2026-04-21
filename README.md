# SailSwarmSwIM

SailSwarmSwIM is a sailing-robot swarm simulator built on top of SwarmSwIM.

This fork extends the original simulator with wind-aware sailing dynamics, sailing-specific agent behavior, waypoint-based navigation, and visualization tools for multi-agent sailing scenarios.

## Overview

The repository adds a sailing extension to the SwarmSwIM framework for simulating autonomous sailboat agents in a 2D environment with wind.

It is aimed at research and experimentation on topics such as:

- sailing robot dynamics
- wind-aware navigation
- waypoint following and tacking behavior
- multi-agent sailing scenarios
- regatta-style demonstrations and experiments

## Demo

https://github.com/user-attachments/assets/78136534-73f5-4b2f-b991-e2a12eed649c

<<<<<<< HEAD

## Main sailing components
=======
Key features include:
- Wind physics simulating linear gusts/lulls and directional turbulence
- Extendable sailing force mechanics and tacking behavior
- Waypoint navigation
- Couzin-zone flocking (repulsion / orientation / attraction)
- Angular-slow flocking variant (smoother heading dynamics)
- Aggregation toward a shared goal via connectivity-based clustering
- Aggregation v2 with behavioral force fields (F_sep + F_coh + F_agg)
- CPA-based collision avoidance with CoLREGs sailing rules
- Acoustic and visual sensor models
>>>>>>> origin/main

The sailing-specific code is located in:

- `SwarmSwIM/sail_extension/physics.py`  
  Wind field generation and sailing-related physics utilities.

- `SwarmSwIM/sail_extension/sail_agent.py`  
  Sailing agent behavior, navigation logic, and waypoint handling.

- `SwarmSwIM/sail_extension/wind_plotter.py`  
  Visualization of wind, agents, and waypoints.

- `SwarmSwIM/sail_extension/regatta.xml`  
  Example sailing scenario configuration.

- `SwarmSwIM/sail_extension/sail_agent_default.xml`  
  Default sailing-agent parameters.

A runnable example is provided in:

- `SwarmSwIM/regatta_nav.py`

## Features

- Wind-aware sailing simulation
- Extendable sailing-agent mechanics
- Waypoint navigation
- Tacking-oriented behavior
- Wind-field visualization
- Multi-agent sailing scenario support

## Installation

Clone this repository and install it in editable mode:

```bash
git clone https://github.com/praked/SailSwarmSwIM.git
cd SailSwarmSwIM
pip install -e .
<<<<<<< HEAD
=======
```

The installation should conclude with:
```
Successfully built SwarmSwIM
Installing collected packages: SwarmSwIM
Successfully installed SwarmSwIM-x.x.x
```

---

## Navigation Behaviors

| Script | Behavior | Metrics CSV |
|--------|----------|-------------|
| `SwarmSwIM/behaviors/flocking_nav.py` | Couzin-zone flocking (ZOR/ZOO/ZOA) | `results/flocking/` |
| `SwarmSwIM/behaviors/flocking_ang_slow.py` | Angular-slow flocking variant | `results/flocking_slow/` |
| `SwarmSwIM/behaviors/aggregation_nav.py` | Connectivity-based aggregation | `results/aggregation/` |
| `SwarmSwIM/behaviors/agg_nav_v2.py` | Force-field aggregation (F_sep/F_coh/F_agg) | `results/aggregation_v2/` |
| `SwarmSwIM/behaviors/regatta_nav.py` | Regatta / waypoint racing | — |

Each script can be run directly from the repo root:
```bash
python -m SwarmSwIM.behaviors.flocking_nav
python -m SwarmSwIM.behaviors.aggregation_nav
```

---

## Running Experiments

Batch scripts live in `scripts/` and must be run from the **repository root**:

```bash
bash scripts/RunFlocking.sh       # 10-seed flocking experiment → results/flocking/
bash scripts/RunAggregation.sh    # aggregation experiment      → results/aggregation/
bash scripts/ScriptFlocking.sh    # full parameter sweep        → results/flocking/
bash scripts/120thread.sh         # high-parallelism sweep      → results/flocking/
```

Outputs land in `results/` (gitignored). Each subfolder has its own `README.md` describing the metrics schema and how to reproduce.

---

## Environment Variables

All parameters can be tuned via environment variables without editing code.

### Flocking (`flocking_nav.py`, `flocking_ang_slow.py`)

| Variable | Default | Description |
|----------|---------|-------------|
| `SWARM_ZOR` | `2.0` | Zone of repulsion radius [m] |
| `SWARM_ZOO` | `10.0` | Zone of orientation radius [m] |
| `SWARM_ZOA` | `18.0` | Zone of attraction radius [m] |
| `SWARM_T_MAX` | `400.0` | Simulation duration [s] |
| `SWARMSWIM_DOMAIN` | `35.0` | Arena half-size [m] |
| `SWARMSWIM_SEED` | `42` | Random seed |
| `SWARM_METRICS_FILE` | `results/flocking/...csv` | Output CSV path |
| `SWARM_HEADLESS` | `0` | Set to `1` to disable visualisation |

### Aggregation (`aggregation_nav.py`, `agg_nav_v2.py`)

| Variable | Default | Description |
|----------|---------|-------------|
| `SWARM_AGG_DC` | comm radius | Connectivity distance [m] |
| `SWARM_T_MAX` | `300.0` | Simulation duration [s] |
| `SWARM_METRICS_FILE` | `results/aggregation/...csv` | Output CSV path |

### Collision Avoidance (`sensors/collision_avoidance.py`)

| Variable | Default | Description |
|----------|---------|-------------|
| `SWARM_CA_SAFE_RADIUS` | `4.0` | CPA avoidance bubble radius [m] |
| `SWARM_CA_HORIZON` | `12.0` | CPA look-ahead time [s] |
| `SWARM_CA_BIAS_MAX` | `90.0` | Max heading bias [deg] |
| `SWARM_CA_TIE_MODE` | `colregs` | Tie-break mode: `colregs` / `deterministic` / `random` |
| `SWARM_NEAR_RADIUS` | `safe_radius/2` | Physical collision threshold [m] — used by both the collision counter and the red-agent visual indicator |

---

## Results Structure

```
results/
├── flocking/         Couzin flocking runs
├── flocking_slow/    Angular-slow variant runs
├── aggregation/      Standard aggregation runs
├── aggregation_v2/   Force-field aggregation runs
└── misc/             One-off tests and archived data
```

Each subfolder contains a `README.md` with the metrics column schema and reproduction instructions.

---

## Wiki
Refer to SailSwarm [Wiki](https://github.com/save-xx/SwarmSwIM/wiki). Plugin documentation in progress.
>>>>>>> origin/main
