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


## Main sailing components

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
