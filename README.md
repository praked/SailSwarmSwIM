# SwarmSwIM: Sailing Extension
This plugin adds sailing simulation to SwarmSwIM. 

**Related repositories**
- [sail_examples](https://github.com/aaron4522/swarmswim_examples)
- [swarmswim](https://github.com/save-xx/SwarmSwIM)
- [swarmswim examples](https://github.com/save-xx/swarmswim_examples)
- [swarmswimros](https://github.com/save-xx/swarmswimros)

## Features

https://github.com/user-attachments/assets/78136534-73f5-4b2f-b991-e2a12eed649c


Key features include:
- Wind physics simulating linear gusts/lulls and directional turbulence
- Extendable sailing force mechanics
- Extendable navigation and tacking behavior
- Waypoint navigation


## Installation
The package can be installed Globally to be accessible regardless the location.

---
#### Generic installation
For a generic installation, just run: 
```bash
pip install git+https://github.com/aaron4522/SwarmSwIM.git
```
Note that you will not be able to alter the code.

---
#### Dev installation
For a developer (editable) installation:

Clone the repository in a Folder of your choice:
```bash
git clone https://github.com/aaron4522/SwarmSwIM.git
```

Enter the folder that has been cloned
in the `SwarmSwIM` folder:  
```bash
cd SwarmSwIM/ # Go to download folder
```
  
Install SwarmSwIM as python pakage, `-e` indicates that is editable.
```bash
pip install -e .
```

The installation should conclude with:
```
Successfully built SwarmSwIM
Installing collected packages: SwarmSwIM
Successfully installed SwarmSwIM-x.x.x
```

---

## Running Experiments

Batch experiment scripts live in the `scripts/` directory and must be run from the **repository root**:

```bash
bash scripts/RunFlocking.sh       # Run 10-seed flocking experiment
bash scripts/RunAggregation.sh    # Run aggregation experiment
bash scripts/ScriptFlocking.sh    # Parameter sweep (many seeds/combos)
bash scripts/120thread.sh         # High-parallelism parameter sweep
```

Output directories and CSV metrics files are gitignored and generated locally.

---

## Wiki
Refer to SailSwarm [Wiki](https://github.com/save-xx/SwarmSwIM/wiki). Plugin documentation in progress.
