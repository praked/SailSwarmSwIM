#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------
# Parameter grids
# ---------------------------------------------------------

# Three values for each parameter
ZOR_VALUES=(1.5 2.0 3.0)          # SWARM_ZOR   (repulsion radius)
ZOO_VALUES=(8.0 10.0 12.0)        # SWARM_ZOO   (orientation radius)
ZOA_VALUES=(15.0 18.0 22.0)       # SWARM_ZOA   (attraction radius)
DOMAIN_VALUES=(25.0 35.0 45.0)    # SWARMSWIM_DOMAIN (half-size of domain)

# Number of seeds per parameter set
NUM_SEEDS=10

# Simulation duration (T_MAX in flocking_nav.py)
SIM_TIME=400.0

# Maximum number of seeds to run in parallel for each parameter set
MAX_PARALLEL_SEEDS=4

# Base name for experiments
EXP_BASE="experiment_day2_"

# ---------------------------------------------------------
# Main loop over parameter combinations
# ---------------------------------------------------------

EXP_IDX=1

for ZOR in "${ZOR_VALUES[@]}"; do
  for ZOO in "${ZOO_VALUES[@]}"; do
    for ZOA in "${ZOA_VALUES[@]}"; do
      for DOMAIN in "${DOMAIN_VALUES[@]}"; do

        EXP_DIR="${EXP_BASE}${EXP_IDX}"
        mkdir -p "${EXP_DIR}"

        echo "======================================================="
        echo "Experiment ${EXP_IDX}:"
        echo "  ZOR=${ZOR}, ZOO=${ZOO}, ZOA=${ZOA}, DOMAIN=${DOMAIN}, T_MAX=${SIM_TIME}"
        echo "  Output dir: ${EXP_DIR}"
        echo "======================================================="

        # ---------------------------------------------------
        # Write README for this experiment
        # ---------------------------------------------------
        README_FILE="${EXP_DIR}/README.md"
        cat > "${README_FILE}" <<EOF
# ${EXP_DIR}

This folder contains 10 flocking simulations (seeds 1–${NUM_SEEDS}) with:

- SWARM_ZOR (repulsion radius):   ${ZOR}
- SWARM_ZOO (orientation radius): ${ZOO}
- SWARM_ZOA (attraction radius):  ${ZOA}
- SWARMSWIM_DOMAIN (half-size):   ${DOMAIN}
- SWARM_T_MAX (duration):         ${SIM_TIME} s

Each run writes a metrics CSV:

- \`run01.csv\` … \`run10.csv\`
EOF

        # ---------------------------------------------------
        # Run seeds 1..NUM_SEEDS for this parameter set
        # (up to MAX_PARALLEL_SEEDS in parallel)
        # ---------------------------------------------------
        RUNNING_JOBS=0
        for SEED in $(seq 1 "${NUM_SEEDS}"); do
          # Zero-padded index for file naming (01..10)
          
          SEED_PAD=$(printf "%02d" "${SEED}")
          METRICS_FILE="${EXP_DIR}/run${SEED_PAD}.csv"

          echo "Running: EXP=${EXP_IDX}, SEED=${SEED}, ZOR=${ZOR}, ZOO=${ZOO}, ZOA=${ZOA}, DOMAIN=${DOMAIN}, T_MAX=${SIM_TIME}"
          echo "  -> Metrics: ${METRICS_FILE}"

          SWARMSWIM_SEED="${SEED}" \
          SWARM_ZOR="${ZOR}" \
          SWARM_ZOO="${ZOO}" \
          SWARM_ZOA="${ZOA}" \
          SWARMSWIM_DOMAIN="${DOMAIN}" \
          SWARM_T_MAX="${SIM_TIME}" \
          SWARM_METRICS_FILE="${METRICS_FILE}" \
          SWARM_HEADLESS=1 \
          MPLBACKEND=Agg \
          python3 -m SwarmSwIM.flocking_nav &

          RUNNING_JOBS=$((RUNNING_JOBS + 1))
          if [ "${RUNNING_JOBS}" -ge "${MAX_PARALLEL_SEEDS}" ]; then
            # Wait for current batch of background jobs before launching more
            wait
            RUNNING_JOBS=0
          fi
        done

        # Ensure all remaining background jobs for this parameter set finish
        wait

        EXP_IDX=$((EXP_IDX + 1))

      done
    done
  done
done

echo "All experiments finished."