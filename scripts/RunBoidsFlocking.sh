#!/usr/bin/env bash
set -euo pipefail

# Create output folder if it does not exist
OUT_DIR="BOIDS_FLOCKING_experiment5"
mkdir -p "${OUT_DIR}"

# Set output file inside OUT_DIR to log chosen parameters
PARAM_LOG="${OUT_DIR}/params.csv"

# Fixed parameters
DOMAIN_VALUE="45.0"

# Loop over seeds 1..10
for SEED in $(seq 1 2); do
    # File index: 101..110
    FILE_IDX=$((100 + SEED))
    METRICS_FILE="${OUT_DIR}/run${FILE_IDX}.csv"

    echo "Running: SEED=${SEED}, METRICS_FILE=${METRICS_FILE}"

    (
        export SWARMSWIM_SEED="${SEED}"
        export SWARM_METRICS_FILE="${METRICS_FILE}"
        export SWARM_SEP_FAC=1
        export SWARM_ALG_FAC=1
        export SWARM_COH_FAC=1
        export SWARMSWIM_DOMAIN="${DOMAIN_VALUE}"
        export SWARM_HEADLESS=1
        export SWARM_CRASH_LOGGING=0
        export MPLBACKEND=Agg

        printf "SWARMSWIM_DOMAIN,SWARM_SEP_FAC,SWARM_ALG_FAC,SWARM_COH_FAC\n${DOMAIN_VALUE},$SWARM_SEP_FAC,$SWARM_ALG_FAC,$SWARM_COH_FAC" > "${PARAM_LOG}"

        python3 -m SwarmSwIM.boids_flocking
    )
done


