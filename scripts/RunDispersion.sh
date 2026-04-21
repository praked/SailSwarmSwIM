#!/usr/bin/env bash
set -euo pipefail

# Create output folder if it does not exist
OUT_DIR="DISPERSION_experiment4"
mkdir -p "${OUT_DIR}"

# Set output file inside OUT_DIR to log chosen parameters
PARAM_LOG="${OUT_DIR}/params.csv"

# Fixed parameters
DOMAIN_VALUE="100"
T_MAX="300"

# Loop over seeds 1..10
for SEED in $(seq 1 10); do
    # File index: 101..110
    FILE_IDX=$((100 + SEED))
    METRICS_FILE="${OUT_DIR}/run${FILE_IDX}.csv"

    echo "Running: SEED=${SEED}, METRICS_FILE=${METRICS_FILE}"

    (
        export SWARMSWIM_SEED="${SEED}"
        export SWARM_METRICS_FILE="${METRICS_FILE}"

        export SWARM_RND_POSE=1
        export SWARM_AGENT_COUNT=20
        export SWARM_SPREAD_POSES=1

        export SWARM_HEADLESS=1
        export MPLBACKEND=Agg

        export SWARM_CRASH_LOGGING=0

        export SWARMSWIM_DOMAIN="${DOMAIN_VALUE}"
        export SWARM_T_MAX="${T_MAX}"
        export SWARM_COMM_RADIUS=100

        printf "SWARMSWIM_DOMAIN,SWARM_COMM_RADIUS\n${DOMAIN_VALUE},$SWARM_COMM_RADIUS" > "${PARAM_LOG}"

        python3 -m SwarmSwIM.dispersion_nav
    )
done

