#!/usr/bin/env bash
set -euo pipefail

# Output folder for this batch of aggregation experiments
OUT_DIR="results/aggregation/Random"
mkdir -p "${OUT_DIR}"

# Fixed parameters (adjust as needed)
DOMAIN_VALUE="35.0"      # L_h; arena is [-L_h, +L_h]^2
DC_VALUE="20.0"          # d_c for connectivity (can = comm radius)
T_MAX_VALUE="300.0"      # total simulation time per run [s]

# Loop over seeds 1..10
for SEED in $(seq 1 2); do
    # File index: 101..110
    FILE_IDX=$((100 + SEED))
    METRICS_FILE="${OUT_DIR}/run${FILE_IDX}.csv"

    echo "Running aggregation: SEED=${SEED}, METRICS_FILE=${METRICS_FILE}"

    SWARMSWIM_SEED="${SEED}" \
    SWARM_METRICS_FILE="${METRICS_FILE}" \
    SWARMSWIM_DOMAIN="${DOMAIN_VALUE}" \
    SWARM_AGG_DC="${DC_VALUE}" \
    SWARM_T_MAX="${T_MAX_VALUE}" \
    SWARM_HEADLESS=1 \
    MPLBACKEND=Agg \
    python3 -m SwarmSwIM.behaviors.aggregation_nav
done