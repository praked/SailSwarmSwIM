#!/usr/bin/env bash
set -euo pipefail

# Create output folder if it does not exist
OUT_DIR="50experiment1"
mkdir -p "${OUT_DIR}"

# Fixed parameters
ZOR_VALUE="2.0"
ZOO_VALUE="10.0"
DOMAIN_VALUE="35.0"
# Loop over seeds 1..10
for SEED in $(seq 1 10); do
    # File index: 101..110
    FILE_IDX=$((100 + SEED))
    METRICS_FILE="${OUT_DIR}/run${FILE_IDX}.csv"

    echo "Running: SEED=${SEED}, ZOR=${ZOR_VALUE}, METRICS_FILE=${METRICS_FILE}"

    SWARMSWIM_SEED="${SEED}" \
    SWARM_ZOR="${ZOR_VALUE}" \
    SWARM_ZOO="${ZOO_VALUE}" \
    SWARM_METRICS_FILE="${METRICS_FILE}" \
    SWARMSWIM_DOMAIN="${DOMAIN_VALUE}" \
    SWARM_HEADLESS=1 \
    MPLBACKEND=Agg \
    python3 -m SwarmSwIM.flocking_nav
done