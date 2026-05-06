#!/usr/bin/env bash
set -euo pipefail

# Create output folder if it does not exist
OUT_DIR="results/battery/10experiment1"
mkdir -p "${OUT_DIR}"

# Fixed parameters
ZOR_VALUE="2.0"
ZOO_VALUE="10.0"
DOMAIN_VALUE="35.0"
W_ORI="0.0"
# Loop over seeds 1..10
for SEED in $(seq 1 10); do
    # File index: 101..110
    FILE_IDX=$((200 + SEED))
    METRICS_FILE="${OUT_DIR}/run${FILE_IDX}.csv"

    W_ORI_VALUE=$(awk -v seed=$SEED 'BEGIN {print 0.0 + seed * 0.1}')
    echo "Running: SEED=${SEED}, ZOR=${ZOR_VALUE}, W_ORI=${W_ORI_VALUE}, METRICS_FILE=${METRICS_FILE}"

    SWARMSWIM_SEED="${SEED}" \
    SWARM_ZOR="${ZOR_VALUE}" \
    SWARM_ZOO="${ZOO_VALUE}" \
    SWARM_W_ORI="${W_ORI_VALUE}" \
    SWARM_BATTERY_METRICS_FILE="${METRICS_FILE}" \
    SWARMSWIM_DOMAIN="${DOMAIN_VALUE}" \
    SWARM_HEADLESS=1 \
    MPLBACKEND=Agg \
    python3 -m SwarmSwIM.behaviors.flocking_battery_nav
done