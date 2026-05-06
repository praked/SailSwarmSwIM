#!/usr/bin/env bash
set -euo pipefail

# parameters

ZOR_VALUES=(2.0 4.0 8.0)
ZOO_VALUES=(5.0 10.0 20.0)
ZOA_VALUES=(9.0 18.0 38.0)

NUM_SEEDS=5

SIM_TIME=300

EXP_BASE="results/battery/day1/experiment"

EXP_IDX=1

MAX_PARALLEL_SEEDS=4

RUNNING_JOBS=0


for ZOR in "${ZOR_VALUES[@]}"; do
    for ZOO in "${ZOO_VALUES[@]}"; do
        for ZOA in "${ZOA_VALUES[@]}"; do

        EXP_DIR="${EXP_BASE}${EXP_IDX}"
        mkdir -p "${EXP_DIR}"

        echo "======================================================="
        echo "Experiment ${EXP_IDX}:"
        echo "  ZOR=${ZOR}, ZOO=${ZOO}, ZOA=${ZOA}, T_MAX=${SIM_TIME}"
        echo "  Output dir: ${EXP_DIR}"
        echo "======================================================="
        # Write README
        cat > "${EXP_DIR}/README.md" <<EOF

# ${EXP_DIR}

This folder contains ${NUM_SEEDS} flocking simulations with:

- SWARM_ZOR (repulsion radius):   ${ZOR}
- SWARM_ZOO (orientation radius): ${ZOO}
- SWARM_ZOA (attraction radius):  ${ZOA}
- SWARM_T_MAX (duration):         ${SIM_TIME} s

Each run writes a metrics CSV: \`run01.csv\` … \`run0${NUM_SEEDS}.csv\`
EOF

        for SEED in $(seq 1 "${NUM_SEEDS}"); do
            SEED_PAD=$(printf "%02d" "${SEED}")
            METRICS_FILE="${EXP_DIR}/run${SEED_PAD}.csv"

            echo "Launching: EXP=${EXP_IDX}, SEED=${SEED}, ZOR=${ZOR}, ZOO=${ZOO}, ZOA=${ZOA}"
            echo "  -> ${METRICS_FILE}"

            (
              SWARMSWIM_SEED="${SEED}" \
              SWARM_ZOR="${ZOR}" \
              SWARM_ZOO="${ZOO}" \
              SWARM_ZOA="${ZOA}" \
              SWARM_T_MAX="${SIM_TIME}" \
              SWARM_BATTERY_METRICS_FILE="${METRICS_FILE}" \
              SWARM_HEADLESS=1 \
              MPLBACKEND=Agg \
              python3 -m SwarmSwIM.behaviors.flocking_battery_nav
            ) &

            RUNNING_JOBS=$((RUNNING_JOBS + 1))

            if [ "${RUNNING_JOBS}" -ge "${MAX_PARALLEL_SEEDS}" ]; then
              wait -n
              RUNNING_JOBS=$((RUNNING_JOBS - 1))
            fi

            done

            EXP_IDX=$((EXP_IDX + 1))
        done
    done
done

wait
echo "All experiments completed successfully."
