#!/bin/bash
# Deterministic LiDAR-only student gate: one ego, two NPCs, three laps.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0
start_random_seed="${E2E_START_RANDOM_SEED:-2026}"
if [[ ! ${start_random_seed} =~ ^[0-9]+$ ]]; then
    echo "[ERROR] E2E_START_RANDOM_SEED must be a non-negative integer" >&2
    exit 2
fi
echo "[INFO] E2E NPC effective start_random_seed=${start_random_seed}"

# GNSS is infrastructure-only for Ready/Grounded. The TinyLidarNet controller
# subscribes only to /scan and never consumes GNSS, IMU, or V2X.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles 1 \
    --npcs 2 \
    --boosts 0 \
    --laps 3 \
    --timeout 420.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random on \
    --start-random-seed "${start_random_seed}" \
    --start-random-range 2.0,2.0 \
    --start-random-min-separation 3.0 \
    --ranking off \
    --camera off \
    --lidar on \
    --imu off \
    --gnss on \
    --v2x off
