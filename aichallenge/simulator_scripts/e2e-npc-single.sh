#!/bin/bash
# Deterministic LiDAR-only student gate: one ego, two NPCs, three laps.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

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
    --start-random-seed 2026 \
    --start-random-range 2.0,2.0 \
    --start-random-min-separation 3.0 \
    --ranking off \
    --camera off \
    --lidar on \
    --imu off \
    --gnss on \
    --v2x off
