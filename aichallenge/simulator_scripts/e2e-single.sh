#!/bin/bash
# Deterministic E2E development gate: one vehicle, no NPC, three laps.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# The bundled AWSIM only reaches Ready/Grounded after its GNSS publisher
# exists. GNSS is infrastructure-only: TinyLidarNet does not subscribe to it.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles 1 \
    --npcs 0 \
    --boosts 0 \
    --laps 3 \
    --timeout 420.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random off \
    --ranking off \
    --camera off \
    --lidar on \
    --imu off \
    --gnss on \
    --v2x off
