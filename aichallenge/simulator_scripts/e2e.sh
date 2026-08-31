#!/bin/bash
# Official-upstream-derived E2E practice/reference mode.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# GNSS is infrastructure-only for the bundled AWSIM Ready/Grounded handshake.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 0 \
    --vehicles 1 \
    --npcs 2 \
    --boosts 2 \
    --laps 6 \
    --timeout 10000000.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --start-random on \
    --ranking off \
    --camera cpu \
    --lidar on \
    --imu off \
    --gnss on \
    --v2x off
