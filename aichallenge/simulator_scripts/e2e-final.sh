#!/bin/bash
# Official-upstream-derived E2E final reference mode.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# GNSS is infrastructure-only for the bundled AWSIM Ready/Grounded handshake.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode sync \
    --start-count-seconds 10 \
    --vehicles 4 \
    --npcs 0 \
    --boosts 2 \
    --laps 6 \
    --timeout 420.0 \
    --steer-source ackermann \
    --sound on \
    --collisions on \
    --handicap on \
    --wall-recovery off \
    --start-random off \
    --ranking on \
    --camera cpu \
    --lidar on \
    --imu off \
    --gnss on \
    --v2x off
