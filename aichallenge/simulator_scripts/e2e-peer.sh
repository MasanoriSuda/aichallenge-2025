#!/bin/bash
# Deterministic peer audit: domain 3 ego follows/avoids domains 1 and 2.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# All three domains expose LiDAR for parity. Domain 2 is the configured low-speed
# peer; domain 3 is the MPC/Tiny audit ego. Current MPC runs must pass the E2E
# run admission checks before they can ever be considered teacher candidates.
# V2X is privileged MPC infrastructure and is not subscribed by TinyLidarNet.
exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    --venue citycircuit \
    --start-mode count \
    --start-count-seconds 5 \
    --vehicles 3 \
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
    --ranking on \
    --camera off \
    --lidar on \
    --imu on \
    --gnss on \
    --v2x on
