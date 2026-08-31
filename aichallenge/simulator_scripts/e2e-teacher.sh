#!/bin/bash
# Deterministic MPC teacher collection: one vehicle, no NPC, three laps.

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=0

# LiDAR is the student input. GNSS/IMU are enabled only for the MPC teacher and
# AWSIM readiness/localization infrastructure; they are not student features.
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
    --imu on \
    --gnss on \
    --v2x off
