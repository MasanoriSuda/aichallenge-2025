# Participant EKF

This fork uses the historical EKF implementation whose 13 installed headers
match the current base image. It replaces the underlay EKF in the participant
launch while preserving node/executable names, topics, services, QoS and params.
The filter equations, noise and smoothing remain unchanged.

A callback captures one time for prediction, measurement delays and pose/twist/
odometry publications. TF retains its pose timestamp. Independent clock reads
previously let a5msclock update label a10.000sstate as10.005s. Native tests call
the real node with controlled clock advances; their clock hook is test-only.

UPSTREAM.json records the original files and commit; LICENSE/NOTICE retain
attribution. Original mathematical tests are retained. This narrow repair does
not establish race acceptance or newly validate clock-regression/reset behaviour.
