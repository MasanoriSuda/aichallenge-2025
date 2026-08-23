# Follow live positive gate

## Purpose

Obtain live AWSIM evidence for the exact fresh canonical Follow chain after the warm-start layout repair.

## Gate

- production configuration remains unchanged;
- at least 200 valid-contract Follow attempts are observed;
- warm-start, solve, physical, canonical and runtime counts are aggregated;
- all results remain shadow-only;
- no authority promotion occurs in this steering.

If the natural scenario produces no eligible Follow interval, record the gate as not exercised rather
than changing parameters or bypassing `LowSpeedAvoidance`.
