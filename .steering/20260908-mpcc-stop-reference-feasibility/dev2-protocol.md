# Multivehicle continuation, pending single repetitions

After all three fixed-binary single trials pass, use the same controller,
configuration and maps with `make dev2` and the standard Domain0/1/2 layout.
The steering-only Compose overlay mounts a copy of dev.sh into the simulator
container. Only `--laps unlimited` and `--timeout 10000000.0` become6 and600;
count start, spawn, two vehicles, no NPCs, collision/handicap/wall-recovery
settings stay as in dev2. No repository runtime configuration is changed.
Record this fixture distinction explicitly; it is a development acceptance
scenario, not the baked submission image test.

The initial run is bounded by all vehicles finishing, tactical Recovery or
an840s host deadline. On first tactical Recovery, preserve the boundary and
stop this run; do not repeat until classified. If complete, inspect both
formal result files, all penalties, receive/callback timing and full
ShiftOut/Pass/Return/Idle episodes. Distinguish left/right observations.
Specifically inspect certified Stop's commit, subsequent execution cursor,
current-world proof and resumption. No observed Stop means that acceptance
item is untested, even if the race finishes.

The old Pass longitudinal-progress failure remains unresolved. The reference
repair may reduce Stop rejection and lateral sibling changes, but no speed,
watchdog, wall or target margin has been changed. A recurrence requires a
same-scene comparison before any policy edit. Do not treat race completion
alone as architecture or timing acceptance.
