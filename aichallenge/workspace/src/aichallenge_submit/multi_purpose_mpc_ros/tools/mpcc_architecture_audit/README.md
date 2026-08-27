# MPCC Architecture Audit

Offline, dependency-free evidence tooling for the architecture escape-hatch
process. It does not publish ROS commands and cannot change production
authority.

```bash
python3 -m mpcc_architecture_audit validate-snapshot snapshot.json
python3 -m mpcc_architecture_audit validate-registry registry.json
python3 -m mpcc_architecture_audit classify comparison.json
```

A snapshot is replay-ready only when all immutable numerical, world,
prediction, tactical, warm-start and physical-certificate payloads are present
and SHA-256 sealed. Logs alone must be recorded with `replay_ready=false`.

The classifier never treats local solver failure as proof of physical
infeasibility.
