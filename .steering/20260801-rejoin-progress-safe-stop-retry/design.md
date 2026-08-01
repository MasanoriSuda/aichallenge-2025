# Design

## Root Cause

`LowSpeedRejoin` already owns speed and steering directly, independently of the failed normal MPC.
However, its timeout was measured only from state entry. P2 was turning toward the path on every
sample, but the fixed five-second deadline forced it back into clearance selection while it was
still approximately 66 degrees misaligned. P1 then occupied P2's Reverse rollout and both karts
formed a V2X recovery interlock.

The supervisor also checks normal-MPC health before dispatching `SafeStop`. In aggressive
simulation mode this can prevent the existing bounded retry policy from running when a caller
reports the persistent solver failure directly.

## Changes

1. Track a dimensionless rejoin alignment error:
   `max(abs(e_y)/max_e_y, abs(e_psi)/max_e_psi)`.
2. Refresh the rejoin progress timestamp only after the best error improves by the configured
   minimum ratio. This rejects noise while allowing a genuinely converging gross-angle recovery
   to continue.
3. Interpret `rejoin.timeout_sec` as the maximum time without material alignment progress.
   Static path, current footprint, gear, information-completeness, and completion gates remain
   unchanged.
4. In aggressive simulation mode, let a `SafeStop` with an explicitly recoverable reason reach
   `update_safe_stop()` even if normal MPC is unhealthy. That function still enforces its delay,
   clearance confirmation, and reason allowlist.

## Scope

- `include/multi_purpose_mpc_ros/stuck_recovery_core.hpp`
- `src/stuck_recovery_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `config/config.yaml`
- `test/test_stuck_recovery_core.cpp`

No interface or evaluation-system changes.
