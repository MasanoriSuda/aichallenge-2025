# Design

## Pass-entry lease

Extend gate eligibility, not hold duration.  The gate is eligible at the
ShiftOut boundary and during the first 2.5 seconds / 12 metres of Pass.  A
warning starts the existing bounded hold (`pass_horizon_hold_max_sec` and
`pass_horizon_hold_max_distance`).  When the warning clears, Pass resumes from
a freshly validated path; expiry requests DynamicMissionWait/reselection.

## SafeSeparation lifecycle

SafeSeparation owns a cumulative episode budget.  Same-side replan retention
and target-bound physical-prefix execution are subordinate actions and must not
clear `pass_horizon_safe_separation_active`.  Normal phase/Mission reset remains
the only lifecycle termination, apart from the existing explicit bounded
progress extension.

## Rear-clear retention

`PausedMissionTerminalRequest` receives an opt-in retention flag.  Legacy
FollowPrepare keeps expiry-first behavior.  A healthy DynamicMissionWait uses
the flag so time/distance expiry becomes `Hold` while rear-clear is pending.
Target discontinuity/staleness, forbidden waypoints and other hard faults still
recover, rear-clear returns, and the existing total Mission budget remains the
absolute upper bound.
