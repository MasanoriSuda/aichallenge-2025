# Design

## 1. Prediction-only grace

Add a small pure resolver that combines a hard guard and a predictive guard.

- A hard guard failure always returns unsafe.
- A healthy predictive guard returns safe immediately.
- A failed predictive guard may remain temporarily safe only for an already-latched forward
  completion with strict current-body separation, an unblocked corridor, fresh measured forward
  progress, and `loss_elapsed <= grace_sec`.

The controller owns the loss timer. Recovery remains fail-closed after grace expiry.

## 2. Active Pass clock

Change the mission time accounting from:

`now - first_pass_entry`

to:

`completed_pass_phase_time + current_pass_phase_time`.

Leaving Pass accumulates the active phase duration. Re-entering Pass starts a new active segment.
Mission replacement preserves the accumulated active duration, so it cannot reset the global Pass
budget, while time spent waiting or shifting to the replacement side no longer burns that budget.
The existing 10-second and 32-metre ceilings remain unchanged.

## Impact

- No topic, service, message, launch, or package contract changes.
- One new YAML tuning value controls prediction-only grace.
- Logs distinguish active Pass elapsed time and prediction-grace activation.

