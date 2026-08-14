# Design

## Problem

Replacement commits, paused-Mission resolution and fallback trajectory publication are nested in
one `else-if` authority chain. The runtime condition and output production are therefore easy to
couple incorrectly. The latest run exposed this: the forward-prefix publisher exists, but its
parent condition excludes the tactical rolling-replan state that needs it.

## Refactoring

1. Define a local forward-prefix publisher that takes the resolved hard-fault state and Mission
   side explicitly.
2. Define a local DynamicMissionWait executor with an explicit outcome:
   - not active,
   - output/transition handled,
   - resume current Mission.
3. Keep the invocation under the existing admission branch for this refactor, so runtime behavior
   remains unchanged.
4. Keep replacement order unchanged: MPCC same-side, MPCC cross-side, opponent-side, then paused
   Mission handling.

## Follow-up

The next performance change can move only the executor invocation into the tactical rolling-replan
owner. That change will be reviewable independently from this mechanical refactor.
