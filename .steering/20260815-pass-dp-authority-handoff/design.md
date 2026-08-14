# Design

## 1. Bounded DP Pass authority

Resolve a pure `FrenetDpPassAuthority` policy from the active phase, exact
target/side identity, DP path validity and age, remaining path distance, target
continuity, body geometry and hard-fault signals.

When the policy grants authority, horizon actions that would otherwise rebuild
the same-side path through the legacy single-goal extension are satisfied by
the current DP prefix. The later receding-horizon optimizer still performs the
normal wall, target and kinematic validation. A hard failure therefore keeps
the existing target-bound hold/Recovery behavior.

Outer-side transitions and explicit dynamic-corridor tactical replacements
continue to use their existing atomic preflights.

## 2. Return reference handoff

`begin_validated_return` stores the admitted return horizon distances and
lateral targets before changing phase. Return execution resamples that stored
reference by Return distance traveled, using the existing Frenet-DP reference
resolver and the legacy Return profile as its fallback tail.

The stored reference is cleared whenever Return ends. This makes the first
Return cycle consume exactly the path accepted by preflight instead of
recomputing from a slightly changed state.

## Expected dynamic evidence

- `DP Pass authority retained` appears near a rear-clear revalidation window.
- The same episode does not immediately enter SafeSeparation with
  `same-side lateral adjustment limit exceeded`.
- `Return preflight reference committed` is followed by Return execution, not
  an immediate static-wall/lateral-acceleration Recovery.
- Hard wall/target faults still revoke DP authority and fail closed.
