# Design

## Root-cause hypothesis

The accepted 20-stage run proves the required geometry, but the production
latest-only worker receives a new full solve request on every 40 Hz callback.
During tight wall/obstacle regions the physical-certificate construction can
take tens of milliseconds, replace pending jobs, and contend with the control
callback.  Horizon shortening reduced this tail but removed terminal
feasibility, so proof length and solve cadence must be separated.

## Change

Add a pure cadence resolver to the existing latest-only worker support code.
The production controller retains a small semantic identity consisting of:

- control intent;
- Mission/intent generation;
- target identity;
- execution side.

The resolver submits when no predecessor solve exists, the semantic identity
changes, retained production authority is unavailable, the clock resets, or
the configured interval elapses.  A deferred cycle still evaluates the last
actually published certified artifact against the current world; it does not
obtain authority from age alone.

Pre-entry Gate A solving is unchanged.  This experiment changes computation
frequency only and does not change which artifact can become production
authority.

## Candidate value

- Baseline: 0.025 s (40 Hz)
- Candidate: 0.050 s (20 Hz)
- Publication and current-world revalidation: 0.025 s (40 Hz), unchanged

## Rollback

Remove the cadence mechanism and restore unconditional per-callback production
submission if the experiment is rejected.  A rejected scheduling abstraction
must not remain as another dormant production path.
