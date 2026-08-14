# Requirements

## Purpose

The latest `make dev2` run proved that the short runtime-validation lease can keep a fresh
Frenet DP path authoritative.  The remaining dominant failure is that the target/wall
corridor is evaluated only for one implicit longitudinal timing.  When that timing closes
the lateral corridor, the Mission falls back to waiting or Recovery even though arriving
earlier or later can expose a feasible gap.

## Required behavior

- Evaluate several bounded longitudinal timings for the same pass side before rejecting it.
- Include an aggressive arrival, intermediate arrivals, and a zero-closing hold candidate.
- Build the target-aware lateral corridor using the arrival time of each candidate.
- Select the fastest candidate whose lateral DP cost remains close to the best feasible
  corridor; otherwise select the lower-cost timing.
- Execute the same closing speed that was used to predict and select the corridor.
- Re-evaluate the timing during the existing rolling MPCC-lite refresh; do not extend the
  runtime-validation lease.

## Constraints

- Preserve wall, vehicle-footprint, emergency-front-risk, solver-recovery, and forbidden
  waypoint hard gates.
- Keep target ID and pass-side ownership rules unchanged.
- Keep the legacy single-timing behavior as a disabled-policy fallback.
- Do not change ROS topics, messages, services, launch entry points, or result schemas.
- Do not modify generated output or the user's `aichallenge/result-summary.json` change.

## Definition of done

- Unit tests cover longitudinal profile selection and its fail-closed validation.
- Both local and cloud parameter files enable the same policy.
- Startup and candidate-rejection logs expose selected speed/closing speed and profile count.
- The package builds and its unit tests pass in the repository Docker environment.
