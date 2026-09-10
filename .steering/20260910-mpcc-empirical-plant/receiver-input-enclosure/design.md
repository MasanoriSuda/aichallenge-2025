# Receiver input uncertainty prototype and adoption boundary

Controller baseline `cbcda112`, repository checkpoint `7b9a6093`. Full M1–M6
implementation, validation and local commits remain authorised. No push.

## Broken invariant and bounded comparison

Publication does not establish longitudinal application. The local receiver
selects the latest pending input in Unity Update, independently of the steering
receipt and mechanical delay. The current immediate longitudinal prediction can
certify a Stop after its actual stopping reserve has already disappeared. See
[the receiver audit](../receiver-schedule-design.md) for exact failures,
producer boundaries, A/B/C/D comparison and preserved counterexamples.

This prototype represents acceleration as a set in the existing native body
map, uses one common held float32 steering/brake policy, and follows every
enclosed state through nominal rest. It never authorises a command. The held
steering policy is a bounded diagnostic alternative to the existing path-tracked
Stop; its nominal path is not silently substituted for the executed trajectory.

The centered midpoint-map enclosure splits rest and sign discontinuities.
Cartesian translation is separated exactly from the Jacobian. A fixed initial
heading frame avoids artificial lateral width from world-axis boxes. Full
original footprint/margins are included before enclosing its corners. Endpoint
box hulls cover the existing interpolated body path; analytic maximum peer speed
encloses peer motion over each interval. This does not prove the continuous
physical plant, arbitrary contact, or real-arithmetic correctness of libm.

Validation so far: 3,584 native scalar point checks including rest boundaries,
200,018 adjacent-float checks against `nextafter`, 64,000 footprint corners,
and 2,017,600 native scalar trajectory checks including extreme and switching
inputs. Synthetic full-rest enclosure alone fell from 20.5–69.2 ms to
4.9–16.4 ms without widening model limits or changing time steps. Saved low-speed
world comparisons take approximately 4–6 ms for range plus physical checks;
these are standalone observations, not callback acceptance.

The preserved r1/r2 world runs expose and retain invalid time-step/absent-input
diagnostics and world-axis overestimation. The r3 fixed-heading comparison admits
all inputs from −3 to the existing native +1.37 m/s² limit before each hypothetical
horizon, then the same −3 input. It conditionally clears949 and994 at100ms;
their next saved inputs950 and995 fail. All200ms arms fail. Point-delay endpoint
oracles are kept separately from whole-range results. Captured829 lacks original
observation provenance and remains unavailable. No actual publication input is
reconstructed from a later scene or relabelled as present.

## Predeclared independent receiver-profile experiment

The200ms assumption fails existing application-r1 startup: both receivers retain
a packet until its source age reaches209.103501ms. Do not discard startup or
claim200ms as an established bound. Application-r2 maximum closed-interval age is
184.070773ms. These are observations, not scheduling guarantees.

Before running application-dev2-r3, declare a **250ms empirical candidate** for
maximum applied-packet age in ROS source time. This is a conservative model
hypothesis, not a configured receiver delay or an inferred universal bound.
An old positive input and overwritten packets remain possible throughout it.
All other model, solver, safety and runtime parameters remain at the baseline.

Independent diagnostic acceptance:

1. Every logged selection matches receive sequence, source timestamp and exact
   float32 input, and receipt precedes application. No probe errors.
2. For every complete consecutive application interval on both receivers,
   the preceding selected packet's age immediately before replacement is
   nonnegative and at most250ms. Include startup, moving and stopping periods.
   Report both maxima and all violations. The final open interval is censored.
3. Source-time regression or missing selection identity makes the result
   inconclusive. The original DLL and protected user JSON files are restored.
4. The instrumented run is input-model evidence only; any moving Emergency
   remains a failed race run. Do not retry an unchanged rejected model or enlarge
   this profile after a failure without identifying the new cause.

The participant cannot observe the private receiver state. Any later adoption
must state this empirical assumption, monitor public observation consistency,
and reject outside the declared model context. This does not turn a measured
maximum or retrospective private trace into a prospective transport guarantee.

## Production promotion boundary

No prototype-only clearance result may execute. Before promotion:

- Move the shared body arithmetic into one kernel used by nominal transitions
  and the range calculation; remove the diagnostic transcription.
- Represent possible applied inputs from the causal publication history and
  common future command sequence. Preserve the separate steering channel.
- Carry profile, initial ranges/epochs, common controls and resulting tube in
  the immutable problem/artifact and every async/retained identity check.
- Candidate generation, physical prefix, normal/terminal Stop, materialization
  and final serialization must certify the same common controls. Extend through
  complete rest for all admitted responses; no per-branch future controller.
- Retain the existing QP authority, wall/peer margins, command bounds and budgets.
  Compare current nominal proof versus the declared uncertainty interpretation
  on frozen earlier inputs before selecting a production slice.
- Validate model/observation error separately from queue uncertainty. The native
  map's numerical enclosure does not establish real-vehicle error bounds.
- Run focused/package tests, build, frozen replay and live multi-tick Stop,
  then the remaining same-HEAD M4–M6 acceptance. No current run closes those steps.

Rollback remains `cbcda112`; current changes are standalone diagnostics only.
