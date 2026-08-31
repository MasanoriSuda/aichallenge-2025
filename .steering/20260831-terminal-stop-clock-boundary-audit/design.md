# Design: terminal Stop / normal-resume architecture audit

## Observed causal boundary

At decision 2451 the vehicle is in Cruise with a nearly stopped `d2` about
5.45 m ahead and about 4.11 m/s closing speed.  The recorded normal candidate
loses its recursive terminal contingency and a separately certified Stop
successor correctly takes authority.  On the next cycle the normal execution
clock has been discarded, the retained candidate is evaluated as a bootstrap
candidate from cursor zero, and its expected pose is already about 1.30 m from
the current control pose.  Later `steering-unreachable`, Recovery and prolonged
Stop are downstream observations; they are not yet accepted as the cause.

## Hypotheses

| Hypothesis | Supporting evidence | Refutation |
|---|---|---|
| H1 persistent lifecycle defect | Stop transition invalidates the executed-clock join and only old candidates remain | B/C/D also fail on the frozen current world |
| H2 direct candidate-generation defect | A and direct left/right B fail at obstacle disjunction rows | a smooth/lattice C cannot construct a certified bundle |
| H3 live single-SQP limitation | right B reaches maximum iterations with a small failed-iterate normalized residual | C fails but the same candidate succeeds under bounded offline multi-SQP D |
| H4 physical infeasibility | short distance, high closing speed and narrow hairpin may leave only Stop | every A--D candidate fails and an independent physical certificate proves no feasible trajectory |
| H5 model/certificate mismatch | a solve may pass while exact wall/dynamic/terminal proof rejects | no solved arm fails only the exact proof |

Current confidence before C/D: H2 or H3 is medium; H1 and H4 remain possible.
The existing A/B evidence is insufficient to select a repair.

## Audit-only implementation

Extend the existing architecture comparison only for Cruise/Follow snapshots:

1. start each side from the same stateless current-world B seed;
2. enumerate a bounded family of smooth transition stages while preserving the
   recorded obstacle tube and physical constraints;
3. evaluate C once with the unchanged seven-state solver;
4. evaluate D on the identical sealed candidate with bounded multi-SQP;
5. emit all arm identities and proof outcomes without exposing any publisher
   API.

The comparison executable has no ROS publisher and cannot alter normal or Stop
authority.

## Repair gate

No production repair is selected in this Slice until the exit classification
is known.  In particular, keeping the pre-Stop execution clock is not assumed
safe: an external Stop may legitimately invalidate that old trajectory.
