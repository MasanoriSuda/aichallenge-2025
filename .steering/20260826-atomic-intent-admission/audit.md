# Root-cause audit

## Evidence

In `output/20260826-124000/d1/autoware.log`:

- 68 Follow -> ShiftOut atomic admissions solve successfully but fail the
  physical proof with `stage-wall-rejected`.
- The proposal becomes `canonical_intent=shiftout` before admission completes.
- the retained candidate is then rejected by intent mismatch or steering
  reachability;
- production reports `retained-proof-unavailable` and publishes canonical
  Emergency;
- subsequent callbacks return to Follow and repeat the proposal.

The publisher boundary itself is healthy in the same run: 3324 candidate joins
succeed with zero reject and executed sequence reaches 2942.  Therefore this is
not the preceding publication-ledger defect.

## Earliest violated invariant

A proposed normal intent must not replace the effective normal intent until an
executable six-state plan for that proposal has crossed Gate A.  Failed
proposals are observations, not authority transitions.

## Falsification

- Solver unable to formulate: false; the transition solver returns `solved`.
- Wall guard too strict: not established and irrelevant to authority loss; a
  rejected path must not remove the current owner.
- stale executed ledger: false after the preceding Slice; sequence advances.
- adoption-order defect: directly supported by the proposed intent and
  physical rejection occurring in the same decision; highest confidence.

## Intermediate falsification

`output/20260826-140054/d2/autoware.log` showed that the first all-V2X join was
not sufficient:

- 43 Follow -> Cruise decisions reported
  `previous_world=follow-target-observation-unavailable`;
- proposed Follow artifacts also intermittently failed with the same reason;
- the target still existed in the current Cartesian obstacle set.

This disproved the assumption that a stateless Cartesian-to-course projection
was equivalent to the continuity-constrained observation which built the
Follow problem.  The duplicate projection was removed from the current Follow
path.  It remains only as a fail-closed recovery of evidence for a previously
published Follow intent whose new proposal is not Follow.

## Acceptance evidence

Static validation after the final repair:

- `make autoware-build`: 25 packages passed;
- `colcon test --packages-select multi_purpose_mpc_ros`: 51/51 targets passed;
- package results: 1924 tests, zero errors, failures or skips (prior run), with
  the final run again reporting 100% target success.

Dynamic validation in `output/20260826-141125`:

- ordinary Cruise -> Follow and Follow -> Cruise transitions joined normally;
- Follow -> ShiftOut was accepted when its exact six-state proof passed;
- five physically rejected ShiftOut proposals (`stage-wall-rejected`) retained
  the current Follow plan with `resolution=previous-retained` and
  `previous_world=accepted`;
- current Follow no longer produced
  `follow-target-observation-unavailable`;
- two ShiftOut -> Follow decisions with a genuinely unavailable V2X dynamic
  observation rejected both alternatives.  They were not converted into an
  uncertified hold.

The observed result satisfies the invariant: a tactical proposal cannot remove
normal authority, but an old intent is not retained without current-world
proof.
