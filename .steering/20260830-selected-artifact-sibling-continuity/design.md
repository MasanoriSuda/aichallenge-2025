# Design

## 1. Lifecycle owner

The certified Store, not the global latest-world bank, owns execution
lifecycle.  Each Store slot carries an immutable pair:

- selected certified plan;
- optional same-epoch opposite-side certified sibling.

Candidate, published-Bundle source and executed slots each preserve the pair
which crossed that specific lifecycle boundary.

## 2. Pair validation

Pair admission validates both plans independently and proves:

- both are Cruise or Follow normal dynamic-avoidance artifacts;
- identical sequence and snapshot time;
- identical problem epoch fields;
- opposite `dynamic_obstacle_side_sign` values in `{-1, +1}`;
- each fingerprint is correctly sealed for its side;
- no plan is accepted as its own sibling.

The helper is data-only and has no publisher surface.

## 3. Producer transaction

The dual current-world producer already joins both branch evaluations.  After
selection, it sends the selected plan and, if certified, the opposite plan to
one Store replacement call.  The old selected-only replacement edge is
removed for this population.

## 4. Current-world consumption

Retained evaluation keeps its existing order:

1. candidate;
2. published-Bundle source;
3. executed plan.

Only after all fail does it inspect their associated siblings, in the same
order, deduplicated by artifact identity.  Each sibling is evaluated by the
unchanged `evaluate_rate_resolved_track_cruise_plan` path.  It is selected only
when that function returns complete production authority and a stateless
current-world Bundle.

The global latest-world bank remains diagnostic and may continue invalidating
old epochs.  It is no longer the source of executable sibling continuity.

## 5. Diagnostics

Record which Store lifecycle slot supplied the sibling, its source sequence,
side and exact current-world rejection/adoption reason.  This distinguishes:

- sibling unavailable at producer time;
- sibling present but physically stale;
- sibling current-world proof rejected;
- sibling current-world proof accepted.
