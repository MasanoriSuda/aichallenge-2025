# Design

## Existing authorization boundary

No new permissive gate is added.  The controller already sets
`forward_escape_allowed` only when all of the following hold:

- competition attack mode and forward escape are enabled;
- a frozen minimum-motion corridor is active;
- the Pass front cap has already been released;
- target identity and course progress remain continuous;
- current vehicle footprints are separated;
- no target position jump is present.

The normal short-horizon wall/body checks remain mandatory every cycle.

## Priority change

`resolve_safe_separation()` retains this order:

1. rear-clear and Return;
2. short-horizon safety;
3. duration/distance bounds;
4. authorized forward escape;
5. confirmed target-ahead recover-behind fallback;
6. ordinary longitudinal separation.

Previously step 5 ran before step 4.  At 2 m front clearance it therefore
selected Recovery even though the forward corridor was still authorized.

## Competition settings

- closing delta: `0.8 -> 2.0 m/s` (aligned with committed Pass closing speed)
- forward window: `0.75 -> 3.0 m`
- bounded duration: `3.0 -> 5.0 s`
- bounded travel: `8.0 -> 12.0 m`

These values do not change acceleration or speed hard limits.  The window is
long enough to cover the observed 2 m recover-behind threshold, while a target
beyond 3 m still falls back to Recovery.  A continuously unsafe corridor or an
expired bound also keeps the existing Recovery behavior.

