# Requirements

## Objective

Improve launch and mid-speed acceleration without driving the frozen lateral
policy beyond its evidenced speed distribution.

## Root cause

The existing fixed acceleration owns both transient acceleration and
drag-limited steady speed.  On the paired default-seed test, `0.8 m/s2`
reached `6.515 m/s` and failed against a wall, while `0.6 m/s2` stayed at or
below `4.564 m/s` and finished without penalty.  The commands had been labelled
seed 2035, but an infrastructure audit proved AWSIM received default seed 2026.
Spatial training speed has maximum `4.790 m/s`.

## Constraints

- Keep packaged production at `0.6 m/s2` until the candidate passes both Gates.
- Do not change lateral weights, steering authority or obstacle distances.
- The governor may only reduce a positive acceleration request; it may not
  weaken an existing brake request.
- Missing or stale speed while the governor is enabled must prohibit positive
  acceleration.
- The feature is opt-in until promotion and its limit must be present in runtime
  provenance.
- First test the frozen failing seed 2035 with `0.8 m/s2` and a `4.6 m/s` cap.
- An admitted candidate then needs an independent NPC seed before promotion.
