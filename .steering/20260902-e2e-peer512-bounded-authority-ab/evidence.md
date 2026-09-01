# Evidence

## Frozen comparison

Authority-disabled A:

- run: `/output/20260902-e2e-recurrent-shadow-isolated-single`
- recurrent SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- laps: `84.3783 / 84.0335 / 83.9885 s`
- total: `252.4002 s`
- penalty / stall: `0 / 0`
- hidden reset / recurrent error: `0 / 0`
- production inference weighted mean: `5.651 ms`

Bounded-authority B:

- run: `/output/20260902-e2e-peer512-authority-single-b`
- same recurrent artifact and production identities as A
- authority correction bound: `+/-0.24 rad`
- laps: `84.5332 / 83.9685 / 83.7486 s`
- total: `252.2503 s`
- penalty / stall: `0 / 0`
- recurrent coverage: `99.9852%` (`6739 / 6740`)
- authority applications / clips: `6739 / 7`
- weighted / maximum applied correction: `0.00620 / 0.24000 rad`
- inference weighted mean / maximum: `8.570 / 98.70 ms`
- minimum scan frequency: `19.80 Hz`
- skipped recurrent inference / hidden reset: `1 / 1`

The B total is `0.150 s` faster than A, approximately `0.06%`.  This is not a
material performance benefit.

## Rejection cause

The strict recurrent-authority analyzer rejects B for both:

- `recurrent hidden state reset count exceeded threshold`;
- `recurrent authority run contains skipped inference`.

The first reset is reported at scan 5,330, timestamp
`1788305606.025307252`.  The orchestrator stops the rosbag after detecting
Finish at `1788305610.278758860`, so the reset occurred about four seconds
before Finish and is part of the race rather than shutdown noise.

The affected reporting interval contains one spatial skip, one recurrent skip
and a `98.70 ms` inference tail.  Both policies share the production speed
freshness admission.  This timing supports the hypothesis that synchronous
current-sample recurrent authority can delay callback servicing enough to lose
one speed admission, but interval telemetry cannot prove the exact callback
ordering.  No threshold is relaxed on that basis: the observable contract
violation alone is sufficient to reject the Gate.

## Decision

Reject peer-512 bounded authority at the single-vehicle Gate and do not execute
the three-vehicle authority run.

The authority-disabled async observation architecture remains accepted.  The
recurrent artifact stays external and shadow-only, and packaged production
defaults remain unchanged.  Do not retry this Slice or add retained/stale
authority to hide the failed current-sample admission.  A future authority
attempt requires a separately approved architecture that bounds current-sample
compute without publishing old diagnostic results.
