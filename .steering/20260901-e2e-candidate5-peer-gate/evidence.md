# Candidate5 peer gate evidence

## Frozen run

- run: `output/20260901-103656`
- candidate SHA-256:
  `f84e802fc6976906ddb062e1f5ddd509119a39602b55051b25e519b761c0f9e3`
- candidate path in all four launch logs:
  `/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/20260901_102515/candidate.npy`
- runtime mode in all four launch logs: `fixed_lidar_brake`
- production checkpoint was not overwritten

The run was stopped after d2 yielded a finalized low-speed failure.  The
practice harness remained in AWSIM `Start`, so this is a failure snapshot and
not a Finish admission.

## Finalized bag analysis

| domain | distance [m] | duration [s] | mean speed [m/s] | longest post-start low speed [s] | result |
|---|---:|---:|---:|---:|---|
| d1 | 859.06 | 395.04 | 2.168 | 0.00 | pass |
| d2 | 117.40 | 404.21 | 0.281 | 123.51 | **fail** |
| d3 | 1156.45 | 414.29 | 2.783 | 0.00 | pass |
| d4 | 1237.39 | 422.79 | 2.925 | 0.00 | pass |

d2 became stationary at 280.70 s and remained stationary until the finalized
bag end at 404.21 s.  At the start of that interval:

- acceleration command: `0.0 m/s2`
- steering command: `+0.538 rad`
- front LiDAR: `1.476 m`
- right-front LiDAR: `1.142 m`
- right-side LiDAR: `1.001 m`
- left-front LiDAR: `3.178 m`

The final runtime status repeatedly reported
`longitudinal_safety_active=51/51`, `front_m=1.62--1.66` and
`safety_reason=slow-clearance`.  Therefore this is not the old defect in which
positive acceleration was published while physically stuck.  The longitudinal
safety layer correctly removed acceleration after the lateral policy entered a
contact trap.  Steering a stationary kart cannot create the missing lateral
escape.

## Comparison and decision

The production four-peer baseline had two domains trapped by symmetric peer
interaction.  Candidate5 learned 53.1% of the admitted teacher's novel
correction offline and passed the single-vehicle gate, but still trapped d2 in
the same final world.  The admitted all-teacher run completed all four domains
without a post-start stall.

Candidate5 is rejected and must not replace production.  Another global
fine-tune is also rejected as the next step: candidate5 materially regressed
independent normal validation while failing to reproduce enough of the lateral
escape, whereas an fc4-only candidate preserved normal behavior but learned
only 2.6% of the correction.

The next bounded experiment separates responsibilities:

1. freeze the admitted production base policy;
2. learn only the steering delta between `precontact_teacher` and the historical
   `gap_teacher` on the same LiDAR state;
3. train with zero-delta normal anchors so normal racing remains unchanged;
4. keep the residual disabled in the production launch until independent
   validation and closed-loop gates pass.

This is an ML lateral residual, not a deterministic runtime teacher or another
threshold patch.
