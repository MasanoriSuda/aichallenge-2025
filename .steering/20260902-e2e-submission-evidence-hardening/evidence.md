# Evidence

## Frozen run revalidation

The packaged single-vehicle freeze was re-run through the strict competition
analyzer with all production expectations supplied:

- raw checkpoint path and SHA-256: pass;
- residual checkpoint path empty: pass;
- spatial checkpoint path and SHA-256: pass;
- spatial base-steering input enabled: pass;
- spatial authority enabled with `1.2 rad` correction limit: pass;
- recurrent checkpoint path empty and recurrent authority disabled: pass;
- Finish 3/3, penalty 0, motion admission: pass.

Input: `output/20260902-e2e-submission-freeze-single`

The existing peer freeze was independently converted into a competition
report for domain 3.  It fails for all expected reasons:

- motion admission failed;
- race not finished (`1/3` laps);
- crash penalty count `1`;
- longest low-speed interval `54.914 s`.

Input: `output/20260902-e2e-submission-freeze-peer-v2`

The schema-v2 readiness audit therefore remains
`single-vehicle-candidate-only`, with both mixed-peer motion and competition
Gates reported independently as failed.

Generated revalidation reports were written only under `/tmp`; immutable
`output/` evidence was not edited or committed.

## Tests

Focused host tests with third-party pytest auto-loading disabled:

```text
22 passed in 0.07s
```

Complete TinyLidarNet suite in the Autoware development image:

```text
258 passed in 2.13s
```

Critical flake8 checks (`E9,F63,F7,F82`) and `git diff --check` passed.

The repository documents `pre-commit run -a`, but `pre-commit` is not
installed on either the host or the current Autoware development image.  No
dependency was installed for this evidence-only slice.

## Production impact

No participant controller, launch default, checkpoint, speed, acceleration,
braking, steering, topic, or service contract changed.
