# Evidence

## Scope

This slice changes offline evidence tools, tests, and submission documents
only.  Controller code, checkpoints, launch defaults, ROS interfaces, speed,
acceleration, braking, and steering are unchanged.

## Artifact identity

The development container independently hashed the source and installed
runtime artifacts:

```text
raw source  de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa
raw install de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa
spatial source  f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c
spatial install f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c
```

The strict readiness replay used the install paths as the runtime artifacts
and the participant-package paths as the source artifacts.  Artifact identity
passed.

## Frozen evidence replay

The replay generated all intermediate reports in the container's `/tmp`; no
file under `output/` was modified.

Single run: `output/20260902-e2e-submission-freeze-single`

- competition runtime contract: pass;
- 3/3 laps, penalty 0, stall 0;
- spatial coverage: `5838 / 5838 = 1.0`;
- spatial error/non-ok/stale: `0 / 0 / 0`;
- spatial authority applied: `5838` scans;
- single competition and spatial Gates: pass.

Peer run: `output/20260902-e2e-submission-freeze-peer-v2`, E2E Domain 3

- spatial coverage: `3919 / 3923 = 0.9989803722`;
- spatial error/non-ok/stale: `0 / 0 / 0`;
- spatial authority applied: `3919` scans;
- race: 1/3 laps, one crash penalty;
- longest low-speed interval: `54.914067918 s`;
- motion, competition, and spatial Gates: reject.

The final schema-v3 readiness classification is therefore still
`single-vehicle-candidate-only`.  The stricter evidence chain preserves the
previous conclusion while proving the production runtime contract and actual
spatial execution.

## Tests

Focused host tests with third-party pytest auto-loading disabled:

```text
37 passed in 0.16s
```

Complete TinyLidarNet suite in the Autoware development image:

```text
268 passed in 2.32s
```

`py_compile`, critical flake8 checks (`E9,F63,F7,F82`), and
`git diff --check` passed.

## Remaining submission work

- Record the final seed-2037 video after the evidence tooling is committed.
- Run the same strict competition and spatial Gates on that fresh recording.
- Fill the team name and public video URL, render the Marp PDF, and generate
  the final submission archive and archive SHA.
- Do not claim mixed-peer qualification; its frozen Gate remains failed.
