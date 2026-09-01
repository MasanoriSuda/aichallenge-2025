# Evidence

Generated run (not committed):
`output/20260902-e2e-speed-committed-seed2033`.

## Immutable setup

- world: one ego and two NPCs (`e2e-npc-single`);
- random seed: 2033, unseen during design and offline replay;
- runtime mode: `speed_committed_teacher`;
- frozen base checkpoint:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.

## Strict result

| Metric | Value |
|---|---:|
| laps | 104.087 / 91.050 / 91.000 s |
| total lap time | 286.137 s |
| Finish | 3/3 laps |
| penalty | 0 |
| distance | 1,019.732 m |
| mean / max speed | 3.3468 / 4.4079 m/s |
| low-speed / positive-accel stall | 0 / 0 s |
| minimum front LiDAR range | 2.5866 m |

The motion gate and strict competition gate both passed.  Artifact hashes are:

- competition analysis:
  `7f84599004c711cb54beb5fac3841b16393b3eb43395b8b0ad5dfda1f91c97aa`;
- result summary:
  `d6c4b84cbb6906558e9aebdc1a129bf7ee7949341028b59a6e11be96c252e124`;
- result detail:
  `1ac82aa614f1683eaf401a4caab3ce0ecbf3c20d8888053877bc1ec03fb66c22`;
- motion analysis:
  `290158edab502c652f09c9659d322a0cabd8a37e858cdf318f66118881fee155`.

## Sequential teacher replay

Replaying the recorded 6,100 scans with synchronized wheel speed produced:

- 1,099 active teacher samples;
- 54 side acquisitions and 54 confirmed releases;
- 10 pending and 8 confirmed side switches;
- 661 steering samples differing from the historical teacher by at least
  0.02 rad;
- 10 acceleration samples differing from the historical teacher;
- no committed-side conflict, bilateral-pinch brake or stale-input event.

The minimum recorded front range stayed 2.55 m farther from contact than the
seed 2032 historical-teacher failure (0.035 m).  This is closed-loop evidence
that early temporal side commitment, rather than emergency braking, changed
the failed interaction outcome.

## Decision

The diagnostic teacher candidate is admitted as an executed successful source
for the next dataset slice.  This does not promote it to production authority.
Relabeling must preserve its distinct runtime identity and synchronize each
scan with fresh wheel speed; the historical `precontact_teacher` provenance
must not be reused.
