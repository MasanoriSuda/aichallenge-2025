# Evidence

## Certified source

- run: `output/20260902-e2e-final-speed-committed-teacher-all-v2`;
- strict competition report SHA-256:
  `a41899daae2de204bdca07777299341a0ebccb3dfdbff1338e9d77cb7faf9703`;
- base checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`;
- all four domains: Finish 6/6, penalty 0, stall 0.

## Raw peer dataset

Ignored artifact root:
`dataset/speed_committed_final_peers_v1`.

| Domain | Samples | Material corrections | Max causal speed age [s] | Certificate SHA-256 |
|---|---:|---:|---:|---|
| d1 | 4,474 | 1,446 | 0.063576 | `6d74e04dd7e61cbee183abf4eb228f80b9fdab8cd835049f1d329aa17be7f1ca` |
| d2 | 4,337 | 1,440 | 0.060364 | `fe2d0049be0e8ffc02999ff196b3b483beef596f70e46dba6dd1be15cdd97872` |
| d3 | 4,371 | 1,019 | 0.055248 | `a458b6da2ccccf07db2113206a2517c93b4b5fe4d3e9ffd67e511d96ca9ccf4c` |
| d4 | 4,314 | 888 | 0.054245 | `9972f42cf75bd420e75fb0575e6f6ae29cd9061b2503b47b62707b21f791a0d9` |

Every scan is retained in timestamp order and every speed sample is the latest
preceding observation.  The first 50 ms extraction correctly failed because
three d1 samples were outside that stricter offline contract.  Re-extraction
at the actual 100 ms runtime freshness admitted every scan; no default was
relaxed.

Raw metadata SHA-256 values in d4/d1/d3/d2 sequence order are:

- `8721b2dbdc315ad6b5353ae072b1b053a9b480cfb5f413a043fffa509bc4419b`;
- `12e9123f134a17503220006de05387a1253a8088dd258f3df4dddf28166616f1`;
- `f6570a95eeb1aa1fd429eaf2ced511e512aadec58145a09a37a06eb10e15b8a8`;
- `91a70224bb3247357e8ba25f582288d2a70b9a3fbe0866df51247cb73f5fc9ef`.

## Coverage comparison

| Source | Samples | Material rate | Mean abs correction | Max abs correction |
|---|---:|---:|---:|---:|
| existing NPC seed-2034 train | 6,180 | 15.84% | 0.01919 rad | 0.63264 rad |
| final peer train | 17,496 | 27.39% | 0.03477 rad | 0.89667 rad |

The peer run contributes 4,793 material corrections, 427 side acquisitions,
4,223 maintained-side samples, 31 confirmed side switches and 13 bilateral
pinch samples.  This is a material interaction-coverage gain rather than only
four more normal laps.

## Recurrent derivative

`dataset/speed_committed_recurrent_peer_augmented_v1` combines:

- train: seed 2034 plus all four correlated final-world peer sequences;
- validation: independent seed 2033 only.

It contains 29,776 samples across five train sequences and one validation
sequence.  Manifest SHA-256:
`6f769c1d10d4662d371e61ea2aa6ffa32e970e7a6233923931b79f295d336190`.

## Decision

Train a separate peer-augmented projected-conv5 recurrent candidate using the
same frozen base, frozen production spatial model and independent-normal
anchor contract.  Compare it with the previous recurrent candidate offline
before any runtime connection.  Production authority remains unchanged.
