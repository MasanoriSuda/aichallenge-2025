# Evidence

## Immutable inputs

- recurrent corpus manifest SHA-256:
  `6f769c1d10d4662d371e61ea2aa6ffa32e970e7a6233923931b79f295d336190`
- frozen raw TinyLidarNet SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- previous recurrent checkpoint SHA-256:
  `bcd1652a31215be58b258b66fb301884863d3d2c1179932b35b1d05079a21304`

The valid comparison candidate is:

`checkpoints/conv5-recurrent-final-peers-v3-frozen-nospeed/20260902_063629`

- training manifest SHA-256:
  `b0e11c7f60688f88d91259f22abdf0e69691086a946b142a407c64ad94917227`
- training summary SHA-256:
  `583c29d1d559bab43d7967d168ccf957534df4dc7af03e04c388acdde7054324`
- checkpoint SHA-256:
  `1ab48ad834b64ba53f6016e4891ab2b1572278dbe080ea4c549ec340238a3a93`

Manifest comparison found no unexpected difference from the previous
candidate.  The intended differences are the train/output roots and explicit
0.100 s successor speed-sync contract.  Newly recorded CLI fields retain their
effective historical defaults (`direct`, direction weight 1.0).

## Speed freshness contract

The first training attempt stopped before model construction because the
peer corpus records the teacher runtime's 0.100 s causal speed freshness while
the recurrent loader silently required 0.050 s.  The four peer runs observed
maximum speed ages of about 0.054--0.064 s, within their executed runtime
contract.

Training and evaluation now keep 0.050 s as the default and admit a looser
contract only through explicit `--max-speed-sync-delta-sec`.  The selected
value is persisted in the training manifest and evaluation report.  A focused
test proves that a 0.100 s dataset is rejected by default and accepted only
when 0.100 s is requested.

## Invalid exploratory run

`conv5-recurrent-final-peers-v2-nospeed/20260902_063313` improved offline
metrics, but it is not admissible evidence.  It accidentally used CLI defaults
instead of the frozen candidate settings: hidden dimension 512 instead of 64,
speed embedding 64 instead of 16, material weight 2 instead of 5, normal
weight 1 instead of 5, and four workers instead of zero.  Its converted output
must not be packaged or granted runtime authority.

## Valid frozen comparison

Both candidates use a 0.02 rad deployment deadband.  Lower MAE is better.

| Dataset / metric | Previous | Peer-augmented | Change |
|---|---:|---:|---:|
| seed2033 all | 0.014210 | 0.016130 | +13.51% |
| seed2033 material | 0.055790 | 0.058742 | +5.29% |
| seed2033 anchor | 0.005930 | 0.007645 | +28.92% |
| seed2035 all | 0.013821 | 0.016077 | +16.32% |
| seed2035 material | 0.054414 | 0.059754 | +9.81% |
| seed2035 anchor | 0.005416 | 0.007034 | +29.87% |
| independent normal correction | 0.001966 | 0.002920 | +48.47% |

Seed2033 report:
`output/20260902-e2e-peer-frozen-v3-seed2033-gate.json`
(SHA-256 `0cef300fd23af6de7da66e7331461d6695496baa3b14eaed46d3bda5064b2e2f`)

Seed2035 report:
`output/20260902-e2e-peer-frozen-v3-seed2035-gate.json`
(SHA-256 `4b6df18c76947e39b099b836c1d8d664e0730bba005689b8ee846e104c7858f3`)

Seed2033 fails `full_validation_not_worse` and `unseen_not_worse`.
Seed2035 passes absolute gates but is consistently worse than the previous
candidate.  A single passing seed cannot override the regression on the fixed
development boundary and the direct old/new comparison.

## Decision

Reject the peer-augmented candidate.  Do not convert the valid candidate, do
not add it to a launch file, and do not run a closed-loop shadow experiment.
The evidence says that naively concatenating four correlated peer runs changes
the learned distribution in an unhelpful way under the frozen small model.
Future work must be a separate hypothesis, such as sequence/source-balanced
sampling or a deliberately reviewed capacity change, not a retry hidden in
this Slice.

## Verification

- focused recurrent suite: 28 passed
- complete TinyLidarNet workspace suite: 222 passed
- Python byte-code compilation: passed
- `git diff --check`: passed
- ROS topics, launch files, packaged checkpoints and runtime authority: unchanged
