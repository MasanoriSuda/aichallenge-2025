# Evidence

## Contract tests

- targeted provenance and dataset tests: `51 passed`;
- complete TinyLidarNet test suite: `184 passed`;
- Python syntax compilation: pass;
- production checkpoint and runtime configuration: unchanged.

Negative tests reject a wrong run identity, domain, control mode, checkpoint,
artifact hash and embedded source identity.  The production-normal builder and
teacher relabeler now share the same successful-run validator.

## Strict seed-2031 extraction

Generated, ignored dataset:
`aichallenge/ml_workspace/tiny_lidar_net/dataset/certified_precontact_seed2031_v1`.

| Item | Value |
|---|---:|
| accepted samples | 6,244 |
| material correction (`abs(delta) >= 0.02 rad`) | 660 (10.570%) |
| mean absolute correction | 0.011704 rad |
| p95 absolute correction | 0.054366 rad |
| maximum absolute correction | 1.055873 rad |
| successor-vs-old-teacher material difference | 135 samples |

Outcome certificate SHA-256:
`45d29495974e6d6c2d9a8fa63e2389c3f9558d8a6bdd500af1a5f00821a04a0d`.

The certificate binds the exact seed-2031 source bag to:

- base checkpoint `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`;
- competition analysis `11bea9117e27234864075932ec53f114d44821666d71c696b05882470a926094`;
- result summary `9a003a36ddb271ef3e621c98d49f2f6447e7ff096646ffb74eed5f6c7a7f3cca`;
- result detail `30dda51ee4c45f8f6258e6f455354cf9d857e442c4897561201be2e207ac60bc`;
- motion analysis `68d34a6f5acadac2af0b6aa4ba3a64cf9503c846bf8d7b4a9bad5040100b3318`.

## Decision

Seed 2031 is admissible as one hard teacher train sequence.  Training remains
blocked because a second run-disjoint successful teacher sequence has not yet
been certified for validation.  Reusing a failed or alternative-policy run as
validation would recreate the supervision ambiguity this slice removes.
