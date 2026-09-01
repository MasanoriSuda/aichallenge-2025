# Evidence

Run directory (generated, not committed):
`output/20260901-e2e-certified-precontact-seed2031`.

## Immutable gate

- world: `e2e-npc-single`, one ego and two runtime NPCs;
- random seed: `2031`;
- control mode: `precontact_teacher`;
- base checkpoint SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`;
- result-summary SHA-256:
  `9a003a36ddb271ef3e621c98d49f2f6447e7ff096646ffb74eed5f6c7a7f3cca`;
- result-detail SHA-256:
  `30dda51ee4c45f8f6258e6f455354cf9d857e442c4897561201be2e207ac60bc`;
- motion-analysis SHA-256:
  `68d34a6f5acadac2af0b6aa4ba3a64cf9503c846bf8d7b4a9bad5040100b3318`.

## Result

| Metric | Value |
|---|---:|
| laps | 103.9225 / 90.2950 / 99.1202 s |
| total | 293.3377 s |
| final position | 1 |
| penalties | 0 |
| distance | 1,030.188 m |
| longest low speed | 0 s |
| longest positive-acceleration stall | 0 s |
| mean / max speed | 3.3049 / 4.5192 m/s |

Both `analyze_e2e_run.py --fail-on-stall` and
`analyze_e2e_competition.py --fail-on-rejection` passed.  The output is
admissible as an executed teacher demonstration, but no dataset is generated in
this slice because the existing metadata schema cannot yet bind these outcome
artifacts to every derived sample.
