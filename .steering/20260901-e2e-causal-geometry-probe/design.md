# Design

Use a compact trainable 1D CNN per frame to preserve angular locality, append a
small speed/base context embedding, and feed the resulting token to a
unidirectional GRU.  The model emits three diagnostic action logits at each
timestep.

Training uses 32-frame causal chunks with an eight-frame burn-in and 16-frame
stride.  No chunk crosses a sequence.  Validation replays each complete run in
timestamp order, carries hidden state only within that run, and resets between
runs.  This is materially different from the rejected fixed 1/8-frame delta
MLP and from the old direct-policy GRU that lacked the current normal contract.

Class weighting uses sample counts; long successful runs retain their natural
mass.  Normal sequences are explicit neutral examples with real synchronized
speed.  The same material threshold and immutable validation tokens are used as
the static action probe.

Acceptance requires repeatable gains in balanced accuracy and material sign,
normal false-material no worse than the projected static baseline, and
peer/focus-tail preservation.  Otherwise the observation/label contract, not
model capacity, becomes the next root-cause target.

## Result

The causal geometry probe improved aggregate failure-action separability over
the frozen projected static baseline, but failed the production-preservation
contract:

| Seed | Balanced accuracy | Material sign | Normal false material | Focus sign | Focus tail sign | Peer sign |
|---|---:|---:|---:|---:|---:|---:|
| 2026 | 0.92137 | 0.95126 | 0.13025 | 0.95683 | 1.00 | 1.00 |
| 2027 | 0.92627 | 0.95039 | 0.10258 | 0.95863 | 0.50 | 1.00 |
| 2028 | 0.92623 | 0.94517 | 0.11439 | 0.94604 | 1.00 | 1.00 |

The projected static baseline means were 0.89906 balanced accuracy, 0.90455
material-sign accuracy and 0.08959 normal false-material.  Every causal seed
increased false intervention on successful normal driving, and seed 2027 lost
one of the two material actions in the frozen focus tail.  The gain therefore
does not justify an offline continuous residual candidate or runtime export.

This closes the current representation search.  Static and causal models can
learn the heuristic labels better, but additional capacity does not reconcile
the fact that the current teacher asks for material intervention on 6.20% of
admitted successful-normal samples.  The next slice must redesign the
teacher/normal supervision contract using run outcome and intervention
necessity; it must not add another model family, runtime threshold or normal
sampling ratio.
