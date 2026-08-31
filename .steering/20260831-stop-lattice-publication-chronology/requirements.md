# Requirements: Stop-lattice publication chronology

## Objective

Fix the asynchronous provenance defect which discards a newer Pass
Stop-lattice result merely because its producer-local artifact sequence is
smaller than an older ShiftOut artifact sequence.

## Frozen evidence

- baseline: `fec9133e`;
- run: `output/20260831-091516/d1`;
- failure decision: `1832`;
- snapshot fingerprint: `ea08829bc110efca`;
- ordinary Pass authority: `terminal-contingency-unavailable`;
- available alternate: ShiftOut source sequence `954`, rejected as
  `intent-mismatch`;
- Pass normal source sequence: `295`;
- Stop-lattice mailbox: `rollback=1` after the Pass result completed;
- same-snapshot terminal audit: rule-based Stop and all 128 lateral targets
  failed exact wall proof, while the seven-state Stop and steering lattice
  passed the unchanged exact wall and dynamic proof chain.

## Constraints

- Do not change production authority, solver settings, physical clearance,
  timeout, lease, grace period or fallback policy.
- Do not permit an artifact from the wrong intent to execute.
- Preserve exact source identity matching before a Stop-lattice result becomes
  the current-world alternate.
- Order asynchronous publication by a chronology which is global across
  normal-intent producers.
- Reject stale completion from an older control decision even if its local
  artifact sequence is numerically larger.

## Definition of done

- The mailbox accepts a newer control decision with a smaller local artifact
  sequence.
- The mailbox rejects an older control decision with a larger local artifact
  sequence.
- Consumer watermarks use the same chronology key as publication.
- Existing exact identity join remains mandatory.
- Focused tests, package build and full package CTest pass.
- A bounded dynamic run shows no sequence rollback across a ShiftOut/Pass
  producer transition and, when the condition occurs, a Pass-compatible
  Stop-lattice candidate is visible rather than the stale ShiftOut artifact.
