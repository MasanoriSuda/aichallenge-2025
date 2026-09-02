# Evidence

The material uses these frozen sources:

- packaged single: `output/20260902-e2e-submission-freeze-single`;
- packaged NPC seed 2037: `output/20260902-e2e-bounded-pace-packaged-seed2037`;
- independent NPC seeds 2035/2036:
  `output/20260902-e2e-bounded-pace-seed2035-rerun` and
  `output/20260902-e2e-bounded-pace-seed2036`;
- rejected peer Gate: `output/20260902-e2e-submission-freeze-peer-v2`;
- readiness report:
  `output/20260902-e2e-submission-freeze-peer-v2/e2e-submission-readiness.json`.

No production model, parameter or launch default is changed by this Slice.

Verification:

- Marp front matter parsed and the deck contains nine slides;
- all three referenced packaged/NPC competition reports exist;
- the video analyzer command matches the current CLI and verifies raw model
  identity, runtime mode, acceleration and speed cap;
- `git diff --check` passed;
- team name, public URL, screenshots and representative video frames remain
  explicit user-fill placeholders.
