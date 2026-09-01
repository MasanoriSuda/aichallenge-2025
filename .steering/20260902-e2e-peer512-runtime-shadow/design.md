# Design

Convert
`conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016`
with the existing converter.  Launch the deterministic `e2e-single` gate with
the converted checkpoint and expected SHA-256 while leaving
`TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED` unset/false.

Analyze the finalized run with the normal motion/competition gates and
`analyze_recurrent_shadow_run.py`.  This first gate checks artifact loading,
NumPy parity, shared Conv5 execution, hidden-state lifecycle and runtime timing.
It does not test peer interaction quality; that requires a later multi-vehicle
shadow run only if this gate passes.

## Invalid v1 run and root cause

The first launch failed before publishing a drive command because the runtime
constructed a 64-wide GRU from a duplicated YAML default before loading the
512-wide artifact.  This was a deployment-contract defect, not model evidence.

The converter now embeds the complete numerical construction contract in the
NumPy artifact.  Runtime reads that immutable contract before model
construction, validates the production input/steering boundary, and then
strictly loads only the weight set.  Partial metadata fails closed.  Legacy
diagnostic artifacts retain their explicit-config path, while startup logs
state whether the loaded contract is `self-described-v1` or `legacy-config`.

The regenerated artifact is:

- source PyTorch SHA-256:
  `10297e9484537d3c63f014050a25162e989f4edc3f7f5359af6a2c0501180e57`
- NumPy SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- artifact schema: `1`
- runtime hidden dimension: `512`
