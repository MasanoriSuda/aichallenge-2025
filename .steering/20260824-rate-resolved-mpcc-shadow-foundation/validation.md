# Validation

## Static result

- `git diff --check`: pass.
- `make autoware-build`: 25 packages succeeded.
- Correct workspace test root:
  `/aichallenge/workspace`, not the stale `/aichallenge` build tree.
- `colcon test --packages-select multi_purpose_mpc_ros`: 41/41 CTest
  targets passed; 1,808 tests, zero errors, failures, or skipped tests.
- New `test_mpcc_rate_resolved`: five tests passed.
- `mpc_controller_cpp` link command contains no
  `libmulti_purpose_mpc_ros_mpcc_rate_resolved` dependency.

The final `colcon test-result` also printed an existing workspace-wide warning
about a missing `build/joycon_contract_guard/package.xml`; it did not belong to
the selected package and did not change the zero-failure package summary.

## Failure found during validation

The first CMake declaration used `ament_auto_add_library`. Link inspection
showed that `ament_auto_add_executable` consequently linked the new library
into `mpc_controller_cpp` even though no symbol was called. This violated the
shadow isolation requirement.

The library is now a normal static CMake target linked only by its test target.
Its source/include contract is explicit, and the production executable is
physically unchanged at link time.

An initial test invocation accidentally targeted the stale
`/aichallenge/build` tree and reproduced two old ABI-mismatch crashes. The
authoritative `/aichallenge/workspace/build` tree passed completely. This is
recorded so those stale results are not misclassified as a controller
regression.

## Acceptance

The mathematical foundation is accepted. It has no production authority and
does not justify publication. The next Slice must assemble a complete
six-state QP in shadow, with its own row semantics, scaling, warm-start
lineage, timing, and exact physical proof before any canonical schema or
publisher change.
