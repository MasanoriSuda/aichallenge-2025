# Design

## Causal chain

```text
canonical solve/preparation failure
  -> safe_failure_control creates an uncertified deceleration/path command
  -> node-level crawl or Dynamic Escape continuation rewrites speed/steering
  -> command publishes without CanonicalNormalCommand identity
  -> final trace records LegacyNormalBypass
```

The qualification hold is the same architectural defect one stage earlier: it clears the failed
candidate identity, copies the prior ordinary command for one cycle and deliberately clears the
fallback flag.  This hides the authority loss from final arbitration.

## Repair

1. Keep `safe_failure_control()` only as an Emergency deceleration producer.  It may preserve and
   rate-limit steering while stopping, but it no longer authorizes positive-speed crawl or lateral
   continuation.
2. Delete the qualification hold.  Candidate qualification failure records/quarantines the failed
   target/side and then follows the same Emergency path.
3. Remove node-level crawl/continuation arbitration and their configuration.
4. Map solver fallback and executed-solution wall hold to `EmergencyOverride`.  The latter remains a
   safety supervisor action, not a normal trajectory owner.
5. Remove `LegacyNormalBypass`; any certified normal request must satisfy the complete canonical
   identity contract.

## Expected behavior change

Only abnormal solver/preparation failure cycles change.  They now decelerate through explicit
Emergency instead of moving under an uncertified helper command.  Successful fresh/retained MPCC
cycles are unchanged.

## Rollback

Rollback this Slice as one commit if fault-injection or dynamic smoke testing shows an authority
contract regression.  Do not restore crawl/continuation as a second normal controller.
