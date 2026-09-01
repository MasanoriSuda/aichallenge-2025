# Requirements

## Objective

Run the admitted recurrent adapter as an explicit, bounded lateral-authority
experiment while keeping the packaged production default unchanged.

## Frozen inputs

- raw production SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- spatial production SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- recurrent candidate SHA-256:
  `8fe1bceb90fbbc09115fe91cd65e14e31472016b0777a42e9d7b585baef1aca6`

## Constraints

- Recurrent authority is default-off and requires an explicit checkpoint and
  expected SHA-256 identity.
- Authority may not exist without the admitted spatial production authority.
- The first experiment clips recurrent correction to `+/-0.24 rad`; this is an
  experiment bound, not a new packaged production setting.
- Missing/stale speed, watchdog reset, identity mismatch or inference failure
  preserves the already-valid spatial production command.
- No recurrent result may change acceleration authority.
- Do not package the recurrent checkpoint or promote it by this implementation
  alone.

## Acceptance

- Shadow-disabled and authority-disabled production outputs remain identical.
- Explicit authority applies only admitted finite correction and reports every
  application and clip.
- A single-vehicle three-lap run finishes with zero penalties, stalls, skips,
  errors and hidden resets.
- Production competition Gate and recurrent runtime Gate remain passing.
- A candidate is not eligible for packaging until an NPC run also demonstrates
  benefit or non-regression.
