# Requirements

## Objective

Test whether full spatial geometry plus short temporal state can distinguish the
successor teacher's left/neutral/right correction on the frozen combined split.

## Constraints

- diagnostic classification only; no steering checkpoint or runtime path
- frozen candidate3 supplies both base steering and frozen features
- validation sequence identities remain untouched, especially peer d3
- all normalization statistics come from train only
- compare compact/static, spatial/static and spatial/temporal representations

## Definition of Done

- three variants use the same labels, split, optimizer class and admission data
- per-sequence support and errors are visible
- peer d3 small material support is reported, not hidden in aggregate metrics
- next architecture decision follows from measured temporal gain
