# Design

## Current issue

`minimum lateral motion` は既にnearest-safe goalを生成するが、左右の必要移動量が
同程度でも、既存の幾何preferred sideへ戻る。これはカーブ前でインが空いていても
アウトを選ぶ原因になり得る。

## Selection policy

新規Overtake entryでのみ、次の順序で選択する。

1. executableな片側候補だけならその側
2. base racing lineを維持できる候補
3. カーブ内側の必要横移動が外側 + 許容差以下なら内側
4. それ以外は必要横移動が小さい側
5. 完全同値かつ内外不明なら既存preferred side

許容差は `v2x_overtake_minimum_motion_inner_preference_max_extra_shift` とし、
暫定値を0.30 mとする。0.0 mでは既存tie policyを維持する。

## Continuity

- `locked_pass_side != 0` では本選択を再実行しない。
- ShiftOut後のearly side replan、lateral progress、stable time制限を維持する。
- イン候補がguard/preflight不成立なら、許容差に関係なく選ばない。

## Diagnostics

既存V2X debug reasonへ、左右のminimum shift、base-line clear、inner sideを追加する。

## Verification

- core testで、内側が許容差以内なら選ぶことを確認する。
- 許容差を超えて余分に動く内側は選ばないことを確認する。
- 既存のbase-line優先、最小移動、候補なしテストを維持する。
