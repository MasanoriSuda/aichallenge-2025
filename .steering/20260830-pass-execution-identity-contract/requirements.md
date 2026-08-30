# Requirements

## Objective

`output/20260830-145407` で、Pass開始後に物理的に有効な反対側branchが存在するにもかかわらず、normal authorityが消えてEmergency Stopへ落ちた根本原因を修正する。

## Observed causal chain

1. decision 1850で戦術phaseはShiftOutからPassへ遷移した。
2. Pass artifactがjoinするまで、published ShiftOut artifact sequence 1198がatomic admissionにより実行を継続した。
3. 現側のlegacy OvertakeLine wall corridorが不成立となり、`stage_corridor.active` が0へ落ちた。
4. canonical execution identityが `overtake_line_output.active && stage_corridor.active` に従属していたため、Pass Mission identityまで消失した。
5. Pass問題とsibling adoptionの評価scopeが作られず、decision 1902でEmergency Stopとなった。
6. 後着の同一epoch branch結果では現側が不成立、反対側がcertifiedだったが、authority喪失後だった。

## Constraints

- Mission resume rule、lease、grace、timeout、fallbackを追加しない。
- solver tolerance、wall/vehicle clearance、速度設定を変更しない。
- legacy corridor不成立を安全とみなさない。
- 物理wall proof、dynamic obstacle proof、publisher境界の証明を迂回しない。
- production authorityの所有者を増やさない。

## Definition of Done

- canonical execution identityは、戦術Mission identityとstage corridor availabilityを別契約として扱う。
- stage corridorが一時的にinactiveでも、current-world Pass problemを再構築できる。
- 現側不成立時に既存のsame-epoch sibling adoption経路を評価できる。
- 単体・source-contract testとfull buildが通る。
- dev2で、少なくともidentity消失を示すPassの`intent-mismatch`が再発せず、
  live Passがseven-state normal authorityを取得できることを確認する。
- 別のrejectionが残る場合は本修正へ混ぜず、独立したfailure snapshotとして固定する。
