# Follow effective-gap constraint

## Purpose

Follow MPCCの車間制約をvirtual progress単体ではなく、実際のFrenet縦位置
`progress + lag`に対して課し、canonical obstacle certificateを作れる定式化へ直す。

## Earliest violated invariant

現行QPは`progress <= target_progress - hard_gap`を課す一方、world poseは
`course_frame(progress) + lag * tangent`で復元する。正のlagを持つ解は実車体が制約より前へ出るため、
virtual progress上の合格だけでは物理hard gapを証明できない。

## Scope

- Follow contractへhard gap identityを保持する。
- 各stateへ`lag + progress <= target_progress - hard_gap`の線形制約を追加する。
- solver row provenanceへFollow effective-gap rowを追加する。
- solve後も同じ式で物理gapを検査する。
- Track/CruiseおよびOvertakeのQP行列を変更しない。

## Non-scope

- Follow canonical command生成とpublisher接続。
- hard gap値の調整。
- solver tolerance、iteration、scaling変更。
- lateral target footprint回避。

## Acceptance

- 正のlagでvirtual progressだけなら通るfixtureが新制約で拒否される。
- zero/negative lagの既存Follow contractは成立する。
- Follow以外のconstraint layoutは不変。
- build/testを通し、runtime authorityはlegacyのまま。
