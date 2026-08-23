# Follow row provenance audit requirements

## Purpose

Follow shadowの残存physical row certificate拒否を、row番号の手計算ではなく、solverが
観測した物理値・上下限・許容差とextended QPの意味へ一意に対応付ける。

## Scope

- solver境界でworst physical rowのvalue/projected/lower/upper/violation/toleranceを保持する。
- extended five-state QPのconstraint rowをkind/field/stageへ型付き変換する。
- Follow shadow決定ログへ上記provenanceを出す。
- rejected rowの発生条件を次のdynamic runで分類する。

## Constraints

- solver setting、tolerance、weight、wall margin、physical boundを変更しない。
- clamp、repair、retry、fallback、flagを追加しない。
- Follow shadowのauthorityを昇格しない。
- production command、ROS topic、parameter schemaを変更しない。

## Definition of Done

- malformed/out-of-range rowをfail-closedで分類するunit testを持つ。
- dynamics/state box/input box/curvature-rateの各rowが正しいstageへ対応する。
- solver拒否ログだけで物理値、bound、許容差、semantic fieldを特定できる。
- 追加観測から次の構造仮説を一つに絞る。観測前に挙動修正しない。
