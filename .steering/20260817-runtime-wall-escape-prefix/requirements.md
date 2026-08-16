# Requirements

## Background

`e702df3` の実走 `output/20260817-082938` では、runtime wall warning 後の
target-bound progress extension は停止できた。一方、center contraction は5回とも
採用されず、同じ側を保持した後に壁・車両接触へ進んだ。

- full Mission lateral acceleration rejection: 3回
- physical-clearance goal unavailable: 2回
- center contraction accepted: 0回
- `Pass -> Return -> Idle`: 0回

現行center contractionは、現在位置からの退避だけでなく、残りPassとReturnまでを
一度に成立させる。このため、直近の壁を避けられる短い経路があっても、将来のReturn
制約で候補全体が棄却される。

## Goal

- runtime wall warning時は、Mission全体ではなく現在位置からの短い中央寄り退避prefixを
  先に成立確認する。
- prefix採用後もtarget、pass side、Pass進捗、front-cap releaseを維持する。
- prefixの先は既存のrolling replan / receding-horizon処理へ戻す。
- prefixを作れないときは、危険な同じ側を壁接触まで保持しない。

## Constraints

- actual wall contact、hard wall margin、map unavailableは従来どおりhard faultとする。
- 現在車体がtargetと分離していること、およびphysical target separationを下回らないことを
  prefix採用条件に残す。
- launch、topic、service、Domain、result JSON schemaは変更しない。
- ユーザーの未コミット変更である`config.yaml`の操舵・遅延設定と
  `aichallenge/result-summary.json`は変更しない。

## Definition of Done

- centerward候補は短区間prefixだけでwall・横加速度preflightされる。
- 採用時に残りPass/Returnを固定経路として検証済みとは扱わない。
- prefix不成立か再計画回数枯渇時は`HoldCurrentSide`ではなくMission handoffを要求する。
- core unit test、package test、buildが成功する。
