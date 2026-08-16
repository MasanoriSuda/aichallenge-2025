# Design

## Overview

既存の二つの機構を接続する局所修正とする。

- target-only conflict: last-feasible execution prefix を短時間保持
- runtime wall preplan: fresh same-side Mission または center contraction を生成

現状は target-only progress extension が runtime wall warning を知らず、center contraction は nominal target separation を満たせないと候補を生成しない。この隙間で `HoldCurrentSide` が選ばれ、壁接触まで同じ経路を延長している。

## Wall-aware target-bound budget

`TargetBoundExecutionHoldRequest` に `wall_preplan_warning` を追加する。

- short repair budget 内: 既存どおり wall-validated prefix を保持可能
- short repair budget 超過後: `wall_preplan_warning == true` なら progress extension 不可
- actual wall hard fault: 既存どおり即時不可

これにより一時的な optimizer gap は吸収しつつ、既知の壁接近方向を Mission-wide absolute budget まで延長しない。

## Physical-clearance center contraction

center contraction の目標を二段階で解く。

1. nominal center separation (`max(configured separation, physical + prediction margin)`)
2. nominal が wall interval に収まらない場合だけ physical center separation

physical fallback は以下を全て満たす場合に限定する。

- Pass または ShiftOut の active execution
- locked target continuity が有効
- current body footprints が非重複
- 現在の ego が selected pass side にあり、物理中心間隔を保っている
- wall hard fault なし
- bounded centerward goal が存在
- ShiftOut/Pass path の wall / lateral acceleration preflight が成立

これは target guard の撤廃ではない。入口で採用する通常 Mission は robust clearance を維持し、壁接近後の実行可能な脱出補正にだけ物理境界を使う。MPC の current-body hard guard は残る。

## Logging

center contraction の採用ログに `clearance=nominal|physical` と使用した中心間隔を記録する。`HoldCurrentSide` には contraction reject reason を記録し、次回走行で候補欠落理由を確認できるようにする。

## Compatibility

- config key は追加しない。既存の `runtime_wall_center_contraction_enabled` と最大補正量を利用する。
- local `config.yaml` にあるユーザー未コミットの操舵・遅延変更は変更対象外とする。
- launch、topic、service、Domain、result schema は不変。
