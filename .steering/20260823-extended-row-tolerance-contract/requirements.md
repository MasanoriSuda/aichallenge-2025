# Requirements: extended MPCC row tolerance contract

## Purpose

Overtake canonical fresh shadow が、OSQPから成功解として返された5-state解を
最初の予測状態の横制約違反で棄却する原因を特定し、物理単位ごとの制約契約を
solver受理条件まで一貫させる。

## Observed behavior

- `stage=0` の横制約違反が約 `0.057-0.097 m`。
- 当該行の許容差は約 `0.0164 m`。
- その周期のextended solverは成功解を返すため、legacy変換経路はその解を利用できる。
- canonical shadowだけが横制約契約を再検証し、正しく棄却している。

ここでの`stage=0`は現在状態x0ではなく、最初の予測状態x1のbox rowである。

## Constraints

- 横境界の拡幅、許容差の緩和、primalのclampは行わない。
- OSQPのepsやwall margin等のparameter tuningは行わない。
- Overtake production authorityは昇格しない。
- Overtake canonical候補を生成するlive/左右branch solverの受理規約を、
  後段の行別物理証明と一致させる。
- 既にproduction昇格済みのTrack/Cruiseへ、未監査の受理規約変更を波及させない。
- `aichallenge/result-summary.json`はユーザー変更として触れない。

## Definition of Done

- Overtake/Followのmixed-scale QPで各constraint rowの物理許容差を超える解を
  成功扱いしない。
- Overtake live solverと左右branch、および既に同契約のFollowが
  `RowToleranceNormalized`を使う。
- Track/Cruiseは本slice前のsolver policyを維持し、同一bag replayでsolve-failureを
  増やさない。
- package buildと全テストが成功する。
- 同一bag replayでOvertake canonical shadowのlateral contract通過率を確認する。
- 計算時間、solver failure、callback overrunの退行を報告する。
