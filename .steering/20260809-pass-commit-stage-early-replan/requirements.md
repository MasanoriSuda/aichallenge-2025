# Requirements

## 目的

直近の `make dev2` 走行で残った、追い越し開始後に相手の横移動へ対応できず
Recoveryへ落ちる事象と、Direct Pass開始同周期に予測情報が未引継ぎのため
Recoveryへ落ちる事象を局所修正する。

## 対象事象

1. `Idle -> Pass` の同一制御周期で `locked_target_seen=false` のまま
   Pass horizon判定が走り、`fresh target prediction unavailable` になる。
2. ShiftOut開始後、相手が選択側へ寄る予測が見えても、候補安定待ち中に
   3.5 mのno-returnへ入り、代替側を評価・採用できない。
3. 実車体重複、壁接触、EmergencyBrake、solver異常は従来どおりhard faultとする。

## 制約

- 一般的なpredicted-overlap graceは広げない。
- 左右切替は完全な代替Missionが動的・静的preflightを通った場合だけ行う。
- SideBySide以降は左右切替を行わない。
- ROS 2 topic/service/message契約を変更しない。
- `aichallenge/result-summary.json` の既存ユーザー変更に触れない。

## Definition of Done

- Direct Pass開始周期は、選択済みMissionを保持して次周期のlocked-target観測を待つ。
- commit段階が純粋関数で判定される。
- ShiftCommitted中は予測横侵入を早期に検出し、代替Mission評価へ反映する。
- no-returnは車体が横並びになる直前の2.0 mへ寄せるが、代替Missionのpreflightは維持する。
- core単体テストと対象package buildが成功する。
