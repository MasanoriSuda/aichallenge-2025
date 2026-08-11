# Requirements

## 目的

前方の低速車を捕捉中に、未実行のMission候補が1周期成立しないだけで1秒間の左右探索禁止に入る現象を解消する。

## 要件

- Idle/Follow中のMission候補全棄却は、side retry cooldownをarmしない。
- 次周期以降も同一targetの左右候補を再評価できること。
- ShiftOut/Pass/Recovery中の壁違反、実行経路破綻、Mission timeoutなどは従来どおりcooldown対象とする。
- emergency front risk、wall/footprint、solver、target continuityのhard guardは緩和しない。
- ROS 2 interface、launch、configの契約は変更しない。
- 古い横経路を実行せず、ShiftOut開始時は現周期の検証済みMissionを必須とする。

## 対象外

- rear-clear/Returnを入口時に必須とするMission成立条件の変更
- 壁余裕、gap幅、横加速度上限、速度上限の調整
- committed Mission中の左右切替え方針の変更

## Definition of Done

- 探索missと実行失敗のcooldown方針がpure coreで単体テストされる。
- 候補探索全棄却で`ShiftOut geometry retry cooldown`がarmされない。
- 実行失敗系のcooldownは残る。
- `make autoware-build`と対象packageの単体テストが通る。
