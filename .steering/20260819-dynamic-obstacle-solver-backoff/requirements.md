# Requirements

## 背景

`20260819-201236` の試走では、動的障害物 lateral escape 導入前の毎周期 solver
failure は大幅に減った。一方、tracking solve に失敗した同じ target / side が固定
0.5 秒の quarantine 終了直後に再採用され、再び OSQP maximum iterations へ入る
低周期ループが残った。

また従来の quarantine は全候補共通であり、片側の solve failure が反対側の安全な
候補評価まで停止していた。

## 要求

- tracking solver failure を target ID と pass side の組で記憶する。
- 同一候補の連続失敗時は再試行間隔を段階的に延ばす。
- 片側の backoff 中も反対側候補を評価可能にする。
- 反対側も成立しない場合は通常 Follow の速度capを維持し、失敗候補の横境界を
  tracking MPCへ漏らさない。
- tracking solve が成功した候補だけ、その候補の失敗履歴を解除する。
- EmergencyBrake、既存solver recovery、明示Overtake Missionの優先順位を変えない。

## 対象外

- OSQP内部設定・重み・horizonの変更
- Overtake Missionの候補ランキング変更
- Recovery / Reverseの変更
- ROS topic、service、message契約の変更

## Definition of Done

- 同一target/sideのholdが `0.5 -> 1.0 -> 2.0 -> 4.0 s` と増加する。
- 反対sideおよび別targetは同じholdで遮断されない。
- 成功または8秒以上の失敗間隔で履歴がリセットされる。
- core単体テストと `make autoware-build` が成功する。
- ユーザー既存の生成物変更をコミットしない。
