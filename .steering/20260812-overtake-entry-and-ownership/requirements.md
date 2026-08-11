# Requirements

## 背景

`output/20260812-083252` では、前方車を捕捉していても新規 Mission の完全成立待ちで
Follow を継続し、成立後の ShiftOut 中にも一周期の locked-target 判定欠落で
Overtake/Follow が反復した。5 回の追い越し開始はすべて Pass へ到達した一方、
Pass -> Return は 0 回だった。

## 目的

- 完全な ShiftOut/Pass/Return Mission がまだ成立しない間も、安全な body-clear
  候補を使って縦速度を準備する。
- frozen Mission の ShiftOut/Pass 実行中は、新規 candidate search の失敗だけで
  Behavior を Follow へ戻さない。
- Pass 中の短時間の horizon 不成立では、hard fault がない限り Mission を保持して
  再評価する。

## 制約

- 壁接触・壁余裕違反、現在車体の確認済み重複、EmergencyBrake、target identity
  不連続、solver recovery は従来どおり hard fault とする。
- body-clear 候補は横経路の実行権を持たず、base trajectory 上の速度準備だけに使う。
- ROS 2 topic/service、提出インターフェース、評価基盤は変更しない。
- `aichallenge/result-summary.json` の既存変更は触らない。

## Definition of Done

- body-clear 可能だが rear-clear/Return が未成立の候補で bounded pre-arm が成立する。
- active ShiftOut で同一 target の一周期の幾何判定欠落があっても Behavior ownership が残る。
- hard fault では従来どおり ownership が解除される。
- pure core の単体テストと対象 package build が成功する。

