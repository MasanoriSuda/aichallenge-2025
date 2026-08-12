# Requirements

## 目的

並走状態で相手が横へ寄った際、回復可能な横接触用のContactContinuationが0.5 m/s境界でON/OFFし、locked targetに対するSafetyBrakeへ落ちて追い越しを手放す現象を解消する。

## 実測

`20260813-015134/d1`では次の連鎖を確認した。

- Pass中、前方速度capは解除済みで`desired_v=11.11 m/s`だった。
- ContactContinuationは相対横速度0.45 m/sで開始した。
- 0.52 m/sで解除され、0.47 m/sで再開するチャタリングが発生した。
- 実overlap付近で再び解除され、前方距離0.08 mのSafetyBrakeにより`Pass -> FollowPrepare`となった。
- ShiftOutからPassへ6回進んだが、Return完遂は1回だった。

## 要件

1. ContactContinuationの新規開始条件は従来の相対横速度0.5 m/s以下を維持する。
2. 一度開始した同じPassのContactContinuationは0.8 m/s以下まで保持する。
3. 保持中は既存のfull-speed forward escape、locked-target SafetyBrake抑制、壁内の分離バイアスを再利用する。
4. target不連続、前後2.5 m外、横離隔0.75 m未満、高いclosing、低速、時間超過、進捗停止は従来どおりfail-closedとする。
5. 壁接触・壁内で分離不能・solver faultは緩和しない。

## 非対象

- 相手側へ意図的に操舵する衝突目標
- 壁余裕の縮小
- ContactContinuation時間上限の延長
- 前車捕捉・pre-arm条件の変更

