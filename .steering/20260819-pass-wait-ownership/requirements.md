# Requirements

## 背景

`output/20260819-093908`では非同期Mission配送は復旧した一方、Pass中に一時的な
target/wall制約競合が発生するとDynamicMissionWaitへ移行し、rear-clear未成立のまま
約0.8秒でIdleへ戻った。その直後にSafetyBrake、壁接触、Stuck Recoveryへ進んだ。

## 目的

- Passでforward-completionをcommit済みの場合、短い再選択期限だけでMission所有権を
  手放さない。
- rear-clear、Mission総予算、target異常、物理壁接触など既存の終了条件は維持する。
- 新しい追い越し戦術、速度上限、安全余裕の変更は行わない。

## 変更範囲

- DynamicMissionWait保持判定
- controllerから保持判定へ渡すcommit snapshot
- 純粋関数の回帰テスト

## 非対象

- 壁・車体寸法パラメータの攻撃化
- SafetyBrake閾値変更
- Recoveryアルゴリズム変更
- MPCCモデル・重み変更
