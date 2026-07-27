# Requirements

## 目的

開始時に検証済みのOvertakeLineが、V2X Behaviorの新規候補判定の一時的な
不成立だけで`ShiftOut -> Recovery`へ落ち、同じ対象を直後に再取得する
チャタリングを解消する。

## 根拠

`output/20260727-234145/d1`のP1では次を確認した。

- `Idle -> ShiftOut`: 20回
- `ShiftOut -> Recovery`: 37回（Recovery中の再試行を含む）
- `ShiftOut -> Pass`: 2回
- `Pass -> Return`: 0回
- `reason=locked target no longer executable`: 32回
- `same target gap reacquired during recovery`: 19回

代表例では、locked targetの観測と進捗は連続しているにもかかわらず、
Behaviorが`Overtake -> Cruise`へ1周期切り替わった直後にRecoveryへ入り、
約0.3秒後に同じtarget・同じsideでShiftOutを再開している。

## 変更範囲

- commit済みShiftOut / PassのBehavior drop継続条件
- pure helperと単体テスト
- MPC暫定仕様

## 制約

- 新規Overtake開始条件は緩和しない。
- locked targetの位置ジャンプ、観測期限切れ、進捗不連続は継続しない。
- pass側侵入、live execution corridor不成立、明示禁止WP、Emergencyは継続しない。
- actual footprint wall、static wall、横加速度、solver failureのRecoveryを維持する。
- Recoveryから安全な空きへ再取得する機能は維持する。
- ROS topic/service、Domain、評価成果物の契約は変更しない。

## 完了条件

- commit済みShiftOut / Passでは、soft curve、hard curve、完遂距離、
  cooldownなど新規候補用の再判定だけを理由にRecoveryへ落ちない。
- target・corridor・壁・衝突に関する実行時hard guardは維持する。
- 対象packageの単体テストとビルドが成功する。
- 実走効果はユーザーの`make dev2`で確認する。
