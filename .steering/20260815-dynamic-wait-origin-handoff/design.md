# Design

## 方針

### 1. target-bound hold を Pass 専用から committed execution 用へ一般化

既存の target-bound Pass hold を、次の実行局面で利用する。

- `Pass`
- `ShiftOut` かつ lateral ShiftOut 完了済み

保持対象は現在の物理的に成立する同側 prefix のみとし、target-only の最適化失敗を
phase failure に変換しない。前回 horizon の再利用時は、Mission generation、side、
現在 phase が一致することを要求する。

### 2. DynamicMissionWait の長期保持を origin/commit/prefix で制限

短い再選択期限を越えて rear-clear まで保持できるのは、次を全て満たす場合だけとする。

- origin が `Pass`、または no-return / cross-side commit 後
- 前周期に安全な DynamicMissionWait forward prefix を実行している

pre-no-return の未完了 ShiftOut は 0.75 秒または設定距離で Mission を解放する。
既存の side retry block により失敗側を一時抑止し、反対側を含む新規候補評価へ戻る。

### 3. 局所リファクタリング

Pass 固有に見える target-bound hold の core API・状態名を execution 名へ変更し、
ShiftOut/Pass 共通の責務を明示する。保持可否は pure function と単体テストで固定する。

## ログでの期待値

- lateral 完了済み ShiftOut の target-bound 不成立:
  `target-bound execution hold started ... phase=ShiftOut`
- pre-no-return ShiftOut の再選択不成立:
  `dynamic Mission wait released for fresh search ... origin=ShiftOut`
- 同条件で 15 秒 Mission budget expiry まで待たない。
- hard fault 時は execution hold が開始されない、または即解除される。
