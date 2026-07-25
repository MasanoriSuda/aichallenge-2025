# Design

## 方針

既存の `Return -> Pass` 再獲得とは分けて、Recovery 専用の判定を追加する。
Recovery は中央線へ戻る途中なので、再開先は `Pass` ではなく `ShiftOut` とし、
現在横位置から再度 pass-side line を生成する。

## 再開条件

- 機能が設定で有効
- Recovery の最低 phase hold が経過
- stable な同一 target ID
- locked target の course progress が連続
- 同一 pass side
- target の rear-clear が未完了
- gap/corridor が現在有効
- 追い越し zone・危険度・cooldown が実行を許可
- solver recovery/re-entry gate が解除済み

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`
  - Recovery 再獲得の純粋判定を追加
- `mpc_controller_cpp.cpp`
  - config 読み込みと `Recovery -> ShiftOut` 遷移を追加
- `config/config.yaml`
  - 競技シミュレーション向けに機能を有効化
- `test/test_v2x_overtake_core.cpp`
  - 許可条件と fail-closed 条件を追加

## リスク抑制

新しい corridor が選ばれても、既存の actual footprint wall check、static wall clamp、
lateral acceleration feasibility check は ShiftOut 側で引き続き適用される。
solver 起因の Recovery は re-entry gate が解除されるまで再開しない。
