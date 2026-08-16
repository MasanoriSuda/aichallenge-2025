# Requirements

## Goal

相手車両の将来予測だけが一時的に不成立になった場合に、壁が成立している追い越し Mission を即座に破棄して Follow / Recovery へ戻さない。

## Evidence

- 対象走行: `output/20260817-072813`
- `Idle -> ShiftOut`: 15 回
- `ShiftOut -> Pass`: 10 回
- `Pass -> Return -> Idle`: 1 回
- Recovery 遷移: 8 回
- 主な失敗理由:
  - `optimized horizon escaped target separation bounds`
  - `physical target separation conflicts with wall bounds`
  - `optimized horizon failed physical revalidation`
- 既存の target-bound execution hold は動作しているが、初期サンプルでの target/wall 区間衝突が target-only と分類されず、保持経路を通らない。
- ShiftOut 未完了時は保持対象外のため、予測チャタリングで Mission を失いやすい。

## Required behavior

1. receding-horizon の失敗所有者を target / wall / physical / invalid に分類し、文字列理由から制御判断を推測しない。
2. target separation と wall bounds の区間衝突は target-only failure として既存の再計画保持へ渡す。
3. Pass または ShiftOut 完了後は、既存の last-feasible same-side prefix を保持する。
4. ShiftOut 未完了では、古い横移動を継続せず、現在横位置を短時間保持して左右候補を即時再評価する。
5. ShiftOut 保持は Pass より短い時間・距離で打ち切り、進捗による延長は行わない。
6. 壁接触、壁余裕不成立、壁サンプル欠損、実車体の非回復接触、緊急制動、solver recovery、禁止区間は従来どおり hard failure とする。

## Non-goals

- Recovery / Reverse の性能変更
- 車体寸法・壁余裕・追い越し clearance の攻撃化
- ROS topic / service / result schema の変更
- フル MPCC への置換

## Definition of Done

- target-only failure が型付きで呼び出し側へ伝播する。
- ShiftOut target-only hold に専用上限を持つ。
- core 単体テストが hard fault と budget の境界を確認する。
- `multi_purpose_mpc_ros` の build/test が成功する。
