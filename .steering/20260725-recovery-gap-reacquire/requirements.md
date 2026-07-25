# Requirements

## 目的

追い越し失敗後の `OvertakeLine::Recovery` 中に、同一ターゲット・同一追い越し側の
実行可能な空き corridor が再成立した場合、Recovery 完了まで待たずに追い越しを
再開できるようにする。

## 制約

- Recovery の moving-Follow 速度制御は維持する。
- solver recovery/cooldown 中は再開しない。
- ターゲット ID、追い越し側、V2X course progress が不連続な場合は再開しない。
- EmergencyBrake、追い越し禁止区間、実行 corridor 不成立時は再開しない。
- Recovery 開始直後は既存の phase hold 時間だけ再開を抑止する。
- ROS 2 topic/service 契約は変更しない。

## 完了条件

- Recovery 中の安全な再獲得条件を純粋関数で単体テストできる。
- 条件成立時に `Recovery -> ShiftOut` へ遷移する。
- 条件不成立時は従来どおり Recovery を継続する。
- 対象 package のビルドと単体テストが通る。
