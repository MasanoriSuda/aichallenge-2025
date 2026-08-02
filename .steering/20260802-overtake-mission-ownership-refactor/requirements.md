# 要件

## 目的

追い越しの新規 Entry 判定と、開始済み ShiftOut / Pass の実行判定が同じ制御関数内で混在している状態を局所的に整理する。

## 背景

`output/20260802-122031/d1/autoware.log` では、OvertakeLine が Pass を継続している周期にも Behavior 側が新規候補相当の条件を再評価し、Follow や SafetyBrake へ遷移する事象が残った。次の性能修正に先立ち、mission phase と速度所有権を単一の判定結果として扱えるようにする。

## 制約

- 今回はパラメータ、閾値、状態遷移の優先順位を変更しない。
- ROS 2 topic / service / message の契約を変更しない。
- 参加者実装 `aichallenge_submit/` の範囲に閉じる。
- 既存の追い越し・復帰・SafetyBrake の保護条件を維持する。

## 完了条件

- Idle / FollowPrepare / ShiftOut / Pass / Return / Recovery の役割が純粋関数で分類される。
- 新規 Entry、committed execution、paused mission、locked target の速度所有権を同じ結果から参照する。
- 分類の単体テストが追加される。
- `multi_purpose_mpc_ros` のビルドと単体テストが通る。
