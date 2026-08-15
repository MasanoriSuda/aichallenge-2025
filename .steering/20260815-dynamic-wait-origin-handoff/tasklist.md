# Tasklist

- [x] 最新走行の失敗遷移と現行コードを照合する
- [x] 要求・設計・非対象範囲を記録する
- [x] target-bound hold を committed execution 用へ局所リファクタする
- [x] 完了済み ShiftOut の target-only 不成立で前進 prefix を保持する
- [x] DynamicMissionWait の延長条件を origin/commit/prefix で制限する
- [x] core 単体テストを追加・更新する
- [x] package test / build を実行する
- [x] 差分をレビューし、ユーザー生成物を除外してコミットする

## Definition of Done

- pre-no-return ShiftOut 由来の待機が短期期限後に Recovery なしで終了する
- lateral 完了済み ShiftOut は target-only optimizer conflict で即 FollowPrepare へ落ちない
- hard fault を前進 prefix で迂回しない
- `test_v2x_overtake_core` が成功する
- `make autoware-build` が成功する

## Validation

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 611 tests passed
- `pre-commit`: ホストにコマンドがないため未実行
