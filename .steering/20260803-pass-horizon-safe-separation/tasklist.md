# Tasklist

## Steering

- [x] post-fix runのPass/extension/Recoveryを集計する
- [x] requirements/design/tasklistを作成する

## Core

- [x] atomic commit判定結果を理由付きで返す
- [x] dynamic距離の微小非前進だけではfresh replacementを棄却しない
- [x] SafeSeparation actionを純粋関数化する
- [x] core単体テストを追加する

## ROS adapter

- [x] live rear-clear rolloutを残横移動距離へ変更する
- [x] current body footprint非重複をHold条件へ追加する
- [x] extension失敗からSafeSeparationへ遷移する
- [x] same-side goal固定と前後分離速度参照を適用する
- [x] failure/actionログを追加する
- [x] SafeSeparation設定を追加する

## Verification

- [x] `test_v2x_overtake_core`（325件成功）
- [x] `git diff --check`
- [x] Release build（`make autoware-build`内）
- [x] `make autoware-build`（25 packages成功）
- [x] ROS interface差分なしを確認する

## Dynamic verification

- [ ] `make dev2` 6周以上
- [ ] `rear_clear_window`即時発火数
- [ ] same-side extension成功/失敗理由
- [ ] SafeSeparation開始/前方分離/後方分離/timeout数
- [ ] `Pass -> Return -> Idle`完遂率
- [ ] side-by-sideからの`Pass -> Recovery`数

## Verification note

- 2026-08-03: `make autoware-build`成功。既知のsetuptools deprecation warningのみ。
- 2026-08-03: `test_v2x_overtake_core`は325件すべて成功。
- 動的項目は次回`make dev2`のログで確認する。
