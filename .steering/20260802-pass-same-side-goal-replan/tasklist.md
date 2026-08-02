# Tasklist

## Steering

- [x] 最新runと直前runを比較する
- [x] extension 0/3の実装要因を特定する
- [x] requirements/design/tasklistを作成する

## Core

- [x] predicted overlap replan要求をPass horizon actionへ追加する
- [x] replacement lateral shift distance計算を純粋関数化する
- [x] atomic goal adjustment上限を通常平滑化値から分離する
- [x] extension失敗理由を分類する

## ROS adapter

- [x] 予測重複timerをhorizon判断前に一度だけ更新する
- [x] predicted/current targetから同側最小安全goalを生成する
- [x] adjusted goalでrollout、Pass距離、full preflightの順に評価する
- [x] replacement pathとmission stateをatomic更新する
- [x] 状態変化ログを追加する

## Verification

- [x] 追加単体テスト
- [x] `test_v2x_overtake_core`
- [x] `git diff --check`
- [x] Release build
- [x] `make autoware-build`
- [x] ROS interface差分なしを確認する

## Dynamic verification

- [ ] `make dev2` 6周以上
- [ ] predicted-overlap replan要求数
- [ ] same-side extension成功/失敗数と理由
- [ ] Pass -> Return -> Idle完遂率
- [ ] SafetyBrake中断数
- [ ] wall/solver Recovery数
- [ ] Reverse要求数
- [ ] 45～46秒台のクリアラップ維持
