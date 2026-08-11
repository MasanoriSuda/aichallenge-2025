# Tasklist

- [x] 最新ログと Pro 指摘を照合する
- [x] 既存変更と現行状態遷移を確認する
- [x] body-clear setup candidate を実装する
- [x] active Mission の target continuity 判定を修正する
- [x] soft horizon loss の Mission 保持条件を修正する
- [x] pure core 単体テストを追加する
- [x] 対象 package をビルドする
- [x] 実走確認項目を記録する

## 検証結果

- `g++ -std=c++17 ... -fsyntax-only v2x_overtake_core.cpp`: 成功
- `make autoware-build`: 25 packages 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test 成功

## 次回 make dev2 で確認する値

- `reason=overtake entry setup` が完全Mission成立前に出ること
- OvertakeLineがShiftOut/Pass中の `Overtake -> Follow` が0回になること
- `Pass -> Return -> Idle` が発生すること
- `SafeSeparation aborted: short horizon unsafe` 直前に0.5秒の tactical
  revalidation が実際のprediction loss起点で働くこと
- 壁接触、現在車体重複、Emergency、solver recoveryが従来どおり中断すること
