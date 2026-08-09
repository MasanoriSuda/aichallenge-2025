# Tasklist

- [x] 現行の V2X 追跡と Mission 失効経路を確認する
- [x] 要件・設計を記録する
- [x] 他車速度・加速度推定を追加する
- [x] rollout の縦加速度・横速度減衰を追加する
- [x] fresh same-side Mission replacement を追加する
- [x] 単体テストを追加する
- [x] package build / test を実行する
- [x] 動的確認項目を記録する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  857 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功

## 実走で確認する項目

- 相手が横へ一時的に振れた後、横速度を Pass 全域へ外挿して候補を棄却し続けないこと
- 相手が加減速した場合に、rear-clear 予測と実際の抜き切り時刻の乖離が減ること
- 現 Mission が失効した場合、同じ側が再び成立すれば新 generation として再採用すること
- hard fault、壁 pin、前進不能時は従来どおり Recovery へ入ること
- `ShiftOut -> Pass -> Return` の成功数、Mission replacement 回数、Recovery 理由を比較すること
