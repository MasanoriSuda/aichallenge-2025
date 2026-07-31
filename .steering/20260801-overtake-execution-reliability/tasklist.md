# Tasklist

- [x] 最新ログの4回のShiftOutと残る2失敗を整理
- [x] 壁clamp失敗とsolver失敗をコード経路へ対応付け
- [x] 安全条件と対象外を定義
- [x] 横加速度bounded targetの共通計算を追加
- [x] static mapで車体実寸を再検証するreachable projectionを追加
- [x] 高い旧side横速度でearly side replanを抑止
- [x] 診断ログと設定を追加
- [x] 単体テストを追加
- [x] ビルドとpackageテストを実行
- [x] 差分レビューと試走確認項目を整理

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 225/225成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25成功
- `git diff --check`: 成功

## 試走確認項目

- 起動ログのearly replan設定に`vlat=0.25 m/s`が表示されること。
- 前回wp187付近で`replan_vlat_ok=0`の間に、直接`side=-1 -> 1`へ反転しないこと。
- 同区間で反転直後のOSQP連続失敗と`solver failure threshold reached`が再発しないこと。
- 壁clamp区間で`static_reachable=1`が出た場合、車体接触なしでShiftOutが継続すること。
- `actual footprint intersects static wall`またはmap不明時は従来どおりRecoveryすること。
