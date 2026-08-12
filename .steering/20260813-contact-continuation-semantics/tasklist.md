# Tasklist

- [x] 最新実走とPro案の差分を照合する
- [x] V2Xで利用可能な姿勢・速度・壁情報を確認する
- [x] NearContactPrearmとContactContinuationを分離設計する
- [x] core分類器とcontroller状態を変更する
- [x] local/cloud設定を更新する
- [x] 分類器・Prearm・証拠欠落・壁guardのテストを追加する
- [x] 対象packageをビルドする
- [x] core単体テストを実行する
- [ ] 次回`make dev2`で動的効果を確認する（ユーザー実施）

## 静的確認結果

- `make autoware-build`: 25 packages成功。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功。
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`: 1009 tests、error/failure/skip 0。
- 境界確認: near-contact単独は不採用、impact/actual overlapは条件付き採用、active時の証拠欠落は有界保持、相対ヨー超過・姿勢不明・壁余裕不足・front-contact normalは不採用。
