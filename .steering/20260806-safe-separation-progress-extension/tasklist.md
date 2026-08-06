# Tasklist

- [x] 最新ログと現行 SafeSeparation の終了経路を照合する
- [x] SafeSeparation state に進捗アンカーと延長回数を追加する
- [x] pure core に進捗延長判定と終了理由を追加する
- [x] controller で局所枠を一度だけ再設定する
- [x] config と起動時ログを追加する
- [x] README に動作と安全境界を記載する
- [x] unit test を追加・更新する
- [x] package build/test を実行する
- [x] 差分を確認し、ユーザー所有の変更を保持する

## Definition of Done

- 進捗が新鮮な forward escape は局所 12 m 上限で直ちに Recovery へ落ちない。
- 進捗なし、短期安全不成立、絶対上限到達では延長しない。
- ログから終了理由と延長回数を判別できる。
- 対象 package の unit test と build が成功する。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功
- `colcon test-result --verbose --test-result-base build/multi_purpose_mpc_ros`: 837 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
- 動的な `make dev2` 効果確認はユーザー試走対象
