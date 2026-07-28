# LowSpeed静的壁preflight整合化 Tasklist

- [x] 最新runのLowSpeed開始〜wall stop時系列を確認する
- [x] candidate側とlive guard側のwall判定差を特定する
- [x] 静的footprint path掃引APIを追加する
- [x] path掃引のclear/collision/map外/invalid回帰テストを追加する
- [x] stopped local path plannerへforced side評価を追加する
- [x] LowSpeed開始前の静的preflightとauto反対side再試行を追加する
- [x] 実行中の再評価にも同じpreflightを適用する
- [x] `docs/spec/mpc-integration.md`へ設計を反映する
- [x] 対象package build/testを実行する
- [x] `git diff --check`と最終差分レビューを行う

## 静的検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test target成功
- `colcon test-result --verbose`: 706 tests、0 errors、0 failures、0 skipped
  - 既存のstale `build/joycon_contract_guard/package.xml`欠損診断は出たが、
    test result集計には失敗なし
- 最終build後の`test_v2x_overtake_core` / `test_recovery_footprint`: 2/2成功
- `git diff --check`: 成功

## 実走確認（ユーザー実施）

- `make dev2`で停止車へ接近した際、direct開始直後の
  `wall clearance margin violated`が再発しない。
- 最初のsideが静的壁不成立で反対sideがclearなら、反対sideを選ぶ。
- 両side不成立時は壁へ向けてdirect controlを開始しない。
- bypass開始後も物理接触・map外では既存live guardが即時停止する。
