# Tasklist

- [x] 2走の共通失敗経路を確認
- [x] requirements/designを作成
- [x] 低速車専用static wall preflightをGap plan共通処理へ局所リファクタ
- [x] 動的横回避candidateへstatic execution preflightを追加
- [x] candidate decision traceへpreflight結果を追加
- [x] solver continuationを検証済み回避経路基準へ修正
- [x] Pass同側replan拒否ログへ数値根拠を追加
- [x] 単体テスト
- [x] `make autoware-build`
- [x] `git diff --check`
- [x] コミット

## Definition of Done

- 壁掃引不成立の動的回避candidateがauthorityを取得しない
- 検証済み動的回避の横変位だけを理由にsolver継続を拒否しない
- 壁・姿勢・緊急状態・連続solver失敗は従来どおり継続を拒否する
- 失敗理由を1本のdecision traceとsolver continuationログから判別できる
- ROS/evaluation interfaceに差分がない

## Validation

- `docker compose run -T --rm --no-deps autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 30/30 tests成功、
  1421 tests / 0 failure
- `ctest -R "test_mpc_velocity_limit|test_overtake_decision_trace"`: 2/2成功
- `git diff --check`: 成功
- `colcon test-result --verbose` は対象package 0 failure。ただし既存build成果物の
  `joycon_contract_guard/package.xml` 欠損警告あり（今回差分外）
