# Tasklist

- [x] 最新走行と変更前走行を比較する
- [x] wall warningからhard failureまでの経路を特定する
- [x] front-cap再適用増加を確認する
- [x] runtime wall fallback pure policyを追加する
- [x] 中央寄り縮退Missionと安全なReturn fallbackを実装する
- [x] front-cap acquire/hold条件を分離する
- [x] unit testを追加する
- [x] `make autoware-build`を実行する
- [x] 差分をレビューする

## Verification

- `make autoware-build`: 25 packages成功
- `ctest -R test_v2x_overtake_core --output-on-failure`: 1/1成功
- local/cloud configの追加5項目: 一致
- `git diff --check`: 成功
- ROS topic/service、評価JSON、提出物構造の変更なし

## 動的確認

- `make dev2`でwall warning後のactionを確認する
- `runtime wall center contraction accepted`後にhard wall Recoveryへ入らないことを確認する
- `runtime wall speed-preserving Return`は十分な前方距離時だけ発生することを確認する
- front-cap reapply回数・総時間、Overtake平均速度、rear-clear率を前走と比較する
