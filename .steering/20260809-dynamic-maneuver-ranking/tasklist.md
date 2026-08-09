# Tasklist

- [x] Pro案と現行Mission指標を照合する
- [x] 変更範囲と非対象を記録する
- [x] maneuver ranking policyをcoreへ実装する
- [x] 設定値と読み込みを追加する
- [x] controllerの左右再評価へ統合する
- [x] ranking差分ログを追加する
- [x] core単体テストを追加する
- [x] package build/testを実行する
- [x] 動的確認項目を記録する

## Static verification

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25成功
- `colcon test-result --verbose`: 939 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 問題なし
- `joycon_contract_guard/package.xml`不在の既知警告は今回の対象外

## Dynamic verification

`make dev2`で低速車を含む同一条件を走らせ、次を確認する。

- `OvertakeLine opponent side opportunity pending`に`rank`と各advantageが記録される
- `rear-clear time advantage`または`horizon progress advantage`が0.25秒継続した場合だけatomic replacementされる
- no-return後、body overlap中、target不連続時にside replacementされない
- 同一Missionのreplacement countが1を超えない
- side切替回数、Pass完遂率、rear-clear時間、最低速度、壁/接触/Recovery回数
- 反対側が速くてもreserveが0.05 m超悪化する候補を選ばない
- 反対側が広くてもrear-clear時間または最低速度が0.25超悪化する候補を選ばない
