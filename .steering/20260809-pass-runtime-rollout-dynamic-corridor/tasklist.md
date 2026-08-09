# Tasklist

- [x] 最新ログと現HEADの差分を確認する
- [x] 参加者・評価インターフェース契約を確認する
- [x] runtime forward-completionを共有rolloutへ統一する
- [x] same-side dynamic corridor refreshを実装する
- [x] target Mission total budgetを実装する
- [x] 設定と起動時ログを追加する
- [x] core単体テストを追加・更新する
- [x] 対象packageをbuild/testする
- [x] 動的確認項目を記録する

## Static verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result`: 878 tests、0 errors、0 failures、0 skipped
- 実走確認は未実施。下記Dynamic verificationを次の`make dev2`で確認する。

## Dynamic verification（ユーザー試走）

- `ShiftOut -> Pass`回数と`Pass -> Return -> Idle`回数
- runtime rear-clear予測距離と実績誤差
- dynamic corridor commit回数とgoal変位
- ContactContinuation後のPass完遂可否
- Mission total budget到達回数
- wall/solver Recovery、Reverse、70秒超lap
