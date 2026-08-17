# Tasklist

- [x] 現行MPCの状態・入力・QP・warm-start契約を確認する
- [x] requirements/designを作成する
- [x] `mpcc_progress` pure C++ helperを実装する
- [x] QP構築をstage reference生成とdynamics生成へ局所分離する
- [x] progress MPCC modeとmode切替resetを統合する
- [x] configと起動ログを追加する
- [x] helper単体テストを追加する
- [x] package build/testを実行する
- [x] 変更をコミットする

## 実走確認（ユーザー実施）

- [ ] 40 Hz callbackの連続deadline missがない
- [ ] OSQP failure/cold resetが増えない
- [ ] hairpinでwaypoint/progress branch jumpがない
- [ ] `ShiftOut -> Pass -> Return -> Idle`完遂率が改善する
- [ ] Mission破棄とRecoveryが増えない
