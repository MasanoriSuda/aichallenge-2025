# Tasklist

- [x] 最新ログでpreflight通過と追従QP失敗をattempt単位で照合する
- [x] margin-escapeをfootprint validation契約として常時引き継ぐ
- [x] dynamic escapeのmaximum-iteration失敗へcold retryを追加する
- [x] tracking traceへplanning/solver contextを追加する
- [x] 純粋関数とtraceの単体テストを追加する
- [x] 対象packageをビルド・テストする
- [x] ユーザー生成物を除外してコミットする

## Definition of Done

- margin-escapeで中心が通常境界内でも物理解検証が省略されない。
- 単発のwarm-start最大反復失敗では、即座にside backoffへ入る前にcold solveを試す。
- 次回ログだけで、失敗がpreflight、tracking contract、warm/cold solver、物理解検証の
  どこにあるか区別できる。
- 既存の車両edge・実寸壁接触を緩和しない。

## Validation

- `make autoware-build`相当（`docker compose run ... autoware-build`）:
  成功、25 packages。
- `test_v2x_overtake_core`: 786件成功。
- `test_overtake_decision_trace`: 7件成功。
- `test_persistent_osqp`: 6件成功。
