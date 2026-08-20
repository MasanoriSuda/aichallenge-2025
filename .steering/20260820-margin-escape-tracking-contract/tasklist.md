# Tasklist

- [x] 最新ログでpreflight採用と追従失敗の同一attemptを照合する
- [x] 壁所有edgeだけを段階復元する純粋関数を実装する
- [x] dynamic escape candidateへ追従境界契約を接続する
- [x] QP成功解の実寸footprint検証を追加する
- [x] decision traceへ境界契約を追加する
- [x] 通常、継承成功、車両edge非緩和、不正入力の単体テストを追加する
- [x] 対象packageをビルド・テストする
- [x] ユーザー生成物を除外してコミットする

## Definition of Done

- preflightで許可した初期壁余裕重複をQPが同じ復元距離で表現できる。
- 車両所有edgeと実寸壁接触は緩和されない。
- 採用後のQP失敗を、境界契約または物理解検証の理由まで追跡できる。
- 既存および追加単体テストが通る。

## Validation

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 既存を含む784件成功
- 追加wall-margin contract 5件: 成功
- `test_overtake_decision_trace`: 7件成功
- `mpc_controller_cpp` 最終差分再ビルド: 成功
