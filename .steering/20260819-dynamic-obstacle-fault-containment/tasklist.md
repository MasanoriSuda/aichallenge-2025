# Tasklist

- [x] 6周ログから失敗経路と頻度を抽出する
- [x] Target-only hold、SafeSeparation、Return fallbackの現行実装を照合する
- [x] Target-only conflictの動的repair budgetを実装する
- [x] preplan warningとhard wall faultの責務を分離する
- [x] SafeSeparation InvalidInputをsoft failureへ接続する
- [x] rear-clear済みReturnのlast-feasible/handoff fallbackを実装する
- [x] 純粋関数の単体試験を追加する
- [x] `make autoware-build`を実行する
- [x] `colcon test --packages-select multi_purpose_mpc_ros`を実行する
- [x] 変更対象だけをコミットする

## 動的確認（ユーザー試走）

- [ ] `SafeSeparation aborted: invalid input`によるRecoveryが0件
- [ ] rear-clear後の`Return -> Recovery`が0件
- [ ] Target/壁競合によるMission破棄が現行13件から減少
- [ ] `Pass -> Return -> Idle`完遂率80%以上
- [ ] 壁接触回数が増えていない
