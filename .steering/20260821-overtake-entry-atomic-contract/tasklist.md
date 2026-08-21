# Tasklist

- [x] physical execution certificateをMissionへ追加
- [x] extended MPCC実軌跡をcertificateへ保存
- [x] current-state revalidationを実装
- [x] Mission freezeとFSM遷移を原子的な順序へ変更
- [x] lateral bound intersectionをpure contract化
- [x] invalid boundをsolver前にfail-closed化
- [x] entry/bound rejection traceを追加
- [x] 回帰テストを追加
- [x] package build/testを実行
- [x] 差分レビュー後にコミット

## Validation

- `make autoware-build`: 成功
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros --output-on-failure`:
  30/30成功
- `pre-commit`: ホストにコマンドがなく未実行
