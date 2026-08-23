# Requirements: OSQP row-contract root-cause audit

Status: root cause confirmed; two numerical correction candidates were
dynamically rejected and removed.

## Objective

Track/Cruise の5状態MPCCについて、OSQPが `solved` と返した後に実行境界検査が
曲率・加速度・速度・仮想進捗速度を拒否する因果を特定する。

本Sliceは閾値調整やfallback追加ではなく、QPが保証する制約と実行層が要求する
制約を同じ契約へ戻すことを目的とする。

## Constraints

- `aichallenge/result-summary.json` の既存変更へ触れない。
- racing parameter、wall margin、OSQP iteration上限を先に調整しない。
- solver失敗を実行後のclampで隠さない。
- 既存のROS 2 topic/service/launch契約を変えない。
- 根本原因を動的証拠で反証できるまでproduction authorityを変えない。

## Failure-first acceptance

1. mixed-unit QPで、全体の大きい行が小単位の制約許容誤差を拡大するケースを再現できる。
2. row別の違反カテゴリ・stage・絶対値・許容値を一意に説明できる。
3. 修正する場合、OSQPの制約と実行境界検査が同じ行契約を満たす。
4. `make autoware-build` と対象packageの全テストが通る。
5. 動的確認で `execution-primal-reject` を減らし、solve failure・callback overrun・物理証明拒否を増やさない。
