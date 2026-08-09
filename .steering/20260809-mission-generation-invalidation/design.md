# Design

## Observed failure

最新ログでは SafeSeparation 後に同一 side・同一固定 Mission を再開し、
直後に同じ `outer pass becomes inside before rear-clear` または
`same-side rollout cannot reach rear clearance` で失敗する遷移が反復していた。

## Changes

### 1. Generation-scoped invalidation

`OvertakeLineState` に失効した Mission generation を保持する。動的 Mission
待機へ入る原因が発生した時点で現在 generation を失効させ、Recovery 後の
Mission 保持も禁止する。

失効は generation 単位なので、完全 preflight 済みの反対側 Mission または
新規 Mission を commit して generation が更新された場合は解除する。

### 2. Dynamic wait policy

失効 Mission の待機中は次の順で処理する。

1. rear-clear 済みなら Return
2. hard fault / target 不連続 / body overlap なら Recovery
3. 完全成立した反対側候補があれば atomic replacement
4. 評価途中なら Hold
5. 評価完了後も反対側がなければ Recovery

失効した現在経路は、現在側が一時的に feasible と再評価されても
`ResumeCurrent` しない。

### 3. Initial Mission admission

rear-clear 予測地点までにコース上の side role が反転し、全幅切替が必要な
候補は、scheduled outer transition とその preflight が成立している場合だけ
初期候補として採用する。切替が未検証なら Follow / pre-arm を継続する。

## Expected effect

- 同じ失敗理由による `Pass -> FollowPrepare -> Pass` の高速反復を除去する。
- 最初から完遂できない固定経路への commit を減らす。
- 速度パラメータ変更によらず、追い越し Mission の実行整合性を上げる。

