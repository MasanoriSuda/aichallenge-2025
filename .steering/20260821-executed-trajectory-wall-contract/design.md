# Design

## 1. 二段階の物理証明

入口candidateに付与する既存の`physical execution certificate`は、左右branchが生成した
参照経路の証明として維持する。ただし、これはsolverが実際に追従して生成する状態列の
証明ではない。

制御周期ではsolver成功後、発行予定の状態列`e_y[k]`を取り出し、同じstage geometry、
同じwall bounds、同じrequired physical clearance、同じ`SweptFromCurrentPose`で再検証する。

```text
branch reference certificate
        ↓
Mission / corridor / solver
        ↓
executed-solution certificate  ← 発行前の最終契約
        ↓
command publish
```

## 2. 検証対象

`MpcProblem::progress_execution_context_active`が成立する`ShiftOut/Pass/Return`を対象とする。
solverのlegacy表現へ変換済みの状態列から各stageの`e_y`を取得し、
`progress_execution_path_distance_m`とstage lower/upperを使って検証する。

検証入力が欠落・非有限の場合もfail closedとする。

## 3. failover

### 未走行entry

`ShiftOut`開始直後で、phase走行距離がほぼ0 mの実行解が不成立なら、当該Missionを
実行済みとみなさない。

- unsafe solver解は発行しない
- 直前の安全な制御を1周期だけbounded holdする
- 不成立sideへretry blockを設定する
- Mission/DP/async cacheを無効化する
- `Idle`へ戻し、反対側を含むfresh searchを許可する
- solver failure counterは増やさない

### 走行開始後

既に横移動した`ShiftOut/Pass`と`Return`は従来のDynamicMissionWait/Recovery契約を使う。
途中で単純に`Idle`へ戻すと横位置の所有者を失うためである。

## 4. ログ

新規の`Overtake executed solution wall contract`ログへ以下を出す。

- decision / episode / generation / target / phase / side
- validation scope
- reference certificateの有無とrequired clearance
- executed solutionのstage数
- resultとreason
- failover action (`publish`, `entry-rollback`, `dynamic-replan`, `recovery`)
- wp_id

既存の最終`Overtake control decision`と同じdecision IDで照合できるようにする。

## 5. 非対象

- Hold/Returnを含む常時Race MPCC一本化
- wall marginの設定変更
- candidate生成の`no complete or receding branch candidate`改善
- solver計算時間最適化
