# Design

## 1. Gap plan 共通の静的壁preflight

既存の停止・低速車用preflightを `GapPlanStaticWallPreflight` として共通化する。
候補点のyawは基準trajectoryのyaw固定ではなく、隣接する候補点の接線から求め、横移動中の
車体回頭も掃引判定へ反映する。

動的横回避候補は次の順で評価する。

1. GapPlannerの可行性
2. 現状態からのreachable bridge
3. 車体＋壁のstatic execution preflight
4. target/side単位のsolver backoff
5. authority判定

preflightで落ちた候補はbridge rejectedとして扱い、既存の反対側探索へ流す。

## 2. solver継続契約

`SolverFailureContinuationRequest` に `execution_path_validated` と
`tracking_envelope_valid` を渡す。

- `execution_path_validated`: 今周期のreachable bridgeとstatic wall preflightが成立
- `tracking_envelope_valid`: 動的回避中は姿勢誤差を確認。レーシングライン基準の`e_y`は
  回避そのものを誤差扱いするため使用しない

現在footprint、緊急状態、failure budget、速度のgateは維持する。

## 3. ログ

`Overtake decision trace` のcandidateへ以下を追加する。

- `preflight=evaluated/feasible`
- `preflight_reason`
- `preflight_poses`

Pass同側replanの横移動上限拒否には数値を埋め込む。これにより、次走で
「候補生成不良」「壁掃引不成立」「solver backoff」「Pass横移動量不足」を分離できる。

## 互換性

内部C++型とログ文字列のみを変更する。ROSインターフェース、launch、yaml schema、
評価成果物schemaは変更しない。
