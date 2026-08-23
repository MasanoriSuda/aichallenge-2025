# Follow shadow failure audit design

## Observed behavior

`output/20260823-134218/d1/autoware.log`では、moving front vehicleに対するFollow
shadowが長いaccepted windowを持つ一方、約30 msのmaximum-iterations failureと、
accepted cycleに挟まれた`certified-bound-violation`が発生した。

## Hypotheses

### H1: Follow contractの周期間変化がQPを不安定化している

- Support: target observation generationと距離は周期ごとに更新される。
- Falsification: 入力と境界が滑らかなまま同じfield/stageだけが数値許容差を僅かに超える。
- Evidence: ego speed、target gap/speed、initial velocity reference/upper、terminal progress interval。

### H2: solver収束不足が境界棄却として表面化している

- Support: 同じrunにmaximum iterationsとexecution-primal rejectionが混在する。
- Falsification: solverが十分収束しても入力の不連続に同期して大きな境界違反が出る。
- Evidence: rejected field/stage/value/violation/toleranceとsolve detail。

### H3: warm start provenanceがFollow target更新へ追従していない

- Support: warm identityはintent/formulation/horizon/stage geometryを検査するが、target observation generationを含めない。
- Falsification: cold/reset cycleでも同率で失敗し、warm cycleとの相関がない。
- Evidence:既存warm/reset telemetryと失敗境界の同時刻照合。

## Instrumentation change

`FollowShadowCycleResult`へ診断値を保持し、Follow shadowの状態変化ログへ出す。
計算・制約・採用判定・最終制御出力は変更しない。

## Root cause conclusion

Follow contractはtarget由来のpolicy velocity limitを次stageのhard state upperへ
そのまま適用していたが、requestに現在ego速度がなかった。そのため、例えば
ego 8.87 m/s、policy limit 3.0 m/sでも次stageから3.0 m/sを要求し、最大制動
3.0 m/s^2では到達不能なQPを生成していた。

観測されたmaximum-iterationsはこの不可到達制約の結果であり、solver設定不足が
最上流原因ではない。修正は現在ego速度と経過時間から
`max(0, v_ego - braking_deceleration * elapsed)`を作り、hard upperをその包絡より
下げない。policy reference、hard gap、progress upperは緩和しない。

## Residual numerical risk

到達可能包絡を入れた後も、最大制動包絡ちょうどを使う区間では可行領域の数値余裕が
小さい。OSQPのglobal infinity-norm終了条件ではsolvedでも、行別の実行認証では
acceleration、curvature、predicted velocity、virtual progress speedが拒否される周期が
残った。これはFollow policyやwall marginではなく、混合単位QPのglobal収束契約と
row-wise execution certificateの不一致である。許容差を緩めず、次Sliceで等価なrow
scalingまたはsolver termination contractの統一を検討する。

## Authority audit result

- `ControlIntent::Hold`にはruntime producerがなく、既存のwall/solver holdは安全出力境界である。
- `ControlIntent::Stop`の実producerはSafetyBrakeで、emergency supervisorに残す。
- したがって通常longitudinal authorityの次の昇格候補はFollowであり、Hold/Stopを先にcanonical化しない。
- Followは残存するrow-wise certificate rejectionがあるため、production authorityへは昇格しない。
