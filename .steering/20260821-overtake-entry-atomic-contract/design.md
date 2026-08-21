# Design

## 1. Physical execution certificate

左右extended MPCCが解いたstage-wise lateral trajectoryについて、以下を一つの証明書として
`OvertakeMissionCandidate`へ保存する。

- branch side
- solve時刻とcourse progress
- required wall clearance
- path distance列
- lateral trajectory列

sideだけをMissionへ引き継ぐことは禁止する。

## 2. Current-state revalidation

新規entry採用前に、証明書を現在のcourse progressへrebaseし、Mission referenceとのtrust envelopeを
適用した最終横軌跡を作る。その最終軌跡を現在のstage boundsとswept footprintで再検証する。

検証済み軌跡だけをcached Missionへ保存する。証明書がstale、horizon不足、bound外、壁接触なら
現在のracing/Follow経路を維持する。

## 3. Atomic entry commit

通常追い越しentryでは次の順序を固定する。

1. candidateと証明書を検証
2. PassPlanを構築
3. frozen Missionと検証済み実軌跡をcommit
4. FSMをShiftOut/Passへ遷移

1〜3のいずれかが失敗した場合、FSMはIdleのままとする。

## 4. Lateral bound fail-closed

wall margin適用後、またはGapPlannerとのintersection後に`upper < lower`となったstageを
`LateralBoundContract`不成立として記録する。QP行列構築用のplaceholderは実行不可とし、
`get_control()`でsolver起動前にbounded fallbackへ移る。

## 5. Trace

状態変化時に次を一行で記録する。

- `Overtake entry commit rejected/accepted`
- source、target、side、certificate age
- source/current course progress
- required wall clearance
- rejection reason
- lateral bound failure stage、lower/upper、source

既存のchange-aware最終decision logは維持する。
