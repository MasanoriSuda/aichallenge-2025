# Design

## 1. margin-escape契約

preflightで`margin_escape_used=true`となったcandidateは、次のどちらかとして追従へ
引き継ぐ。

1. 車体中心が通常wall bound外
   - 既存どおり、壁所有edgeだけを復元距離まで段階的に戻す。
2. 車体中心は通常wall bound内
   - 境界は一切広げず、`footprint-validation-only`契約を有効にする。
   - QP成功後のstage境界検証と実寸footprint swept-path検証を必ず実行する。

これにより、preflightがfootprint基準、追従が中心点基準という診断上・安全上の
不一致を解消する。margin footprintそのものをhard constraintへ変換して広げることは
しない。

## 2. dynamic escape cold retry

通常のpersistent OSQP solveはwarm startを維持する。次の全条件を満たす最初の失敗に
限り、同一QPをwarm startなしで一度だけ再実行する。

- dynamic obstacle lateral escape authorityがactive。
- warm startが実際に適用された。
- solver statusがmaximum iterations reached。

最初のfailureでpersistent workspaceはreset済みなので、再試行は新しいworkspaceの
cold solveになる。cold solve成功時はcandidateをtracking-qualifiedへ進める。失敗時は
二つのfailure detailを結合し、従来のfallback/backoffへ渡す。

## 3. 決定ログ

tracking traceへ以下を追加する。

- `preflight_mode`
- `preflight_margin_escape` と `preflight_margin_clear`
- `tracking_contract_active/reason/max_relax`
- `corridor_width` と `target_adjust`
- `cold_retry=attempted/succeeded`
- `initial_solver_reason`

planning traceにも`footprint-validation-only`を出し、中心境界緩和と物理解検証だけの
契約を区別する。

## 影響範囲

- dynamic obstacle lateral escapeの追従solveだけ。
- Overtake Mission、Recovery、通常MPC/MPCCのsolve policyは変更しない。
- cold retryによる追加計算は失敗周期だけに限定する。
