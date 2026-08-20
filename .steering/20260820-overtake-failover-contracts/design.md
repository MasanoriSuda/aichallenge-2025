# Design

## 1. Solver bounded continuation

通常Cruise用のsolver crawlとは別に、動的障害物回避中だけ使う短時間の縮退判定を
純粋関数として追加する。

許可条件は、simulation、control有効、solver fallback、dynamic escape有効、
emergencyなし、静的footprint clear、追従誤差内、連続失敗回数が操舵保持周期以内、
現在速度と速度上限が有限、であること。

許可時はMPC内で作成済みのfallback steeringを保持し、速度目標を現在速度以下に限定、
加速度を0以下に限定する。継続失敗または任意の安全条件不成立時は従来どおり
forced stopへ移る。

最終出力sourceは `solver-bounded-continuation` とし、entry/exitとblock理由をログへ残す。

## 2. DynamicWait atomic handoff

DynamicMissionWaitへ遷移したcall siteは、そのまま空の `OvertakeLineOutput` を返さない。
FollowPrepareへ状態を更新した直後に同じ `update_overtake_line()` を一度再評価し、
wall-feasibleなforward prefixまたはlateral holdを同じ制御周期で発行する。

再評価でも横経路が成立しなければRecoveryへ移るため、無制限再帰にはならない。

## 3. Runtime replacement contract

same-side / cross-side / progressiveの分岐前に、全runtime replacementへ共通契約を適用する。

- candidate feasible
- dynamic validityが現在時刻を包含
- target clearanceが評価済みかつ非負
- minimum path wall clearanceがlive required clearance以上

不採用時は `target-clearance-unchecked`、`target-overlap`、
`wall-contract-shortfall`、`prediction-expired` などを一行で記録し、
古いMissionをtransactionalに維持する。

## ログ

- `Overtake control decision.control_source=solver-bounded-continuation`
- `output=solver-failure-dynamic-escape-hold`
- bounded continuation entry/exitにはfailure count、速度、footprint、誤差を含める。
- Mission置換拒否には契約名と実測値/要求値を含める。
- DynamicWaitの決定ログで `dynamic-wait-without-lateral` を0件にする。
