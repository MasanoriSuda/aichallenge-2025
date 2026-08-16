# Design

## 背景

`20260816-083931` では measured-state rebase が3回採用された一方、19件のfresh候補が完全なsource coverageとtarget boundsを持ちながら execution horizon で棄却された。古いprefixではなく、DP生成と実行検証の制約差が次のボトルネックである。

## 方針

### 1. Static lateral interval

`recovery_footprint` に、あるreference poseの横区間を一定刻みで評価し、preferred lateralに最も近い連続したcollision-free区間を返すutilityを追加する。

- 車体矩形と追加wall clearanceを含めて判定する。
- occupied / unknown / out-of-map は不成立とする。
- 内部障害物で分断された区間を一つのboundsとして結合しない。

### 2. Reachable execution envelope

`v2x_overtake_core` に、DP sampleのhard boundsと以下の到達可能区間を交差するpure policyを追加する。

```text
center(t) = current_ey + current_lateral_velocity * t
reachable(t) = center(t) +/- 0.5 * max_lateral_accel * t^2
t = max(0.15, path_distance / current_speed)
```

これは実行horizon validatorのモデルと一致させる。

### 3. DP sample integration

既存のraw gap / target / wall bounds生成を小さなsample builderへ集約し、次の順で交差する。

1. lane/gap bounds
2. physical/robust target bounds
3. static footprint wall interval
4. current-state reachable interval
5. time-aligned target prediction

preferred boundsはsoft costのままとし、hard intervalを狭めない。hard static intervalまたはreachable intervalが空なら、その枝をDP solve前に不成立とする。

静的壁区間は制御1周期で1度だけ遅延計算し、左右候補と縦速度候補間で共有する。これにより同じoccupancy gridの重複走査を避ける。

### 4. Last feasible behavior

rolling refreshのatomic promotionとlast-feasible保持は現行を維持する。新しいexecution envelopeが不成立でもactive pathを上書きしない。

## 設定

`v2x_overtake_mpcc_frenet_dp_execution_envelope_enabled: true`

ローカル・提出configの双方で有効化する。

## ログ

既存のrate-limited pendingログへ、wall/lateral/static-mapの各フラグを追加する。周期ログは増やさない。
