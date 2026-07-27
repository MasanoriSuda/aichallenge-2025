# Design

## 現行の問題

gap plannerは各MPC horizonでV2X車両を次式により予測している。

```text
pred_x = x + vx * (age + horizon_t)
pred_y = y + vy * (age + horizon_t)
```

`v2x_prediction_use_path_time=true` は `horizon_t` をコース距離から求めるが、
相手の予測軌跡自体は直線のままである。ヘアピンでは、相手が接線方向へ直進すると
誤認したcorridorと、次のV2X観測で実際に旋回した相手位置が不連続になり得る。

## 変更方針

### 1. コース投影

既存の `project_forward_course_progress` を再利用し、現在時刻へage補正した相手位置を
MPC参照コースへ投影する。

投影結果として次を利用する。

- egoから相手までのコース前方距離
- 相手のコース横位置
- 相手速度をコース接線へ射影した速度

探索範囲はMPC horizon距離、予測中の最大移動距離、車体長から導出し、
空間的に近い別のヘアピン枝を無制限に探索しない。

### 2. horizon予測

各horizonにおける相対位置を次式で求める。

```text
target_s(t) = target_forward_s + target_along_speed * horizon_t
relative_s(t) = target_s(t) - ego_horizon_course_distance
relative_d(t) = target_current_course_lateral
```

相手のyawはV2X契約に含まれないため、短い既定1秒horizonでは横位置一定を採用する。

### 3. fallback

以下の場合は現行Cartesian予測を使用する。

- 新設定が無効
- 参照コースへの投影が無効
- 投影値または予測入力が非有限
- 相手が導出したコース探索範囲外

fallbackは安全条件の緩和ではなく、現行互換経路である。

### 4. 設定

```yaml
v2x_prediction_use_course_progress: true
```

既定値はコード上 `false` とし、設定欠落時の既存互換性を維持する。
競技用sim設定では `true` を明示する。

### 5. ログ

起動時に予測horizon、path-time有効状態、course-progress有効状態を1行だけ出す。
周期ログは追加しない。

## 影響範囲

- gap plannerの候補生成とlive execution corridor判定が同じ予測を利用する。
- Follow/SafetyBrakeの前方車分類は既存common-progress処理を維持する。
- V2X topic/message型、MPC出力、評価システムには影響しない。

## 検証

### 単体

- コース投影有効時に進行度が接線速度分だけ進む。
- 横位置が保持される。
- 無効設定、無効投影、非有限値でCartesian fallbackとなる。
- 既存course projectionテストを維持する。

### dev2

`output/20260727-085009` をbaselineとして次を比較する。

- 完全追い越し回数: baseline 2
- `locked target no longer executable`: baseline 22
- 新規 `Idle -> ShiftOut`: baseline 27
- `ShiftOut -> Pass`: baseline 7
- 壁margin・横加速度違反と接触回数
- P1の各周lap time
