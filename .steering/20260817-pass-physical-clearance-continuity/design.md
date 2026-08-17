# Design

## 予測速度の整合

空間軌道の target overlap 時刻に使う速度を次で統一する。

```text
prediction_ego_speed = max(minimum_speed, current_ego_speed, planned_ego_speed)
```

計画速度が現在速度より低い場合、車両が瞬時に減速したとは仮定しない。計画速度が現在速度より高い場合は従来どおり計画速度を使う。これにより、現在の closing momentum で先に相手へ到達するサンプルへ物理 target bound が残る。

この速度は速度指令を変更せず、次の空間予測だけに使う。

- 初期および rolling Frenet-DP corridor の target constraint
- rolling candidate を実行権限へ昇格する直前の atomic target-bound validation

## 物理離隔の継続

既存の Frenet-DP corridor は、target overlap window 中に車体実寸の center separation を hard bound、robust separation を preferred bound としている。今回、その overlap window の時刻だけを実際の運動量と一致させる。

これにより Entry で許可された physical-clearance pathが、`closing=0`候補の楽観予測によってPass途中で相手側へ戻ることを防ぐ。全サンプルへ固定離隔を強制したり、壁余裕を下げたりはしない。

## 局所リファクタ

速度選択を純粋関数へ切り出し、候補生成と実行時検証が同じ規則を使用する。単体テストで以下を固定する。

- 現在速度が計画速度より高い場合は現在速度を保持
- 計画速度が高い場合は計画速度を使用
- 不正入力は無効
