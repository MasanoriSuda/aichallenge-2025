# Design

## 原因

従来の直接制御は、操舵目標全体を
`atan(wheelbase * max_ay / speed^2) / steering_gain`で対称clipしていた。
この制限は直線では有効だが、曲線上では基準経路を走るための操舵も同時に削る。
今回のログでは通常制御の約 -0.30 radが約 -0.06 radまで戻され、外側へ逸走した。

また、`pass_corridor_enforced`は「自車中心が選択区間内」を表すだけなのに、
直接制御の初期phaseと`Pass`移行を単独で決めていた。

## 修正方針

### 1. Pass admission

直接制御は必ず`Shift`から開始する。次をすべて満たした周期だけ`Pass`へ進める。

- pass corridorが現在も成立
- 横偏差・姿勢偏差が許容値内
- locked targetを継続観測しposition jumpがない
- 現在footprintが分離
- 予測が有効で、予測sweepも分離

### 2. Curve-preserving steering envelope

横加速度から求める操舵量を絶対上限ではなく、直前操舵からの追加補正幅として使う。

```text
correction_limit = atan(L * max_ay / v^2) / steering_gain
lower = previous_steering - correction_limit
upper = previous_steering + correction_limit
target = clamp(unconstrained_target, lower, upper)
```

その後に既存の操舵レート制限と最大操舵を適用する。ROS command直前の再clipも同じ
非対称boundsを使い、後段で再びゼロ中心へ縮めない。

壁stop時だけboundsを`[0, 0]`にする。

## 影響

- 曲線走行中の低速車回避開始でも、直前の曲線追従操舵を保持できる。
- 横方向の追加攻撃量は従来の`max_ay`相当で段階的に増える。
- target予測が不確実な間は3.0 m/sのShift phaseを維持し、6.0 m/s Passへ早上がりしない。
