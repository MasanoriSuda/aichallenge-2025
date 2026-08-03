# Requirements

## Background

暫定1位の走行資料
`.steering/20260803-another-people-log3` は、参照線・走路境界・相手車両・
操舵・加減速を同じGMPCCホライズンで扱っている。ログ上の構成は
`N=20`, `dt=0.12 s`, 先読み`2.40 s`, `game=2`であり、6周合計は
303.84秒、最速40.55秒だった。

比較対象の`.steering/20260803-another-people-log2`は6周合計380.72秒、
最速43.08秒だった。差の大部分は最高速ではなく、交通処理中の停止・
Reverse・長い低速区間にある。

現行品には既に次が存在する。

- `N=20`の`target_ey`/`target_epsi`ホライズン
- frozen overtake mission path
- static wall footprintと横加速度のホライズン検証
- target lock、pass side固定、rear-clear判定
- Pass horizon extensionとSafeSeparation
- committed Passの速度floor/hold

したがってGMPCCをそのまま新設するのではなく、既存ホライズンへ相手との
前後反転・前進量を評価する要素を段階的に追加する。

## Objective

1. 現在のPass horizon修正を動的確認し、比較可能なbaselineを固定する。
2. 左右候補を「横へ到達できるか」だけでなく、rear-clearまで前進できるかで評価する。
3. 採用したmission pathと速度計画を同じgenerationで固定し、Pass中の別系統速度capを減らす。
4. 有効な追い越し軌道上では、Emergency・壁・車体重複以外の理由による失速を減らす。
5. 全面GMPCC化の必要性を、段階導入の実測結果から判断できる状態にする。

## Constraints

- ROS 2 topic、message、service、launch、提出物の契約は変更しない。
- `aichallenge_system`は変更しない。
- 現行のwall、Emergency、actual footprint overlap保護は維持する。
- target ID、pass side、mission generationはcommit後に固定する。
- 既存の`target_ey`/`target_epsi`ベクトルとmission pathを再利用する。
- 暫定1位のソースコードはなく、ログから得た挙動を参考に独自実装する。
- グローバル`a_max=1.0 m/s^2`は本作業では変更しない。
- `1.0/1.37/2.8`の加速度A/Bはsim-onlyの別steeringで扱う。
- multi-stroke K-turn復帰は追い越し性能変更と混ぜず、別steeringで扱う。
- ユーザーの既存変更と`.steering/20260803-pass-horizon-safe-separation`を巻き戻さない。

## Acceptance criteria for the first increment

- 現行SafeSeparation版の`make dev2` 6周baselineを記録する。
- 左右各候補について、rear-clearまでの予測時間・予測距離・最低速度・
  wall/overlap余裕を同じ評価結果から取得できる。
- 候補採用後はpathと速度計画を同一generationで固定する。
- mission中に相手の横揺れだけでsideを変更しない。
- 有効なcommitted corridor上では通常Follow capを重ねない。
- Emergency、別車両、actual overlap、wall infeasibleでは従来どおり減速・中止できる。
- unit test、Release build、`make dev2`で回帰がない。

## Success metrics

- `+8 m ahead -> -8 m behind`の追い越し所要時間
- `ShiftOut -> Pass -> Return -> Idle`完遂率
- committed Pass中の負加速度指令時間
- Pass中の最低速度
- 同一targetへのside変更回数
- SafetyBrake、Recovery、Reverse、停止時間
- 6周合計と55秒超過ラップ数
