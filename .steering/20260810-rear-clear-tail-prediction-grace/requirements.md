# Requirements

## 背景

`output/20260810-095847/d1/autoware.log` では、`SideBySide rear-clear tail` が
target_s=-0.03 mで発動した。その約0.36秒後、target_s=-0.77 mまで前進したにもかかわらず、
予測sweepの一時的不成立でtail所有権を失い、既存のprediction-only graceが0.06秒だけ
有効な状態でlocal distance limitからRecoveryへ遷移した。

## 要求

- SideBySideCommitted、target_s<=0、現車体分離、実行corridor clearのrear-clear tailは、
  既存prediction-only grace中も同じsideで前進を継続する。
- grace時間、fresh progress条件、forward-completion latch条件は既存値を変更しない。
- grace終了、現車体重複、corridor block、target不連続、壁・EmergencyBrake・solver fault、
  absolute Pass limitでは従来どおり中断する。
- ROS 2 topic、message、service、launch、yamlパラメータは変更しない。

## 制約

- 新しいgraceやパラメータを追加しない。
- 予測overlapを物理clearとして扱わない。
- `output/`、rosbag、`aichallenge/result-summary.json`を変更しない。
