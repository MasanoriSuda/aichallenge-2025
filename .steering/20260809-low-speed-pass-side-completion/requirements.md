# Requirements

## 背景

`output/20260809-234842/d1/autoware.log` では、commit 済み Overtake から
LowSpeedAvoidance への引継ぎと、停止車左側の 3.43 m corridor 選択までは成功した。
しかし Pass 開始後に対象車が前方投影から側方へ移ると、停止車 local path が inactive
となり、`live vehicle corridor unavailable` で速度 0、stall、stuck recovery、Reverse が
連鎖した。

## 要求

- 検証済み停止車 corridorへ既に進入した Pass 中は、対象車が前方から側方または
  rear-clear 判定帯へ移っただけで速度 0 にしない。
- 継続は、前方車が残っておらず、側方／clearance 車両が存在し、保持中の横目標が
  現在のコース境界と静的壁 preflight を通る場合に限定する。
- Shift 中、正面に車両が残る場合、live local path が明示的に infeasible な場合、または
  保持経路の壁 preflight が不成立の場合は従来どおり停止する。
- 車両群が clear になった後の clear hold、Rejoin、MPC handoff は既存仕様を維持する。
- ROS 2 topic / service / message 契約と速度・壁余裕パラメータは変更しない。

## 制約

- `aichallenge/result-summary.json` の既存変更には触れない。
- `output/` と rosbag は変更しない。
- 実 overlap や壁接触を無条件に無視する実装にはしない。
