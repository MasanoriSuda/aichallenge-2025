# Design

## 原因

停止車 planner は前方距離 `s <= 0` の車両を投影対象から除外する。その結果、対象車が
正常に側方へ移った Pass 後半でも `planner_output.active == false` になる。一方 Behavior
FSM は同じ車両を `has_side_vehicle` または `has_low_speed_clearance_vehicle` として保持する。

現行の corridor guard は active な direct control について local path inactive を一律停止
とするため、正常な front-to-side の前後関係反転を corridor 消失と誤認する。

## 方針

1. corridor guard の入力を構造体化し、direct-control phase と車両分類を明示する。
2. live local path が active の場合は従来どおり feasible / infeasible を最優先する。
3. local path inactive の場合、次の全条件でのみ保持 Pass を許可する。
   - phase が Pass
   - front vehicle がない
   - side または low-speed clearance vehicle がある
   - 保持中 pass target が現在 horizon の base bounds 内
   - 保持中 pass target の静的壁 swept-footprint preflight が成立
4. 上記以外の inactive は停止する。関連車両が完全に消えた場合は既存 clear-hold / Rejoin
   が所有するため corridor guard では停止しない。
5. 保持 Pass の開始・終了を一度だけログへ出し、実走で誤停止解消を確認できるようにする。

## 安全境界

- current physical footprint と壁余裕の毎周期 guard は残す。
- live planner が active かつ infeasible を返した場合は保持 Pass で上書きしない。
- Shift では保持経路を使わない。検証済み corridor へ物理的に入った Pass だけを対象とする。
- 前方車が残る場合は planner inactive を許容しない。

## 非対象

- low-speed pass 速度、加速度、wall clearance の調整
- 通常 Overtake の side ranking
- Stuck Recovery の距離・速度調整
