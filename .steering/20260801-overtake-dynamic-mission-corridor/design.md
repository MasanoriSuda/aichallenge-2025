# 設計

## 1. 動的corridor照合

既存gap plannerは各horizon pointについて、V2X予測車両のinflated footprintを除いた
free intervalを`target_active/lb/ub`として生成している。従来の全区間preflightは、この
intervalをpass goal一点の共通区間としてだけ使い、ShiftOut/Return途中の横位置を
照合していなかった。

各active sampleについて固定mission profileを次のaffine形へ変換する。

```text
ey(s) = intercept(s) + coefficient(s) * pass_goal
```

`lb(s) <= ey(s) <= ub(s)`からsampleごとのpass goal区間を導出し、全sampleと既存の
candidate goal区間を交差する。交差が空ならmissionを採用しない。

## 2. 最小横移動goal

動的corridorから得たgoal区間をminimum-motionとstatic-wall preflightの双方へ渡す。
これにより、最終Pass位置だけなら成立していても、車体が重なり始める時点までに
横離隔を確保できない候補は、必要最小限だけgoalを外側へ補正する。壁内に補正できない
場合はFollowへ残す。

## 3. 固定後の所有権

`mission_path_frozen=true`のShiftOutでは、Behavior側のearly side replan assessmentと
entry preflightを無効化する。実行中は固定goalをOvertakeLineが所有し、static wall、
actual footprint、target lossなどのruntime hard guardのみを継続する。

## 4. interface影響

変更は`multi_purpose_mpc_ros`内部policyだけである。設定値、ROS interface、Domain、
result schemaへの変更はない。
