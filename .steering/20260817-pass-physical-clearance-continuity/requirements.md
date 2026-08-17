# Requirements

## 背景

`output/20260817-224842` では、スタート直線の左右開き幅は正しく認識され、広い側から ShiftOut できた。一方で 38 回の ShiftOut に対し rear-clear 後の Return 完遂は 1 回だけで、Pass から FollowPrepare が 20 回、Recovery が 12 回発生した。

代表例では、rolling Frenet-DP が `closing=0.00 m/s` の候補を採用したが、実車は約 5.0 m/s、対象車は約 4.0 m/s で接近を継続していた。候補の target-bound 検証は計画速度 4.0 m/s を用いて接近を遅く見積もり、実行後に target separation / wall bound が衝突して Mission を失っていた。

## 目的

- Pass 候補の相手予測速度を、計画速度だけでなく現在の自車運動量も含めて評価する。
- 現在速度を維持している間、物理 target clearance を外れる rolling path を採用しない。
- robust margin は soft preference、車体実寸離隔と hard wall clearance は hard bound とする既存方針を維持する。
- 数値 solver failure、Recovery、ROS 2 interface には変更を加えない。

## 制約

- ユーザー変更中の `config/config.yaml` と `aichallenge/result-summary.json` は変更しない。
- wall clearance、vehicle footprint、closing-speed の設定値は変更しない。
- target 消失、実接触、壁 hard fault は従来どおり fail closed とする。
