# Requirements

## 目的

全V2X車両を通常走行中の動的障害物として扱う入口判定を、固定距離と低速閾値だけに依存しない予測ベースへ変更する。

## 要求

- 既存のcourse-progress探索範囲内で、レーシングラインと現在または予測上重なる前方車両を評価する。
- 相対速度と追い越しentry距離へ到達する予測時間から、通常GapPlannerへ渡す時点を決める。
- 15 km/h級の移動車両も、追いつく見込みがある場合は動的障害物候補にする。
- 停止・極低速車には既存の連続観測確認を維持する。
- スタートグリッドgrace中は専用breakout処理を優先し、新authorityを発動しない。
- 固定距離は発動条件ではなく、既存のfront-progress探索上限だけを安全な計算境界として用いる。
- ROS topic、service、launch、提出物の契約を変更しない。

## 対象外

- Pass実行中のwall/physical revalidation失敗の修正。
- OSQP収束失敗とfallback制御の修正。
- Recovery、Reverse、接触継続処理の変更。
