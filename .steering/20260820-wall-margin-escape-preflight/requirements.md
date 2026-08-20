# Requirements

## 目的

`output/20260820-123454` で顕在化した、現在姿勢の壁余裕不足によって
壁から離れる動的障害物回避候補まで静的壁 preflight が棄却する欠陥を修正する。

## 要求

- 車体実寸 footprint が壁または未知セルへ接触する候補は従来どおり棄却する。
- クリアランス込み footprint だけが現在姿勢で接触している場合、短い中心方向
  escape に限って候補を許可できるようにする。
- escape 中も実寸 footprint の接触は許可せず、余裕層は上限距離内に完全復帰し、復帰後の再接触を許可しない。
- 壁沿い前進では同じ壁でも raster 接触セル数が増減するため、セル数は棄却条件ではなく診断値として記録する。
- 設定された壁余裕へ短距離で復帰できない候補は棄却する。
- 通常の wall-clear candidate の判定は変えない。
- 決定ログから通常 clear、margin escape、実寸衝突、余裕復帰失敗を区別できる。
- ROS topic、service、message、提出物の契約は変更しない。

## 対象外

- wall clearance 値そのものの調整
- solver weight と OSQP 設定の調整
- Recovery / Reverse 全体の再設計
- 評価基盤の変更
