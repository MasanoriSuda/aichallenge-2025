# Requirements

## 目的

非同期MPCCが解いた追い越し経路を実行中のDP経路へ昇格するとき、現在の車両状態・直前の実行経路から連続的に接続し、昇格直後のtracking解除や壁hard faultを減らす。

## 背景

`output/20260819-021234` では solved source の実行昇格は動作した一方、次が残った。

- 198件中117件で横補正が既定上限0.35 mに到達
- QP境界余裕0.1 m未満が105件、0 mが33件
- 昇格後にtracking/degraded理由のauthority解除が発生
- `physical target separation conflicts with wall bounds`、`solution hard wall contact`、`optimized horizon failed physical revalidation` が残存

現状の昇格処理はMission基準のtrust envelopeで点ごとに制限するだけで、通常のrolling DP更新が使うpreserved prefix・smooth blend・実測状態からの横加速度到達可能性制約を適用していない。

## 要件

- solved source昇格時にも既存のDP refresh stitchを適用する。
- 現行実行参照を短区間保持し、候補MPCC経路へ滑らかに接続する。
- 実測横速度と横加速度余裕により、昇格直後に到達不能な横移動を制限する。
- stitch後の経路に従来のhard wall・QP bound物理検証を再適用する。
- hard guard、source age、last-feasible期限は緩和しない。
- 追い越し余裕パラメータや壁余裕を一律に増やさず、既存の攻撃性を維持する。
- ROS topic/service、提出インターフェースを変更しない。
- `aichallenge/result-summary.json` の既存変更を変更・コミットしない。

## Definition of Done

- solved source昇格経路がtrust envelopeとcontinuous stitchの両方を通る。
- stitch利用、reachability制限、未制限横加速度が診断ログで確認できる。
- pure helperの単体テストが通る。
- `make autoware-build` が通る。
- 変更を単独コミットする。
