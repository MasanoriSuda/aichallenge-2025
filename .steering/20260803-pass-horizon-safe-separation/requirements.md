# Requirements

## 背景

現HEAD `f83d889` のpost-fix走行 `output/20260803-221549` では、d1で
`ShiftOut -> Pass` が11回成立したが、11回すべてがPass開始直後に
`rear_clear_window`再計画を要求し、same-side extension成功は0回、
`Pass -> Recovery`は11回、`Pass -> Return`は0回だった。

extension失敗は、atomic commit拒否7回、全体経路の横加速度超過2回、
outer strategyのside反転2回である。またPass中のlive rear-clear予測は、
既に完了したShiftOut後にも設定上のShiftOut距離を再度加えている。

## 目的

- Pass開始後のrear-clear再予測を、現在横位置から残る横補正量で計算する。
- freshかつstatic Pass終端を前進させるreplacementを、短いdynamic距離の微小変動だけで棄却しない。
- atomic commit失敗理由を個別にログへ出せるようにする。
- extension不能時は、車体非重複を確認して同じ横位置を維持し、前後分離してからReturn/Recoveryへ移る。
- side-by-side状態から基準線へ即横断するRecoveryを減らす。

## 制約

- ROS 2 topic、message、service、launch、提出物の契約は変更しない。
- target IDとpass sideはSafeSeparation中も固定する。
- wall contact、Emergency、solver recovery、target jump時の保護は緩和しない。
- current body footprintが非重複でない場合はbounded holdへ入れない。
- completion guardのkinematic rollout統合は別作業とする。
- `aichallenge/result-summary.json`のユーザー変更と
  `.steering/20260803-fable-gptpro`は編集しない。

## Acceptance criteria

- Pass live rolloutが固定ShiftOut距離を二重加算しない。
- extension commitの各棄却理由を単体テストと実行ログで識別できる。
- fresh replacementはstatic Pass終端が前進すれば、dynamic距離の微小な短縮だけで棄却されない。
- Hold/SafeSeparation開始にはcurrent body footprint非重複が必要。
- SafeSeparation中は同側goalを維持し、相手が前なら負のclosing、後ろなら正のclosingで前後分離する。
- 前方または後方の分離成立後だけRecoveryまたはReturnへ遷移する。
- 追加・既存単体テストと対象package buildが成功する。
