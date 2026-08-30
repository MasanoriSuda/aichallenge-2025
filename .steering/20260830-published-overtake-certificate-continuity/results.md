# Results

## 観測された現象

修正前の`output/20260830-154652`では、canonical Pass artifactがpublishされた直後に
ShiftOut専用adapterが`intent-mismatch`でcertificate ownerを失い、legacy wall warningから
Pass-entry physical gate、FollowPrepare、Recoveryへ波及していた。

## 根本原因

actually-published execution ledgerはatomic phase handoffによりShiftOut predecessorまたはPass
successorを保持する。一方、rolling-prefix／Pass-entry certificate consumerだけが常に
`ControlIntent::ShiftOut`を要求していた。このproducer/consumer identity contractの不一致が
Pass開始後の二重certificate ownershipを再開していた。

## 実施した変更

- ShiftOut専用alignmentをphase-compatibleなpublished Overtake alignmentへ置換した。
- tactical ShiftOutではShiftOutだけを許可した。
- tactical PassではPass successorを優先し、未publish中だけShiftOut predecessorを許可した。
- target、Mission generation、side、immutable source、publication cursorの既存検証は維持した。
- Pass-origin DynamicMissionWaitも同じpublished adapterへ接続した。
- adapterが採用したintentをdecision logの状態変化として明示した。
- legacy projection、wall margin、solver、速度、lease、timeout、fallbackは変更していない。

## 静的検証

- `git diff --check`: 成功
- isolated source-contract: 87 passed
- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 59 test targets成功
- `colcon test-result --verbose`: 2241 tests、0 errors、0 failures、0 skipped
- `joycon_contract_guard/package.xml`欠損の既知診断は今回差分と無関係

## 動的検証

最終run: `output/20260830-161148/d1/autoware.log`

- published Pass alignment: 2回とも`active=1 expected=1 intent=pass cursor=available`
- `ShiftOut -> Pass`: 2回
- `Pass -> Return`: 2回
- `Return -> Idle`: 2回
- Pass中のpublished `intent-mismatch`: 0回
- Pass中のpublished `side-mismatch`: 0回
- Pass-entry physical gate hold: 0回
- `Pass -> Recovery`: 0回

episode 1はその前段のShiftOut中に`actual footprint wall margin violated`で一度Recoveryへ入ったが、
再開後はPass/Return/Idleまで完遂した。episode 2はShiftOut/Pass/Return/Idleを連続完遂した。

## 残っている懸念

先行run `output/20260830-160431`では、Pass current-world Bundleをpublishした周期に
`Published stateless sibling Bundle adopted: side=1->-1`が発生し、次周期にMission sideとの
`side-mismatch`でcertificateを失った。このrunでは再現しなかったため今回のintent adapter修正へ
混ぜない。次Sliceでは、sibling採用とMission homotopy/side stateが同一atomic decisionで更新されるかを
producerからconsumerまで監査する。

また、ShiftOutのactual wall margin violationは本SliceのPass phase-boundary defectとは別原因であり、
clearance調整をせずfailure snapshotとcurrent-world geometryの一致を先に調べる。

## 次回確認

- opposite sibling採用周期のpublished side、Mission side、decision traceを同一decision IDで照合する。
- sibling採用後のside変更が正式なhomotopy commitか、consumer stateだけの後追い変更かを判定する。
- 6周acceptanceではPass published intent continuity、Pass Recovery、wall hard faultを継続集計する。
