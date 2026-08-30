# Requirements

## Objective

`output/20260830-154652`で観測した、canonical Pass artifactがpublishされた直後に
Pass-entry certificate ownerが消失し、旧wall preplan gateからDynamicMissionWait／Recoveryへ
移るphase-boundary defectを修正する。

## Root cause

episode 1では次の順序が成立した。

1. tactical FSMが`ShiftOut -> Pass`へ進む。
2. atomic admissionはPass proofがjoinするまでpublished ShiftOutを保持する。
3. ShiftOut artifactの間はpublished execution alignmentが有効である。
4. decision 1590でcanonical Pass current-world Bundleが正常にpublishされる。
5. published alignment helperは`ControlIntent::ShiftOut`だけを要求するため、Pass artifactを
   `intent-mismatch`として失う。
6. `published_execution_pass_gate_valid=false`となり、同じ周期系列でgeneric wall warningが
   Pass-entry gateを再起動する。
7. current-side hold prefixが成立しなくなり`Pass -> FollowPrepare -> Recovery`へ波及する。

問題はPassの物理証明不足ではなく、actually-published canonical certificateをShiftOut専用の
identity adapterが読めないことである。

## Constraints

- wall/vehicle clearance、solver、速度、操舵parameterを変更しない。
- lease、grace、timeout、fallback、Mission resume ruleを追加しない。
- hard wall contact／margin violation／sample unavailableは従来どおり拒否する。
- publishedでないcandidateやshadow artifactをauthorityへ昇格しない。
- target、generation、side、immutable source、publication cursorのidentity検証を維持する。
- ShiftOut専用経路を横に残さず、published Overtake execution identityへ置換する。

## Acceptance

- tactical ShiftOut中はpublished ShiftOutだけを受理する。
- tactical Pass中はpublished ShiftOut predecessorとpublished Pass successorを受理する。
- Pass-origin DynamicMissionWaitでもactually-published Pass prefixを再利用できる。
- canonical Pass publish後に`Published ... intent-mismatch`でcertificate ownerを失わない。
- valid published Pass certificateだけを理由にwarning-band二重gateを起動しない。
- hard wall faultはcanonical certificateがあっても抑制しない。
- source-contract testがShiftOut専用adapterの不在とphase-compatible identityを固定する。
