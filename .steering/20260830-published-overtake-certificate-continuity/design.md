# Design

## Single certificate owner

Pass-entryの物理authorityは、actually-published canonical execution ledgerが所有する。
そのartifactはphase境界で次のどちらかになる。

- tactical ShiftOut: published ShiftOut
- tactical Pass: atomic handoff前のpublished ShiftOut、またはhandoff後のpublished Pass

同じtarget、Mission generation、sideを持つphase-compatible predecessor/successorだけを許す。

## Adapter

ShiftOut固定の`align_published_shiftout_execution_trajectory()`を、phase-compatibleな
`align_published_overtake_execution_trajectory()`へ置換する。

照合順は現在phaseがPassならPass、次にShiftOutとする。各候補は既存の
`rate_resolved_execution_source::build()`と`build_published()`を必ず通し、raw artifactを直接
authorityにしない。現在phaseがShiftOutならPass artifactは受理しない。

DynamicMissionWaitはorigin phaseがShiftOutまたはPassの場合だけ同じadapterを問い合わせる。

## Wall semantics

published canonical trajectoryがcurrent publication cursorでalignでき、そのtransition certificateが
有効なら、legacy/generic warning-band projectionは第二のcertificate ownerにならない。

これはwall checkの省略ではない。canonical artifactは既にcurrent-world exact wall proofを通過して
publishされている。`actual_wall_physical_contact`、`actual_wall_margin_blocked`、
`actual_wall_sample_unavailable`は別のhard faultとして残る。

## Deleted legacy assumptions

- published executionは常にShiftOutである、というidentity仮定
- Pass-origin DynamicMissionWaitではpublished executionを参照できない、という分岐
- canonical Pass publish後にprojected preflightへ戻る二重certificate ownership

## Non-goals

- candidate geometryの変更
- Pass goalやwall marginの変更
- cross-side戦術の変更
- Mission timingの変更
