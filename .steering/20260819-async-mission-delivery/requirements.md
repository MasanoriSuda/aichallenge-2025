# Requirements

## 背景

`output/20260819-091547` では、P1が停止中のd2を11.28 m前方で捕捉し、
非同期worker内ではcomplete Missionと即時Overtake handoffが成立した。
しかしlive callbackへMissionが届かず、`Idle -> ShiftOut`へ遷移しないまま
SafetyBrake、Stuck Recoveryへ進んだ。

workerが約0.12秒以上かかる区間では、実行中jobの後に新しいsnapshotが投入される。
現行mailboxは「完了jobのsequenceが最新投入sequenceと完全一致する場合」だけ結果を公開するため、
常時1件pendingがあると完了済みの有効結果が無言で破棄され続ける。

## 目的

- 実行中に新しいsnapshotが投入されても、完了した単調に新しい結果をlive callbackへ公開する。
- live callback側のtarget、phase、Mission generation、side、age、hard-fault検証は維持する。
- 新しい戦術、FSM状態、solver、ROSインターフェースは追加しない。

## 制約

- context epochが変わった旧セッション結果は公開しない。
- 既に公開したsequence以下の結果でmailboxを巻き戻さない。
- 未投入sequenceの結果を受理しない。
- `aichallenge/result-summary.json`の既存変更を変更・コミットしない。

## Definition of Done

- newer pending jobが存在しても、同一contextの完了結果を公開できる。
- context不一致、sequence巻き戻り、不正な未来sequenceを拒否する。
- async結果の既存live admission guardを変更しない。
- `multi_purpose_mpc_ros`の単体試験とビルドが成功する。
