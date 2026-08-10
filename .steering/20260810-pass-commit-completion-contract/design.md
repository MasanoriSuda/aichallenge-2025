# Design

## 1. Wall margin 時の Pass 完遂契約

Pass 中の wall 判定を次へ分ける。

1. physical wall contact: `RecoverPhysicalWallContact`
2. margin violation + target rear + rear-clear 未確認:
   `HoldPassForRearClearBeforeWallMarginRecovery`
3. margin violation + rear-clear 確認 + return corridor clear:
   `ReturnBeforeWallMarginRecovery`
4. 上記以外の margin violation: `RecoverWallMargin`

Hold は phase、side、frozen lateral goal を変更しない。現在 goal より壁側へ新しく攻めず、
既存 Pass budget、wall physical-contact guard、solver guardの範囲でrear-clearを作る。

## 2. Commit-stage aware SafeSeparation

`SafeSeparationRequest` に `PassCommitStage` を渡す。

- `Selectable` / `ShiftCommitted`: target が十分前方なら従来どおり RecoverBehind 可
- `SideBySideCommitted`: RecoverBehind 禁止。同側を保持し、少なくとも現在速度または
  target速度を維持して前後関係の改善を待つ
- `RearClear`: Return

絶対／局所 budget 超過や short-horizon hard fault は従来どおり Abort とする。

## 3. Current-overlap debounce の所有権

`CommittedPassForwardCompletionResolution.current_overlap_grace_active` を、将来予測の
成立とは分離する。次を満たす短い current-overlap 未確認期間だけ物理hard guardを通す。

- `SideBySideCommitted`
- target identity / course progress continuity
- current overlap が未確認
- physical wall contact / wall sample lossなし
- target intrusion / EmergencyBrake / solver recoveryなし

このgraceだけではforward-completionを新規成立させない。SafeSeparationを開始可能にする
ための短い橋渡しに限定し、confirmed overlap後は既存ContactContinuationまたはRecoveryが
所有する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: typed action、commit-stage aware resolver
- `mpc_controller_cpp.cpp`: current commit stageの配線とwall action処理
- `test_v2x_overtake_core.cpp`: 回帰テスト
- topic、message、launch、yaml値: 変更なし

