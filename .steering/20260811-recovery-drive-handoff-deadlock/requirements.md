# Requirements

## 目的

20260811-141846 の走行でP1が Stuck Recovery から前方Overtakeへ復帰する途中、Reverseのまま約44秒停止し続けた固着を解消する。

## 直接原因

- 実ギア報告は `Reverse` かつ車速はほぼ0だった。
- pure coreは `RequestDrive` を返していた。
- controllerが過去の `last_commanded_recovery_gear_ == Drive` を根拠に新しいDrive要求を `HoldStop` へ置換した。
- pending中もDrive要求を再送しないため、実ギアがReverseのまま永久待ちになった。

## 制約

- 停止確認前にDriveへ切り替えない。
- freshなDrive報告が来るまでRecoveryを解除しない。
- 追い越し候補、壁判定、solver判定、ROS 2インターフェースは変更しない。
- 再要求ログを40 Hzで大量出力しない。

## 完了条件

- 過去の最終指令がDriveでも、freshな実報告がReverseならDriveを要求する。
- 最初の要求が反映されない場合、停止中はレート制限してDriveを再要求する。
- freshなDrive報告後だけ既存条件に従ってRecoveryを解除する。
- 再現単体テストとビルドが成功する。
