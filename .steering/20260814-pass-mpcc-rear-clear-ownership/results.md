# Results

## 実装結果

receding-horizonのtarget bounds解除条件をbody-clearからrear-clear confirmedへ変更した。
body-clearは引き続き速度capの解除判定に利用できるが、targetが前方または横並びの間は
予測ホライズン上の横障害物として残る。

Returnのlive preflightが不成立で、同じ周期のPassホライズンも不成立になった場合は、
現在横位置を基準に短いcurrent-side holdを再生成する。通常壁余裕、次に設定済みhard
wall余裕で実行成立性を確認し、成立時だけPass phaseとMissionを保持する。壁接触、
静的壁判定不能、EmergencyBrake、solver Recovery、追い越し禁止区間では採用しない。

## ログ変更

従来の `body_release` を `rear_release` へ改名した。Return延期holdが採用された場合は、
receding-horizon reasonに次が出る。

```text
rear-clear Return deferred; retained current-side Pass
```

## 検証結果

- 全25 package build成功。
- `multi_purpose_mpc_ros` の25 test targets成功。
- 合計1110 tests、failure/error 0。
- 追い越しcoreは571 tests、failure/error 0。
- body-clearのみではtarget boundsを解除しないことを単体確認。
- Return延期holdはrear-clear、物理的に成立するcurrent-side horizon、hard faultなしの
  全条件を満たす場合だけ許可されることを単体確認。
- 実行ホライズン不成立後にReturn preflightを開始する経路も、Return成功なら即Returnを
  再構築し、不成立なら同じcurrent-side holdへ合流する順序に統一。

## 動的確認項目

前回 `output/20260814-214306` と比較し、次を確認する。

- targetが前方のPass中は `rear_release=0` であること。
- `body-clear Pass released opponent bounds` が消えること。
- `Pass -> Return -> Idle` の正常完遂が増えること。
- actual overlap、wall Recovery、77秒級の外れ周が増えていないこと。
