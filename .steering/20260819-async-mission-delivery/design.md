# Design

## 原因

`MpccLiteAsyncMailbox`は`latest_submitted_sequence`だけを保持し、worker完了時に

```text
result.sequence == latest_submitted_sequence
```

を要求している。

LatestOnlyWorkerは1 running + 1 pending構成である。計算時間が投入周期を超えると、
running jobの完了前にpending jobが投入されるため、running resultは新鮮かつ利用可能でも
上記条件で破棄される。負荷が継続すると全結果が同じ理由で破棄され、live側は
`tactical candidate generation owned by async worker`のまま候補を受け取れない。

## 修正

mailboxへ`latest_published_sequence`を追加する。
完了結果の公開条件を純粋関数で次のように定義する。

```text
same context
AND result.sequence > latest_published_sequence
AND result.sequence <= latest_submitted_sequence
```

pending jobの存在は公開拒否条件にしない。新しい結果が後から完了すれば、同じ単調更新条件で
mailboxを置換する。live callbackが未消費でも常に最新の完了結果だけを保持する。

live callbackでは従来どおり以下を再検証する。

- target ID
- context epoch
- Mission generation
- OvertakeLine phase
- locked side
- hard fault
- result age

したがって、古い世界状態のMissionをそのまま実行する変更ではない。

## 局所リファクタリング

結果公開条件を`latest_only_worker`の純粋関数へ分離し、mailbox実装と単体試験で同じ意味を共有する。
FSMやMission生成処理の構造変更は行わない。

## 動的確認

停止車両捕捉時に次を確認する。

1. worker内の`complete Mission selected`後、`adopted`が増える。
2. `OvertakeLine: Idle -> ShiftOut`が発生する。
3. 既存hard guardで明示的に拒否された場合は、その拒否理由がログへ残る。
4. `tactical candidate generation owned by async worker`のままSafetyBrakeへ到達しない。
