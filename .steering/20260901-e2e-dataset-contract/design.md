# E2E Dataset Contract Design

## Sequence Identity

入力rootからbag directoryまでの相対pathを正規化し、短いSHA-256 suffixを付けた
`sequence_id`を生成する。basenameだけを使わない。`--seq-dirs`でも絶対path由来の
fingerprintを使い、同一invocation内の衝突は抽出前に拒否する。

## Synchronization

scan timestampを基準にnearest control commandを選ぶ。同期差が
`max_sync_delta_sec`以下のsampleだけを採用する。既定は50 msとし、実bagのLiDAR・
controlがともに20 Hzであることに対応する。

保存物:

- `scans.npy`
- `steers.npy`
- `accelerations.npy`
- `delta_times.npy`
- `scan_timestamps_ns.npy`
- `control_timestamps_ns.npy`
- `metadata.json`

## Run-Level Split

`sequence_id`とsplit seedのSHA-256から決定論的にtrain/validationを決める。
sampleを分割しない。出力は`<outdir>/train/<sequence_id>`または
`<outdir>/val/<sequence_id>`とする。

## Error Policy

個別messageのdeserialize失敗は件数と先頭理由を記録する。topic type不一致、scan
shape変動、必要topic欠損、全sample同期rejectはsequence失敗とし、process全体の
終了コードを非0にする。部分成功と全成功を混同しない。

## Label Provenance

抽出時に`--label-source`を必須とし、`mpc`、`mpcc`、`human`、`student`、`other`
から明示する。trainer既定は`mpc/mpcc/human`だけを教師として受理する。student
bagはfailure観測には使えるが、corrective labelなしに教師へ混ぜない。
