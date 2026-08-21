# Design

## Root cause

Entry 前の Race MPCC branch は V2X target を選択済みだが、OvertakeLine の target lock は Mission 採用後にしか作られない。従来は branch certificate と async lease の両方が locked target provenance のみを参照していたため、Entry 前結果が `invalid-expected` で全破棄されていた。

## Provenance lifecycle

`TargetProvenanceStage` を導入する。

- `Observed`: 現周期で選択された target の course projection と V2X observation generation。
- `Locked`: Mission が所有する同一 target の観測。
- `None`: provenance 不成立。

許可する遷移は `Observed -> Observed`、`Observed -> Locked`、`Locked -> Locked`。`Locked -> Observed` は Mission 所有権の退行として拒否する。

## Controller integration

- 車両走査中に nearest-front / nearest-side と同じ観測から provenance を保存する。
- Entry 前は selected target provenance、Mission 中は locked target provenance を返す単一 helper を用いる。
- physical execution certificate、async result validation、Race MPCC shadow log は同じ helper を使用する。
- target speed tolerance も lifecycle に応じて locked speed または entry target speed を選ぶ。

## Diagnostics

async tactical lease 判定を bool だけでなく reason code 付き resolution として公開する。既存 bool API は wrapper として維持し、周期ログに provenance stage と lease reject reason を追加する。

## Scope control

左右 solver、objective、wall clearance、速度パラメータは変更しない。片側可行時の既存 branch selection は単体テスト済みのため維持する。
