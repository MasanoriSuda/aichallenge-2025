# Publication order and scheduled-prefix ownership

2026-09-11 JST。baseline/rollback 892cbdfc。全M4–M6は未完。

予定公開の数値部品は検証済みだが、計算中の実送信が予定prefixを守ったかを
判定する経路がまだない。現行履歴は同じstampの全値を保存するが、raw clockの
before/after、安定した送信連番、reset前後、意図したcertificateの対応を持たない。
別の指令やEmergencyが途中に入った予定計算を、そのまま採用してはならない。

`PublishedInputLedger`が既存のserialized historyを所有し、送信後に一回だけ
transactionを追加する。historyの値・causal floor・保持期間・同時刻の順序は
既存関数へ委譲して維持する。追加のtransactionは有限件数で保持し、raw clocks、
その送信の元certificate/source/indexを記録する。source metadataは意図した出典で
あり、公開後の検査を通ったauthorityという意味ではない。Emergencyは出典なし。

workerへ渡すsnapshotはhistoryのコピーと不透明なcursorを持つ。異なるledger、
reset、raw clock逆行、必要なtransactionが既に脱落したcursorは照合不可とする。
historyの通常pruningはcursorを偽って失効させず、途中送信を全て取り出せる間は
照合を続ける。raw clockが逆行した場合、既存history更新の意味は変えないが、
次の明示resetまでscheduled snapshotを作らない。

予定prefix照合は全transactionの個数・順序・source・float packet・元の各公開
windowを検査する。early、late、追加/欠落/逆転、異なる同stamp値を拒否する。
この照合は入力時刻と出典の検査だけで、壁/peer/sensor/intentのauthorityを作らない。
最新観測から旧tubeを再anchorするcacheも追加しない。

nodeでは全ての最終normal/Emergency/終了時送信を同じ台帳へ記録し、明示clock resetと
終了時に台帳をresetする。終了時はROS停止済みで最後の送信ができない場合もcursorを
失効させる。既存のnormal proof、single publisher、failsafeおよび公開guardは
維持する。新しい非同期実行へ接続する際に旧同期authorityを同じsliceで退役する。
この段階の追加情報だけで予定送信を許可しない。接続できない未使用経路は削除する。

最低限の検証: 旧historyとの値/順序/pruning/reset parity、同stampの別指令、source
改変、cursorのowner/reset/脱落、raw clock逆行、独立に変えた公開window両端、
prefixの追加/欠落をnative回帰で確認する。全体build/packageと固定commitの新しい
dev2で実記録を検証する。r40/r41失敗は消さず、timing解消やM4完了と呼ばない。


## Validation checkpoint

R236独立nativeは53test/220ms、追加8testを含め全合格。range-loopコピーのwarningを
修正しbuildr69は26packages/5m05s。終了時の直接送信経路も台帳へ統合し、最終
buildr70は26packages/4m13s、testsr58は2480記録/64group/32.0s、エラー/失敗/skip0。
C++warningなし、既存setuptools deprecationのみ。通常・failsafe・終了時の最終送信
3箇所を確認済み。旧normal guard/物理検査は維持する。
[27ファイルの保存証拠](publication-ledger-evidence.json)。
固定commitのdev2-r42は次の確認。公開期限解消、予定normal authority、M4–M6完了は
この結果から主張しない。次は型で区別した予定nominal生成と明示的な旧指令prefix。


固定56464c65のr42は起動時の観測停滞でinconclusive。r43は両Domainで台帳の連番増加・
256件上限・history pruning・通常のsourceと出典なしEmergencyを確認した。
D2decision683で実公開期限を超過し、統合走行は不合格。
[実記録と予定名目予測の証拠](scheduled-nominal-design.md)。
