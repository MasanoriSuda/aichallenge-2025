# Nominal forecast with an explicit committed prefix

2026-09-11 JST。baseline/rollback56464c65。M4–M6は未完。

最初の破綻は現行Requestが公開時刻を観測nowと同一とすること。同期の全証明後に
その時刻のwindowを超える（r43D2decision683:4.274999904→4.304999903、期限
4.299999904）。早期計算には未送信の旧programme prefixを明示して将来のpacketを
組み立てる必要がある。r229/r232は途中追加送信なしの診断であり実行の根拠にしない。

`ScheduledPublicationPrediction`は元のObservationProvenanceと、別の有限予定prefix、
元のnowの状態・予定公開時点の状態・呼び出し側が明示する予定control originを持つ。後続のnominal生成は
旧programmeの指定prefixを含め、新しいfirst packetを選び直す。未送信指令をactual
historyへ偽装する戻り値を作らない。既存nominalと同じく、選択packetをcontrol origin
まで保持する点予測であり、実入力集合・壁・peer・restのcertificateではない。

既存の履歴予測から数値積分だけをprivate engineへ抽出する。旧APIは未来historyを
引き続き拒否し、同じ演算/分割点で結果を返す。新しい入口だけが、元のnow以後の
明示prefixを数値scheduleへ追加し、予定公開境界でも区切る。元の観測・component
時刻・actual historyは戻り値にそのまま保存する。モデル、delay、周期、window、
物理境界・solver設定は変えない。新しいhold権限を与えない。

受入れ: zero-leadで旧prospective結果一致、独立nominal kernel比較で加速/操舵の
異なる遅延と旧Stop prefixの効果を確認。未来history・不正grid・index・欠損を拒否。
保存した旧関数とhistorical/nominal結果を比較し、全build/packageを実行する。
この部品を予定nominal/full-rest/current-world/async単一dispatchへ接続するsliceで
旧同期normal authorityを退役する。予定結果を旧Certificateへ型変換して採用しない。

r42はspawn/初期sensor後に更新が停滞。両Domainの3秒read-only観測ではclock/odom/
vehicle reportなし、制御計算未到達。IMUはprobeのtopic違いで判定対象外。
Unity/DDSの停止箇所は未確定。外部停止理由・Playerログ・復元結果を保存した。
r43は通常起動。両車で台帳sequence増加、256件上限、history pruning、通常sourceと
出典なしEmergencyを観測。clock discontinuities0。D2decision683post失敗を保存し、
全体の走行合格とはしない。両run停止済み、保護成果物のSHA一致を確認済み。


## Numerical comparison

R237は旧関数800入力の全出力bit一致とzero-lead一致を確認したが、新規future比較で
失敗した。1.05+0.1から得た1.1500000000000001と1.125との差は25msをわずかに超え、
既存のdivision/ceilが6区間にする。5ms刻みの独立比較とyaw-rateで0.000104851187806の
差があった。比較の許容値を広げず、future整数wordのapplication境界と積分区間長を
整数nsで計算するよう変更した。旧APIとzero-leadは従来演算を保つ。

R238は時刻変換へpublication-window builderをゼロwindowで呼んだため入口で拒否。
その未採用結果も保存した。R239は既存の厳密ns変換を公開した共通関数を使う。
将来control originは明示引数とし、過去のderived control-origin差から勝手に丸めて
作らない。非grid source/query/channel時刻を整数wordで拒否する。全57test/239ms
合格、旧800入力の全state/path出力bit一致。native独立比較の許容値は1e-11のまま。
これらはnominal点予測の検証で、全入力集合の現在world/authorityは別途必要。


最終buildr71は26packages/5m02s。testsr59は2483記録/64group、エラー/失敗/skip0。
C++warningなし、既存setuptools deprecationのみ。
[1183ファイルの保存証拠](scheduled-nominal-evidence.json)。
次は、この型を予定nominal/full-rest検査とprefix連結に使用し、fresh sensor/worldと
実送信windowを照合する非同期単一dispatcherへ接続する。旧同期authorityの退役は
接続時に行い、この名目予測だけで実行を許可しない。
