# Planned publication experiment

2026-09-11 JST。baseline f661ced6。診断のみ。本番変更・新しいauthorityなし。

r40 D1decision867 は、整数のnominal8874999801nsから8909999800nsへ
34.999999ms進み、20.789259msのcallbackでも25msの公開期限を超えた。
同じrunの882も20.389803msで約30ms進む。今回の丸め修正の対象とは異なる。

次の仮説は、将来の予定公開に対して、元の観測から必要な全prefixと停止までを
先に証明できるかである。既存certificateの時刻だけを動かしてはならない。
診断では元の観測時刻・各sensorの元stamp・公開済み履歴・world/peerの観測時刻・
model・hard boundsを保持する。予定時刻まで追加publishがない仮説の下で、元の
historyから状態を予測し、予定公開のpacketと全prefixを新しく構成・再証明する。
比較は予定時刻+0ms/+25ms。周期・公開windowはどちらも元の25msのまま。
元のsolver sourceは歴史的なsourceのまま、仮説のdecisionとinput identityを分ける。
Followの射影など、再構成できない入力はinconclusiveにする。

この静的比較がAcceptedでも、実際の送信はまだ許可されない。本番化には少なくとも:
- 元の観測時刻と予定公開時刻を別の型/フィールドで所有する。
- 予定より前のpublishを拒否する。現行の「受信済み観測がclockより先」という
  causal floorを予定時刻に流用して早期publishを許可してはならない。
- publishまでの実履歴が証明prefixと一致し、新しいsensor/peer/world/targetの
  情報が証明を壊さないことを検証する。単なるageやtarget ID一致では不十分。
- async完了順の逆転、reset、変更/取消、期限外completion、Stop/rest/restartを扱い、
  古いauthorityやcertificateを再束縛しない。最終送信は引き続き単一authority。
- 旧い同期publish経路を置換する場合は同じsliceで退役させる。新しいhold、grace、
  timeout、隠れたfallback、window拡張による実装にしない。

最初の診断の目的は将来prefixの物理的な成立性と構成費用を調べることだけ。
現在の観測を偽って未来の観測として記録しない。実行条件の設計と動的検証が
揃うまで、shadow結果から本番へ進めない。全M4–M6は継続して未完。

## Explicit scheduling primitives

R229の6件は将来25msのnominal/applied proofがAcceptedだった。しかし現行APIは
observation.nowと最初の予定公開を同じ値として使う。r229の未来viewのままでは
実際の判断から予定公開までの数値prefixは生成されても、その区間のwall/peer
validatorを呼ばない。この診断をそのまま本番採用することはできない。

共通予測器に明示的なscheduled APIを追加する。元のObservationProvenanceは
一切書き換えず、未来programmeのfirst epochを別に保持する。公開済み履歴は
元のnow以前だけを許し、未送信prefixはprogrammeの一部とする。検査は元のnow
から完全停止まで行い、未来のpacketで過去の入力欠損を埋めない。従来APIの
同時刻制約とfingerprintを維持し、scheduled結果は別の型とfingerprintを使う。
予定公開guardはbefore/afterの両方を実際の閉区間で検査し、causal floorを使わない。

このsliceは計算・時刻の共通部品だけで、normal authorityを追加しない。既存の
同期publisherはまだ変更しない。統合時には、実prefix台帳、fresh world/sensor
検査、単一dispatch、async/reset/取消を接続して旧同期実行を同じsliceで退役する。
それまでscheduled結果は実行不可。統合できない場合はこの未使用経路を削除する。
rollbackはこのsliceのfocused revert（baseline f661ced6）。rates、W25、profile250、
物理境界・数値刻み・hard marginsは変更しない。

受入れ: 待機prefixの検査拒否、未来を偽ったhistory/欠損、早期/期限後/逆行の拒否、
独立native schedulesの完全停止までの包含、旧APIのbit/hash/判定維持。共通部品の
検証はM4–M6走行の合格を意味しない。r40通常/r41headlessとも統合不合格。


## Results and next integration boundary

R40通常dev2はD1decision867、callback20.789259msでROSが34.999999ms進み不合格。
R41のNullGfx/noRVizでもD1decision552、26.027607ms/30msROSで不合格。表示を止める
設定は採用しない。Unityのfatal signalはTerminate・context disposal後のteardown
順序で記録され、stackはROS2 executor終了/Finalizeを指す。正確なsignal壁時刻は不明。
先に記録されたactive controller違反と区別する。両runは停止済みで保護成果物を復元した。

`ScheduledInputTube`は元の観測と予定programmeを別に保持する。sourceから元のnowへ
到達した後、待機区間を含めて各native intervalのbody/rigid footprintを検査する。
停止判定は未来の全packetとpositive inputの失効までを覆う。従来のAPI、数値演算、
同時刻条件、hash、v1–v4保存形式は維持する。scheduled hashは別domainであり、
既存のCertificate/adapterに渡せる型ではない。将来の送信履歴はoriginal nowで拒否する。

R230は独立native44test合格。最終suiteは45testを含む全2471記録/63groupが合格。
既存native受信時刻比較を0/75ms lead×連続/整数clock×18armsへ拡張し、72通りの
body/4corner包含と完全停止を確認した。待機prefixのvalidator拒否、入力欠損、
未来を偽ったhistory、早期/期限後/逆行、不変な旧数値結果/異なるscheduled hash、
静止→未来の加速→完全停止の回帰を含む。最後のケースも実車・実走行のrestartではない。

Buildr66:26packages/5m00s、testsr55:2470記録/32.1s。括弧のwarningを修正した
buildr67:26packages/14.2s、testsr56:2470記録/32.0s。静止中の未来加速の回帰を
追加し、buildr68:26packages/4m57s、testsr57:2471記録/32.0s。エラー/失敗/skip0。
残るbuild stderrは既存setuptools deprecationで、C++warningはない。

R231は診断内の非対応Follow参照でcompile失敗。R232はFollowを入口で拒否し、
到達しない診断branchを除去した。本番Followの検査は変更していない。R232はR40の
6件×0/25/75ms、計18通りで元のnowから停止まで全検査がAccepted。25ms待機で5–6、
75msで15–16の追加native区間を検査する。scheduled数値/物理検査単体は5.78–8.41ms。
これはnominal生成・非同期採用・実公開の費用を含まない。元の公開時刻対を25ms先の
予定へ当てはめるclock-only診断は4件を許すが、2件はbeforeが早いので拒否する。
実際にその異なるpacketを送信したという証拠ではない。

R233–235は旧21ケースの物理・Stop・materialization/join/production分類を維持。
259.999994msのactual history gapも拒否のまま。
[1565ファイルの封印済み証拠](planned-publication-evidence.json)。

次の実装は実送信の順序/clock/resetを持つprefix台帳と、予定programmeの照合。
計算中も既存certificateが約束したprefixを実行する必要がある。未完成の新しい証明を
理由に、旧programmeのStopを飛ばしてpositive inputを保持してはならない。未来prefix
を実historyへ偽装せず、明示programmeとして新しい全証明へ含める。新しいsensor/peer/
worldとの照合では旧tubeを新しい観測に再anchorしない。これらを満たす単一dispatcherと
非同期計算を接続するsliceで、旧同期authorityを退役する。M4–M6は依然未完。
