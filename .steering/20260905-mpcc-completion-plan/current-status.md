# Current status and remaining completion work

2026-09-13 JST。M1–M6を追加確認なしで自律実行。必要分をローカルcommit、pushなし。
**全体未完。ゲームStart・車両Start・Rejoin復帰は確認済み。周回・統合受入れは未達。**

描画を保つ200FPS診断single-r31で、時計受信間隔は中央値5.004ms・最大13.948ms。
標準r30の中央値0.600ms・最大294.599msとの差はフレーム上限の関与を支持するが、将来の上限ではない。
4217callback最大23.941ms、通常送信0件。車両Start/Rejoin/周回は未達で、期限問題の修復とは判定しない。
初期501とactive856は証明worker33.429/29.854ms、採用時刻が予約期限を約5ms超過。
同時に履歴も更新済みなので、期限だけを唯一原因と断定せず、元の入力・世代・履歴を再生する。
壁区間2163とRecovery4164は元の判定に完全一致。余裕込み壁接触とコース悪化を正しく拒否。
次は役目を終えた壁区間observerを削除して1263の境界回帰を残し、予約生成・採用の時間と履歴を監査。
無変更の走行を反復せず、25ms期限・物理条件・正常系の認証を維持。全コース/M4–M6は未完。
[直近の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/render-cap-live-audit.md)。

以下はr31走行前の履歴。

single-r30は実送信8440の期限超過で中止。正常送信2420件とは別に失敗した通常送信1件を保持。
元の入力・履歴・証明経路をR836で一致再生し、送信前許可→raw診断終了時点/送信後拒否を確認。
8784callback中644の1件25.030ms、連続超過なし。車両Start/Rejoin復帰1回、周回は未達。
壁区間1263は全地図・元クエリで完全一致。現在は非接触だが、1mm上側の安全余裕込み車体は
壁セル466554に接するため、既存境界ガードの拒否は正当。壁条件を緩めない。
原DLLの標準60FPS制限はbatchmode/nographicsでも解除される。r27は描画だけの比較ではなかった。
次は描画を保つ--target-fps 200だけのsingle-r31（120host秒）で制限の影響を切り分ける。
物理・時計刻み・制御・25ms期限とガードは維持。時計問題・全コース/M4–M6は未完。
[直近の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/full-course-current-audit.md)。

以下はr30走行前の履歴。

7ecd0b36のsingle-r29で車両Start、Rejoin送信5件・復帰完了1回、通常2741送信を確認。
実期限外送信ゼロ、送信前拒否22件からの通常送信なし。4500callback中2件超過、連続なし。
2267は28.585ms（normal join16.687/Recovery10.407ms）で送信前拒否、2286は26.109ms
（Recovery21.738ms、全体CPU14.764ms）。後者の処理内/off-CPU詳細は未確定。
Recoveryの実選択は前進5件だけで、修正分岐の後退実行は未観測。Startを修正効果と断定しない。
壁区間拒否・方向不明の新記録はなし。周回・全体M4–M6は未完。
次は同じ制御コードの全コースsingle-r30（6周/600sim秒、660host秒上限）で未到達範囲を確認。
過去r26/r80の実送信期限問題は未修復で、同じ元のガードと失敗時の中止を維持する。
[直近の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-preference-live-audit.md)。

以下は走行前の履歴。

Recoveryの前進優先を候補順序として実装し、不合格なら残りの既存候補へ進むよう修正。
元の前進評価と接触・段階的・後退の分岐内容を保持。同一クエリ内の同じ前進評価だけ
結果を再利用する。R821旧版2失敗→R822native73、最終source118、R823全地図の比較、
build142全26/package129全2664tests合格。次は固定commitのsingle-r29で実行・時間・復帰を確認。
未評価だった元の後退距離の完全再生、先行通常停止・期限問題・全コース/M4–M6は未完。
[設計・検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-preference-order-design.md)。

以下は候補順序修正前の履歴。

7f263cc2の標準描画single-r28は通常675送信、Rejoin送信3件・復帰完了1回、実期限外送信ゼロ。
4271callback中495の1件25.218ms（初期化14.746ms/Recovery6.402ms）、連続超過なし。
2771は送信前拒否。ゲームStart成立、車両Start・周回は未達。壁区間拒否は発生せず観測なし。
後半のRecovery2906をR818で完全再生。前進3候補は全て壁またはコース条件で拒否され、
「前進優先」の分岐が後退評価まで省く。これはordering-onlyという宣言と不整合。
R819は同じ地図・姿勢で既存後退4m/+0.15rad、8m/+0.10radの物理・コース検証に合格。
未評価の元の後退距離は未保存なので、これらは明示した距離での診断であり実行許可ではない。
次は優先指定が候補を消さない選択処理の回帰・修正・検証。先行する通常停止の原因、
標準描画の期限問題・全コース/M4–M6は未完。
[直近の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/first-interval-live-audit.md)。

以下は走行前の履歴。

初回の壁区間拒否を通常移動前も保存する観測へ修正。先行通常記録は任意の実証拠とし、
元の地図・クエリ・判定・安全条件を維持。R812旧版1失敗を再現、R813native34/source118、
build141全26package、package128全2660tests合格。次は標準描画のsingle-r28で初回実入力を
取得して一致再生する。壁区間の原因・標準描画の期限問題・全コース/M4–M6は未完。
[観測の設計・検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/first-wall-interval-design.md)。

以下は観測範囲修正前の履歴。

single-r27の描画なし診断は通常738送信、4281callback最大24.392ms、実期限外送信ゼロ。
時計の記録側受信間隔中央値5.001ms・最大14.327ms（標準描画r26は中央値0.574ms・
最大296.983ms）。環境差への支持はあるが、経路・場面が異なり描画の原因確定ではない。
Recoveryは後退25/前進3選択、小操舵−0.05radの指令と実負速度を確認。Rejoin送信2回、
復帰完了・車両Start・周回はこのrunでは未達。壁区間拒否は初の通常移動記録より前に
現れ、既存の観測範囲から外れていた。last_physicalは過去値の保持で、連続拒否を意味しない。
次は先行通常移動を必須としない初回実クエリ観測へ置換し、原地図・元の判定を一致再生する。
標準描画の送信期限問題と全コース・M4–M6は未完。
[直近の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/renderer-clock-live-audit.md)。

以下は描画診断前の履歴。

single-r26は実送信2180の期限超過で中止。721正常送信とは別に、期限外の通常指令1件と
直後のfailsafeを保存。元の指令・証明・履歴がR805で一致し、送信前許可→送信後拒否を再現。
callback3.46msでもROS時刻が25ms進む。直前の記録側clock受信間隔は297msで、原因の
描画・GC・DDS等は未確定。単純な5ms余裕や期限拡大は採用しない。次は元DLL・物理・
制御を保つ描画なしsingle-r27診断で時計と送信を比較。r25車両Startは別runの確認済み証拠。
全コース・周回・統合受入れとM4–M6は未完。
[期限超過の再生と次の比較](../20260910-mpcc-empirical-plant/receiver-input-enclosure/full-course-publication-timing-audit.md)。

以下は全コース試行前の履歴。

38a562eeのsingle-r25で車両Startを初確認。通常3321送信、実期限外送信ゼロ。
4573callback中3646の1件が26.73ms（現在検証24.63ms）。その送信は元の期限内で、
送信前拒否19件からの通常送信はない。壁区間拒否・観測・Recovery操縦・Rejoinは未発生。
観測追加の効果とは断定しない。120秒内の周回は未確認。次は同じ制御sourceで
6周・600sim秒、660host秒上限のsingle-r26へ進み、全コースと時間・結果を検証する。
callback超過・未再現の壁区間原因・多車両を含むM4–M6は未完。
[実走結果と全コース検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/current-wall-interval-live-audit.md)。

以下は実走前の履歴。

現在の壁区間が拒否される最初の実入力を、通常走行の認証済み送信後に一度だけ保存する
観測を追加。地図・姿勢・元の区間計算・preferred containmentを保持し、走行判断は維持。
R793native220/source118、build140全26、package127全2658tests合格。
次は固定commitのsingle-r25で実入力を取得し、元の地図・安全条件のまま一致再生する。
壁区間の原因は未確定。車両Start・周回・全体M4–M6は未完。
[観測の設計・検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/current-wall-interval-observation-design.md)。

以下は観測追加前の履歴。

1648f338のsingle-r24は通常552送信、4267callback最大24.30ms、期限外送信ゼロ。
壁補正1回を経て認証されたsource785から3送信を確認。Rejoin送信2件以上、復帰完了なし。
ゲームStart成立、車両Start・周回は未達。後半は現在の物理的な横幅区間が採用されず、
停止コース形状が欠落する。現ログはpreferred containmentと実区間を欠くため、
次は元の入力・地図・区間結果を一度だけ保存して再生し、生成元の原因を確定する。
全安全条件を維持。小操舵の後退実行と全体M4–M6は未完。
[実走監査と次の観測](../20260910-mpcc-empirical-plant/receiver-input-enclosure/full-wall-feedback-live-audit.md)。

以下は実走前の履歴。

完全な壁検証で将来の軌道接触を検出し、元の最大3回内で数値補正を続ける修正を実装。
現在接触・既通過区間の異常は追加補正の対象にせず、最終の壁・他車・停止認証を維持。
R785旧版の実入力テスト失敗→R786native162/source118合格。R787本番ビルドで
Rejoin537/656の認証が成立し、追加3場面の元の拒否を保持。build139全26package、
package126は2655tests合格。次は固定commitのsingle-r24で計算・送信時刻とRejoinを確認。
全体・車両Start・周回・M4–M6は未完。
[設計・検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/rejoin-full-wall-feedback-design.md)。

以下は修正前の履歴。

R776–R782でsingle-r23のRejoin入力を監査。537は現在姿勢が無接触で、3.615秒先の
壁接触を再現した。補正の継続条件へ完全な壁検証を加える診断では、537/656が元の
最大3回内に状態・壁・他車・停止継続を満たす。壁の近似制約を再生成する別方式は
537で拒否、656で合格。本番は未変更。次は現在接触などの負例と追加入力を検証し、
元の上限内の最小修正を実装・build・実走へ進める。全体・M4–M6は未完。
[比較と次の修正条件](../20260910-mpcc-empirical-plant/receiver-input-enclosure/rejoin-full-wall-feedback-audit.md)。

以下は実走直後の履歴。

226e4f84のsingle-r23は通常522送信、4260callback最大23.50ms、期限外送信ゼロ。
認証済みRejoin送信4件以上を確認したが、復帰完了・車両Start・周回は未達。
Recoveryの実選択12件はすべて前進で、小操舵の後退実行は未検証。方向不明の保存もなし。
後半は停止軌道のコース形状を生成できない。保存したRejoin537/538・656/743を再生し、
現在接触・軌道の壁拒否・後段の入力欠落を分離する。全安全条件を維持しM4–M6を継続。
[実走監査と次の調査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-steering-live-audit.md)。

以下は実走前の履歴。

Recoveryの新規後退候補を補助MPCの有効・無効から分離し、既存設定の操舵角を探索する。
保存1921では元の8m候補3件が壁拒否する一方、−0.05/−0.10radは元の壁・コース条件を通る。
R768旧選択失敗→R769native249/source118合格、R770完全地図で元3件の拒否を保持し
−0.05radを選択。距離・設定・壁・V2X・停止・通常権限・期限は維持。build138全26、
package125合格。次は固定commitのsingle-r23で実選択・再検証・時刻と周回を確認する。
全体は未完。先行する後退中の障害停止、車両Start・周回・M4–M6は引き続き未達。
[設計と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-steering-population-design.md)。

以下は今回の修正前の履歴。

1b478b27のsingle-r22:通常1133送信、4329callback最大24.70ms、実期限外送信ゼロ。
認証済みRejoin実送信3件以上・復帰完了1回、準備候補のsource認証も確認。周回は未達。
判断1921の方向選択失敗を完全保存し、R764で全4件を一致再生。8m後退3候補はすべて壁拒否。
先行する後退中の障害停止と、その後の要求距離8mへの増加を分け、同じ地図で候補形状と
元の4m要求を比較する。安全条件は維持。ゲームStart成立、車両Start・M4–M6は未完。
[実入力・再生・次の比較](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-direction-live-audit.md)。

以下は今回の実走前の履歴。

RecoveryのCheckClearanceで方向を選べなかった最初の1件について、実際の候補入力・
地図・物理結果とコース条件を限定保存する観測を実装。走行判断・候補順・安全条件は維持。
R760native245/source118、build137全26/package124合格。保存したセルと入力から元の
壁拒否・コース拒否・合格例を再生できた。次は固定commitのsingle-r22で実入力を取得。
Rejoin準備の実走採用・車両Start・周回・M4–M6は未完。追加確認なしで継続する。
[限定観測の設計・検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-direction-observation-design.md)。

以下は観測実装前の履歴。

6f42d134のsingle-r21:通常408送信、4248callback最大19.53ms、期限外送信ゼロ。
913のRecovery停止後、方向候補を選べずSafeStop。Rejoin準備は適用場面に到達せず、
実走採用は未検証。予約無効7件は期限超過4/期限前3で、914は停止後の世代・履歴拒否。
ゲームStart成立、車両Start・周回は未達。次はRecovery方向候補の完全入力と各拒否を
取得・再生し、元の壁・コース検証を維持して生成元を調べる。M4–M6は未完。
[監査と次の観測](../20260910-mpcc-empirical-plant/receiver-input-enclosure/rejoin-preparation-live-audit.md)。

以下は今回の実走前の履歴。

Rejoinの数値初期化を、操舵準備から加速へ進む候補と元の候補の2件に接続した。
QPの入力・状態制約・目的関数・物理証明・送信期限は維持。R753native208/source118、
build136全26/package123合格。実装で保存537/548の完全物理証明が成立し、元候補と
追加3場面の元の拒否を保持。527の準備候補は現在の壁接触で拒否する。
次は固定commitのsingle-r21。ゲームStart確認済み、車両Start・周回・M4–M6は未完。
[今回の設計と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/rejoin-preparation-design.md)。

以下はこの修正前の履歴。

7468cf46のsingle-r20:通常534送信、認証済みRejoin1件、4257callback最大17.98ms、
実期限外送信ゼロ。1111は送信前拒否。復帰完了・車両Start・周回なし（ゲームStart成立）。
予約無効13件は期限超過9/期限前4で、保存済み577/780は世代失効。期限を緩めない。
後半は壁プロファイルと停止コース形状を生成できずStore704で停止。後段の完全入力は
未取得のため、物理的余裕があるとは断定しない。先に保存済みRejoin548の別世界で
固定準備候補と自由QPを比較し、生成元の修正を絞る。全体M4–M6は未完。
[現在の監査・未解決点](../20260910-mpcc-empirical-plant/receiver-input-enclosure/reserved-order-live-audit.md)。

以下は今回の実走前の履歴。

予約区間優先の候補生成順を実装。元の期限・壁・他車・停止・指令履歴の検証と、
長い候補・短い候補の代替経路を維持する。R740旧2回検証を再現、R741native166/
source118、build135全26/package122合格。R742実ビルドで6入力の合否と合格指紋が一致。
元の遅延入力は約43msで同じ証明を生成。次は固定commitのsingle-r20で実走確認する。
ゲームStartは確認済みだが車両Start・周回・M4–M6は未完。
[今回の設計と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/reserved-program-order-design.md)。

以下は今回の実装修正前の履歴。

40d1664bのsingle-r19:通常2325送信、Rejoin完了2回、4463callback最大22.09ms、
実期限外送信ゼロ。863/1194は送信前拒否。ゲーム全体のStartはUnityで確認済み、
車両Start・周回は未達。復帰後の予約計算を初めて完全取得し、R734で指紋一致再生。
R736は長い候補の壁拒否に約41ms、続く予約区間の候補に約24ms、領域作成に約19ms。
R737の予約区間優先は同じ指令・証明で合計約43ms（診断のみ）。次は追加入力と負例を
検証し、この候補生成順の修正境界を固める。速度・期限・安全条件は維持。
[現在の監査と次作業](../20260910-mpcc-empirical-plant/receiver-input-enclosure/reserved-program-order-audit.md)。

以下はsource修正時点の履歴。

現在は計画の実行区間を非線形再計算し、元の9状態制約を毎回検証する。
違反時の既存補正は、連続した非線形状態列から線形化する。通常権限・モデル・期限・
パラメータ・候補数は維持。R727で旧診断の速度超過見逃しを再現し、R728native179/
source118、build134全26/package121合格。R724自由QP診断、R729固定4候補、
R730追加3場面で合格例と元の壁・計算拒否を保持した。停止操舵準備は本番未採用。
次はローカルcommit固定single-r19。current/retainedの速度制約、復帰後の予約計算、
r80D2実送信期限、Start/周回・M4–M6は未完。
[設計と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/nonlinear-source-state-design.md)。

以下はこの修正前の履歴。

最新: 9c522a95のsingle-r18は認証済みRejoin・Start・周回なし。
4234callback最大19.482284ms、通常357件、送信前拒否・実期限外送信ゼロ。
復帰後記録の成立条件に到達せず、live観測は未検証。R708–R711でsource537の
0.725秒後の壁cell471815接触を再現。R715の停止操舵準備は2候補が物理検証合格。
R716の自由QPは壁を通っても非線形速度が2m/s上限を超え、採用不可。
R717は速度検証と非線形の連続状態列による線形化を比較し、1回の既存補正で
速度・壁・他車・停止継続が成立する診断候補を確認。本番制御は変更していない。
Rejoin診断適用漏れは旧失敗R719→R720全50/source118、build133/package120合格。
R721実ビルドで元の壁拒否を保持。次は元の速度制約の保持と連続した線形化の
実装境界を確定し、追加の保存世界・build/package/liveで検証する。
復帰後の予約計算入力、r80D2実送信期限、物理観測・M4–M6は未完。
[現在の監査と次作業](../20260910-mpcc-empirical-plant/receiver-input-enclosure/rejoin-wall-source-audit.md)。

以下の「次」は以前の時点の履歴。

最新: b3613667のsingle-r17で認証済みRejoin実送信とRejoin完了→Cruiseを2回確認。
全4296callback最大26.41ms、通常856件、最終1772。863は送信前期限拒否、実期限外送信ゼロ。
ただしCruiseは短い加速と制動を反復し、予約済み後続計算が元期限を超過（job1207約148ms）。
1773の通常候補は現在認証済みでも再停止確認でRecoveryが上書きし、最後は進行方向不明の
SafeStop。Start/周回なし。R699–R701で保存、元DLL・ユーザーJSON復元済み。
限定保存の実装はR704native193/source118、build132全26/package119合格。
実際のRejoin記録と、予約計算が試した最大4候補の完全入力・結果・時間を独立保存する。
次はローカルcommit固定single-r18で取得し、native再生と
構造比較で修正箇所を確定する。先行失敗と別の保存枠を使う。速度/期限/安全条件は緩めない。
[現在の監査・次の観測](../20260910-mpcc-empirical-plant/receiver-input-enclosure/post-rejoin-source-audit.md)。

以下は引き継ぎ修正前までの履歴。

最新: a865ed4eのsingle-r15は起動clock停止で有効制御なし。single-r16は候補供給が
Rejoin中も継続し、952/source528の現在証明を受理したが旧ID一致条件が停止で上書き。
R694で実候補から再現、R695でID書き換え案と厳密な現在dispatcher照合を比較。
R696のテスト経路不備を保持・修正し、R697は3実証明経路の243native、source118、
build131全26/package118合格。元証明IDと送信判断IDを保持する引き継ぎへ変更。
次はローカルcommit固定single-r17で認証済みRejoinの実送信を確認する。
single-r16最大21.47ms・343通常送信・期限外ゼロだがStart/周回なし。r80D2の実送信
期限、壁/停止軌道と物理観測、M4–M6は未完。R691は記録範囲外のclock解釈を訂正。
[現在の引き継ぎ修正・根拠](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-rejoin-handoff-design.md)。

以下の「次」は各時点の履歴。現在の次作業は上記を正本とする。

通常証明の数値実装fdee0b8fは、二つの完全な入力予測範囲と、包含を確認した事前計算の座標変換。
Build126全26/package113全2622/source112/native216合格。新規single-r11を120秒確認し、
4255callback最大19.92ms、送信期限拒否・期限外通常送信ともゼロ。局所的な実走時間確認は通過。
ただし通常送信は1047/source651/index13で終わり、1048は元の停止証明期限切れ。
その後もStore651のままで再発進・Start・周回なし。ユーザーJSON・元DLLを保持しruntime停止。

R647/R648で、停止判断1048の両初期化候補804/805が軌道stage71の実際の壁検証を
拒否することを確認。R649は元入力・証明・実指令の一致を保って停止証明期限切れを再現。
R650の小加速2候補は0.0034m/s・約6mmにとどまり再発進の根拠にならない。
より強い加速と、R651の停止操舵準備4候補は壁/軌道条件で拒否。安全条件を変えず記録した。
既存B/C/D/G比較は対象車なしのCruiseに未対応で、走行不能とは判定できない。
R653で最初の壁は左前方cell476325と特定。R655非線形3初期値比較は、合格候補が約4.5cmに
とどまり再発進を確認できず。走行不能とはしない。R654の診断サンプル対応不備も保持。
別の適用漏れとして、準備済みsimulation ReadyをRecovery動作セッションへ接続し、
LowSpeedRejoinの直接速度・加速・操舵生成を除去。速度上限を最適化へ渡し、
現在認証済みRejoin/Stopの原指令だけを通常dispatcherへ戻す。元のStart reset/gear/
launch/Boost/安全条件は維持。R656native228/source115、build127/package114合格。
9950c145のsingle-r12は未合格。4246callback中865が36.77msで元期限を超え拒否、
336通常送信は864で終了。Ready停止確認後、自己取消によるsolver待ちで確認を失う
885回のループを観測。R664で再現し、元の待機順序で確定済み確認を保持する案は
R665全150Core test合格。R662の待機順序を変える案は57回帰失敗のため未採用。
R663は2048の新鮮な空V2X配列を確認。宣言単車なのにRecoveryが不完全扱いし、
旧force-motionが情報欠落を無視していた。宣言台数の接続と壁/V2X/予算bypassの撤去を
実施し、R667全973native/source117、build128/package115、実launch5条件が合格。
次はcommit固定single-r13。R668で865の元入力を再生し、期限問題は別途継続。
[現在の設計・根拠](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-confirmation-clearance-design.md)。
07897bedのsingle-r13で確定保持は動いたが、標準XMLの台数配線漏れで未指定0となり
SafeStopへ進んだ。4236callback最大30.77ms（起動485）、361通常送信は899まで。
期限外通常送信・V2X強制overrideゼロ、Start/周回なし。R674実XML5条件失敗、
接続後R675全5合格、build129/package116合格。次は固定版single-r14とdev2-r80で
実際の台数伝播・完全情報・Recovery/Rejoinを確認する。
[標準起動の配線修正](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-launch-entry-design.md)。
source壁問題・物理観測誤差・全M4–M6は未完。追加確認は不要。
[設計と局所検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-session-rejoin-design.md)、
[停止source比較](../20260910-mpcc-empirical-plant/receiver-input-enclosure/source-cell-native-feasibility-audit.md)。

ea14f80aのsingle-r14は4234callback最大21.13ms、通常送信355件で期限外送信ゼロ。
台数1とV2X完全性が成立し、Recovery前進・Rejoinまで進んだが、認証待ちの停止が
次候補供給も抑止する循環をR681で確認。10回のRejoin中通常送信ゼロ、Store/worker
供給が止まり最後はattempt limitでSafeStop。公開権限を維持した候補計算の配線修正は、R685Core151/source118、build130/package117合格。次は固定版single-r15で実際の供給再開を確認。
同一版dev2-r80は両Domain台数2・相手IDを受信し空publisherなし。ただしD2decision593で
送信中に元期限を約5ms超えたため停止。成功送信ログだけの集計からゼロと判断しない。
R686は元のsource/証明/履歴/実送信を保って送信前合格・後拒否を再現。実送信期限問題は未解決。Start/周回/統合受入れは未完。
[現在の候補供給設計と実走根拠](../20260910-mpcc-empirical-plant/receiver-input-enclosure/recovery-rejoin-supply-design.md)。

以下は先行runの履歴。現在の次作業は上記とする。

この変更前の実装検証はbuild123全26、package110全2618、source106合格。
b368ecabのsingle-r9で停止後の両初期化の失敗入力を取得し、R595–R606で再生・切り分けた。
最初の停止1160後も177回通常送信し、最後は1342。1343は新しい1402の現在壁検証と
古い1374の停止証明期限で拒否。約40秒までStoreは新しい認証済み計画を受け取っており、
ソルバー不足だけでは持続停止を説明できない。370件の後続送信証明失敗のうち369件は壁。
同じ保存世界で非線形の前進軌道は存在するが、新しい認証済みsource1986を使う再構成でも
送信時刻の幅を含む検証が30.209999331で壁拒否となった。次はこの適用入力・現在状態の
検証境界を修正する。制御モデル・安全条件は未変更、再発進・Start・周回・M4–M6は未達。
[今回の原因切り分けと証拠](../20260910-mpcc-empirical-plant/receiver-input-enclosure/post-loss-source-audit.md)。

制御モデル・通常権限は8d9810e0を維持。受信履歴の診断追加をbuild121全26、
package108全2613、旧3失敗/新3合格、過去3ケースの判定保持で検証。標準single-r7/R563–R566で
20判断の受信済み履歴を取得し、位置時刻により新しい値を除外した事実を確認した。
最大1.04m/sまで動き、正常送信1011の後に停止状態で通常権限を失った。全4239callbackは
25ms以内、送信期限違反なしだが、再発進・Start・周回は未達。受信値の補間案を固定し、
force-r3/R567–R569で位置時刻の物理状態と比較した。25判断のうち16件を照合し、
速度平均誤差は改善したが、未支持の最大誤差と旋回の反例が残るため未採用。単純な外挿も
横速度・操舵の悪化から棄却。R570–R575で共通時刻からの連成再構成も比較し、
横速度・ヨーレート・操舵の悪化から未採用。操舵の比較は物理更新前へ訂正した。
実際の適用入力なら操舵機構は最大約2.4e-8radで一致し、入力時刻と車体モデルの誤差を
分離できた。R576–R582で接地方向・3D成分を切り分け、542区間で候補を比較したが、
単独の時刻修正と追加接地モデルは過去区間の反例から未採用。公開IMU姿勢は物理姿勢と
一致する一方、角速度は5msの姿勢差分であり、力学モデルが使う瞬時角速度とは差がある。
この測定定義を含む観測・車体・接地の契約整合は、引き続き未解決。
先に独立したStop/rest/restartの記録欠落を修正した。起動時の停止が保存枠を使うため、
実際の正常送信で走行した後の停止を、送信証拠付きの追加固定枠へ保存する。
R584旧2失敗/新2合格、source106、build122全26/package109全2615、R586過去3判定保持。
single-r8/ec8e822bで停止1052のdue/active記録を取得。R589は元の停止証明期限切れを
正確に再現した。4229callback最大19.29ms、送信期限違反なしだが再発進・Start・周回なし。
新しい計画の認証が供給されない前段は未解決。次は実際の停止判断後の新しいsolver失敗を、
両初期化候補の完全な入力として保存して原因を絞る。
[実走結果と次の観測境界](../20260910-mpcc-empirical-plant/receiver-input-enclosure/post-motion-source-audit.md)。
[記録欠落の修正と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/post-motion-observation-design.md)。
[力の向き・公開観測・残る測定モデル](../20260910-mpcc-empirical-plant/receiver-input-enclosure/force-frame-observability-audit.md)。
[再構成の棄却・時刻訂正・入力切り分け](../20260910-mpcc-empirical-plant/receiver-input-enclosure/received-event-phase-audit.md)。
[物理比較と次の設計](../20260910-mpcc-empirical-plant/receiver-input-enclosure/received-pose-truth-audit.md)。
[今回の結果と次の条件](../20260910-mpcc-empirical-plant/receiver-input-enclosure/received-single-r7-audit.md)。
[診断実装と検証](../20260910-mpcc-empirical-plant/receiver-input-enclosure/received-body-observation-design.md)。
標準single-r6の決定968は
元の予測範囲から後の速度・向きが外れ、現在の壁証明を拒否した。別世界のforce-single-r1も
D1Ready950/source632/job945/index2で停止。R533は同一の履歴・世代・実指令で再現した。

R531–R537で実際の力と適用指令を解析。接地変動は短時間の予測誤差に寄与するが、
全輪接地モデルへの置換は独立した過去区間の誤差を悪化させるため棄却。
過去のholdoutには全輪非接地が最大35msあり、直前の接地状態も将来予測を一貫して
改善しない。観測値を将来の保証へ転用せず、モデル・安全条件は変更していない。
R538–R544で、入力が許容範囲内でも接地変動により速度が予測を外れることを分離。
過去の接地情報を固定して使う案も独立区間の位置・旋回誤差を悪化させたため棄却。
位置スコアには診断側の車体原点/base_link混同があり、R542で元出力を保持して訂正。
本番の座標設定は正しく、解析の共通readerを修正した。R545–R549で接地遷移と
速度・旋回依存のモデルを比較。旧校正は高速・一定加速に偏っていたため、r1の低速区間を
明示して加えた診断候補を固定した。速度誤差は改善するが旋回・旧D1の問題は残る。
Force-single-r2の固定候補検証をR550–R558で完了。Ready972/source643/job967/index2で
停止し、元の壁拒否を再現。新しい1秒先の速度誤差は0.085→0.041m/sに改善したが、
過去D1と旋回・完全な安全証明は未達。今回のsourceは位置より30ms古い速度を保持し、
初期時点から実速度が予測範囲外だった。接地変動と観測時刻差を別々に確認した。
公開値を古い共通時刻へそろえる案も、鮮度・停止境界・旋回の反例があるため未採用。
次は実際の受信履歴と有効時刻を確認し、観測の再構成と物理モデルを一貫して修正する。
追加確認は不要。本番への採用は技術的な受入れ条件を満たしてから行う。
[新規走行の結果・具体的な残作業](../20260910-mpcc-empirical-plant/receiver-input-enclosure/contact-prospective-audit.md)。
[接地モデル監査・証拠](../20260910-mpcc-empirical-plant/receiver-input-enclosure/contact-model-audit.md)。

これらの新規runはStart/周回なし。元DLL・ユーザーJSON復元済み、runtime停止。追加確認/pushなし。
過去の実公開期限問題、r75 Start/update_v_max、全intent/Mission/sibling/Store、Stop/rest/
restart、Recovery/Rejoin/Boost/async、同一最終HEADの単車/dev2各3回、dev3/dev4六周、
gate1–3、同一tar/image/evalは未完。局所合格を全体受入れとしない。

以下はr75開始前までの根拠。

直前の制御baselineは6c7c9615。旧build114全26、tests101全2592/66/source105/C5 161合格。
標準r73のD2 Ready decision529は物理証明合格後、元の25ms公開期限を超過。
別のr74診断は両車停止・未完走。D1/D2の世代は有効でもStoreが停止し、workerがsolver拒否。
r72の進み続けるD1Storeとは異なる世界であり、同じ原因とは扱わない。

R448–R453で保存世界を比較。D1source618は壁の線形問題が不成立だが現在の車体は接触ゼロ。
現在姿勢をキャッシュ用に丸めて余白を足すと2セル接触し、近似の影響を確認した。
全A/B/C/D/Gと既存Hは拒否。物理的に走行不能という証明はなく、合格した修正候補もない。
R450のD1はsolver前処理が実走と異なるため、R452で正しい条件による再現を別途取得した。
R455–R458の姿勢対応の壁近似・独立前処理は数値解に達したが、元の壁証明が接触を検出。未採用。
R460で舵角準備後の前進4候補が元の壁・他車・停止継続証明に合格。
R461では準備後を自由に最適化する9状態QPの1候補も全物理証明に合格。
R462の適応的な準備候補は、許容された微小な負進捗に対して静的コースが足りず再生停止。
R464で同じQP解・指令を維持し、手前の静的点だけを含めると全証明に合格した。
このコース収録範囲のproducerを修正し、build115全26、tests102全2596/source105が合格。
R469の実ビルド再生も同じ解・指令で全証明に合格。その後の標準dev2-r75は上記の未達。
R468では全操作を自由にした初期軌跡のみの準備候補も完全なsource証明に合格。診断限定。
実時間・実走と準備候補統合の合格ではない。

全runtime停止、元のDLL・ユーザーJSON復元済み。
Follow負荷、全intent/Mission/sibling/Store、実Stop/rest/restart、Recovery/Rejoin/Boost/async、
同一finalHEADの単車/dev2各3回、dev3/dev4六周、gate1–3、同一tar/image/evalは未完。
[現在の修正](../20260910-mpcc-empirical-plant/receiver-input-enclosure/course-support-design.md)、
[現在の監査](../20260910-mpcc-empirical-plant/receiver-input-enclosure/r73-r74-wall-source-audit.md)、
[tasklist](../20260910-mpcc-empirical-plant/tasklist.md)。

## Superseded checkpoint and historical evidence

2026-09-10JST. Continue the authorized autonomous M1–M6 task without routine
confirmations. Full completion is not claimed. Baseline is `d0fdd338` (audit repair), with production control `b0478348` after
`226f93e0`shared nine-state migration and map-support repair. The verified terminal
time candidate below is prepared for local commit and fresh runtime. No push.

Empirical simulator acceptance with explicit unverified guarantees is the user's
accepted scope; universal transport/contact guarantees are not required for this
local acceptance. Physical margins, tolerances and failed-run criteria remain.

The shared native body/tire/rest model and normal/Stop/prefix/artifact/async
consumers are implemented. Buildr9passes26packages; tests r7pass2388records,
0errors/failures/skips. Single-r2passes6laps241.24130249023438s/penalty0.
Dev2-r1fails at D1decision938/world4cdc21da9faa0149. Exact replay attributes the
new terminal rejection to the updated peer field. No production repair is yet
justified by a passing integrated dev2run.

The [current dev2 evidence](../20260910-mpcc-empirical-plant/first-dev2-failure.md)
and [terminal-time repair](../20260910-mpcc-empirical-plant/stop-terminal-time-design.md)
are active. A native0.8s free-control feasibility Stop certifies the unchanged938
scene. Production now evaluates explicit native-braking and maximum-support clock
candidates, preserving rear-peer delayed stopping, all hard proofs and per-solve
budgets. Time-specific IDs and zero-cost meaning are sealed. Buildr13passes26
packages, testsr9pass2389records, canonical938replay accepts. Fresh dev2-r2on746ec926fails D2decision991;
[current failure](../20260910-mpcc-empirical-plant/first-dev2-r2-failure.md) is active.
Peer forecast alternatives and coherent tangent seeds were not promoted.

Remaining sequence: repair current dev2root cause, finish M4intent/Stop/restart/
Recovery/Rejoin/Boost/async exercises, same-HEAD single/dev2three-trial campaign,
dev3/dev4sixlaps and gate1/2/3, then the same submitted tar/image/eval and evidence
closure/local commits. [Design](../20260910-mpcc-empirical-plant/design.md),
[tasklist](../20260910-mpcc-empirical-plant/tasklist.md).

## Earlier evidence (historical, not current HEAD acceptance)

Previous direct input audit ae2efe6a:

Generated-only Unity probes preserve original3608 CIL methods after removing4
observation calls. Direct receiver/application run `20260909-actuation-observed-dev2-r2`
records1906receives/438applications, all438selection identities/inputs match,
probeerrors0. Unique source-ns/float32 matches identifyD1/D2(581/609).
Selected packet age median30.650449/25.096941ms, maximum103.655550ms;
application interval maximum183.330866ms is observed, not a bound.
Do not interpret0.13s prediction origin as actual fixed longitudinal application.

FirstD1Emergency933atnow9.834999780has no earlier moving brake pulse.
First brake packets430/431are overwritten by432; actual brake begins9.923955841,
88.956061ms afterfirst request. Old374/386run phase remains unknown.
Nearly constant actual applied1.3296m/s² produces0.365483m/s velocity gain over
0.49s versus canonical wire integration0.651502m/s. Native isolated decoded
rolling-resistancecomponent shows two0.074m/s mismatches. This establishes a
model/input discrepancy, not a full plant fit or repaired coupled acceptance.
Exact371/932Accepted->933rejected pair replays withzero solves; peer-only
substitution preserves both outcomes. All9normal+4supportStop arms reject/Unknown.
Original930actual374+actual929clock incurrent930world also rejects; choosing374
instead of386alone is not sufficient. Schema helper failures, initial token
comparison and observation-r1missingUnityPlayer.log remain preserved.
All runtime stopped, probe mounts removed, originalDLL/userJSON restored.

Previous state epoch repair (1c4f377e) and still relevant baseline evidence:
The prior next step was audit of actual Stop/normal command application and current-world
revalidation at `output/20260909-stop-viability-dev2-r1` D1decision930. A source-to-now
Odometry producer defect is repaired: raw older state previously omitted5/15ms
motion before the fixed0.13s actuator prediction. Native four failures;34state/
longitudinal cases and104source contracts pass after explicit stamped constant-
twist observation propagation. Canonical now pose and raw safety monitoring have
independent owners; async clones preserve both. No actuator delay/gain/solver/
hard limit change.26build/60CTestgroups/2381records pass; first Pose2Dconstructor
compile failure is preserved separately.

Single6laps254.074051s/penalty0,no moving override,callback15.327ms/0overruns;
command/Odometry/clock receipt and source checks have no >50msgaps/duplicates.
Dev2rejects firstD1Emergency930at1.362512395068054m/s. All three captured current
poses uniquely reproduce from their source Odometry atnow, XY/yaw errors0.
D1/D2callback20.969/23.696ms/0overruns, commandreceipt~40Hz/no >50msgap; source
12duplicates each and Odometry/clock~300ms delivery pauses remain.
Normal374at928loses terminal proof; independentStop accepts and386joins. Actual
publication returns to374at929, but930final inspected386terminal peer rejects
-0.0013934640174382285m; independentStop-0.0011199987734020755m. Actual374and386
are separately recorded; do not mix their clocks or infer source selection caused
failure. Runtime also reports actual374independentStop peer reject. Current-world
control speed1.355983828 vs386expected1.229635254m/s needs causal input/application
phase audit. Previously observed Unity10Hz actuation phase remains unmeasured;
a published25msbrake pulse is not proof it was applied. No hold/delay/gain tuning.
All runtime stopped/protected JSON restored. Full acceptance remains open.
See [state epoch repair and latest results](../20260909-mpcc-stop-viability-epochs/results.md).

Previous observation repair4b2474da: complete source-free CertifiedPlan evidence,
18native/104source/26build/2374records pass;384sealedartifacts. Actual387/930Accepted
->931rejected pair replays with zero solves and original physical diagnostic fields
exact. Native tracker old928/929source projection also matches exactly. Legacy
solver-source absence and original parent dynamic-proof absence remain explicit.
See [peer epoch observation](../20260909-mpcc-peer-observation-epochs/results.md).

Previous publication clock36c2a01a: realStore4.403881ms rewind repaired;
33native/26build/2373records pass. Single6laps251.993591s/0penalty passes;
dev2firstD1Emergency930/actual385missing remains a rejected historical run.
Peer-only cross substitution of old928/929flips both outcomes, but subsequent
rawV2X/nativefilter replay reproduces source-time projection exactly. Array or
receipt epoch relabelling is falsified for those observations; no tracker gain
or model repair follows from filter lag. See
[publication clock](../20260909-mpcc-publication-clock-handoff/results.md).

Previous EKF repair 17bcf24b: real installed library clock failure fixed with
one calculation epoch in participant package; 26 packages build, EKF 102 records
0 failures / 30 cppcheck wrapper skips; MPCC 2372 records pass. Recorded-input
54-tick / 2268-value parity is exact. Host cppcheck preserves one unchanged
upstream Eigen false positive. Single six laps passes; dev2 937 rejects.
Controller treatment of older Odometry remains unresolved. See
[EKF results](../20260909-mpcc-ego-viability-transition/results.md).

Previous b19 paired observation: same-376 Accepted 934 / rejected 935 reproduce
without solving; peer-only substitution did not flip either outcome. At final
938, actual publication 376 is preserved but inspected derived Stop 388 lacks
solver-source provenance. Do not substitute a new solve. World 941 architecture
comparisons remain rejected/Unknown, and covariance unit-change hypothesis is
falsified (wire values are standard deviations in metres). See
[peer viability](../20260909-mpcc-peer-viability/results.md).

Previous GNSS repair and941boundary:
The then-next step was bounded current-world peer/terminal viability audit of new
GNSS-repaired dev2 D1world941/actual370. Cumulative low-speed GNSS heading defect
is locally repaired:4of5newactual-nodecasesfail before, all5pass after;
25packages build, MPCC2371records pass, GNSS30records0failures/6cppcheckwrapper
skips (direct cppcheck clean). Same-recorded136/144GNSSfixes update yaw and rotate
lever arm, correcting up to0.460/0.496m versus old node output. No threshold change.
Single6laps254.349091s/penalty0, movingoverride0, callbackmax14.674ms/0overruns.
Dev2stillrejects D1decision941/v1.299626669m/s: exact terminal and final snapshots
both contain370and rejected Request. Zero-solve steering join passes, terminal
peer-0.001206851953m and independent Stop peer-0.000951979315m reject.
D1/D2callbackoverruns0/max20.730/23.434ms; source/observation delivery remains
separate unresolved evidence. All runtime stopped/user JSON restored.
See [exact join and GNSS results](../20260909-mpcc-exact-join-causality/results.md).
Full coupled/intents/dev3/dev4/gates/submission acceptance is not complete.

Previous observer baseline466bba5e and its945diagnosis:
The then-next step was causal audit of exact dev2 D1 world945 / actual and inspected361,
now recorded with its exact rejected Request. The first-event observer repair
passes16native/104source tests,25-package build and60CTestgroups/2371records.
Fixed dev2 captures the first moving Emergency945at1.5802064876844846m/s.
Zero-solve replay reproduces steering-unreachable, terminal peer-0.003647424709m
and independent Stop peer-0.001124094799m. Original361wall/dynamic pass;
current pose/speed/steering join differs. Both Domain callbackoverruns0,
commandreceipt40Hz/no gap>50ms, but source/observation delivery pauses remain.
Coupled race rejected. Later1137is downstream of first Emergency, before teardown.
Use exact publication/request clocks and same-run serialized commands to test
reachability ownership; compare A/B/C/D and full-rest candidate architecture on
this new world before another failure-family patch. No offline candidate is
promoted. All runtime stopped and user JSON restored. See
[final authority observation](../20260909-mpcc-final-authority-observation/results.md).

Previous e5b8c455 evidence and now-repaired recording gap:
The then-next step was to preserve the final-authority observation at dev2 D1 951 and
the actual Stop 379, distinct from first terminal world 949 / normal 365.
The metadata repair in [Stop proof provenance](../20260909-mpcc-stop-proof-provenance/results.md)
passes its failing native regression, 25-package build and 60 CTest groups /
2369 local records. It binds Stop physical proof to the solved artifact's
residual tolerance; strict validation and production candidates are unchanged.
Single six laps pass in253.608948s/penalty0, callbackmax15.511ms/0overruns,
command receipt40Hz/max34.638166ms. However source command timestamps have
11duplicates/backward and two gaps>50ms; odometry/clock delivery pauses225/226ms.
Dev2 rejects at D1decision951/speed1.718736m/s after949selectedStop379.
Both Domains callbackoverruns0/max17.646/21.194ms, commandreceipt40Hz/no gap>50ms;
source timestamps and odometry delivery still have gaps. No coupled finish.
949 captures actual normal365, not379's artifact or951's exact observation.
The terminal snapshot submission suppresses a generic final-authority record,
but the downstream terminal bucket already contains949. Reproduce and repair
that observation gap before guessing current-world Stop authority transitions.
The earlier923comparison has physical Stop witnesses; four explicit candidate
missions show923needs a different accepted candidate from1629, while4264remains
Unknown. No offline formulation was promoted. Runtime is stopped and protected
user artifacts restored. Full acceptance remains open.

Preceding complete-rest baseline ae6862aa and its923/328/925failure:
The complete-rest candidate producer is implemented and locally validated:
25packagebuild,60CTestgroups/2368local records pass. Same-world1629production
join/command passes;4264still rejects. Single-r1finishes6laps252.778778s/penalty0,
moving override0,callbackmax18.373ms/0overruns,command40Hz/receiptmax39.216995ms.
Dev2-r1rejects: D2first visible moving Emergency925at1.823493m/s; no finish.
D1callbackmax31.967ms/3overruns, D2max22.277ms/0. Both command streams40Hz and
no gap>50ms. One causal observation warning per Domain is startup at0m/s
(D1spawned,D2before state observation), not a moving-race failure.
Actual async Stop publication is confirmed near rest (D1source308/decision844,
D2source309/decision827). Moving multi-tick Stop/restart acceptance stays open.
First terminal snapshot is923(fp8706044921014371466) with normal328published
source, not925world and not alternate395. At923alternate395/908joined/selected;
its final command/artifact is not inferred from that log. At925normal328has
cursor2.1s/pose mismatch1.430411m/current control speed1.883513vs expected3.224031,
only publisher-interval continuation and rejected terminal wall proof.
All runtime is stopped. Preserve before/after/rejected outcomes and investigate
source switching/current-state proof plus callback overrun before another run.
See [complete-rest results](../20260909-mpcc-complete-rest-candidates/results.md).

The preceding complete-suffix consumer remains locally validated. On its old
world1629capture, conditional one-solve188tick replay reached rest (170solved
suffix/18existing generated terminal). Preserve this as old offline evidence,
not runtime evidence for the new producer.

A separate Stop pose producer displaced world968's body37.925mm; its stale
projection/bundle coordinate is repaired. The corrected successor correctly
rejects wall. Native wrong-pose150mm and25ms-boundary regressions are preserved.
Final67retained cases/60CTestgroups/2366records and25packagebuild pass.
Single-r1finishes6laps253.323883s/penalty0, moving override0, callbackmax12.918ms
and command receiptmax34.149647ms; source gapmax35ms/no duplicates in this run.
Dev2-r1rejects D1decision4264at8.72m/s: terminal wall reject13 and independent
Stop wall reject, while actual3695is atomically preserved. Retained wall proof
only covers the publisher interval, so it cannot discharge the new terminal
condition. No finished race. Each Domain88reportedwindows,3564/3573cycles,
callbackmax20.292/20.403ms and0overruns. See [complete Stop proof results](../20260909-mpcc-complete-stop-suffix/results.md).

Previous tangent fix8d6ebf8apasses its own single6laps but dev2world968failed.
All164maximum-brake profiles and8rear-QPmethods remain rejected;3of4free-rest
methods are offline positive witnesses. Actual1100/world1629and all older
missing-artifact/solver/clock failures remain separate evidence. Earlier
source/sensor delivery pauses are not erased by the new single clean clocks.

The preceding paired longitudinal observer/clock commit6c4875ed passes one
single six-lap trial253.768982s/penalty0, moving override0, callbackmax16.564ms,
control receiptmax37.079096ms. Its dev2D1world4428terminal wall rejection remains
recorded with missing actual3834artifact and one D2callback overrun28.758ms.
A/B/C/Dcomparison on4428found no accepted bundle. No substitute artifact is
permitted. Single-r1clock-owner rejection, r2Unity startup stall and source/
clock delivery pauses remain in [longitudinal results](../20260909-mpcc-committed-longitudinal/results.md).

Observation slice03e174e5preserves previous actual Stop453/world993. Original
Stop and native current-world rejection are reproduced without re-solving.
Absolute command replacement is rejected across clean single data. The selected
paired observer retains measured-vs-commanded response and passes the old
same-world terminal proof at1.420476m/s. This closes the local producer defect,
not the new integrated failure. Prior same-world A/B/C/Dand fresh Stop arms
remain rejected without physical infeasibility proof.
See [published Stop audit](../20260909-mpcc-published-stop-audit/results.md).
Older dev2 loss 978 and missing Stop 442 remain historical failures, not replaced.
See [side-peer/Stop results](../20260909-mpcc-side-peer-stop/results.md).
The exact same full-body source now admits a native production zero-objective
Stop with unchanged hard constraints, approximately 8.0 ms offline. Old normal A
still fails. New dev2 normal source 381 has an exact rational certificate that its
affine convex branch is infeasible even with current row tolerances; this says
nothing conclusive about physical-scene feasibility. The older normal A's earlier
invalid tangent is repaired with the model adapter's existing declared-box rule.
The raw-primal initial course-window boundary is repaired
with a required separate semantic physical state:25packages/60CTestgroups/
2339records pass. One single trial finishes six laps253.944031s/penalty0,
active moving override0, callbackmax18.913ms/0overruns, receiptmax35.070181ms.
Same-run source/clock publication pauses remain separately documented.
See[initial-state results](../20260909-mpcc-semantic-physical-initial/results.md)
for the earlier zero-objective failure and its baseline evidence.
The first complete-bodydev2run is
rejected with moving Emergency/Recovery and one lap per vehicle. See
[peer results](../20260908-peer-envelope-audit/dev2-results.md). Do not advance
to a submission pass claim or repeat this unchanged trial.

## Completed work

- Astra harness: root/package AGENTS,7Skills,project Astra/high default;
  difficult MPCC audits use an explicit xhigh override. Frontmatter/TOML/links
  and effective configuration checked. No reasoning quality/cost benchmark claimed.
- Semantic target time, Stop feasibility/reference and certified publication,
  stateless successor provenance: local causal repairs and regressions recorded
  in their steering/central experiment registry. Integrated coverage remains open.
- Local AWSIM calibration: UNKNOWN GNSS covariance explicitly configured,
  nominal ego body enclosed by measured footprint, omitted static wall cells
  added and native/editor preservation aligned.25packages build and regression
  evidence retained. None of these alone establishes race acceptance.
- New bounded observation preserves actual published solver source, artifact
  and publication clock before Emergency clears the ledger. No new authority.
  Latest25-package build passes;60CTest groups/2270package-local test records,
  0errors/failures/skips. All7normal intents preserve original solver source.
  Final bounded diagnostic captures actual published source6782at failure7405.
- Coordinate repair now integrates physical Cartesian motion and projects into
  the exact immutable reference frame, retaining the7-state solver order and
  unchanged actuator model. Native invariants24/24pass;25packages build and
  all60CTestgroups pass. Old model artifact identities are rejected. Fixed-control
  source6782error against independent Cartesian ODE is0.231333mmmaximum,
  previously651.9642mm. New solves of6782/7405stillreject wall rows; no
  physical infeasibility claim. Initial6laps252.253662s/penalty0, no active-race
  moving override or Recovery. Callbackmax20.390ms/0overruns, control receipt
  max39.862633ms/0gaps>50ms. Source duplicates coincide with simulator clock
  and sensor publication pause. Later geometry-lineage refusal checks and
  normal/continuation/Stop binding regressions pass; final build25packages,
  all60CTestgroups/2335colconrecords,0errors/failures/skips. The integrated
  campaign with those final checks and peer geometry remains open.

## Current evidence

Corrected-wall trial20260908-wall-map-single-r1:6laps253.198868s/penalty0,
but moving Emergency10699/10700at7.53/7.55m/s. First rejection is delay-prefix
collision at0.105s, before continuation or Stop-terminal proof. The old exact
published10073artifact was not recorded. Fixed-input comparisons do not find
a feasible alternative or prove physical impossibility. Contacted grid cells
contain wall faces at body height, not just overhead geometry.

Observation trial20260908-execution-evidence-single-r1:6laps250.508286s/
penalty0, no active-race moving override, callback20.701ms/0overruns,
control receive max35.215855ms/0gaps>50ms. Failure did not recur, so published
artifact capture was not triggered. The protocol permits at most2additional
same-settings diagnostic trials, solely to obtain the missing failure data.
The earlier rejected result is retained. No pass can replace its explanation.

Observation trial2finishes6laps252.718750s/penalty0 but loses normal authority
at9005/9006at7.65m/s. Here delay prefix and20-stage continuation are clear,
while both terminal Stop references fail. Original source is absent because
the source producer only retained ShiftOut/Pass inputs. A native probe confirms
5of7normal intents missing. The observation producer now retains all7; its
new build/tests and final planned diagnostic now pass observation acceptance.
This does not fix the earlier delay-prefix or terminal failure by itself.

Observation trial3finishes6laps259.300171s/penalty0, but Emergency7405/7406
at7.53m/s. Source6782and actual artifact/publication clock are saved; first
rejection is delay-prefix collision at0.120s, before continuation or terminal
proof. Callback21.207ms/0overruns;receive max35.177231ms/0gaps>50ms.
The three-trial observation campaign is closed, with all outcomes retained.

The actual native transition fails physical coordinate invariance: stationary
body, curvature0.2/m,lag-0.3m,virtualspeed4m/s moves39.9899mm in0.1s.
14of24native analytical cases fail. With captured6782controls, native world
position differs from independent Cartesian motion by47.0783mm at45.123ms.
Same source grid/native sweep: native path clears hard reserve; Cartesian
path rejects at the first sampled4.512msboundary while body margin stays
clear. Correct arc-length equations alone still leave16.0719mm at45.123ms,
because stored curvature and piecewise reference-frame derivatives disagree.
Complete actual-frame equations agree with Cartesian integration offline.
This was a proved model/certificate defect; the current local repair is above.
It is not yet a complete attribution of the live delay-prefix failure. Full-horizon rollout
is conditional: Emergency supersedes actual publication aroundcursor0.05s.

## Order to finish

1. Complete current-world full-rest candidate generation and async promotion
   using the sealed witnesses and new world4264, with all-peer physical proofs.
   Observation bundle and longitudinal observer/clock slices are validated. Coordinate-consistent integration, semantic initial state,
   whole-body peer enclosure and Cartesian peer-plane repairs are implemented
   and committed. Do not repeat their completed native/build work without a
   changed input or newly discovered concern. Preserve the unresolved Stop
   successor course-window boundary and the unmeasured longitudinal phase.
2. For any new physical-versus-solve disagreement, seal the executed artifact,
   current world, serialized inputs and independent observations, then perform
   the bounded architecture comparison before another patch in that family.
   Do not replace wall/peer evidence with margin, delay or solver tuning.
3. Fresh fixed campaign: single six laps, Follow, both Pass/Return directions,
   certified multi-tick Stop/restart and Recovery/Rejoin. Then async identity,
   dev3/dev4 and gate1/2/3. Seal all inputs/binaries and all repeated results;
   check final authority, wall/peer proof, callback/publish timing and every vehicle.
4. Submission: create tar.gz, bake eval image from that artifact, run make eval;
   verify top-level archive/interface/schema and same-run outputs. Update
   registry and P0-P4 tasklist with actual evidence before declaring completion.

Detailed current work: ../20260908-mpcc-delay-prefix-audit/ and
../20260909-mpcc-coordinate-consistency/ and ../20260908-peer-envelope-audit/.
Full acceptance remains P2partial/P3/P4open.


2026-09-10更新: dev2-r2はD2decision991で不合格。生成時の相手車加速度を
捨てるCV予測が接近を見逃す比較を保存し、有限1秒加速度予測を物理QP/proof/
Stop/retainedへ共通化。buildr14:26packages、testsr10:2392records合格。
修正を固定した新しいdev2から統合受入れを継続する。M4–M6は未完。

2026-09-10更新: dev2-r3はD2decision957で不合格。採用したfeedback Stopの認証済み
停止列を破棄して元軌道を実行済みとする欠陥を修正。buildr15:26packages、
testsr11:2393records、本番APIによる954再構成の停止列格納・再証明が合格。
新しいdev2からM4–M6を継続する。詳細は20260910-mpcc-empirical-plant/first-dev2-r3-failure.md。

2026-09-10更新: dev2-r4は停止列の実行を確認したが、D2decision853で静止状態の
時間区間を格納できず不合格。距離ゼロの完全静止を明示する修正はbuildr16:26packages、
testsr12:2394records、同一852/853観測の本番再生で合格。次のdev2からM4–M6を継続。

2026-09-10: f1f213fcdev2-r5reaches ShiftOut/Pass and canonical Rejoin after
relative-progress watchdog abort. Ego moves46.198m; faster peer84.979m, so this
is not physical standstill. The old monitor's tactical-phase stop is classified
separately from final Recovery override. Sixlaps remain incomplete and7adjacent
callback-overrun pairs reject timing. Observation-only nested timing passes
buildr17/testsr13:26packages/2394records. Next: bounded ShiftOut timing capture,
then causal repair and M4–M6. See timing-attribution-design.md in current steering.

2026-09-10:08322c3bdev2-r6startup stalls beforeReady; preserved/inconclusive.
Dev2-r7bounded timing shows D2decision1154marker16.559ms/924points and an adjacent
1155overrun with Recovery16.816ms. Negative-side ShiftOut→Pass→Return→Idle is
observed, but no integrated acceptance. Native identical-point comparison accepts
SPHERE_LIST batching (native prototype and direct production builder comparisons retained). Producer now
replaces complete display and clears legacy IDs; no control/proof/rate changes.
Buildr19andtestsr15pass. Fresh dev2-r8 timing and remaining M4–M6 follow.

2026-09-10:3ab839fadev2-r8fails moving Emergency atD2decision2144 after
negative-side ShiftOut/Pass/Return/Idle. Marker cost0.092–0.329ms; raw adjacent
overruns0/0, but only a short failed run, source duplicates10/10unattributed.
Paired exact2143/2144replay shows predicted control-origin speed drives rejection,
not peer update alone. The newly published acceleration can cancel the assumed
braking prefix used to certify its initial state. Native current provenance
replays exactly; prospective−3prefix yields a feasibleStop. Existing full5sStop
SQP also certifies this world, so it is not physically impossible. Publication
input binding is reopened within M1–M3; see publication-prefix-design.md.
Next: complete candidate/packet-bound native proof, implement shared prospective
prefix binding across normal/Stop/artifact/async, regress/build/run and M4–M6.

2026-09-10: prospective publication candidate implemented under existing autonomous
M1–M6 authorization. Shared native prefix is bound to each actual float packet,
including Stop materialization and final common candidate conversion; Follow ego
origin is updated against the unchanged canonical peer. Buildr21:26packagespass;
testsr17:2401records,0errors/failures/skips. Earlier testsr16 source-layout failure
is preserved. Native r8 replay rejects accelerating command under braking prefix;
matching Stop passes materialization/join/packet and snapshot roundtrip at+0.909311m.
See publication-prefix-design.md and publication-prefix-evidence.json. Fresh
committed dev2-r9 next; no integrated or M4–M6 completion claimed yet.

2026-09-10:99fccb87dev2-r9rejected at initial rest on both cars. Causal
native comparison exposes QPx0equality copied into initial physical corridor;
new prospective prefix moves1–2mm and is rejected before wallproof. Producer
now uses sealed initial physical map support while keeping QPx0/futurebounds.
Buildr24:26packagespass;testsr18:2402records/62groups/0errors/failures/skips.
Nativebeforefails; originalTrack294sourcebuilders solve/prove for both cars.
CurrentD1source840solves/proves, currentD2source819solverrejects at4000iterations,
kept separately. Details/evidence in bootstrap-bound-design.md and JSON.
Freshcommitted dev2-r10next; M4–M6and full integrated acceptance remain open.

2026-09-10: edad4e32dev2-r10 rejected on D1moving Emergency1670. Bothcars
launch; D2negativeShiftOut reachesPass. Ordinary1667/1668 exactnativepair
reproduces terminaldynamic boundary; reboundbraking1668passes1.212136504m.
Actual1669/1670pair missing because stationaryCruise833consumed finalbucket.
D1callback1668=164.662143ms; packetstamp28.284999367 arrives between public
clock28.434999364/28.439999364. Backdatedhistory is a causal hypothesis;
actualphysicsapplicationepoch and heavycallbackproducer need further evidence.
Observation adds separate firstmovingfailure bucket and eachoverrun's joincosts.
Buildr25passes26packages; testsr19passes2402entries/62groups/0errors/fails/skips.
Freshdev2-r11 next; sameM4–M6 remainopen. See moving-failure-observation-design.md.

2026-09-10: ae2137c6dev2-r11 rejected D1movingEmergency2588. Firstmoving
recorder now preserves exact2587/2588/source2047. D2negativeShiftOut→Pass→Return
→Idle observed. Root: normalStop reference ends~0.102m beforecompleteRest;
InvalidLateralPolicy then masksascenterlinewallcollision. Explicitconstantlateral
tail boundedby sealedStopmap fixes exact2588;2587unchanged; fullrest/wall/peer/
productionwirepacket checks pass. Solved-onlydiagnostic and strictsamplerremain.
Buildr26passes buttestsr20oneexistingdiagnosticfailure reportedtwice; corrected
buildr27passes26packages,testsr21passes2404entries/62groups/0errors/fails/skips.
Oldr10D1decision1668peerrejection remains, includingproductionadapter rejection;
itsbackdatedpublication/largecallbackcause remainsopen. Freshcommitteddev2-r12
next; M4–M6and fullintegratedacceptance stillopen. See terminal-reference-design.md.

2026-09-10: 8bfdaa52 dev2-r12 fails D2 moving Emergency933 during certified
Stop. Exact932/933source424 native peer clearance +0.042140739→-0.000833389m.
Same-peer-generation, latest velocity still rises after firstbrake; application
phase unknown in that run. Existing probes reused read-only in separate
application-dev2-r1:627 exact applied-input joins, firstfailureD2 1294 is intent
mismatch after ShiftOut→FollowPrepare→Idle. All evidence retained separately.
Observed prescribed application improves148D2brake-crossing speedMAE0.068265→
0.024975m/s; it is unavailable as production input and no r12 phase is inferred.
Fix known backdating producer: serialize history uses fresh ROSclock immediately
after publish with causal decision lower bound; packetstamp/prospective proof
remain nominal. Before2regressionsfail; buildr28passes26packages/4min48s and
testsr22passes2406records/62groups/0errors/failures/skips/28.78s. Receiver timing
uncertainty remains open. Same1294 actualpublished778 completeStop rejectsCruise
identity but accepts upstreamShiftOut/+10.061819599m/345samples, materialization/
rejoin/productionadapter; observation only, explicitStop handoff repair next.
AandrightB/C/D/G also certify1294; leftarms and2fullrestclocksreject. No
integrated acceptance or M4–M6 closure. See publication-epoch-design.md/evidence.

2026-09-10: aec8d32e publication epoch producer committed. Separate explicit
publishedStop handoff now keeps actual source identity after tactical intent
change. New regression fails old producer; buildr29 passes26packages/4min14s.
Testsr23 newnativepasses but2source-structure references fail; correctedtestsr24
passes2407records/62groups/0errors/failures/skips/29.07s. All forbidden publish/
store checks and generic normal/Stop intent equality remain. Native toolr30
actual1294published778: requestedCruise, proofShiftOut, completeStop345samples,
clearance10.061819599m, materialization/join/adapter and serialized packetmatch.
R12negative933source424 stays peerreject(-0.005110710497m)/no bundle;932unchanged.
Fresh committeddev2-r13next, covering both publicationepoch and Stop handoff
changes. Receiver application model and all M4–M6 remain open; no failed run
is relabelledpass. See intent-stop-design.md and intent-stop-evidence.json.

2026-09-10: e710056d dev2-r13 failed at D2decision1401 duringShiftOut, source828Stopwallreject; laterPass is afterfailure. Exactprior1400request unavailable,1399is historical. Adjacentcallbacks1400/1401=38.366/32.753ms. Stopmodelclosure atzeroerror exposes continuousfuturesteering versus a new heldpublication mismatch; continuouscounterfactual alone restoresall8samples but is never publishable. BoundedA/B/C/D comparison is inconclusive (15rejectedcandidates), fullA-Gtimeout retained. Stopproducer now uses float32heldangles, sealedincrementinputschema, unchangednativebody/rest/wall/peers/packetchecks. Buildr30/r31pass26packages; testsr25two oldassumption/classificationfailures corrected without removing negative checks; testsr26all2409records/62groupspass28.69s. Correctedhistorical1399Stop330samples/bundle/join and all8modelclosurespass. Exact1401stillwallreject; exact933peerreject-0.000891864256m/no bundle. No integrated acceptance claimed. Freshcommitteddev2-r14next, then remainingM4–M6. See stop-packet-schedule-design.md and stop-packet-schedule-evidence.json.

2026-09-10: 21b025f3dev2-r14 failed D2decision1112 unexpected Recovery while Stop604 remains certified. Actualstateless589publication retires frozenMission at1100; old earlyreplan wrongly reopens on absentgeometry. One shared applicability predicate now recognises frozen or actualpublished owner; ordering stays diagnostic for ownedencounters, legacy uncommitted Switch/Abort and physical supervisors remain. Nativebeforefails; buildr32=26packages/4m15s, testsr27=2410records/62groups/0errors/failures/skips/29.62s. Earlier589snapshot boundedA/B/C/D15candidates allsolverreject, Unknown; exact1112world absent, livepositive589publication is separateevidence. R14adjacentoverrunpairs0/0, stillfailed. Fixedcommitdev2-r15 next; all M4–M6 remainopen. See side-replan-ownership-design.md/evidence.json.

2026-09-10: cbcda112dev2-r15 fails beforeShiftOut at D2decision956/wp31/1.47m/s, Stop449peerreject. Current-controller application-r2 also fails1001/wp31/1.15m/s, Stop576; actualreceivertrace binds firstnegative503source11.729999737, afterpositive502application11.764874323, to−3application507at11.866269907. Native exact955/1000historicalinputs pass but sourceassociations are invalid for449/576. DirectfailureStop clearances−0.000425110/−0.000167764m; they remainrejected. Five conditionaldeliveryhypotheses show immediateStopaccepted inputs can fail with50mspositivehold; earliest994stillpasses150ms but995doesnot. No delay or margin adopted, no productionchange, no integratedacceptance. Explicit receiverinput scheduling is the next shared-model/proof slice; see receiver-schedule-design.md and receiver-schedule-evidence.json. All M4–M6 stayopen.
