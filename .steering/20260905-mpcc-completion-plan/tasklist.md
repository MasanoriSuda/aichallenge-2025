# Tasklist

最新の進捗・完遂順序は[current-status.md](current-status.md)。

2026-09-09観測bundle追記: 失敗worldと実行元を原子的に保存し、25package build・
60CTestgroup/2362記録、native14/source104が合格。dev2 D2 decision1629で実行元1100を
取得し、停止末端の後方D1との動的余裕不足を再求解なしで再現（1.555秒、-0.006771m）。
壁検査は未到達。同worldのStop4方式は全て動的拒否、通常B/C/D/Gはfront target不在で
候補生成できず、Aはstage19線形化拒否。次はrear peerを扱う独立停止候補の有界比較。
後続D1制動を含む未来実測でも完全停止経路はpeer円と重なるため、予測変更だけで直る
根拠はない。多車両の完走・P2残項目・P3/P4は未完。
[観測bundleの結果](../20260909-mpcc-failure-observation-bundle/results.md)を参照。

2026-09-09縦応答予測追記: 観測slice03e174e5の後、単純な公開指令への置換を
単車全体の誤差悪化で棄却。同じodometry時刻の加速度と指令を対でfilterするproducerを
実装した。元Stop453/world993のnative再証明は通過。初回単車1938の5ms時計逆転を
producer側で修正し、25package build・60CTestgroup/2359記録が合格。
単車r2はAWSIM時計停止で起動不成立。r3は6周253.768982秒・penalty0、
移動中override0、callback最大16.564ms/超過0、指令受信最大37.079096ms。
変更後dev2はD1 decision4428、8.42m/sで停止末端の壁証明が失敗した。
実行元3834が起動時296の記録と重複扱いになり欠落。新worldのarchitecture比較と
失敗単位のsource/artifact同時保存を次に行う。D2 callback超過1回も未解決。
[縦応答予測の検証記録](../20260909-mpcc-committed-longitudinal/validation-notes.md)を正とし、
P2のdev2受入れとP3/P4は引き続き未完とする。以下は過去の時点の記録。

2026-09-09最新追記: f3b0338cで横並び制約・Stop候補修正をコミット。
観測のみの修正後、dev2 D2 decision993で実行済みStop453を捕捉した。
元のStopは再求解なしで証明を通り、失敗時の再証明も再現した。
公開済みブレーキが遅延予測へ渡されず、予測速度が過大となるproducerを特定。
公開済み指令のみの比較で停止証明が通るが、本番修正・統合受入れは未実施。
次は[当該監査](../20260909-mpcc-published-stop-audit/results.md)の時刻回帰を固定し、
予測producerを修正・検証する。全体P2/P3/P4は未完のまま維持する。

2026-09-09追記: b5308839に他車全体の包含とsemantic物理初期状態をローカルコミット。
単車6周253.944031秒・penalty0を確認。現在は
`../20260909-mpcc-side-peer-stop/`で横並び予測、共通Cartesian他車制約、
零目的Stopの数値表現と通常全7intentへの候補供給を修正・検証中。
同一保存場面の本番Stopは物理証明を通過。25package・60CTestgroup/2346記録が合格。
変更後dev2はD2 decision978で採用済みStopの壁再証明が失敗して不合格。
Stopの実artifactが観測条件で欠落しており、次は記録漏れと当該失敗を監査する。
同runの通常QP381は厳密な有理数dual証明で凸近似の制約矛盾を確認。
物理場面の実行不能は証明されておらず、通常候補と統合受入れは未完。

2026-09-09追記: 座標モデル修正を5cfc50bcへローカルコミット。
完全他車包含の修正は25package build・60CTestgroup/2338記録で合格。
初回dev2は移動中Emergency/Recoveryが出て不合格。現在は
`../20260908-peer-envelope-audit/dev2-results.md`の同一入力監査を進める。
以下の過去runの記述は履歴として保持する。

2026-09-07に自律実行を開始。時間軸修正は`../20260907-mpcc-semantic-target-time/`、Stop問題の監査は
`../20260907-mpcc-pass-terminal-audit/`、現在の公開継続修正は
`../20260907-mpcc-certified-stop-publication/`。
現在の後続候補生成修正は`../20260908-mpcc-stateless-successor/`。
単車6周は完走したが最終周に壁penalty1回で安全受入れ失敗。停止監査は
`../20260908-mpcc-single-stop-horizon/`、Pass進捗監査は
`../20260908-mpcc-pass-progress-audit/`。停止参照の現在の修正は
`../20260908-mpcc-stop-reference-feasibility/`。
停止参照修正後の単車3回は2成功・1失敗。3回目は最終周に急減速し5周timeout、壁penalty1回。
現在の監査は`../20260908-mpcc-single-physical-stop-audit/`。多車両への移行は未実施。
実壁のmap未被覆・車体前端の寸法不足と、GNSS UNKNOWN共分散10→100m²を確認。
GNSSの明示校正は`../20260908-gnss-unknown-covariance/`で22test・全25package build・
単車6周診断（248.778秒、penalty0）を通過。移動中の位置差p95は0.353mから0.068mへ低下。
車体寸法の校正は`../20260908-awsim-footprint-enclosure/`で全25package buildと
MPCC2323testを通過。新しいbinary/configの単車診断は6周249.613秒、penalty0。
壁mapの追加校正は`../20260908-physical-wall-map-calibration/`で回帰・buildを通過。
単車診断は6周253.199秒、penalty0だが移動中Emergency10699/10700で統合受入れ失敗。
現在は`../20260908-mpcc-delay-prefix-audit/`で、公開済み指令のdelay prefix壁余裕侵入を監査中。
停止末端の証明に入る前の失敗であり、Stop参照を再調整する根拠はない。
V2XはGNSS点を配信し、4台共通の車体全体には同点から約1.876mの公称包含半径が必要。
相手形状の監査は`../20260908-peer-envelope-audit/`。局所校正だけで統合受入れとはしない。
全体受入れ・提出物評価が終わるまで完遂扱いにしない。

## 今回の計画作成

- [x] git status、branch、HEAD、対象AGENTSと監査skillを確認。
- [x] 最新steering、元の統合tasklist、正本仕様、interface、launch/config/Makefileを確認。
- [x] 完了済みの構造統合・局所修正と、未完の統合品質を分離。
- [x] decision 1186→1187のログ境界を確認。
- [x] S1/S2のsource・fingerprint・wall binary同一性とSHA-256を確認。
- [x] 保存tubeの移動量と0.25/0.025秒の関係を再計算し、sourceの時間分岐を確認。
- [x] 最新リンクのrun混在、registry未登録、二重build成果物を確認。
- [x] requirements、evidence、design、後続tasklistを作成。
- [x] 文書リンク13件、根拠パス、snapshot hash、Markdown、git diffを最終確認。

## P0 証拠と実行基準

- [x] S1/S2の保全manifest、commit/config/map/image/seedの基準manifestを作成。
- [x] 最新監査と未登録snapshot参照を中央registryへ反映。
- [x] 正規workspaceでbuildし、使用installとバイナリを固定。
- [x] S1のarchitecture comparisonとS2のwarm-start nonlinear oracleを再現。

## P1 時間軸監査

- [x] observation/control origin/semantic time/tube timeの比較表を作成。
- [x] sampling時間のみを変えた比較armと、新candidate fingerprintを作成。
- [x] 加速度・projection・horizon差を一変数比較に分離。
- [x] QP rowからwall/dynamic/terminalまでの最初の相違を分類。
- [x] 不変条件・producer・削除経路・修正前test・合否・rollbackを確定。

## P2 原因修正（P1で確定した範囲）

- [x] 本番組立の問題を再現する失敗test/replayを先に追加。
- [x] producerを修正し、誤った時間列の重複生成経路を削除。
- [ ] 全intent、可変dt、delay、seam、停止target、古いschemaの回帰を確認。
- [x] focused/package tests、build、固定replayを通す。
- [x] 新authority/flag/fallback追加0、既存の物理guard維持を差分確認。
- [ ] 限定dev2で当該failure familyの修正を受け入れる。

## P3 統合受入れ

- [ ] campaignのcommit/config、全シナリオ、反復回数、計測方法を固定。
- [ ] 単車六周のTrack/Cruiseと循環seamを受け入れる。
- [ ] dev2 Follow、左右の完全追い越し、六周を受け入れる。
- [ ] Hold/Stop/再発進、Recovery/Rejoinの独立試験を受け入れる。
- [ ] async遅延・完了順・target/intent更新のidentity検証を受け入れる。
- [ ] dev3六周、非target割込み、全車結果を受け入れる。
- [ ] dev4の走行と負荷を受け入れる。
- [ ] ローカルgate1/2/3を受け入れる。
- [ ] 全反復の安全・authority・callback/publish KPIを報告する。
- [ ] 未観測intent、説明不能な停止、未分類failure familyが残っていないことを確認。

## P4 提出と閉鎖

- [ ] 提出tar.gzとeval imageの同一性・契約を確認。
- [ ] 提出物から`make eval`を実行し、同一runの成果物で合否を確認。
- [ ] 正本仕様、registry、元の統合tasklistの未完項目を証拠付きで更新。
- [ ] 実施コマンド・全run・test結果・削除経路・残る暫定事項を報告。

P0〜P4が合格するまで、MPCC全体の完遂としてチェックしない。
