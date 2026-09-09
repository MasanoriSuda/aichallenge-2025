# 現在の基準からMPCCを完遂する計画

2026-09-09。基準HEADは`1c4f377e`。全体受入れは未完。
初期計画の「構造移行のやり直しは不要」という判断は、現在の物理入力モデルの
不整合を根拠に再検討する。既に閉じた時刻・座標・証明provenanceの修正は保持する。
作業の再承認を待つ計画ではない。既存の自律実装・検証・ローカルコミットの承認を使う。

## 次の実装単位: 指令入力と実加速度の意味を統一

対象不変条件は、同じwire入力がprefix予測・七状態QP・physical rollout・Stop生成・
最終publishで同じ物理モデルを表すこと。最初に現行モデルとの比較を行う。

1. 今回の実受信/適用trace、native抵抗componentの2失敗、371の932→933境界を固定。
   元の374/386/930とは別の世界として扱う。
2. **モデルの比較**は候補方式A/B/C/Dと別軸にする。
   現行のwire=net仮定、因果的な応答推定を全horizonへ統一するモデル、ローカル
   plantの明示的な抵抗/飽和モデルを、同じ入力・初期状態・hard constraintsで比較する。
   観測窓の後半やEmergency後の実績を過去decisionの入力へ入れない。
3. 一つのbiasや遅延を今回だけに合わせない。まず既存のresponse推定が持つ情報と
   provenanceを引き継ぐ案を検証し、単車・dev2の加速/減速/低速で誤差と限界を示す。
   0.37m/s²はローカル設定のcomponent根拠であり、全plantの同定値や2026公式値ではない。
4. 採用案はproblem/source/artifact/current-world proofへモデル値と意味を封印する。
   async途中で可変global値を参照させない。新しいschema/identityで旧証明の実行再利用を
   拒否し、古いsnapshotは診断用として明示して読めるようにする。
5. 七状態の接線・非線形積分・serialized入力・complete Stop/retained/source-free Stopを
   一つの契約へ揃え、旧wire=netの重複経路を削除する。入力・速度の飽和を後段のclampで
   隠さず、問題と証明の同じ境界で扱う。操舵calibration、gain、margin、solverは混ぜない。
6. 直進の正負加速度、ゼロ応答、停止端、異常/古い推定、可変dt、async受渡し、
   final serialized commandの対応をnativeで確認。旧失敗→修正後合格を残す。
   モデル誤差が縮まっても新規候補の完全な物理証明が成立しなければ昇格しない。

## 適用時刻の契約を閉じる

固定0.13秒後に全packetが適用されるという解釈は実測と一致しない。
操舵の応答予測時間と縦入力の最新値選択を分け、公開済みであるだけのpacketを
「実適用が確定した入力列」として扱わない。新しい速度モデルだけで解決扱いにしない。

- 受信順・選択・適用を分けた因果的なreplayで、上書きと可変phaseを再現する。
- 実時刻を測れない本番入力から使える情報と、診断だけの情報を分ける。
- 不確かな適用区間の扱いを明示し、単一phaseへの同定やhold/rate調整で済ませない。
- 移動中の認証済みStopを複数tick継続し、停止・再発進までを実車両応答で確認する。
  一度の25ms公開やoffline Stop合格を完了条件にしない。

## 受入れ・提出までの順番

| 順番 | 実施 | 完了条件 |
|---|---|---|
|1|上記native・記録replay・同一世界の方式比較|producer失敗の解消、全拒否/Unknownの保存、新証明のidentity整合|
|2|`make autoware-build`、対象packageの`colcon test`/`colcon test-result --verbose`|有効な回帰testを含めbuild/test合格。未実施を合格にしない|
|3|単車6周、限定dev2|penalty/説明不能なmoving overrideなし。source/receipt時刻・callback/publishも確認|
|4|Follow、左右Pass/Return、Hold/Stop/再発進、Recovery/Rejoin、Boost、async遷移|全経路の認証済み指令と実応答を観測。未観測を残さない|
|5|固定campaignで単車/dev2各3回、dev3六周、dev4走行、`make gate1/2/3`|全runを報告し、同じHEAD/configで安全・時間・全車結果を受け入れる|
|6|`./create_submit_file.bash`→`./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz`→`make eval`|同じtar.gz由来のimage、interface/評価JSON契約、提出物による合格|
|7|正本仕様・registry・tasklist・必要なlocal commit|全未完項目を証拠で閉じ、暫定2025仕様/TBDを明示。pushはしない|

修正が入ったら影響する検証を新HEADへ揃える。現在の単車254.074秒/6周/penalty0、
26package build・2381testは`1c4f377e`の既存証拠で、新モデルや観測用DLLの合格とはしない。
各sliceのrollbackはその開始HEADへ戻すrevert単位として記録し、ユーザー成果物は触らない。
