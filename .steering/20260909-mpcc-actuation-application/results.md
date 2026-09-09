# 指令の受信・適用と縦運動モデルの監査

基準は `1c4f377e`。参加者の制御コード・パラメータは変更していない。
MPCC全体の受入れは未完。生成した観測用DLLは診断終了時にアンマウント済みで、
元のAWSIM DLL、保護対象の結果JSONは元のSHA-256を維持する。

## 元の930世界: 公開プランの取り違えだけでは解消しない

`output/20260909-stop-viability-dev2-r1` D1 decision930、world fingerprint
157141478484599072。実公開normal374と最後の検査対象Stop386を分けて扱った。
実公開374のsource/artifact/929公開clockを、元の930の物理Requestへ入れた
**診断用の組合せ**は、再求解0回で次の棄却となった。元の観測を上書きしていない。

- steering-unreachable。期待操舵0.14853458163515115rad、上限0.14731454374021924rad。
- terminal peer余裕−0.0025014656576010097m。
- independent Stopもdynamic-path-blocked、余裕−0.0011199987734020755m。

既存replayの入力schema不一致(r1)とsource-free386のlegacy solver状態を
診断コピーへ残した不整合(r2)は失敗として保存。正しい記録済みartifact/sourceを
使ったr3が上記結果。374の本番内部評価を完全に記録したという意味ではない。

元の同一runのraw速度では9.87〜9.905秒で加速が弱まり、9.905〜9.94秒の
平均加速度は約−3.362m/s²だった。25msのStop公開だけから「未適用」とは言えない。
IMUのsensor frame/重力/物理振動を未解決のまま車体縦加速度として代用しない。
Emergency後の未来速度はEmergencyの指令に依存するため、通常継続の予測精度には
そのまま使えない。元runの正確な受信・適用時刻は、後述の別runから転用しない。

## 直接計測の手順と限界

元の`Assembly-CSharp.dll`を読むMono.Cecilツールから、output内へ観測用コピーと
`ObservationProbe.dll`を生成した。元のDLLは変更せず、Composeの読み取り専用
マウントで診断runだけに適用した。

- Ackermann受信後に番号とsource ns、float32加速度を記録。
- 元のactuation lock内で最新入力が選択された時点に、その受信番号を保存。
- 元の`Vehicle.AccelerationInput`代入後に、選択番号、実入力、Unity時刻、ROS時刻を記録。
- 内部emergency topicの即時代入は通常のAckermannブレーキと区別する。

観測呼出し4か所を取り除いたCIL比較で、3608methodの命令・分岐先・例外領域に
差分0。観測用ログの時間負荷まで同一という意味ではない。
最初の検証はmetadata tokenの採番とNaN比較を正規化しておらず170差分となった。
型参照を解決しNaNを同値表現にした最終比較が差分0。両結果を保存。
一時コンテナのdpkg `_chrony`エラーも保存し、ツールは使い捨てコンテナ内への
パッケージ展開に切り替えた。ホストのユーザー定義やパッケージは変更していない。

観測run r1はD1decision938で停止。Unityの標準Player.logを成果物へ保存できず、
受信・適用についてはinconclusive。r2で`-logFile`と実マウント情報を固定した。
`output/20260909-actuation-observed-dev2-r2`はD1decision933で停止し、完走していない。
停止の検出・teardown後までの記録を含むため、その後の事象を最初の原因に混ぜない。

## r2で直接確認したこと

ROS bagのsource nsとfloat32指令の一意一致はD1 581件、D2 609件。
これによりreceiver IDをDomainへ対応付けた。共通キーの曖昧な1件ずつは
対応付けの根拠から除外。bag開始前の受信はbagとの完全照合を主張しない。
全438適用記録は選択した受信番号/source ns/加速度と一致、probe error0。

| | D1 | D2 |
|---|---:|---:|
| 受信 |946|960|
| 適用 |217|221|
| 最後の適用までに上書きされた受信 |729|739|
| 選択packetのsource→適用中央値 |30.650449ms|25.096941ms|
| 同最小〜最大 |5.095011〜103.655550ms|0〜103.655550ms|
| 適用周期中央値 |100.002436ms|100.002371ms|
| 適用周期最大 |183.330866ms|183.330866ms|

これは固定0.13秒後に全公開指令を順番に適用する系ではない。
Unityはrender/到着時刻に依存する最新値選択を行う。計測した最大値は保証上限ではない。

D1最初の移動中ブレーキは、decision933のEmergency、source9.834999780秒。
受信430/431は次の同じ−3m/s²指令で上書きされ、受信432
(source9.889999778秒)がROS9.923955841秒に適用された。
最初のブレーキ公開→実ブレーキ開始は88.956061ms、選択packet自身のageは33.956063ms。
全てAckermann経路で、内部emergency flagはfalseだった。
**このrunの933より前に短いStop指令の取りこぼしはない。**
したがって、それだけを一連の停止の原因とする仮説は棄却する。

## 縦運動モデルに残る不整合

D1のsource9.309999791〜9.799999780秒、raw速度15点の区間では、実適用加速度は
1.32959557〜1.32959855m/s²とほぼ一定。速度増加は0.36548280715942383m/sで、
wire加速度を直接積分した0.6515018146744478m/sと一致しない。
線形fitの傾き0.7583164245243338m/s²、最大fit誤差0.01680008644403097m/s。
この窓から全速度域に共通の定数抵抗を同定したという意味ではない。

ローカルUnityの実装は、入力の飽和・ギヤ処理後にrolling resistanceを加えて
wheel forceへ渡す。設定のrollingResistanceは0.37m/s²で、drag・タイヤ力等も別にある。
一方、canonical seven-stateはwire加速度をそのまま速度微分へ入れる。
既存のpaired response observerによる補正は現在→0.13秒のprefixに限られ、
その後のQP/physical rolloutの入力意味には引き継がれていない。

native `evaluate_temporal_frenet_transition`を使う独立の直進component fixtureで、
v=2m/s、wire=+1/−1m/s²、dt=0.2秒、既知rolling resistanceだけを適用した
physical値と比較すると、**両caseで速度が0.074m/sずれる**。
これは現行モデルの不一致を再現した失敗診断であり、修正後testの合格ではない。
実runの全物理力を再現したfixtureでもない。暫定定数の数値調整へ直結させない。

実際の371について、元の932/933 Requestを再求解0回で再現した。
932はAccepted、terminal余裕+0.0019944623102432502m、independent Stop
+0.0028919971331196059m。15ms後の933はterminal-contingency-unavailable、
−0.0013267385265398612m、independent Stop−0.0012136109652070015m。
相手車の観測だけを相互に差し替え、各時刻へCV投影しても両方の合否は変わらない。
これだけで車体モデルの不整合が933の唯一の原因と確定したとはしない。

同じ933世界で既存の9 normal方式と4 supportStop方式を比較し、全てsolver棄却。
物理的に不可能という証明はないためUnknown。制約やsolver設定は変更していない。
次のモデル修正は[完遂計画の更新](completion-plan.md)に従い、各層の入力意味と
snapshot/async identityを一緒に扱う。今回の直接計測だけで本番モデルを置き換えない。
