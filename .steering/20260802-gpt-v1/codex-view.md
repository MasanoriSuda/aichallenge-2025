# 上位者ログおよびChatGPT Pro分析に対するCodex見解

作成日: 2026-08-02  
対象HEAD: `a84e840 20260802-committed-pass-behavior-ownership`

## 結論

ChatGPT Proの方向性には概ね同意する。ただし、「現行には候補探索がなく、固定4 mの経路を1本だけ判定している」という評価は、現在のHEADには当てはまらない。

上位者ログから学ぶべき本質は、攻撃的な設定値そのものではなく、相手との相対状態を約40 Hzで監視し、横経路と縦速度を一体として追い越し完了まで制御する仕組みである。

現行品で本当に不足しているのは、候補探索そのものではなく、各横経路候補に異なるclosing speedを組み合わせて評価し、採用した横経路と速度プロファイルをShiftOutからbody-clearまで固定して実行する部分である。

## 分析対象と制約

分析対象は以下の2ファイルである。

- `autoware (10).log`
- `gpt-pointout.txt`

今回のフォルダにはrosbag、`result-summary.json`、車両別result detailsがない。そのため、順位、接触、実軌跡、追い越し成功の真値までは検証できない。

ログ中の`COLLISION-SUSPECTED`は制御側による推定であり、AWSIM上の接触真値とは限らない。以下の成功・失敗回数も、ログイベントから明確に対応付けられる範囲での集計である。

## 上位者ログから確認できたこと

ログイベントを再集計すると、次の傾向が確認できる。

| 項目 | 確認値 |
|---|---:|
| `ENGAGE` | 20回 |
| dynamic path | 12回 |
| profile path | 8回 |
| 明確にclearから`OFFSET-RETURN`へ到達 | 少なくとも6 episode |
| ENGAGEからclearまで | 4.09～6.80秒 |
| ENGAGEからclearまでの中央値 | 約5.20秒 |
| `COLLISION-SUSPECTED` | 6ログ、3クラスター |
| Stuck検出 | 5回 |
| `LAT-TTC-ACT` | 416回 |
| `LAT-TTC-CLAMP` | 86回 |
| LAT-TTC giveup | 15回 |
| switchback分岐 | 5回 |

ログにはLap 3が96.87秒、Lap 4が100.79秒という大きな外れ周もある。また、`OFFSET-RETURN`のON/OFF反復と、offsetを長時間保持する区間も確認できる。

したがって、この車両も「20回仕掛ければ安定して20回抜ける完成品」ではない。衝突やスタック、再試行を許容しながら、相手機会に対して繰り返し仕掛ける競技向け実装と評価するのが適切である。

### 上位車から移植価値が高い考え方

- 相手との縦・横相対状態を実行中も閉ループで監視する
- 相対速度と到達時間からENGAGE距離を動的に決める
- 横移動と縦速度を別々の後付け制限として扱わない
- body-clearまでの速度源を段階的に切り替える
- lateral TTC、footprint、ICC、line capを速度仲裁へ反映する
- clear後に通常ラインへ戻す処理を明示的に持つ

### そのまま移植すべきでない点

- 上位ログの`a_max: 1.37 m/s^2`。現行ソフトは運営上限として`1.0 m/s^2`を維持する
- 極端に大きい横偏差重みや操舵上限
- LAT-TTC判定直後の複雑なgiveup・switchback
- 衝突・スタック込みの攻撃性
- `OFFSET-RETURN`のチャタリング

## ChatGPT Pro分析で同意する点

### 1. ShiftOut中のmission ownershipが弱い

現HEADで追加されたBehavior ownershipは、主にPass成立後を保護する。ShiftOut中にも速度維持処理はあるが、「選んだ時空間計画をbody-clearまでBehaviorと速度仲裁の両方が所有する」という状態にはなっていない。

そのため、横へ出る途中に通常Follow、front cap、SafetyBrake側の判定が強くなり、追い越しが追従に戻る余地が残る。

### 2. 横経路とclosing speedの同時評価が必要

同じ横移動でも、相手速度、自車速度、前方距離、ShiftOut距離によってbody-clear時刻が変わる。

したがって、次の単位で評価する必要がある。

```text
side × 横目標 × ShiftOut距離 × closing speed
```

採用後は、選んだ横経路と速度プロファイルを同じmissionとして固定し、実行中は実測進捗との差だけを補正する構成が妥当である。

### 3. コース座標上の相手予測が必要

相手の横速度は、瞬間的なV2X位置差をそのまま利用するとカーブや座標変換誤差で揺れる。

単純に既存のcourse progress予測フラグを再度有効化するのではなく、`s_dot`、`d_dot`をフィルタし、予測信頼度とbranch continuityを持たせるべきである。

### 4. 追い越し専用MPCモードはA/B候補になる

追い越し時だけ横偏差追従を強める考え方は妥当である。ただし、横追従遅れが主因であることをログで確認してから導入すべきである。

現在までにOSQP fallbackやsolver failureが発生しているため、上位車の重みをそのままコピーすると数値条件を悪化させる可能性がある。まずは通常値の1.5倍、2.0倍程度の限定的なA/Bが妥当である。

### 5. lateral TTC制御は有効だが段階導入が必要

上位ログでは`LAT-TTC-ACT`が416回、clampが86回あり、giveupやswitchbackも発生している。これは有効な制御要素である一方、判定が頻繁に作動している証拠でもある。

最初はフィルタ済み横接近傾向を使った速度維持・速度抑制のみとし、左右switchbackは後段で追加するのがよい。

## ChatGPT Pro分析で補正が必要な点

### 現HEADには既に候補探索がある

現在の`mpc_controller_cpp.cpp`には、以下の探索が存在する。

- 左右のpass side評価
- nominal 4 mに対するShiftOut距離候補
  - 2.5 m
  - 3.25 m
  - 4.0 m
  - 5.0 m
- corridor内の複数横目標候補
- 動的mission corridorの全区間評価
- 壁・footprint・横加速度preflight
- body-clear期限による候補順位付け
- 採用したShiftOut距離のmission固定

対象実装:

```text
aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/
  src/mpc_controller_cpp.cpp:6180付近
```

したがって、次に必要なのはcandidate latticeの新規作成ではなく、既存latticeへclosing speed軸を追加することである。

### 現在のbody-clear期限はhard gateではない

body-clear期限に間に合わない候補は、現在の実装ではmission entryを必ず拒否する条件ではなく、候補選択上のペナルティとして扱われる。

```text
mpc_controller_cpp.cpp:6338付近
```

これは「Followし続けるより、実行可能な中で最もマシな候補へ仕掛ける」という競技向けの攻撃方針である。しかし、選択したclosing speedではbody-clear不能な候補を採用すると、次の負の経路になり得る。

```text
ShiftOut開始
  -> 横移動完了前に前後距離が詰まる
  -> Follow/front cap/SafetyBrakeが介入
  -> 失速またはRecovery
  -> 相手から再び離される
```

body-clear期限を単純にhard gateへ戻すだけでは、再び追従し続ける車になる。先にclosing speed候補を増やし、「速度プロファイルを変えれば期限内にbody-clearできるか」を評価したうえで採否を決める必要がある。

### 50 Hz化は主対策ではない

上位者ログも実質約40 Hzで動作している。したがって、40 Hzから50 Hzへ変更するだけで追い越し性能が大幅に改善する根拠はない。

優先すべきなのは周期向上ではなく、各周期で更新している相対状態を横経路・縦速度の閉ループ制御へ正しく反映することである。

## 推奨する次の実装順序

### P0-1. 既存候補探索へclosing speed軸を追加する

候補ごとに少なくとも以下を予測する。

- body-clear時刻
- hard-distance到達時刻
- ShiftOut完了時刻
- 最大必要横加速度
- 壁・相手footprintとの最小余裕
- Pass完了までの距離

現行の`a_max: 1.0 m/s^2`の範囲で実現できる速度プロファイルのみを候補とする。

### P0-2. 採用した横経路と速度プロファイルをmissionへ固定する

mission開始後は、相手の小さな横揺れだけで左右や目標横位置を変更しない。

実測進捗に応じてclosing speedを補正するが、以下のhard abort以外では通常Followへ所有権を戻さない。

- 実footprint重複
- 壁接触または実行経路消失
- locked targetの大きな位置ジャンプ・消失
- solverによる制御不能

### P0-3. ShiftOut用の進捗監視を追加する

少なくとも以下をログ化する。

- selected side、goal、shift distance、closing speed
- predicted body-clear time
- actual body-clear time
- predicted/actual lateral progress
- remaining body-clear margin
- speed owner
- Behavior owner
- front cap/SafetyBrake介入理由

body-clearの遅れが横追従由来か、速度仲裁由来かを切り分けられるようにする。

### P1. 追い越し専用MPCモードを小幅A/Bする

横追従が計画より明確に遅れている場合だけ、追い越し中の横偏差・heading重みを段階的に増やす。

```text
A: 現行
B: 横追従重み 1.5倍
C: 横追従重み 2.0倍
```

評価対象は横偏差だけでなく、steering saturation、OSQP fallback、wall contactも含める。

### P2. フィルタ済みlateral TTC制御を追加する

最初は同じ側を維持したまま、横接近傾向に応じてclosing speedを調整する。switchbackは、同一側が一定時間物理的に成立しない場合に限定して後から追加する。

## 効果確認基準

初回のA/Bでは、追い越し成功率だけでなく内部経路を確認する。

- Entryからbody-clearまでの時間
- ShiftOutからPassへの遷移時間
- body-clear期限超過回数
- ShiftOut中のBehavior owner喪失回数
- front cap再適用時間
- SafetyBrake介入回数
- wall/solver Recovery回数
- 同一targetへの再ENGAGE回数

段階的な合格基準は以下とする。

1. ShiftOut中の不要なFollow復帰を解消する
2. body-clear前の失速を解消する
3. 12 km/h車に対して1回のmissionでPassまで到達する
4. 接触・wall Recovery・solver Recoveryを増やさない
5. 最終的に20試行連続で再現性を確認する

## 最終見解

ChatGPT Proの「固定パラメータの攻撃化ではなく、時空間計画と実行ownershipを改善すべき」という主張は正しい。

一方、現HEADは既に複数のShiftOut距離・横目標・左右候補を探索しているため、これを捨てて別Plannerを作り直す必要はない。

次に行うべき作業は、既存候補探索を次の形へ拡張することである。

```text
現状:
  side × 横目標 × ShiftOut距離

次段階:
  side × 横目標 × ShiftOut距離 × closing speed
```

そのうえで、採用した横経路と速度プロファイルをShiftOutからbody-clearまで一体として所有・実行する。

上位者実装の価値は、加速度1.37、極端な操舵、衝突覚悟の設定ではなく、「相手との相対状態を見ながら抜き切るまで速度源を管理する」という実行層の構造にある。
