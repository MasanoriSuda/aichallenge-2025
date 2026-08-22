# Audit

## 観測された現象

`output/20260823-065700`ではlap 1を46.681秒で完了後、waypoint 53付近で
実速度が`10.089 -> 1.424 m/s`へ`0.120 s`で低下した。直前指令は前進かつ
正加速度のままで、canonical five-state Track/Cruiseはfreshな物理証明済み解を
選択していた。Recoveryは速度低下後に開始した。

## 問題が発生するまでの因果関係

1. Track/Cruise canonical MPCCが解と静的wall certificateを生成した。
2. controllerは前進・正加速度・小さい右操舵を指令し、実操舵も追従した。
3. controllerの静的occupancy-gridモデル上では、実測poseの車体footprintは
   少なくとも`1.0 m`の検索範囲内でclearだった。
4. それにもかかわらず、AWSIM由来の実速度だけが通常制動包絡を大幅に超えて低下した。
5. 既存コードが外部衝突指標として期待した`/aichallenge/pitstop/condition`は
   current AWSIMに存在せず、静的証明外で生じた変化の直接証拠が残らなかった。
6. 下流では低速・停止として観測され、その後Recoveryへ遷移した。

## 根本原因

このSliceで証明できた根本欠陥は、wall certificateの誤判定そのものではなく
**証明・観測境界の不完全さ**である。certificateは設定済み静的occupancy gridだけを
証明する一方、AWSIM側のcollider／penaltyとの同値性は保証していない。さらに、その
差を観測する想定だったcondition topicが現環境に存在しないため、外部速度不連続を
controller起因の減速と区別できなかった。

AWSIM内の具体的なcolliderまたはpenalty sourceまでは、現ログだけでは特定していない。
したがって「AWSIM壁colliderが必ず1 mずれている」とは結論しない。

## 根拠

- 失敗時cross-track errorは約`-0.238 m`。成功runの同地点には約`-0.244 m`があり、
  tracking error悪化では分離できない。
- 失敗poseをC++と同じstatic-grid footprint samplerで再現するとclearanceは
  検索上限`0.9999 m`以上。
- 指令操舵`-0.0631 rad`に対し実操舵`-0.0585 rad`で、大きな符号・scale不一致なし。
- controllerは減速指令を出しておらず、観測加速度は約`-72 m/s^2`。
- V2X targetなし、Recoveryは事後、事故前callback overrunなし。
- `output/20260823-075629`は同じ制御で6周を
  `46.111 / 43.285 / 43.965 / 42.390 / 43.310 / 43.360 s`で完走し、
  同種の速度不連続とRecoveryは0回だった。
- `/aichallenge/pitstop/condition`はbag metadataにもcontroller baselineにも現れず、
  current AWSIMではpublishされていない。
- observer接続後の`output/20260823-081219`は46.386秒で1周完了し、正常走行中の
  `Abrupt measured speed loss`は0件だった。

## 既存パッチとの関係

- legacy steering gain `1.5`の再適用は原因仮説を反証し、既に撤回済み。
- wall margin、solver weight、speedの変更は、この証拠からは根拠がないため行わない。
- Recoveryは原因ではなく事後応答なので、このSliceでは変更しない。
- 新しいfallback、timeout、feature flag、authority leaseは追加しない。

## 修正方針

制御動作を変えず、外部速度不連続を入力境界でchange-onlyに観測する。観測イベントには
前後速度、時間差、観測加速度、直前command、生pose、静的map sample、condition topicの
可用性、decision IDを一行で残す。将来condition topicが存在する環境ではrosbagにも保存する。

## 実施した変更

- configured braking envelopeを超える急減速だけを報告するpure monitorを追加。
- odometry callbackに診断observerを接続。authorityとcommandは変更しない。
- pitstop conditionのbaseline／transitionを、pose・command・静的wall observation付きで記録。
- 開発用bag allowlistとlocalization-scope extractorへconditionを追加。
- pure monitorとbag extractorへfailure-first testを追加。
- 同地点比較・static-grid再現用のoffline extractorをsteering artifactとして追加。
- `multi_purpose_mpc_ros`全テストは1,632件、error/failure/skip 0件。

## 削除・整理できた処理

collision判定の`delta > 30`という既存動作は互換維持のため削除していないが、subscription内の
匿名処理を`handle_pitstop_condition()`へ集約した。新observerは既存collision判定を代替せず、
欠落topicを理由にcollision stateを推測・強制設定しない。

## 残っている懸念

- AWSIM collider／penalty sourceの直接topicは未確認で、物理源の最終特定は次回再現待ち。
- 6周runには開始直後の1秒窓でcallback overrunが1回あり、Track/Cruise昇格Gateの
  「連続overrun 0」と分けて精査が必要。
- 6周runには`execution-primal-reject`が97回あり、既知のmixed-unit semantic-row問題は
  別Sliceで根本監査する。

## 次回試走で確認すべき項目

1. 正常走行で`Abrupt measured speed loss`が出ないこと。
2. 再発時に同イベントが速度低下の最初の周期で一度出ること。
3. `command`が前進のままか、controller自身が先に制動したか。
4. 同時点の`map_sample`が`clear/contact/out-of-map/invalid`のどれか。
5. condition topicが存在する環境ならtransitionとの時系列差。
6. その後のauthority遷移をdecision IDで追跡できること。
