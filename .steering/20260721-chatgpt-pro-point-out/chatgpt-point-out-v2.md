今の設定だとうまくオーバーテイクできずに追突してそのまま復帰できないことがあるのだが何が原因だろうか？
10m 53s考えました
結論

ログなしの静的診断ですが、かなり確度が高い原因は、次の二段事故です。

横に十分出切る前に、前車を「前方障害物ではない」と扱ってしまう
追突後は、その停止を「前車に合わせて意図的に止まっている」と判定し、スタック復帰を開始しない

さらに復帰へ入れても、V2X完全性や後退回廊判定が厳しく、最後は SafeStop に入りやすい構成です。

1. 追突する最有力原因：0.75 mで前車ブレーキ判定を外している

現在、他車と接触しないための横方向中心間距離は 1.45 m と定義されています。これは自車と相手車の半幅を足した値です。

一方、追い越し中に前車を「十分横に離れた」と判断する閾値は、

v2x_overtake_pass_front_overlap_lateral_clearance: 0.75

です。

コードではPass中に横差がこの0.75 mへ到達すると、その対象を前方衝突判定から除外し、ラッチします。
実際の前車判定も、ラッチ後は front_overlap から除外されます。

つまり、

無接触に必要：1.45 m
前方ブレーキを解除：0.75 m

となっています。

まだ横方向に約0.70 m重なり得る段階で、前車ブレーキを無効にしているということです。

しかも明示的な追い越しラインの基本オフセットも 1.2 m なので、相手が基準軌道上にいれば、最終的なPassラインでも車体幅モデル上は約0.25 m重なります。

接触許容方針として意図的な値だと思いますが、これは「軽く擦りながら抜く」よりも、相手の後端へ入り込んでリアクォーターまたは後面を押す状態になりやすいです。

2. 横移動中の接近速度が速すぎます

現在の主要値は、

追い越し開始可能な前方距離：4.0 m
ShiftOut距離：4.0 m
ShiftOut最大接近速度：2.0 m/s
Pass前半の接近速度：1.0 m/s
ハードカーブ内からの新規追い越し：有効
横加速度上限：6.0 m/s²
一時的なgap消失保持：1.0秒

です。

追い越しライン側では、0.75 mの横クリアランスがラッチされる前はPassの接近速度を1.0 m/sへ落とします。しかしラッチ後は、再び最大2.0 m/sを使えるようになります。

したがって典型的には、

中心間4 m付近でShiftOut開始
前車へ最大2 m/sで接近
横差0.75 mへ到達
前車がgeneric front brakeから消える
接近速度が再び最大2 m/sへ増える
しかし車体としてはまだ横に重なっている
前車後端へ接触

となります。

しかもV2Xはコードコメント上、約1 Hzになる場合があります。
2 m/sの相対速度なら、1更新の間に約2 m詰まります。4 mから追い越しを始める場合、前車がカーブで少し減速しただけでもかなりギリギリです。

3. ハードカーブ攻撃設定が、この問題を増幅しています

現在は内差し・外回りの両方をハードカーブ認識後でも開始でき、入口条件もかなり短くなっています。設定自体は、

前方4 m
prepare 3 m
ShiftOut 4 m
横加速度6 m/s²
wall clearance 0.1 m
residual gap 0.2 m

という攻撃設定です。

この最新の組み合わせは、単体テストとビルドは通っていますが、記録上はdev3実走未実施です。

ハードカーブでは、

前車が急減速する
経路座標系の横差が変化する
内外判定が切り替わりやすい
自車の横移動能力が落ちる

ため、直線で成立する 4 m / 2 m/s / 0.75 m がそのまま成立しません。

4. 追突後に復帰が始まらない原因：deliberate_stop

接触後、自車が前車を引き続き認識していると、V2X状態はほぼ SafetyBrake または Follow になります。

現在のコードは、次の状態をすべて「意図的な停止」としています。

LowSpeedAvoidance
SafetyBrake
Follow && has_front_vehicle
has_danger_vehicle

スタック検出器は、意図的停止であり、協調後退条件を満たさない場合、観測時間をリセットして DeliberateStop として除外します。

協調後退へ進めるのは、前車速度が 0.20 m/s 以下の場合だけです。

そのため追突後に前車が、

少し動いている
接触で押されている
V2X速度推定が0.2 m/sを少し超えた
位置ノイズから微小速度が出た

というだけで、

自車は動けない
前進要求はある
でもSafetyBrakeだから意図的停止
前車速度が0.2 m/s超なので協調Recovery対象外
スタック観測を毎周期リセット

となります。

これは「追突後、ずっと止まったまま復帰へ入らない」という症状にかなり一致します。

5. 復帰へ入っても、前車が後退経路を塞いだ扱いになる

停止車両については、選択した後退rolloutに沿って「後退することで距離が改善するなら許可」する実装になっています。

ただしその処理が使われるのは、前車速度が 0.20 m/s 以下の場合です。

0.20 m/sを超えると、より粗い moving_corridor 判定へ切り替わります。そこで前車の膨張円が後退回廊と少しでも重なると、後退禁止になります。

つまり同じ配置でも、

前車速度 0.19 m/s：後退すれば離れるので許可される可能性
前車速度 0.21 m/s：moving corridor overlapで禁止

という大きな挙動差が出ます。

過去の実験でも、停止車向けrolloutのデッドロックは修正された一方、移動車は引き続き moving_corridor / corridor_overlap で後退停止対象となっています。

これも「同じように追突したのに、復帰できる場合とできない場合がある」理由になります。

6. V2XまたはAWSIM statusが一瞬欠けるだけでも後退できません

復帰開始には、

V2Xメッセージが新鮮
想定車両数が正確に2台
ID欠落なし
重複IDなし
不正sampleなし

が必要です。

加えて、AWSIM Boostが停止中であることを0.5秒以内のstatusから確認する必要があります。

どちらかが成立しないと、復帰安全判定はその時点で終了し、後退を許しません。

追突直後は複数車が密集し、V2X速度推定やposition jumpが不安定になりやすいため、まさにこの条件が外れやすい場面です。

7. 一度SafeStopへ入ると、ほとんどの原因では自動再試行しません

現在は、

aggressive_sim_recovery_enabled: false

です。

この状態では、ClearanceWaitTimedOut の一部を除き、SafeStopは基本的にそのまま維持されます。ManeuverDirectionUnknown、ContactNotImproving、GearReportTimedOut、SolverUnsafeなどでは再試行しません。

またRecovery中にodomや非有限指令などのfailsafeが一度発生すると、recovery_fault_latched_ が立ちます。
以後はセッションリセットまでfailsafeを出し続けます。

したがって復帰不能は、大きく次の2種類です。

Recovery自体へ入っていない
Stuck detector: reject=deliberate_stop
Recoveryへ入ったが永久停止へ落ちた
maneuver_direction_unknown、contact_not_improving、solver_unsafe、recovery fault remains latched
最小限の修正案

まず追突を減らすA/B設定は、次がよいです。

v2x_overtake_pass_front_overlap_lateral_clearance: 1.15
v2x_overtake_shiftout_max_closing_speed: 1.2
v2x_overtake_pass_unlatched_max_closing_speed: 0.5
v2x_overtake_active_gap_loss_hold_sec: 0.4
v2x_overtake_guard_min_front_distance: 5.0

特に重要なのは、理想的には 0.75 mのパラメータを2つへ分けることです。

# 横移動が成立したとみなす値
v2x_overtake_phase_lateral_clearance: 0.75

# 前車ブレーキ判定から除外する値
v2x_front_brake_exclusion_lateral_clearance: 1.15～1.35

今は「Passへ移ったとみなす閾値」と「前車への衝突回避を解除する閾値」が同じなのが根本的に危険です。

復帰側は、次の変更が効きます。

coordinated_stop_front_speed_mps: 0.5

ただし単純な閾値変更より、

collision hintあり
自車速度0.15 m/s以下
0.4秒進捗なし
前車との距離が改善していない

なら、SafetyBrake中でも deliberate_stop を上書きしてRecoveryへ入れるべきです。

また前車が0.2 m/sを少し超えていても、選択した後退rolloutで全sampleの距離が単調改善するなら、moving_corridorではなくrollout clearanceを使う方が合理的です。

ログで確認すべき順序

該当runの autoware.log では、以下が並んでいればほぼ確定です。

OvertakeLine: Idle -> ShiftOut
OvertakeLine: ShiftOut -> Pass
Pass front-overlap exclusion latched ... lateral=0.7x
OvertakeLine debug ... closing=2.00
Collision detected!
Stuck detector ... reject=deliberate_stop

またはRecoveryへ入った後、

v2x_message_complete=0
v2x_gate=moving_corridor/corridor_overlap
maneuver_direction_unknown
contact_not_improving
stuck recovery fault remains latched until session reset

のどれかが出ているはずです。

最有力の根本原因は、横差0.75 mで前車衝突回避を解除するのが早すぎること。その追突後に、SafetyBrakeを意図的停止として扱うためRecovery判定まで抑止されることです。