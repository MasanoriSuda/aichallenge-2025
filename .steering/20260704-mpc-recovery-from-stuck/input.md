現行品(MasanoriSuda側)は前進前提なので巻き込み事故等により壁にしょうめんしょうとつしたとかなるとそこから復帰できません、考えられうる案を出して星取表を作ってください
10m 29s考えました
結論

この問題は、MPCの追従性能というより制御モードが前進しか持っていないことが本質です。したがって、MPCをMPCCへ置き換えるだけでは解決しません。

おすすめは、

**通常走行のMPC/将来のMPCCとは別に、後退可能な`Recove時バランスがよいのは、

スタック検知＋ギア管理＋有限個の後退マヌーバ候補を安全評価して選ぶ方式

です。最初からHybrid A*やギアを含むハイブリッドMPCCへ行くより、かなり作りやすく、事故原因も切り分けやすいです。

現行実装で復帰できない理由

コード上も前進専用になっています。

MPCの速度入力下限は明示的に0.0なので、最適化問題から負速度は出ません。 cpp
Eigen::Vector2d umin;
umin << 0.0, -std::tan(cfg.delta_max) / model->length;


solver失敗時のfallbackも速度を必ず`0.0`以上に丸め、最終的に減速停止するだけです。 、現在速度はMPCへ渡す前に絶対値化されています。

```cpp
mpc_->update_current_speed(std::abs(actual_v));

衝突時刻からis_collidingも作られていますが、現在は未使用です。つまり、衝突を検出できても制御モードは変化しません。 publisherにもGearCommandはなく、AckermannControlCommandとBoost関係だけです。 2026公式インターフェースには、

/control/command/gear_cmd
/vehicle/status/gear_status
NEUTRAL=1
DRIVE=2
REVERSE=20

が定義されています。また、AWSIMではAckermannControlCommand.longitudinal.speedは未使用で、longitudinal.accelerationが車両入力として使われます。したがって、MPCで負の目標速度を出すだけでは後退せず、ギア制御を追加する必要があります。

案の星取表

すべて星が多いほど良い評価です。「実装容易性」は星が多いほど作りやすく、「安全・検証性」は挙動を限定してテストしやすいことを意味します。

案	正面衝突からの復帰力	実装容易性	安全・検証性	現行MPCとの親和性	SIM・実車共通化	総合
1. AWSIM自動姿勢補正・手動復帰に任せる	★★☆☆☆	★★★★★	★★★☆☆	★★★★★	★★☆☆☆	★★☆☆☆
2. 経路再投影だけ行い、前進で再試行	★☆☆☆☆	★★★★★	★★★★☆	★★★★★	★★★★★	★★☆☆☆
3. 固定時間・固定距離の直進バック	★★★☆☆	★★★★★	★★☆☆☆	★★★★★	★★★★☆	★★★☆☆
4. 壁方向を見て後退操舵するRecovery FSM	★★★★☆	★★★★☆	★★★★☆	★★★★★	★★★★☆	★★★★★
5. 有限運動プリミティブ探索	★★★★★	★★★☆☆	★★★★★	★★★★☆	★★★★☆	★★★★★
6. 後退専用ローカルMPC	★★★★☆	★★☆☆☆	★★★★☆	★★★★☆	★★★★☆	★★★★☆
7. Hybrid A* / Reeds–Shepp復帰経路計画	★★★★★	★★☆☆☆	★★★★★	★★★☆☆	★★★★☆	★★★★☆
8. ギアを含むハイブリッドMPC/MPCC	★★★★★	★☆☆☆☆	★★☆☆☆	★★☆☆☆	★★★☆☆	★★☆☆☆
1. AWSIM自動姿勢補正・手動復帰に任せる

2026ルールでは、壁衝突時の自動姿勢補正が予定されていますが、スタックを完全には防げないと明記されています。また、SIM決勝・実機決勝の手動復帰も現在は未定扱いです。SIM予選はコード提出後の自動走行なので、手動復帰を主戦略にはできません。 価**

実装不要
最終fallbackとしては有効
SIM予選では期待できない
自動姿勢補正の仕様変更に依存する
手動復帰まで順位を大きく落とす可能性がある

競技用の一次対策にはせず、最後の救済手段として残します。

2. 経路再投影だけ行い、前進で再試行

現在のupdate_states()は車両位置に最も近いwaypointを毎周期探し直しているため、経路への再投影自体はすでに行われています。 、正面が壁に接触している場合、

壁
██████████
    ↑
    車

前進可能な軌道は存在しません。参照waypointや横目標を変更しても、車両はその場で舵を切れないため抜け出せません。

軽い側面接触や、壁へ完全には押し付けられていないケースには効きますが、正面衝突対策としては不十分です。

3. 固定時間・固定距離の直進バック

最小実装は次です。

衝突またはスタック検知
    ↓
停止
    ↓
REVERSEへ切り替え
    ↓
低加速度で1m程度直進バック
    ↓
停止
    ↓
DRIVEへ切り替え
    ↓
MPCへ復帰
長所
正面衝突にはかなり効く
実装量が少ない
挙動が単純でテストしやすい
通常MPCを変更しなくてよい
短所
後方に他車や壁があると危険
前輪が壁際へ向いた状態のまま下がる
斜め衝突や前角が引っ掛かった状態では抜けない場合がある
後退距離の固定値がコース状況に合わない

MVPとしてはありですが、後方安全確認なしの固定バックは本番投入しない方がよいです。

4. 壁方向を見て後退操舵するRecovery FSM

固定バックを一段賢くした案です。

前方の壁法線、車両方位、左右の空きから、

直進バック
左へ操舵してバック
右へ操舵してバック

のどれを使うか決めます。

正面衝突       左前角衝突      右前角衝突

  壁              壁               壁
██████          ██████           ██████
  ↑車              ↖車              車↗
   │                ╲                ╱
直進バック     後退アーク       後退アーク

ただし、「左前をぶつけたら右へ切る」のように符号をハードコードするより、候補軌跡を短時間rolloutした方が安全です。後退時は速度符号が逆になるため、操舵角とyaw変化の関係も前進時とは逆になります。

この案は単体でもよいですが、次の有限運動プリミティブ探索と組み合わせるのが最も堅実です。

5. 有限運動プリミティブ探索：最推奨

本格的な経路探索ではなく、あらかじめ用意した少数の復帰操作をシミュレーションして選びます。

候補例は以下です。

R0:  直進バック
RL:  左操舵バック
RR:  右操舵バック
RLF: 左操舵バック → 右操舵前進
RRF: 右操舵バック → 左操舵前進

各候補をkinematic bicycle modelで1〜2m先までrolloutし、次の評価値で選びます。

Score=w
1
	​

⋅最小壁距離+w
2
	​

⋅衝突地点からの離脱量−w
3
	​

⋅経路方位誤差−w
4
	​

⋅後退距離−w
5
	​

⋅操舵変化

以下を満たさない候補は除外します。

静的壁との衝突なし
後方他車との衝突なし
コース外へ出ない
車体矩形を含めた余白を確保
後退終了地点から前進復帰可能
最大後退距離・最大実行時間以内
この方式がよい理由
正面衝突では自動的に直進バックを選べる
斜め衝突では後退アークを選べる
後ろに他車がいればバックせず待機できる
Hybrid A*ほど複雑ではない
数個の候補だけなので計算負荷が小さい
全候補をログやRVizに表示でき、理由を説明しやすい

現行コードはoccupancy grid mapとV2X位置をすでに扱っているので、壁と他車の安全評価を再利用できます。通常走行のMPCはそのまま残せます。

6. 後退専用ローカルMPC

通常MPCとは別に、復帰専用MPCを用意します。

NormalMPC
  状態: 前進のみ
  目的: 経路追従・高速走行

RecoveryMPC
  状態: global x, y, yaw
  ギア: 呼び出し前にREVERSEへ固定
  目的: 壁から離れ、安全な復帰poseへ移動

復帰専用MPCは、ギアを離散変数にせず、1回のsolve中はREVERSE固定にします。

目標poseは例えば、

現在位置から後方1m
壁から十分離れた位置
元の経路へ前進復帰できる向き

です。

長所
滑らかな後退制御になる
車体制約や操舵速度制約を自然に入れられる
固定プリミティブより状況適応性が高い
短所
復帰目標poseを別途作る必要がある
初期状態が壁へ食い込んでいると制約が即座にinfeasibleになり得る
solver failure時のさらに下のfallbackが必要
通常走行MPCとの二重保守になる

有限プリミティブで足りないケースが多かった場合の二段目として適しています。

7. Hybrid A* / Reeds–Shepp

前進・後退を含む自動駐車系の局所経路探索を使います。

現在pose
  ↓
Hybrid A*
  ↓
後退・前進を含む復帰経路
  ↓
Pure Pursuit / MPCで追従
  ↓
通常経路へ復帰

壁と壁の間に斜めに挟まれた場合や、複数回切り返さないと脱出できない場合に強いです。

一方、レーシングカートの競技環境では、

復帰処理としては重い
動く他車を扱うため再計画が必要
切り返し回数が増えると他車との接触リスクが増える
通常走行とは全く別のplannerが増える

というコストがあります。

有限プリミティブでカバーできない事故パターンが実測された後に導入する位置付けです。

8. ギアを含むハイブリッドMPC/MPCC

最適化変数として、

gear
k
	​

∈{DRIVE,REVERSE}

を持たせる案です。理論的には最も統合的ですが、ギアが離散変数なので、通常のQPでは扱えません。

候補は、

MIQP
mixed-integer MPC
gear sequenceを列挙して各QPを解く
非線形MPCCでforward/reverse modeを切り替える

などです。

しかし競技車両では、

solve時間を保証しにくい
gear chattering対策が必要
ギア変更中の状態をモデル化する必要がある
infeasible時の原因が分かりにくい
通常走行の信頼性まで巻き込む

ため、最初の実装としてはおすすめしません。

MPCC化とスタック復帰は分離する方が安全です。

推奨アーキテクチャ
                    ┌───────────────────┐
Odometry ──────────▶│                   │
GearStatus ────────▶│ RecoverySupervisor│
Collision event ───▶│                   │
Map / LiDAR ───────▶│                   │
V2X ───────────────▶│                   │
                    └─────────┬─────────┘
                              │ mode
          ┌───────────────────┼──────────────────┐
          │                   │                  │
          ▼                   ▼                  ▼
     Normal MPC        Recovery Planner      Safe Stop
          │            / Primitive Search        │
          └───────────────────┬──────────────────┘
                              ▼
                       Command Arbitrator
                       ├─ GearCommand
                       └─ AckermannControlCommand

重要なのは、MPCノードとRecoveryノードが同時に車両commandをpublishしないことです。最終publisherを1か所に集約します。

Recovery FSM
NORMAL
  │
  ├─ 衝突イベント
  └─ スタック条件継続
       ↓
SUSPECT_STUCK
       ↓
STOP_AND_CONFIRM
       │  車速ゼロを確認
       ↓
SELECT_MANEUVER
       │
       ├─ 安全な後退候補なし ──▶ WAIT_FOR_CLEAR / REMOTE_STOP
       │
       ▼
SHIFT_TO_REVERSE
       │  gear_status=REVERSEを確認
       ▼
REVERSE_MANEUVER
       │
       ├─ 後方障害物接近 ──▶ STOP_AND_ABORT
       ├─ timeout ────────▶ RETRY_OR_ABORT
       └─ 離脱条件成立
              ↓
STOP_BEFORE_SHIFT
              ↓
SHIFT_TO_DRIVE
       │  gear_status=DRIVEを確認
       ▼
FORWARD_REALIGN
              ↓
REACQUIRE_PATH
              ↓
NORMAL
スタック検知

衝突通知だけに依存しない方がよいです。既存コードの衝突検知は利用できますが、現在のis_collidingは破棄されています。 合条件にします。

スタック候補 =
  自動制御が有効
  AND 通常MPCが前進を要求
  AND 実車速がほぼゼロ
  AND 経路進捗が増えない
  AND 以下のどれか:
      ・衝突イベントあり
      ・前方壁距離が小さい
      ・大きな横偏差 / 方位偏差

除外条件も必要です。

除外:
  SafetyBrake中
  前走車待ちのFollow中
  Start前
  control disable中
  gear切替中
  後方に他車がいる

開発時の初期値例としては、

|v| < 0.1〜0.2 m/s
前進要求が0.8〜1.5秒継続
経路進捗が0.1〜0.3m未満

程度からログを見て調整できます。これらは公式値ではなく、誤検知評価用のローカル初期値です。

後退前の安全確認

後退復帰は、スタックから抜ける一方で後続車へ突っ込む新しい事故を作り得ます。最低限、次をhard conditionにします。

車速がほぼゼロになってからギア変更する
gear_statusで切替完了を確認してから加速度を出す
後方車体swept areaに壁がない
V2Xで後方車両が接近していない
後方情報が不明な場合は、短距離・低速のcreepに限定する
最大後退距離、最大実行時間、最大試行回数を設ける
条件を満たさなければ停止を維持する

公式インターフェース上、control_cmd.longitudinal.speedはAWSIMでは未使用です。したがって、REVERSEギア時の加速度符号やギア遷移タイミングは、小さな検証ノードでAWSIMと実車の双方を確認し、GearManager内へ閉じ込めるべきです。

現行コードへの変更点
1. ギアI/O追加
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
gear_command_pub_ =
  create_publisher<GearCommand>("/control/command/gear_cmd", 1);

gear_status_sub_ =
  create_subscription<GearReport>(
    "/vehicle/status/gear_status", 1, ...);
2. signed velocityを保持する

現状は、

mpc_->update_current_speed(std::abs(actual_v));

ですが、後退中のSupervisorには符号付き速度が必要です。

recovery_supervisor_->update_signed_velocity(actual_v);
mpc_->update_current_speed(std::abs(actual_v));  // 通常MPC側は当面維持

通常MPCまで一度に負速度対応へ変えず、責務を分離します。

3. 衝突フラグをRecoverySupervisorへ渡す
const bool collision_recent =
  last_colliding_time_.has_value() &&
  (current_time - *last_colliding_time_).seconds() < collision_window;

recovery_supervisor_->update_collision(collision_recent);

現在のように(void)is_colliding;で捨てないようにします。

4. command arbitration
switch (recovery_supervisor_->mode()) {
  case Mode::Normal:
    publish(normal_mpc_command);
    break;

  case Mode::Recovery:
    publish(recovery_command);
    break;

  case Mode::SafeStop:
    publish(stop_command);
    break;
}

Normal MPCとRecoveryの二重publishを防ぎます。

5. 復帰後に内部状態をリセット

復帰完了時には少なくとも、

MPCの予測制御列
前回操舵
fallback履歴
overtake target lock
V2X pass-side lock
solver failure counter
参照waypoint
low-pass filter履歴

を再初期化します。

現在のupdate_states()は現在位置から最近傍waypointを選び直すため、経路への再投影には使えます。 、後退直後は車両方位が経路と逆向きになる可能性があります。その状態ですぐ通常速度へ戻さず、

低速前進
→ heading errorが閾値以下
→ lateral errorが閾値以下
→ 通常v_maxへ戻す

という再合流期間を設けます。

実装の優先順位
第一段階

RecoverySupervisor＋直進バック

feature flagで無効化可能
gear command/status実装
スタック検知
後方安全確認
低速直進バック
MPC状態リセット
再合流時の速度制限
第二段階

3候補のプリミティブ探索

reverse straight
reverse left
reverse right

車体矩形のrolloutで最も安全な候補を選びます。

第三段階

二段マヌーバ追加

reverse left  → forward right
reverse right → forward left
第四段階

実走ログで有限プリミティブが足りないことが確認された場合のみ、

後退専用MPC
Hybrid A*

を追加します。

必須テストケース
ケース	期待結果
壁へ真正面から接触	直進バックを選択
左前角が壁へ接触	安全な後退アークを選択
右前角が壁へ接触	反対側の後退アークを選択
後方に他車がいる	バックせず停止・待機
後方に壁がある	短距離候補または復帰不可判定
Followで一時停止	スタックと誤判定しない
SafetyBrake中	Recoveryへ遷移しない
衝突イベントが来ない	速度・進捗監視から検知
gear statusが返らない	加速せずSafeStop
後退中に他車が接近	即時停止
復帰中に再衝突	再評価し、試行回数上限で停止
経路と逆向きに復帰	低速で方向を合わせてからMPC復帰
最終的な推奨

現行構成に対しては、次が最適です。

通常走行は現行MPCまたは将来のMPCCに任せ、スタック時だけ独立したRecoverySupervisorへ切り替える。RecoverySupervisorはGearCommandを管理し、直進・左・右の後退プリミティブを地図とV2Xに対してrolloutして選択する。

これなら、

通常走行性能を巻き込まない
正面衝突と斜め衝突の両方を扱える
後方車両がいる場合は安全に停止できる
SIMと実車で同じ状態機械を使える
将来MPCCへ変更してもRecovery部分をそのまま維持できる

という構成になります。