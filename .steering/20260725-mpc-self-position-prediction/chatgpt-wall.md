GNSS + IMU + 車輪速センサから自己位置を推定するのですが、
GNSSの遅延やIMU(地磁気)が精度不足…など、実車特有の課題があると思うのでまずはこれらの情報をROSBAGから取って可視化するのがいい？

はい。最初にROSBAGへ生データと中間データをそろえて、同じ時刻軸で可視化するのが一番堅い進め方です。

ただし、単にRVizで軌跡を重ねるだけではなく、

どのセンサが、何秒遅れて、どの条件で、どの程度ずれるか

を数値化できる記録・解析環境にするのが重要です。

現在のrosbag設定は、制御指令・EKF最終出力・加速度などが中心で、GNSS/IMUの生データやEKF入力を記録していません。これでは原因を追いにくいため、まず記録対象を増やすのがよいです。

まず記録するトピック
rosbag_topics:
  # 時刻・座標変換
  - /clock
  - /tf
  - /tf_static
  - /diagnostics

  # GNSS生データ
  - /sensing/gnss/nav_sat_fix
  - /sensing/gnss/pose
  - /sensing/gnss/pose_with_covariance
  - /sensing/gnss/gnss_fixed

  # IMU
  - /sensing/imu/imu_raw
  - /sensing/imu/imu_data

  # 車輪速・車両状態
  - /vehicle/status/velocity_status
  - /vehicle/status/steering_status
  - /vehicle/status/gear_status

  # EKFへ入るデータ
  - /localization/imu_gnss_poser/pose_with_covariance
  - /localization/twist_estimator/twist_with_covariance_raw
  - /localization/twist_estimator/twist_with_covariance

  # EKF出力
  - /localization/kinematic_state
  - /localization/pose_with_covariance
  - /localization/twist_with_covariance

  # 制御との関係確認
  - /control/command/control_cmd

実際のトピック名は、実車上で次を実行して確認してください。

ros2 topic list | sort
ros2 topic info -v /sensing/imu/imu_raw
ros2 topic info -v /sensing/gnss/nav_sat_fix

特に、GNSS・IMU・車速の元メッセージを必ず残すことが重要です。加工後の値だけでは、補正ロジックが悪いのか、元データが悪いのか分離できません。

1. GNSSで見るもの
タイムスタンプ遅延

各メッセージについて、

t
age
	​

=t
receive
	​

−t
header
	​


を計算します。

見るべき値は、

平均遅延
最大遅延
標準偏差
受信周期
周期のジッタ
パケット抜け
遅延が速度・旋回・通信負荷で変わるか

です。

ただし、bagに保存される受信時刻だけではROSノード内部の厳密な受信時刻を取得できない場合があります。診断用ノードで受信直後に、

source_stamp
receive_stamp
source_age
interval

を専用debug topicへpublishする設計が確実です。

現在はSIM用にEKFの pose_additional_delay=0.3 s が設定されていますが、コード中にも実測前提の暫定値と記載されています。実車側は既定 0.0 s です。

したがって、実車では最初から0.3秒を入れるのではなく、ログから測って決めるべきです。

GNSS位置の品質

次を時系列とXY散布図で表示します。

FIX状態
position_covariance
停止中の位置散布
連続サンプル間のジャンプ距離
GNSS位置差分から求めた速度
車輪速との差
GNSS位置差分から求めた進行方向
IMU/EKF yawとの差

停止中のGNSS散布を見ると、最低限の再現精度が分かります。

標準偏差 x
標準偏差 y
95%半径
最大ジャンプ量

を走行ごとにJSONやCSVへ保存すると比較しやすいです。

GNSS遅延の推定

直線走行や旋回開始時に、

GNSS位置差分から得た速度
車輪速
GNSS軌跡から得たyaw rate
IMU angular_velocity.z

の相互相関を取ります。

例えば、

τ
∗
=arg
τ
max
	​

corr(ω
GNSS
	​

(t),ω
IMU
	​

(t−τ))

でGNSS側が何秒遅れているかを推定できます。

ただしGNSS位置差分は低速時に非常に不安定なので、

速度3 m/s以上
FIX良好
連続サンプル間距離が一定以上
異常ジャンプなし

などの条件で評価します。

2. IMUで見るもの
最初は地磁気よりジャイロを優先する

自己位置推定で最も使いやすいのは通常、

angular_velocity.z

です。

地磁気による絶対方位は、レーシングカートでは次の影響を受けやすいです。

モーターや大電流配線
車体金属
バッテリー
電装品
場所ごとの磁気環境
センサのhard iron / soft iron誤差

したがって、地磁気yawを真値扱いしない方が安全です。

地磁気は、

初期方位の補助
長時間yaw driftの弱い拘束
異常検出

程度から始め、走行中の主方位はGNSS進行方向、ジャイロ、車両運動モデルで構成する方が安定します。

停止時ジャイロバイアス

車両を停止させて30～60秒記録し、

b
ω
z
	​

	​

=
N
1
	​

k=1
∑
N
	​

ω
z,k
	​


を計算します。

見るものは、

平均値
標準偏差
起動ごとの差
時間経過による変化
温度との関係

です。

現在の imu_corrector 設定では、z軸オフセットはほぼ0、標準偏差は 0.03 rad/s と固定されています。これは実車計測から調整すべきパラメータです。

0.03 rad/s のバイアスが未補正で残った場合、単純積分では10秒で約0.3 rad、約17度ずれます。

yaw rateの整合性

以下を同時にプロットします。

IMU angular_velocity.z
EKF yawの時間微分
車両ステータスのheading rate
v * tan(steer) / wheelbase

車両モデル上のyaw rateは、

ω
model
	​

=
L
vtanδ
	​


です。

現在のMPC設定では車両長さパラメータとして 1.087 m が使われています。

比較結果の読み方は次のようになります。

症状	疑うもの
IMUだけ常時オフセット	ジャイロバイアス
IMUだけ符号が逆	座標軸・TF
IMUが約90度系統的にずれる	IMU取付姿勢・frame変換
操舵モデルだけ大きい	操舵角スケール、実舵角、スリップ
高速コーナーだけモデルと不一致	タイヤスリップ
全信号の形は同じだが時間がずれる	timestamp・通信遅延

このリポジトリでは imu_link にyaw -π/2 の取付回転が設定されています。IMUのorientationやangular velocityを扱う際は、base_linkへのTF変換が正しく行われているか必ず確認してください。

3. 車輪速で見るもの
車速スケール

高品質GNSSが得られる直線区間で、

k
v
	​

=
v
wheel
	​

v
GNSS
	​

	​


を求めます。

現在の設定は、

speed_scale_factor: 1.0
velocity_stddev_xx: 0.2

になっています。

実車ではタイヤ径、空気圧、荷重などにより、1.0から数％ずれる可能性があります。

区間を分けて、

低速
中速
高速
加速中
減速中
直線
旋回中

で比較してください。

スリップ判定

車輪速とGNSS速度がずれる条件を確認します。

急加速時：駆動輪スリップ
強い減速時：ロック、速度推定遅れ
コーナー中：横滑り、GNSS差分速度の誤差
バンプ通過時：車輪速瞬間変動

車輪速を常に正しい値としてEKFへ強く入れると、スリップ時に位置推定を引っ張ります。走行条件に応じて車輪速の共分散を変える方が理想です。

4. EKFで必ず見るもの

センサ単体だけでなく、EKFがそのセンサをどう受け取ったかも必要です。

最低限可視化したいのは、

GNSS入力位置
EKF出力位置
推定yaw
推定速度
共分散
GNSS残差
センサage
GNSS更新の採用・棄却
初期化時刻
EKF更新停止

です。

理想的にはEKF内部から、更新前残差を出します。

ν
k
	​

=z
k
	​

−H
x
^
k
−
	​


さらに、

NIS=ν
k
⊤
	​

S
k
−1
	​

ν
k
	​


を計算すると、GNSS共分散が妥当か判断しやすくなります。

単に、

GNSS位置 - 更新後EKF位置

を計算しても、EKFがGNSSへ寄った後なので残差が小さくなり、正しい評価にはなりません。

5. 推奨する走行データセット

いきなりレース走行だけ取ると、原因が混ざります。以下を個別に収録するのがよいです。

静止試験

30～60秒停止。

確認項目：

GNSS散布
ジャイロバイアス
地磁気yawの揺れ
車輪速ゼロ点
timestamp周期
直線一定速

低速・中速でそれぞれ走行。

確認項目：

GNSS遅延
車輪速スケール
yawの直進バイアス
位置差分速度
左右の定常旋回

左右両方向を同じ程度の速度・舵角で走行。

確認項目：

IMU z軸符号
左右非対称
ステアリングオフセット
ヨーレートゲイン
地磁気方位の場所依存
加減速試験

確認項目：

車輪スリップ
GNSS速度の追従遅れ
車輪速・加速度・制御指令間の遅延
周回走行

最後にレースコースを複数周します。

確認項目：

周回閉合誤差
同一地点での再現性
コーナーごとの系統誤差
制御と自己位置の相互作用
解析画面の構成

1回の走行について、最低でも次のグラフを自動生成するとよいです。

1. XY軌跡
   GNSS / EKF / 参照経路

2. センサ時刻
   source age / interval / dropout

3. 位置品質
   GNSS covariance / FIX / jump distance

4. 速度
   wheel speed / GNSS差分速度 / EKF速度

5. 方位
   EKF yaw / GNSS course / magnetic yaw

6. yaw rate
   IMU / EKF微分 / bicycle model

7. EKF
   covariance / innovation / NIS / reject

8. 制御
   steering command / steering status
   acceleration command / actual acceleration
実装の進め方

おすすめは次の順です。

rosbagへraw GNSS、raw IMU、車輪速、TFを追加
停止・直線・旋回の試験bagを収集
PythonでbagからParquetまたはCSVへ変換
PlotlyなどでHTMLレポートを自動生成
同じbagを使ってLocalizationだけを再実行
delay、IMU bias、速度scale、共分散をA/B比較
最後にコース経路との横偏差を評価

最初からEKFパラメータを手で触るのではなく、まずセンサ品質と遅延を測るのがポイントです。

特に優先度が高いのは、

GNSSの実測遅延
IMU z軸の静止バイアスとTF
車輪速のスケール
各センサのtimestamp整合
GNSS covarianceの妥当性

です。ここを押さえれば、その後のEKF調整はかなり論理的に進められます。