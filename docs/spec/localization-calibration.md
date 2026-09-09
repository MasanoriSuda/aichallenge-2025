# 自己位置入力の校正

2026-09-09確認。使用中のローカルAWSIMに対する校正であり、2026公式仕様や
実車GNSSの精度保証ではない。センサーモデル・ノイズ・遅延・取り付け位置を
変更した場合は再確認する。

## ローカルAWSIMの絶対姿勢と初期方位

`20260909-force-step-observed-single-r1`と`dev2-r1`の同じsource時刻で、
Rigidbody姿勢を実装のROS座標変換へ通し、raw IMU姿勢と比較した。
1124組のbody→IMU回転はyaw **+π/2**、その固定変換との差は最大1.11e-7rad。
従来のsimulation用TFは−π/2で、姿勢をbodyへ戻すとπradの誤りを生む。
simulation用校正を+π/2へ修正し、実車用の校正は別に保持する。

初期方位を近傍経路の接線で置換する処理も、実際の初期車体方位を表していない。
同じ2台走行のD1では、発進中のodometry方位誤差が最大約0.3854radだった。
ローカルAWSIMのGNSSとIMUは20Hzで同じsource stampを持つ（D1 138/138、
D2 143/143組を記録から確認）。`use_imu_orientation=true`ではこの組だけを処理し、
TFでIMU姿勢をGNSSアンテナ座標へ変換してからアンテナ位置をbodyへ戻す。
ゼロ/非有限quaternion、姿勢未提供、時刻不一致、TF欠落ではposeを作らない。
位置medianは同一時刻を保つためbuff_epoch=1を要求する。

simulation起動は`initial_pose_heading_source=measurement`を使い、最初の受信と
`/set_initial_pose`の両方で同じ観測姿勢を初期値にする。生IMU quaternionを
別frameのGNSS姿勢へ代入する旧fallbackは削除する。共分散・EKF gainは変えない。
nodeの明示既定と実車起動は`raceline`を維持する。2026公式環境へ移す際は、
IMU絶対姿勢の提供・基準・取付TF・時刻契約を別途確認する。

根拠と検証状況は`.steering/20260909-mpcc-measured-initial-heading/`。
上記は実装契約であり、統合走行の受入れはその記録が揃うまで未完了。

## 共分散が不明なGNSS

`NavSatFix.position_covariance_type=UNKNOWN`のゼロ配列は精度ゼロを意味しない。
`racing_kart_gnss_poser`の`unknown_position_covariance`で、根拠のある位置分散を
明示する。単位はm²、有限かつ正であることを起動時に検証する。
既知の共分散はこの値で置換しない。

| 呼出元 | UNKNOWN時の分散 | 根拠・扱い |
|---|---|---|
| node / package launch既定 | 10.0m² | 従来の未校正値を維持 |
| `reference.launch.xml`、simulation=true | 0.1m² | 現ローカルAWSIMの明示校正 |
| 同、simulation=false | 10.0m² | 実車の既定を維持。別途センサー校正が必要 |

上位launch引数は`simulation_gnss_unknown_position_covariance`と
`vehicle_gnss_unknown_position_covariance`。既存のEKF追加遅延は両方0秒を維持する。

使用中AWSIMの`StreamingAssets/Vehicle/vehicle.yaml`では、GNSS位置ノイズ標準偏差
1e-9度、遅延0、20Hz。実行ファイルは位置をMGRS float/整数表現から緯度経度へ変換し、
共分散UNKNOWNのまま公開する。0.1m²はその量子化・微小ノイズより十分大きく、
位置差から向きを推定してアンテナのレバーアームを戻す誤差も考慮した保守的な観測分散。
物理的な誤差上限や全シミュレータ共通の値として使用しない。

旧経路はUNKNOWN→GNSSPoserの固定10m²→imu_gnss_poserの品質分類100m²となり、
EKFへ標準偏差10mの観測を渡していた。新経路は明示校正0.1m²が既存の品質分類を
通って0.1m²で届く。EKFのprocess model、gain、gyro、姿勢の重みは変更しない。

## 検証根拠と限界

`20260908-mpcc-stop-reference-single-r3`のsource250.599994秒で、raw UNKNOWN、
EKF入力100m²、GNSSとEKFのglobal Y差約0.50mを確認した。同じ記録済み位置・速度を
同じ初期姿勢から2個のEKFへ入力し、分散だけ100→0.1m²へ変えた比較では、
250.605秒のY推定が43169.6645→43169.1254mとなった。
元のEKF内部履歴は保存されていないため、これは観測分散の寄与の比較であり、
元走行の内部状態を完全再現した試験ではない。

再現手順・証拠は`.steering/20260908-gnss-unknown-covariance/`と
`.steering/20260908-mpcc-single-physical-stop-audit/`。
ROS入出力回帰では、UNKNOWNの明示値・既定値、既知共分散の保存、無効な校正値の拒否を確認する。

校正後の単独診断`20260908-gnss-calibration-single-r1`は6周248.777924秒、penalty0。
記録されたEKF位置入力はすべて0.1m²だった。同じ方法でセンサー時刻へ補間して比較すると、
移動中のXY誤差は旧失敗走行の平均0.202m/p950.353mから、平均0.033m/p950.068mとなった。
異なる閉ループ走行の比較であり、複数反復の安全受入れや独立した真値計測ではない。

同じ失敗走行で、実際の壁メッシュが占有地図の空き領域へ入り、車体メッシュが
MPCCの公称前端を超えることも分かった。校正変更だけで地図・車体モデルの整合や
レース全体の安全受入れが完了したとは扱わない。
