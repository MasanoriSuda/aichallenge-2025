# 自己位置入力の校正

2026-09-08確認。使用中のローカルAWSIMに対する校正であり、2026公式仕様や
実車GNSSの精度保証ではない。センサーモデル・ノイズ・遅延・取り付け位置を
変更した場合は再確認する。

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
