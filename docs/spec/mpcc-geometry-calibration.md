# ローカルAWSIMのMPCC形状校正

2026-09-08確認。使用中のローカルAWSIM assetに対する校正であり、
2026公式仕様や実車寸法を確定するものではない。

## 自車の公称車体

有効な非triggerのconvex MeshCollider `GoKart1/Colliders/Collider` を
実際のTransform階層から`base_link`へ変換した。平面投影の外形は次のとおり。

| 向き | assetの最大範囲 | 採用する公称範囲 |
|---|---:|---:|
| 前 | 1.614851201m | 1.615m |
| 後 | 0.378506349m | 0.510m |
| 左 | 0.767926137m | 0.768m |
| 右 | 0.767158703m | 0.768m |

旧前端1.49mは既存margin0.05mを加えても実形状を包まない。
公称値を外側1mm単位へ丸め、既存の後端とmarginは維持する。
`config.yaml`、`config_for_cloud.yaml`、C++既定値へ反映する。
`recovery_footprint_`はRecoveryだけでなく通常MPCCの壁proofにも供給される。
局所tracking reserve0.20mは変更しない。

[独立fixture](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/fixtures/awsim_body_projection.json)
に33頂点の投影凸包と元assetのSHA-256を保存した。両設定がmarginを消費せず
全頂点を含むことを回帰testで確認する。これは公称平面車体の確認であり、
車体のroll/pitch、姿勢推定誤差、PhysX contact skinを測定したものではない。

## 自他車の横寸法の分離

`mpc.v2x_vehicle_radius`は相手単体の半径ではなく、既存plannerが使う
自他車の合計横距離である。両車の半幅0.768mから1.536mとする。
旧実装では1.45mを維持したまま自車を広げると、相手半径の引き算で
相手側が0.682mへ縮んでいた。plannerの横寸法は独立した回帰testで確認する。
prediction marginと位置不確実性は公称寸法と分け、変更しない。

**この横寸法修正だけでは相手車体の前後端を円形モデルで包めない。**
V2X messageに姿勢はなく、現在のphysical worldは相手を円で表現する。
ローカルassetと配信コードを確認したところ、V2Xの基準点はGNSSアンテナであり、
4台共通の全車体を同点から包む球の半径は1.875424947m（外側丸め1.876m）となる。
修正前の円は公称0.768mに不確実性を足したもので、車体投影が重なる例に対して
実際のC++判定が正の0.822m余裕を返す。この例は平面形状の反例であり、
3Dエンジンの衝突や他guardも含む指令採用を実証したものではない。
[相手形状監査](../../.steering/20260908-peer-envelope-audit/design.md)に根拠を保存する。
相手全形状とRecoveryの半径を一貫して扱う修正は以下のとおり。
多車両の安全受入れを横寸法testだけで完了扱いにしない。

2026-09-09の修正では、公称全車体を`mpc.v2x_peer_body_radius_m: 1.876`へ
分離し、通常の現在world／保存world／Recoveryで共有する。
`mpc.v2x_vehicle_radius`は合計横距離として維持する。
旧`stuck_recovery.rear_safety.vehicle_radius_m`は廃止し、残る設定は明示的に
読込エラーとする。移行先は上記の共通公称半径。予測marginと位置不確実性は
別に加算し、自車幅の引き算で相手形状を決めない。
V2X topic・message・Domainと評価schemaは変えない。
現在worldと保存worldには共通producerで公称半径・予測margin・位置不確実性を
供給する。Recoveryも同じ公称値を使い、既存の位置不確実性のみを加える。
[独立3D凸包fixture](../../aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/fixtures/awsim_peer_body_envelope.json)
は705頂点から202頂点へ縮約し、4台のGNSS基準点に対する包絡を確認する。
多車両受入れはまだ未達。

## 壁マップ

校正前の`env/final_ver3/occupancy_grid_map.pgm`には実際のcollision meshの未被覆があった。
失敗run `20260908-mpcc-stop-reference-single-r3`の接触候補位置
MGRS(89668.655266,43168.549964)がfree cellにあった。
native C++はYAML原点をセル中心として使うため、この規約での最寄り
occupied cell中心まで0.599555m、境界まで0.548311mとなる。
corner原点として計算した当初の約0.555m中心距離は訂正する。

現在は実壁の近鉛直面（非退化、`abs(normal.z)<=0.5`）を追加投影し、
元の占有セルを全て保って4,567セルを追加した。セルと投影三角形の交差を
native座標規約で求める。全高さの投影なので過剰占有になり得る。
選択外の傾斜面や他のcolliderを含む完全な3D世界の被覆は主張しない。

地図YAMLの`preserve_occupied_cells: true`により、形状から得た占有を
経路・回廊側の小島ノイズ除去で消さない。最終物理検証は元画像を直接参照するため、
小島除去の修正は候補生成側との不一致を解消する。本番C++と経路検証ツールの
全570,009セルで一致し、
元経路は公称車体と0.25m余裕を含めた連続掃引検証を通過した。
キー省略または`false`の旧地図は既存の小島除去を維持する。
受け付ける値は小文字`true/false`。他の値は読込エラーにする。

生成手順・assetハッシュ・修正前失敗・検証結果は
[校正steering](../../.steering/20260908-physical-wall-map-calibration/requirements.md)と
[生成根拠](../../.steering/20260908-physical-wall-map-calibration/map-provenance.json)に保存する。
新しいmap/binaryで6周253.199秒、penalty0を記録したが、走行中のEmergency10699/10700で
統合受入れは不合格。遅延予測区間の壁条件との干渉を監査中。旧mapの成功を引き継がない。

GNSS共分散の校正と位置差の測定は[自己位置校正](localization-calibration.md)を参照。
局所build/testや単発完走は、固定条件での反復・多車両・提出物評価の代わりにはならない。
