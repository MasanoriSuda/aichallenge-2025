# Results

## 結論

`output/20260718-174716` で再現した「最後尾D2が前方D1を後退障害物と誤認し、全車が
step 0のまま停止する」V2Xデッドロックは解消した。`make dev3` の確認run
`output/20260718-181438`では、D2が9 step、合計2.055 m後退してLowSpeedRejoinへ移り、
`rejoin_complete`で通常制御へ復帰した。その後D1もcoordinated recoveryを開始し、
Reverseへshiftして実際に後退した。

このrunのレース全体は完走していない。D3は静的mapの接触セルが30個以上あり全候補が
`contact_worsened`、D2は一度復帰した後に通常MPCで `e_y=6.412 m`まで逸脱し、いずれも
`maneuver_direction_unknown`でSafeStopした。D1も移動中のD2を従来moving corridorで検出し、
0.058 m後退後に `escape_not_confirmed`でSafeStopした。これらは今回の停止車rollout判定とは
別の課題として扱う。

## 実装確認

- 停止車には選択済みrollout全姿勢と向き付き自車footprint・膨張他車円のsigned clearanceを使う。
- 初期重なりから離れる操作だけを許可し、悪化または新規重なりを拒否する。
- 移動車には従来の予測moving corridorを維持する。
- blocker ID、判定方式、reject理由、初期・最小・最終clearance、reject距離を状態ログへ出す。
- D1では `d3 / rollout_separation / initial_overlap_worsened` を検出し、旋回候補を拒否した後、
  clearな `reverse_straight`へ再選択できた。
- D1後退中に動いていたD2は `d2 / moving_corridor / corridor_overlap` として従来どおり停止対象になった。

## 検証結果

- `make autoware-build`: 成功、25 package。
- `test_recovery_footprint`: 31 / 31成功。
- `output/20260718-174716`相当のD2配置を使うRight rollout単体テスト: 成功。
- `make dev3`: 対象V2Xデッドロック解除は成功、レース全体の継続走行は上記別障害により不成立。
- 実験後に `make down` を実行し、3台とAWSIMコンテナを停止した。

`colcon test-result --verbose` には今回と無関係な既存結果として、`test_path_core` の
`RemovesOneEndpointFromConfiguredFinalVer3Trajectory`失敗と
`joycon_contract_guard/package.xml`欠損が残る。今回追加した対象test自体は個別実行で全件成功した。
