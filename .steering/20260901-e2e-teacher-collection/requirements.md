# E2E Teacher Collection Requirements

## Objective

TinyLidarNetと同一のLiDAR・コース・開始条件で、MPCの正解commandをrun単位で記録する。
提出entryのE2E既定値やruntime入力契約は変更しない。

## Constraints

- `aichallenge_submit.launch.xml`の既定controllerは`tiny_lidar_net`のまま維持する。
- student入力は750点LiDARだけとし、GNSS/IMUをmodelへ追加しない。
- GNSS/IMUはMPC教師とAWSIM handshake/localizationのためだけに有効化する。
- 教師runは1台、NPCなし、固定start、3周、LiDAR onとする。
- controller選択をファイル差し替えや一時編集に依存させない。

## Acceptance

- `make e2e-teacher`でMPCを明示選択する。
- `make e2e-single`はTinyLidarNetを明示選択する。
- system launchからsubmit launchまで`control_method`が一意に伝播する。
- 教師bagにLiDARと最終control commandが記録される。
- 教師runが3周完了し、dataset extractorで`label_source=mpc`として抽出できる。
