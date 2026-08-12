# Requirements

## 目的

直近走行`output/20260812-224210`では、7回の追い越し開始に対して
`Pass -> Return`完遂は4回で、3回の壁余裕違反と接触後の低速固定が大きな
ラップロスになった。既存のdynamic Mission waitはMission失敗後に動作するため、
actual footprintの壁余裕違反には間に合わない。

また、Mission候補選択は1候補の数値異常をselector全体の異常として扱い、同じ
探索集合に残る実行可能候補まで失う。この二つを局所修正し、hard guardを弱めずに
実行可能な追い越し枝を保持する。

## 要件

- Mission候補の数値異常は候補単位で棄却する。
- selector request自体の異常は従来どおり`valid=false`とする。
- 不正候補と正常候補が混在する場合は正常候補を選択できる。
- `v2x_overtake_line_min_wall_clearance: 0.15`を維持する。
- actual footprintへhard wall marginに加えて0.10 mの予告帯を評価する。
- 予告帯だけが壁へ触れた時点ではRecoveryへ入れない。
- 現在位置から完全検証済みのfresh same-side Missionがあればatomicに置換する。
- fresh same-side Missionがなければ、次の候補評価を即時要求し、現Missionを維持する。
- actual wall contact、0.15 m wall margin違反、sample unavailableは従来どおりhard faultとする。
- runtime wall replanはMissionあたり2回、0.50秒cooldownでboundedにする。
- 同じ反対側候補が同じ横goalで棄却された場合は0.25秒commitを再試行しない。
- ROS topic/service、評価schema、車体寸法、速度・制動上限を変更しない。
- `config.yaml`と`config_for_cloud.yaml`を同値にする。

## Definition of Done

- candidate-local fault isolationの単体テストがある。
- runtime wall preplan admissionの単体テストがある。
- cross-side rejection retry throttleの単体テストがある。
- hard wall faultの既存単体テストが成功する。
- `multi_purpose_mpc_ros`の単体テストとbuildが成功する。
- 動的確認ではwall preplan request/replacement、wall Recovery、Pass完遂率を確認できる。
