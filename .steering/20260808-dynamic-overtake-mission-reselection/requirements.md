# Requirements

## 目的

追い越し開始時に固定した側が、相手の移動やコース曲率反転によって実行不能になった場合に、
同一 target の Mission を直ちに破棄せず、現側・反対側・追従待機を動的に再選択する。

## 背景

`output/20260807-234257/d1` では9回のPassに対してReturn完遂は2回、7回がRecoveryとなった。
失敗7回中6回はPass horizon / footprint予測 / SafeSeparation系で、実壁余裕違反は1回だった。
既存の反対側shadow評価はShiftOut/Passかつtarget前方3.5 m以上に限定され、
`FollowPrepare`では同じ側しか再検証しない。このため、現在側が塞がると反対側の好機を待たず、
同じ側へ即再開するかRecoveryへ落ちる。

## 要件

- frozen Mission中は5〜10 Hz相当で左右の完全Mission候補を再評価する。
- `FollowPrepare`も動的再選択の対象に含める。
- no-return前で反対側候補が連続して成立した場合だけ、Mission全体を原子的に置換する。
- no-return後は反対側へ横断しない。
- 現側が不成立で反対側も未成立の場合、hard faultでなければ`FollowPrepare`でtargetを保持する。
- 待機中は両側を再評価し、現側復旧なら同側再開、反対側成立なら置換して再開する。
- 通常の候補不成立と、壁接触・実車体重複・Emergency・solver failureを分離する。
- 横並び後の無理なside flipは許可しない。
- 既存の壁、車体、横加速度、Return成立性のhard gateは緩和しない。
- 現在のユーザー変更である壁余裕0.10 mは変更しない。

## 変更範囲

- `multi_purpose_mpc_ros`のV2X Behavior出力、OvertakeLine状態、反対側再計画接続
- SafeSeparation / Pass horizonのsoft failure遷移
- pure core policyと単体テスト
- `docs/spec/mpc-integration.md`

## 非対象

- フルMPCC、複数lateral knotの同時最適化
- V2X予測モデルの変更
- 加速度、最高速度、制動、壁余裕の調整
- actual wall contact、Emergency、solver failure時のRecovery緩和
