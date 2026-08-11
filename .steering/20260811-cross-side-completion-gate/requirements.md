# Requirements

## Objective

Pass実行中の反対side置換が、全幅切り返し後に未分離速度capへ落ち、追い越しを失敗させる経路を閉じる。

## Evidence

- `output/20260811-100221/d1/autoware.log`
- wp127で`opp_no_return=1`の後、失速によりtargetが再び前方へ離れた。
- wp130でPass進捗15.87 m後にsideが`-1 -> 1`へ置換された。
- 置換後はPassのままfront-cap latchを解除し、`closing=0.50 m/s`と実速約1.389 m/sへ落ちた。
- active Pass 10.04 sでabsolute time limitに到達した。

## Required behavior

- 一度side-by-side/no-returnに入ったMissionは、targetが再び前方へ離れても反対sideを再許可しない。
- SafeSeparation中の反対side全幅切り返しを禁止する。
- no-return前の反対side置換は、rear-clearの時間・距離予算と速度条件を満たす完全candidateに限る。
- Pass初期に許可した反対side置換はShiftOutとして実行し、candidateのclosing-speed rolloutとruntime policyを一致させる。
- 同側の動的refresh、hard fault、rear-clear後のReturnを壊さない。

## Constraints

- ROS topic/service/interfaceは変更しない。
- パラメータの単純な攻撃化は行わない。
- 既存のtransactional replacementとrollbackを維持する。
- ユーザーの既存変更は巻き戻さない。
