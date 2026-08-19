# Requirements

## 目的

新規追い越し開始時の実行権限を整理し、ローカルの完遂証明ゲートが拒否した
progressive entryを後段のMPCC authorityまたはcached prefix leaseが再採用する
迂回経路をなくす。

## 背景

`output/20260820-074643/d1/autoware.log`では、車間4〜6 mで
`Overtake completion-proof gate rejected local-only entry`が出た直後に、
`MPCC-lite fresh progressive prefix lease selected for entry`から追い越しを開始していた。
その後、Pass entry physical gate、壁交差、SafetyBrakeなどで完遂できていない。

## 変更範囲

- `multi_purpose_mpc_ros`の追い越しentry admission
- progressive prefixの純粋関数と単体テスト
- entry採用元、完遂証明結果、拒否理由を示すログ

## 制約

- ROS topic、message、service、launch、評価インターフェースを変更しない。
- complete Missionは従来どおり完遂証明ゲートの対象外とする。
- 実行中Missionの同側receding prefixは新規entryゲートの対象外とする。
- パラメータ値、MPC数式、Recovery仕様は変更しない。
- ユーザー所有の走行結果とcrash blobは変更しない。

## Definition of Done

- MPCCの新規progressive entryが共通の完遂証明を通過しない限り実行権を得ない。
- dual MPCC authorityとcached prefix leaseの両経路を回帰テストまたは共通純粋関数で保護する。
- ログからentry source、admission結果、completion-proof理由を識別できる。
- 対象単体テストと`make autoware-build`が成功する。
