# Requirements

## 背景

`output/20260814-065705`では追い越しを2回開始し、どちらも`ShiftOut -> Pass`へ
到達したが、`Pass -> Return -> Idle`は0回だった。

1回目では同じ側の短期軌道が`prefix=1/1/admitted`となった一方、
`same_admit=0`、`authority=none`のまま古いMissionを継続し、最終的に
`optimized horizon escaped target separation bounds`でFollowPrepareへ落ちた。

同側置換の最小間隔に使っている`mission_planner_generated_at_sec`は、active Missionの
動的予測更新でも毎周期更新される。このため置換後経過時間が増えず、成立した同側
prefixを実行できない。

## 目的

- 同側MPCC-lite prefixを、既存のhard-feasible admissionとtransactional replacementを
  通した上でactive Missionへ反映する。
- 置換クールダウンを予測更新時刻から独立させる。
- rear-clear前にfreshな同側prefixが成立している場合、古いMissionの失効だけで
  FollowPrepareへ落ちないようにする。
- full-track transitionの入口契約では、実際のtransition preflight結果を参照する。

## 制約

- actual wall contact、target discontinuity、非回復body overlap、runtime hard faultは
  緩和しない。
- no-return後およびSafeSeparation中のprefix置換は許可しない。
- topic/service/Domain/提出インターフェースは変更しない。
- MPCC-liteの非同期化は本変更に含めない。

## 完了条件

- 同側のadmitted prefixが`ReplaceActive` authorityを得る単体テストが通る。
- 置換間隔の基準が動的予測生成時刻から独立する。
- full-track transitionは実preflight成立時だけ完全Missionとして扱われる。
- 対象packageのbuild/testが成功する。
