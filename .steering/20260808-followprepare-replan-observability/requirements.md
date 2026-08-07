# Requirements

## 背景

`20260808-063457`では12回のShiftOutが全てPassへ到達した一方、Return完遂は1回だった。
dynamic Mission waitは8回発生したが、same-side再開とalternate Mission置換はともに0回だった。

待機中のログは一貫して`body_clear=0`、`opp_eval=0`、`opp_reason=body overlap`だった。
FollowPrepareをshadow replan対象へ追加した一方、locked targetの車体・予測離隔はShiftOut/Pass中しか算出しておらず、FollowPrepareでは未計算値がfalseとして扱われていた。

## 目的

- FollowPrepareでも凍結Missionのlocked target幾何を観測できるようにする。
- 反対側Missionを評価できない状況をdynamic waitへ入れない。
- `opp_no_return`を幾何欠損やreplacement回数切れと混同しない。

## 制約

- 車体重複中やno-return後にコース全幅を横断しない。
- actual wall contact、Emergency、solver failureのfail-closed挙動を維持する。
- FollowPrepareをPassとして扱わず、front brakeやPass latchの抑止条件を拡張しない。
- `config.yaml`の既存ユーザー変更（壁余裕0.1 m）を変更しない。

## 完了条件

- paused frozen Missionのlocked targetに対して車体・予測離隔が出力される。
- no-return前かつalternate評価可能な場合だけdynamic waitへ入る。
- no-return後のSafeSeparationは従来のsame-side/RecoverBehind/Recovery判断へ戻る。
- buildと対象パッケージテストが成功する。
