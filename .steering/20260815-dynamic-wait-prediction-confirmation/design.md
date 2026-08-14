# Design

## 原因

直近走行ではDynamicMissionWaitのfull closingが2.0 m/sと0.5 m/sの間で反転し、
SafetyBrake仲裁も同時にON/OFFしていた。判定が生のpredicted footprint sweep 1サンプルと
直前周期の`full_closing`状態へ依存していたためである。

また、fresh same-side Passへ再開してもsame-target Missionの15秒時計は維持されるため、
長いDynamicMissionWait後にはrear-clear予測を実行する時間が残らずRecoveryへ落ちていた。

## 変更

### 1. 予測重複の共有確認

既存の`v2x_overtake_pass_predicted_overlap_confirm_sec`（現設定0.25秒）を再利用する。
DynamicMissionWait中にcurrent body separation、target continuity、prediction validityが成立する間、
predicted overlapの連続時間を専用stateで保持する。

- sweep clear: 即時にpath acceptable
- overlap 0.25秒未満: transientとしてpath acceptable
- overlap 0.25秒以上: path unacceptable、bounded closingへ移行
- current overlap / prediction invalid / target jump: 即時fail-closed

確認結果はforward prefix、Behavior SafetyBrake仲裁、same-side Pass handoffで共有する。
下流仲裁が生の1サンプルを再評価して共有結果を上書きしないようにする。

### 2. Forward authorityの分離

handoff条件を「直前周期がfull closing」から「直前prefixが壁検証済み」へ変更する。
予測path acceptableは現在周期の共有確認結果で別途評価する。これにより、confirmed overlap中は
bounded closingを維持し、clearへ戻れば同じ壁検証済みprefix上で直ちにfull closingへ戻せる。

### 3. Mission期限の限定延長

fresh same-side Pass continuationがrear-clear prediction feasibleの場合だけ、残りMission時間と
`predicted_rear_clear_time + completion reserve`を比較する。不足分だけ累積extensionへ追加する。

- Mission start時刻は変更しない。
- 15秒設定では累積延長上限を3.75秒とする。
- completion reserveは`clear_confirm + prediction delay`、最低0.25秒とする。
- cross-side replacement、ShiftOut再開、予測不成立では延長しない。

MPCC-lite shadowの残時間とruntime budgetは同じeffective limitを参照する。

## 動的確認点

- `dynamic Mission forward prefix active`の`full=1/0`が単発周期で反転しない。
- DynamicMissionWait中の`front_danger_suppress=1`が維持される。
- same-side replacementで`front_cap_handoff=1`となる。
- 必要時のみ`deadline_extend=x/3.75`が0より大きくなる。
- `Pass -> Return`が発生し、Mission budget Recoveryが減る。
