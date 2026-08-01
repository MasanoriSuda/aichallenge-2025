# 設計

## 責務境界

`mpc_controller_cpp.cpp`は次だけを担当する。

1. ROS/V2X/modelから現在値を収集する。
2. 純粋policyへrequestを渡す。
3. resolutionをFSM stateと制御出力へ反映する。

判定式は`v2x_overtake_core`へ集約する。

## 1. 横経路目標policy

現在は次がcontroller内で別々に組み立てられている。

```text
fixed corridorまたはtarget-relative goal
  -> current wall-feasible interval
  -> optional target separation
  -> execution goal
```

この合成を1つのrequest/resolutionへ抽出する。固定corridorとtarget separationの
現行優先順位は変更しない。今回の目的は、次の性能修正で「固定goalを本当に固定するか」
を1箇所で変更・検証できるようにすることだけである。

## 2. mission lifecycle policy

Recovery完了後に`FollowPrepare`へ戻す条件を純粋関数へ抽出する。
現行のtarget継続、wall contact、solver recovery、forbidden waypoint、rear clear条件を
そのまま保持する。timeout追加やmission破棄条件の変更は行わない。

## 3. committed-pass geometry

behavior側のfront-danger抑制とline側のfront-cap policyで重複している次を共通化する。

- ego/target車体長からの縦方向clearance。
- current body separation後のside-by-side escape geometry。
- predicted footprint overlapの生観測。

timer所有権は従来どおり`OvertakeLineState`に残す。

## 4. closing reserve policy

車体未分離時のprotected distanceとclosing-speed budget計算を抽出する。
既存`resolve_adaptive_shiftout_closing_speed()`を内部利用し、controllerは残横距離と
現在値を渡すだけにする。

## 次の性能修正との境界

今回扱わない項目は次のステアリングで行う。

- `FollowPrepare`の時間・距離上限。
- ShiftOut/Pass/Returnを含む全区間candidate path。
- 採用後のcorridor goal固定方法。
- rear clearance後の同一target再捕捉禁止。
