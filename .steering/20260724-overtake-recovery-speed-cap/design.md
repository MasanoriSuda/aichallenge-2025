# 追い越しRecovery急減速調整 設計

## 原因

OvertakeLineが壁余裕、static wallと横加速度の不整合、またはlocked target実行不能で
Recoveryへ移行すると、専用のreachable hard limitとして3.0 m/sがMPC horizonへ
適用される。ベースラインでは速度指令の負加速度が実速度低下より先に発生し、
実速度8.125 m/sから3.652 m/sまで低下した。一方、対象d2は約5.4 m/sで走行を
継続したため、Recoveryが安全な横復帰だけでなく追い越し機会の消失も引き起こした。

## 最小変更

```yaml
v2x_overtake_recovery_velocity_limit_enabled: true
v2x_overtake_recovery_velocity: 5.0
```

3.0 m/sから5.0 m/sへ上げるが、制限自体は無効化しない。5.0 m/sは今回のd2速度
約5.3〜5.6 m/sをわずかに下回るため、Recovery中の接近を抑えつつ、3.0 m/sまでの
大きな失速を避けるための保守的な第一候補とする。

## 分離方針

この実験ではRecoveryへ落ちる上流条件を変更しない。したがって期待する効果は
「追い越し成功率の即時改善」ではなく、まず次の因果だけである。

1. Recovery開始
2. reachable velocity limitが5.0 m/sへ収束
3. d1がd2より大幅に遅くなる時間と車間拡大量が減る

Recovery回数が同じでも急減速だけが改善すれば、直接原因への効果は成立とする。
その後に限り、target実行不能、wall abort、再試行cooldownを別作業で扱う。

## 効果確認

1. 起動ログで`recovery_v=5.00`またはRecovery debugの`v_limit=5.00`を確認する。
2. d1/d2のMCAPから以下を時刻同期する。
   - `/control/command/control_cmd`
   - `/localization/kinematic_state`
   - `/localization/acceleration`
   - `/v2x/vehicle_positions`
   - `/clock`
3. Recovery開始ごとに、開始前0.1秒から開始後2秒の最低速度、最低加速度、
   V2X車間を抽出する。
4. ベースラインの代表値8.125 -> 3.652 m/s、最小-2.484 m/s2と比較する。
5. Autowareログからphase遷移、abort理由、wall/contact/stuckを集計する。

## 採用判定

- 採用候補:
  - Recovery中の3 m/s方向への収束が消える。
  - d1がd2より遅い時間または車間拡大量が明確に減る。
  - wall/contact/stuckが増えない。
- 不採用:
  - 壁接触、SafetyStop、Stuck Recovery実行が新規発生する。
  - Recovery中の横復帰が完了しない。
  - 急減速または追い越し再開性が改善しない。

## インターフェース影響

参加者parameterの値だけを変更する。ROS 2 topic/service/message、launch entry、
Domain、評価結果schemaへの影響はない。
