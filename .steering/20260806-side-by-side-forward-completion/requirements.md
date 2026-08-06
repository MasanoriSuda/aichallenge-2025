# Requirements

## 目的

外まくりの Pass が前方車との並走まで進んだ後、Pass horizon の再計画・上限判定で
Recovery と SafetyBrake へ落ち、前方車と壁の間に挟まれる事象を減らす。

## 対象事象

`output/20260806-230417/d1` では、次の二つの失敗が確認された。

- 対象が約 0.37 m 前方、車体非重複、front cap 解放済みの状態から、次周期に
  `Pass horizon hard limit` で Recovery へ落ち、直後に前方距離 0.12 m の
  SafetyBrake となった。
- 対象が約 0.80 m 後方、車体非重複、forward escape 有効の状態でも、Pass 全体の
  32 m 上限が先に成立し、Return 前に Recovery へ落ちた。

## 必須要件

1. front cap 解放済みの minimum-motion Pass で対象が前方 3 m 以内に入り、現在車体が
   非重複なら、同じ横側を保持する前方完遂モードへ早期に入る。
2. 前方完遂中は、予測 overlap だけを理由とした同側横目標の再計画を行わない。
3. 前方完遂中の速度参照は現在速度を下回らず、対象速度へ設定済み closing speed を
   加えた値を使用する。
4. 単発の current footprint overlap は既存確認時間中だけ許容し、確認済み overlap、
   壁接触、EmergencyBrake、target discontinuity、solver recovery は従来どおり停止・
   Recovery を許可する。
5. 前方完遂モードへ既に入っている場合、Pass 全体距離上限で即時中断せず、現在の
   SafeSeparation 局所枠内だけ完遂を継続する。絶対上限到達後の局所枠再延長は禁止する。
6. rear-clear 2 m の連続確認後に Return する。単なる対象中心の後方化だけでは横へ戻らない。
7. ROS 2 topic、service、message、評価 schema、`a_max=1.0 m/s^2` は変更しない。

## 非対象

- 物理壁接触後も前進を強制する処理
- 確認済み車体 overlap の無視
- 車体寸法、wall margin、EmergencyBrake 閾値の緩和
- stuck recovery 全体の再設計

## Definition of Done

- 並走へ入った committed Pass が horizon 再計画より先に前方完遂を選択する。
- 絶対 Pass 距離上限付近でも、rear-clear まで有限の追加距離を走れる。
- 安全条件不成立時は従来どおり fail closed となる。
- pure policy の回帰試験と対象 package の build/test が成功する。
