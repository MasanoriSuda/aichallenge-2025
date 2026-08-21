# Design

## 原因

DynamicEscape の新解は solver と接続経路の検証後にも、実行時壁ゲートで2回連続の再認定を要求されていた。その最初の周期に速度を落とし、MPC warm start を一定速度・一定操舵列で上書きしていたため、次周期の候補が再び作り直されていた。

候補が一時的に消えた場合は直前の安全解を最大0.35秒保持していたが、毎周期その解の先頭指令を再生していた。経路と指令の時刻がずれ、保持中にも horizon を平坦化するため、last-feasible の意味が失われていた。

## 変更方針

1. live DynamicEscape candidate は、solver前後検証と同周期の物理壁検証を通過したら1回で昇格する。通常経路へ戻す exit handoff は従来どおり連続2回を要求する。
2. retained execution は経過時間と `model->Ts` から stage cursor を算出する。指令は cursor の stage、warm start はその stage 以降を前詰めし、末尾を終端値で埋める。
3. retained execution を出力している周期は、wall hold の一定値同期で MPC horizon を破壊しない。
4. retained solution は attempt、target、side が exit contract と一致する場合だけ利用する。
5. handoffログへ `incoming_admitted/promoted`、`published_source`、`retained_stage/remaining` を追加する。

## 安全境界

- 保持時間上限0.35秒は変更しない。
- stage cursor が制御horizon外へ出た解は期限内でも失効させる。
- 現在footprint不正、予測接触、予測out-of-map、必要壁余裕未達は従来どおりhold/replan対象とする。
