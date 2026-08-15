# Design

## 入口判定

従来の新規entryは、15 m以内、実相対速度0.3 m/s以上を0.3秒、実行可能Mission、hard guardのANDだった。

次の二経路へ分離する。

1. 従来経路: 実相対速度確認後に実行する。
2. 検証済みMission経路: 現在周期の実行可能経路、body-clear、entry front reserve、計画closingを満たす場合は、実相対速度確認を待たずに実行する。

後者はpre-armの時間切れやcooldownを実行根拠にしない。毎周期の現在Missionとhard guardを根拠にする。

## 初動closing

Mission選択時のentry front reserveは、計画closing speedでbody-clearまでに消費する縦距離を既に含む。これを通過した凍結Missionに対し、設定上のentry距離をもう一度protected distanceとしてadaptive capへ入れると、同じ安全余裕を二重計上する。

検証済みentry reserveを持つ凍結MissionのShiftOutでは、この一段目のadaptive capを省略する。車体が横分離するまでのphysical longitudinal reserve、front-risk、emergency guardは維持する。

## 局所リファクタリング

- 検証済みMissionによる即時entry条件をpure functionへ分離する。
- 凍結Missionがentry closing profileを所有する判定をcontroller helperへ集約し、重複する速度仲裁2箇所で共有する。
