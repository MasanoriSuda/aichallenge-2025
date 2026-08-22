# Design

## Causal chain

1. Track/Cruise は legacy MPC が production command を所有する。
2. shadow MPCC certificate は実際の現在姿勢を swept path の先頭へ追加する。
3. `evaluate_clear_footprint_path()` が index 0 を reject する場合がある。
4. controller は index 0 を stageへ写像せず、loop末尾のstage診断を残す。
5. current production pose contact が MPCC stage 19 failure として報告される。

## Change

1. current actual pose を horizon pose loop 前に同じclearance footprintで検査する。
2. current pose の sample unavailable/contact を専用reasonにする。
3. swept path failure index を pure contract helper で `CurrentPose` / `HorizonStage` /
   `Invalid` へ写像する。
4. index 0 または不正indexではstage診断を残さない。
5. current pose reject を1秒telemetryで独立集計する。
6. outcome抑制keyへcertificate reasonを含め、同じreject status内の原因変化も即時記録する。

この変更は同じphysical footprintを同じgridへ照合する順序とprovenanceだけを変更する。
合否、trajectory、command、authorityは変更しない。

## Rejected alternatives

- current poseをswept pathから除外する: 初期接触から候補へ抜ける区間の安全性を失う。
- index 0をstage 0と表示するだけ: production current poseとcandidate stage 0を混同する。
- wall marginを調整する: 原因分類前のパラメータ変更であり採用しない。
