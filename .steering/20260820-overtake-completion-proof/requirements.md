# Requirements

## 背景

2026-08-20 の2走では、Pass中の挟み込み検知と同側横逃げは発動したが、
0.12〜0.37秒後に壁・相手の物理制約が両立不能となりMissionが失効した。
入口で採用された候補には、rear-clear/Returnを後続rolling replanへ委ねる
progressive候補と、短い相手横予測で成立したcomplete Missionが混在している。

## 目的

- 一時的に広い入口より、rear-clearまでの相手・壁余裕を優先する。
- 相手の未観測横移動を有限の不確実性としてPass rolloutへ反映する。
- rear-clear未証明候補が、反対側へ変更できない近距離まで進入することを防ぐ。
- 拒否が「壁」「相手予測」「完遂未証明」のどれかをログだけで判別できるようにする。

## 制約

- ROS 2 topic/service、提出物、Domain契約を変更しない。
- complete Missionと停止・極低速車の明確な通過候補は維持する。
- 全progressive entryを禁止せず、十分な前方距離と再計画時間がある場合は許可する。
- 走行ログを周期ごとに増やさず、採用拒否の状態変化または既存debug周期へ集約する。

## Definition of Done

- future interaction reserveが入口幅より先に左右順位へ反映される。
- Pass rolloutが設定された横不確実性を物理離隔から差し引く。
- 近距離の未完遂progressive候補を純粋関数で拒否できる。
- 拒否理由、front distance、time-to-no-return、予測余裕がログに残る。
- 対象単体テストとパッケージビルドが成功する。
