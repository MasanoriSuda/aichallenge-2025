# Design

## 1. Solver failure crawl

MPC が失敗しても、simulation、control enabled、V2X Cruise、前方車なし、正の crawl speed の全条件を満たす場合は fail-operational crawl を選ぶ。速度は設定値と有効速度上限の小さい方とし、操舵は既存の低速横・方位誤差フィードバックを使って基準線へ戻す。

SafetyBrake、Follow、Overtake、LowSpeedAvoidance、前方車検出中、実時間動作では従来の減速 fallback を維持する。

## 2. Stopped solver observation

simulation aggressive recovery 中に solver failure が継続し、実測速度が停止閾値以下の場合、pose correction と waypoint association の離散ジャンプを走行進捗として扱わない。solver fallback 継続時間と停止時間を維持し、既存の 2–3 秒ゲートから bounded recovery へ進める。

## 3. Reverse-first fallback release

coordinated stop は初回 Reverse intent の理由として保持するが、solver 起因の reverse-only episode とは分離する。初回 Reverse 候補不成立で SafeStop に達した後は forward fallback を明示的に解禁する。solver reverse-only は引き続き Forward を禁止する。

## 4. Non-worsening contact escape

RequireImprovement で候補が得られなかった接触状態では、aggressive simulation recovery に限り `AllowNonWorsening` で同じ bounded step を再評価する。新規接触または接触セル増加は引き続き拒否する。

Forward creep は現行 MPC の最大加速度 1.0 m/s2 を使い、速度上限 1.0 m/s、1 step の時間上限 2.0 秒とする。距離・step 数・V2X corridor の既存上限は維持する。
