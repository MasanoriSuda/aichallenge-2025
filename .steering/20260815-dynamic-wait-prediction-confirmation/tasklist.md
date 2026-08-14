# Tasklist

- [x] 直近走行のfull closing / SafetyBrake / Mission budgetを照合
- [x] DynamicMissionWait専用predicted-overlap確認clockを追加
- [x] forward prefix、Behavior仲裁、Pass handoffへ確認結果を共有
- [x] current overlap・壁・予測欠損のfail-closedを維持
- [x] fresh same-side Pass用の累積上限付きMission期限延長を追加
- [x] MPCC-lite shadowとruntime budgetのeffective limitを統一
- [x] pure core単体テストを追加
- [x] `make autoware-build`成功
- [x] package test 25/25成功（集計1148 tests / 0 failures）
- [ ] `make dev2`で動的効果確認（ユーザー実施）

## 動的合格条件

- 単発predicted overlapで`full=1/0`とSafetyBrakeがチャタリングしない。
- 連続0.25秒以上のpredicted overlapではbounded closingへ落ちる。
- fresh same-side Pass再開直後に0.5 m/sへ不要に戻らない。
- `Pass -> Return -> Idle`が少なくとも1回成立する。
- current overlap、壁接触、prediction invalid時はauthorityを維持しない。
