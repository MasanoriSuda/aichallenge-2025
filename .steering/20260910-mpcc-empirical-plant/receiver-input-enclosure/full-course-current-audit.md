# Current full-course failure and exact replay

2026-09-13. HEAD521b80b9/control7ecd0b36, build142/package129.

single-r30は実送信8440の期限超過で中止。正常送信2420件とは別に失敗した通常送信1件を保持。
元の入力・履歴・証明経路をR836で一致再生し、送信前許可→raw診断終了時点/送信後拒否を確認。
8784callback中644の1件25.030ms、連続超過なし。車両Start/Rejoin復帰1回、周回は未達。
壁区間1263は全地図・元クエリで完全一致。現在は非接触だが、1mm上側の安全余裕込み車体は
壁セル466554に接するため、既存境界ガードの拒否は正当。壁条件を緩めない。
原DLLの標準60FPS制限はbatchmode/nographicsでも解除される。r27は描画だけの比較ではなかった。
次は描画を保つ--target-fps 200だけのsingle-r31（120host秒）で制限の影響を切り分ける。
物理・時計刻み・制御・25ms期限とガードは維持。時計問題・全コース/M4–M6は未完。

Actual8440: nominal218.419995132, decision218.424995117, before/guard_end
218.444995117, raw_end/after218.449995117, deadline218.444995132.
There were only15ns of observed ROS-time headroom at the pre-send guard.
The actual route was OutsideDomain(5), followed by an accepted independent
CurrentPhysicalProof. The serialized domain is the offered parent. R836 keeps
exact source/current/history/wire/epoch and reproduces the original guards.
R834 incorrectly required the CurrentDomainProof route; its failed diagnostic
is retained. This is not a new physical failure. An extra check after raw would
catch8440 but not earlier r26/2180, whose raw_end remained inside the window.
No complete clock-race repair is claimed from another check or5ms allowance.

Interval1263 at21.194999526: preferred lateral0.8337737291834197,
clear run[-3.4949261567311485,0.8337737291834197], guarded upper bound
0.8327737291834197. All146 sampled poses, selected interval and full map
fingerprint1403875087638600848 reproduce exactly. Original temporal and
reconstructed preferred poses agree and are clear. The +1mm anchor has no
physical body contact but its original normal expanded footprint hits occupied
cell466554. Preserve the1mm boundary guard and0.2m normal lateral clearance.
Retire the temporary observer after the one-factor r31 comparison, with an
exact-boundary regression against the same map fixture. No interval repair.

Original DLL hash703e18fad4e3cf68111a559190edb7060e901988a04c409d84c80331dd45a172
is unchanged. Existing sealed R311 CIL and R312 method equivalence retain its
provenance. RuntimePerfMetrics.Awake passes60 to ParseTargetFps;
ApplyCap sets targetFrameRate=-1 for batchmode or Null graphics. Positive
graphical caps set vSyncCount=0 and targetFrameRate to the requested value.
The supported --target-fps option changes rendering scheduling, with no
fixedDeltaTime change in these methods. This revises attribution of r27, whose
flags changed both rendering and cap. It does not establish an upper bound on
future clock bursts or publication execution time.

Runtime stopped and protected artifacts restored. Every post-failure send is
retained; monitor10hostsec polling is not instantaneous. Full-course acceptance
failed before the intended600sim/660host bound. [Evidence](full-course-current-audit-evidence.json).
