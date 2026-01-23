# FTG 実装ステート

このファイルは [FTG戦略ドキュメント](ftg_strategy.md) に記載されている安全策・実装項目を、実際のコード変更に合わせて細かく更新するためのステート管理用ドキュメントです。作業中は該当行を編集し、実装状況（未着手・進行中・完了・再検討）とコメントを残してください。

| 項目 | ステータス | 参照 | 備考 / 次のアクション |
| --- | --- | --- | --- |
| Safety Bubble（bubble_radius_mm） | 完了 | docs/planning/ftg_strategy.md#6-plannerftgの内部ステップ安全策込み | `PlannerFTG::applySafetyBubble` で最近傍点周囲を mask する仕様を追加（`rpi/src/planning/PlannerFTG.cpp`） |
| Disparity Extender（disparity_threshold_mm） | 完了 | docs/planning/ftg_strategy.md#6-plannerftgの内部ステップ安全策込み | `PlannerFTG::extendDisparities` で差分閾値超過を安全側に塗りつぶす（`rpi/src/planning/PlannerFTG.cpp`） |
| 前処理の平滑化（移動中央値/平均） | 完了 | docs/planning/ftg_strategy.md#6-plannerftgの内部ステップ安全策込み | `ScanPreprocessor` に移動平均/中央値（`smoothing_mode`）を追加（`rpi/src/perception/ScanPreprocessor.cpp`） |
| 欠測(INF)の扱い | 完了 | docs/planning/ftg_strategy.md#4-データモデル角度ビン距離配列へ正規化 | `ScanPreprocessor`/`PlannerFTG` 側で `planning::kInvalidDistance` を障害物扱いする保守的方針を採用（`rpi/src/perception/ScanPreprocessor.cpp` + `rpi/src/planning/PlannerFTG.cpp`） |
| gap中心寄り補正 | 完了 | docs/planning/ftg_strategy.md#6-plannerftgの内部ステップ安全策込み | `PlannerFTG::clampGapAimPoint` で目標を gap 中心寄りに制限（`rpi/src/planning/PlannerFTG.cpp`） |
| Params 上書き導線 | 完了 | docs/planning/ftg_strategy.md#5-車幅を可変にする核心inflation | `main` で `--vehicle-width` 等を受け取り `Params` に反映（`rpi/src/app/main.cpp`） |
| PlanOutput の distance_mm 活用（ESP32 連携） | 完了 | docs/planning/ftg_strategy.md#3-データモデル | `Sender::send` に distance_mm を渡し `DrivePayload.dist_mm_le` に反映（`rpi/src/comm/Sender.cpp` + `rpi/src/app/lidar_to_esp.cpp`） |
| Safety-focused regression test plan | 実装済 | docs/planning/ftg_tests.md | 単体/結合/回帰の層ごとにケースを記録し、今後テスト実装の進捗をステータス更新予定 |
| Doctest skeleton tests | 完了 | docs/planning/ftg_tests.md#4-テストディレクトリ--cmake構成 | `rpi/tests/` に `doctest` スケルトンと `CMakeLists.txt` を追加し、`test_planner_ftg_stop.cpp` / `test_speed_policy.cpp` を実装・ビルド・ラン済み |
| Integration tests (pipeline) | 完了 | docs/planning/ftg_tests.md#6-結合テスト設計 | パラメータ差分で安全側の速度低下を検証するケースを追加 |
| Golden scan regression fixtures | 完了 | docs/planning/ftg_tests.md#7-回帰テストgolden-scan-fixtures | corridor/chicane を追加し 4 種のフィクスチャに拡張 |
| Sender デバッグログ | 完了 | docs/planning/ftg_strategy.md#6-plannerftgの内部ステップ安全策込み | `Sender::send` に定期ログ出力を追加（`rpi/src/comm/Sender.cpp`） |

## 更新ルール
1. コードの変更を着手した時点で該当行を `進行中` に変更し、着手日・担当（任意）を追記する。
2. 完了時にはステータスを `完了` にし、具体的なファイル/関数名をリンクまたは記載する。
3. 方針決定や議論が必要な項目は `要方針決定` のまま短い議論ログを残す。
4. 追加の安全策（リスク緩和や新しいdocs項目）を後から加える場合、新しい行を末尾に追加。
