# mc_bridge 定義・実装ステータス（タスクA）

## 目的
mc_bridge の **契約（定義）を先に凍結**し、B/C が並行開発できる状態を作る。
実装は次フェーズで行う。

## 範囲（今フェーズ）
- `docs/contracts/mc_bridge_v0.1.md` の作成
- `/mc/drive_cmd` の出所（telemetry ミラー）を仕様化
- 既存 docs への反映（最小仕様/ROS2 topic spec）

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| docs/contracts/mc_bridge_v0.1.md | 完了 | 2026-01-31 | 定義のみ（実装は次フェーズ） |
| docs/contracts/min_spec_v0_1.md 更新 | 完了 | 2026-01-31 | telemetry ミラー前提を反映 |
| docs/ros2/topic_spec.md 更新 | 完了 | 2026-01-31 | /mc/drive_cmd 出所を明記 |
| rpi/ros2_ws/src/mc_bridge 実装 | 実装済み（未検証） | 2026-02-05 | telemetry decode と /mc/* publish を追加 |
| RunIdCache スレッドセーフ化 | 完了 | 2026-01-31 | MultiThreadedExecutor 対応 |
| demo_period_sec ガード追加 | 完了 | 2026-01-31 | 0 以下の無効値を防止 |

## 残タスク（順序付きTODO）
| 順番 | タスク | 状態 | 期限 | 依存 | 根拠 | 備考 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | mc_bridge 契約の凍結 | 完了 | 2026-01-31 | - | B/C が並行実装できる最小ブロッカー解除 | docs/contracts/mc_bridge_v0.1.md |
| 2 | /mc/drive_cmd 出所の仕様化 | 完了 | 2026-01-31 | 1 | 可視化/デバッグの前提を固定するため | telemetry ミラー前提 |
| 3 | mc_bridge 実装 | 実装済み（未検証） | 2026-02-05 | 1,2 | 仕様凍結後に実装すべき | telemetry decode を追加 |
| 4 | RunIdCache スレッドセーフ化 | 完了 | 2026-01-31 | 3 | Executor 複数スレッド時の競合防止 | Lock 導入 or 単一スレッド前提の明記 |
| 5 | demo_period_sec ガード追加 | 完了 | 2026-01-31 | 3 | 無効値でのタイマー起動を防ぐ | warning を出して停止 |

## 進行メモ
- 2026-01-31: mc_bridge 定義のステータス管理を開始。
- 2026-01-31: mc_bridge v0.1 定義を作成し、/mc/drive_cmd の出所を明記。
- 2026-01-31: mc_bridge の run_id ガード骨組みを追加（/mc/log publish 前のガード）。
- 2026-01-31: RunIdCache に Lock を導入し、run_id の参照/更新をスレッドセーフ化。
- 2026-01-31: demo_period_sec の無効値ガード追加に着手。
- 2026-01-31: demo_period_sec が 0 以下のとき demo_log を無効化するガードを追加。
- 2026-02-01: telemetry.sock を読んで /mc/status /mc/drive_cmd /mc/hils_state /mc/log を publish。
