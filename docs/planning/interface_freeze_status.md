# インターフェース凍結・実装ステータス（担当A）

## 目的
RPi↔ESP32の既存プロトコル/安全設計を契約として凍結し、B/Cが車体なしで並行開発できる状態を作る。

## 範囲（Day1成果物）
- v1インターフェース文書の整備（RPi↔ESP32 / IPC / Logging / HILS）
- `shared/proto` にIPC用メッセージ型（v1）を追加
- RPi側 Logger API の一本化（mc_core へ統合）

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| docs/interfaces/protocol_rpi_esp32_v1.md | 完了 | 2026-01-27 | 既存仕様の凍結版作成 |
| docs/interfaces/ipc_topics_v1.md | 完了 | 2026-01-27 | IPCトピック定義の凍結 |
| docs/observability/logging_v1.md | 完了 | 2026-01-27 | Logger/LogRecord契約化 |
| docs/testing/hils_v1.md | 完了 | 2026-01-27 | HILS v1の要件整理 |
| shared/proto: IPCメッセージ型追加 | 完了 | 2026-01-27 | v1型の追加・固定 |
| RPi Logger API 統一 | 完了 | 2026-01-27 | serialdをmc_coreへ |

## 残タスク（順序付きTODO）
| 順番 | タスク | 状態 | 期限 | 依存 | 根拠 | 備考 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | sim_esp32d（HILS代替）実装 | 未着手 | 2026-01-30 | - | 机上E2Eとfault injectionの前提になるため先行 | DRIVE→STATUS/TTL/KILL/AUTO再現 |
| 2 | proto/ログ単体テスト拡張 | 未着手 | 2026-01-31 | - | 仕様固定の回帰防止を早期に確保するため | LogRecord往復/境界条件 |
| 3 | 統合テスト（seriald + sim_esp32d + racerd） | 未着手 | 2026-02-01 | 1 | sim_esp32d完成後に机上E2Eを成立させるため | 最小E2E |
| 4 | HILS fault injection（遅延/欠損/CRC不正） | 未着手 | 2026-02-02 | 1 | sim_esp32dがあれば再現可能、早期に安全検証 | sim_esp32dで再現 |
| 5 | ローカル実行環境（docker or make target） | 未着手 | 2026-02-02 | 1 | 物理アクセス不要の開発を確立するため | 机上実行の導線 |
| 6 | Metricsログ（CPU/温度/メモリ） | 未着手 | 2026-02-03 | - | 後続の閾値設定の前提となるため | 低頻度で記録 |
| 7 | CI整備（unit + integration） | 未着手 | 2026-02-04 | 2,3 | unit/integrationが揃った後に常時実行へ | 常時実行 |
| 8 | systemd運用・ログローテ手順 | 未着手 | 2026-02-05 | 5 | 実行環境が固まってから運用手順を確定 | v1は外部ローテ前提 |
| 9 | 性能監視の閾値設定（温度/CPU） | 未着手 | 2026-02-05 | 6 | メトリクス取得後に閾値を決めるため | アラート条件 |

## 進行メモ
- 2026-01-27: ステータス管理開始。v1ドキュメント作成完了。
- 2026-01-27: IPCメッセージ型追加とLogger統一まで完了。
- 2026-01-27: 残タスクの順序付きTODOを定義。
