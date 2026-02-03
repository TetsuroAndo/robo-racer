# telemetry TCP ブリッジ 状態管理

## 目的
- RPi 上の `seriald.telemetry.sock` を TCP 経由で PC へ配信し、
  PC 側の `mc_bridge` が実データを受け取れるようにする。
- 観測専用の安全性（送信遮断）を TCP でも維持する。

## スコープ
- RPi: `seriald` に telemetry TCP 配信を追加（送信不可）
- PC: `mc_bridge` に TCP 入力オプションを追加
- ドキュメント: 運用メモとテスト手順を追加

## 仕様（暫定）
- RPi 側: `seriald` に `--telemetry-tcp-port` を追加
  - 0/未指定: 無効
  - デフォルト: 0（無効）
- TCP: `SOCK_STREAM` で接続を受け、
  - 受信（POLLIN）は即切断（telemetry送信禁止）
  - 送信は telemetry と同じフレームを配信
- PC 側: `mc_bridge` に `telemetry_tcp_host` / `telemetry_tcp_port`
  - 指定時は TCP を優先
  - 未指定時は従来の UDS を使用

## ステータス
| 項目 | 状態 | 期限 | 備考 |
| --- | --- | --- | --- |
| seriald: telemetry TCP 配信追加 | 実装済み（未検証） | 2026-02-03 | 送信遮断・複数接続対応 |
| mc_bridge: TCP 入力追加 | 実装済み（未検証） | 2026-02-03 | UDS fallback を維持 |
| 運用メモ/テスト手順更新 | 未着手 | 2026-02-03 | docs/planning に追記 |

## 進行メモ
- 2026-02-03: 状態管理を開始。
- 2026-02-03: seriald に `--telemetry-tcp-port` を追加。
- 2026-02-03: mc_bridge に `telemetry_tcp_host/port` を追加。
