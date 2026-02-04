# PR26-30 統合後レビュー修正ステータス

## 目的
PR26-30 統合後のレビュー指摘（スレッド安全性、互換パス、テレメトリー配信、安全対策）を整理し、実装修正の進捗を追跡する。

## ステータス凡例
- 未着手
- 進行中
- 完了
- 再検討

## 対象項目
- [完了] TelemetryEmitter の status_ 競合対策（ロック/スナップショット化）
- [完了] Process::proc 内の static 状態をメンバーに移動
- [完了] seriald TCP クライアントの non-blocking 設定
- [完了] seriald IPC/TCP 送信の部分送信/エラー処理
- [完了] seriald 互換ソケットパス（/run/roboracer）修正
- [完了] lidar_to_esp / main の scan_id 取り扱い修正
- [完了] telemetry_hz の入力検証
- [完了] Telemetry.cpp の数値パース例外処理
- [完了] ros2-mc-bridge の HOST 取り扱い安全化
- [完了] /tmp ソケット接続前の権限チェック（serialctl / seriald_client）

## メモ
- 仕様確認: docs/interfaces/ipc_topics_v1.md, docs/interfaces/ipc_payloads_v1.md, docs/ros2/topic_spec.md
