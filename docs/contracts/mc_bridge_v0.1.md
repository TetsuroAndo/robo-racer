# mc_bridge 契約 v0.1（定義のみ）

## 目的
`seriald` の telemetry から受け取った mc_proto を ROS2 topic に変換し、
B/C が **実機なしでも ROS2 上で可視化/検証**できる状態を作る。
実装は次フェーズとし、**定義（契約）だけを先に凍結**する。

---

## 入力

### ソケット
- `seriald.telemetry.sock`（観測専用）
  - 既定: `/tmp/roboracer/seriald.telemetry.sock`

### フレーム種別
- UART 受信フレーム（ESP32→RPi）
- **RPi→ESP32 送信フレームのミラー**
  - DRIVE / MODE_SET / KILL 等
  - `/mc/drive_cmd` 可視化のため **必須**

---

## 出力（ROS2 topic）

### 必須
- `/mc/status`（`mc_msgs/Status`）
  - `mc::proto::StatusPayload` に対応
- `/mc/hils_state`（`mc_msgs/HilsState`）
  - `mc::proto::HilsStatePayload` に対応
- `/mc/log`（`mc_msgs/LogRecord`）
  - `mc::proto::LogPayload` の要約
- `/mc/drive_cmd`（`mc_msgs/DriveCmd`）
  - 送信ミラーされた `DrivePayload` を可視化

### 任意（デバッグ）
- `/mc/ack`（`mc_msgs/LogRecord` で代用可）

---

## QoS（最低限）
- `/mc/status` `/mc/hils_state` `/mc/drive_cmd`: `reliable`
- `/mc/log`: `best_effort`（ドロップ可、rate limit あり）

---

## タイムスタンプ/メタデータ
- `header.stamp`: **受信時刻**を付与（mc_protoに時刻が無い前提）
- `run_id`: 環境変数 or `/mc/run_id` を受け取り、`LogRecord.run_id` に **必ず** 付与
  - `run_id` が取得できない場合は `/mc/log` を **発行しない**（完全トレーサビリティ優先）
- `seq`: mc_proto の `seq` を各 message に含める

---

## 例外処理（最小）
- COBS/CRC decode 失敗 → `/mc/log` に WARN（`event=decode_error`）
- seq 飛び → `/mc/log` に WARN（`event=seq_gap`）
- 受信ゼロ継続 → `/mc/log` に INFO（`event=no_frame`）

---

## 受け入れ基準（定義）
- B/C が **質問なしで mc_bridge 実装を開始**できる具体度
- `/mc/drive_cmd` の出所（ミラー）と目的が明確
- エラー時の出力先が `/mc/log` に統一される
