# ROS Topics 契約 v0.1（凍結）

## 目的
B/C が実機なしで ROS2 側の実装を並行できるよう、topic/型/frame_id を最小で凍結する。

## フレーム（凍結）
- `base_link`
- `laser`
- `map/odom` は次フェーズで追加・凍結

## Topics（凍結）

### /scan
- 型: `sensor_msgs/msg/LaserScan`
- `header.frame_id`: **`laser` 固定**
- `angle_min/max`, `angle_increment`, `range_min/max` を必ず設定

### /tf_static
- 型: `tf2_msgs/msg/TFMessage`
- `base_link -> laser` の static transform を publish
- 変換値は `rpi/config/frames.yaml` で管理

### /mc/status
- 型: `mc_msgs/msg/Status`
- `mc::proto::StatusPayload` と意味一致

### /mc/drive_cmd
- 型: `mc_msgs/msg/DriveCmd`
- `mc::proto::DrivePayload` と意味一致

### /mc/log
- 型: `mc_msgs/msg/LogRecord`
- `run_id` は必須（`/mc/run_id` と一致）

### /mc/hils_state（推奨）
- 型: `mc_msgs/msg/HilsState`
- `mc::proto::HilsStatePayload` と意味一致

### /mc/run_id（運用補助）
- 型: `std_msgs/msg/String`
- QoS: `transient_local` 推奨
- bag/ログの run_id と一致させる

### /mc/bridge_diag（任意）
- 型: `diagnostic_msgs/msg/DiagnosticArray`
- bridge の decode/CRC 異常を通知

## 変更ルール
- v0.1 以降の変更は **追加のみ**（既存 topic/型の破壊的変更は禁止）
- 破壊的変更は v0.2 以降で再凍結する
