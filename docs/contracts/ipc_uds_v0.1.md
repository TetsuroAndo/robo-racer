# IPC UDS 契約 v0.1（control/telemetry 分離）

## 目的
観測系（ROS/SLAM/学習）からの誤送信を防ぎ、制御と観測を安全に分離する。

## ソケット
### control
- 役割: **送信権あり**（制御系）
- 既定: `/run/roboracer/seriald.sock`
- 移行期間の互換: `/tmp/roboracer/seriald.sock`
- 原則: **送信者は 1 プロセスのみ（racerd）**

### telemetry
- 役割: **観測専用（送信禁止）**
- 既定: `/run/roboracer/seriald.telemetry.sock`
- 移行期間の互換: `/tmp/roboracer/seriald.telemetry.sock`
- ルール: クライアントが `send()` した場合は **即切断**（または無視 + WARN）
- 複数クライアントの同時接続を許可する

## 配信内容
- UART 受信フレーム（ESP32 → RPi）を telemetry へ broadcast
- **RPi → ESP32 送信フレームも telemetry にミラー**（/mc/drive_cmd 可視化のため）

## 互換・移行
- `/run/roboracer/` へ移行を進める
- 互換期間中は `/tmp/roboracer/` を残し、段階的に切り替える
