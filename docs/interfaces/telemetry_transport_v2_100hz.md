# 送受信（ESP32↔RPi）修正ドキュメント：100Hz 化と帯域見積（v2）

## 0. 背景

現状 ESP32 は `STATUS_INTERVAL_MS = 50`（=20Hz）で

- `STATUS`（速度/舵角/FAULT/age）
- `IMU_STATUS`（a_long/v_est/yaw/flags/age）

を送っている。

速度を物理一致させる（閉ループ制御を詰める）には、

- 過渡の観測点が 20Hz だと粗い
- RPi 側の制御・解析（プロット/同定）が遅れる

よって **100Hz（10ms周期）**へ引き上げる。

## 1. 現状仕様（ワイヤ上のサイズ）

プロトコルの raw frame は：

- `Header` = 9 bytes 固定（"MC", ver, type, flags, seq, len）
- `CRC16` = 2 bytes
- payload は type ごと

payload サイズ：

- `STATUS payload` = 10 bytes
- `IMU_STATUS payload` = 12 bytes

raw frame size：

- STATUS: 9 + 10 + 2 = **21 bytes**
- IMU_STATUS: 9 + 12 + 2 = **23 bytes**

この raw を COBS encode して末尾に delimiter `0x00` を付けるので、実際の UART 上は

- おおむね raw + 1〜数バイト + delimiter（小さいフレームなら **+2 前後**）と見てよい。

## 2. 100Hz 化したときの帯域見積

UART は `SERIAL_BAUD = 921600`（8N1想定）なので、実効は約 1/10：

- 921600 bps / 10 ≒ **92160 bytes/s**

100Hz で STATUS + IMU_STATUS を両方送る：

- STATUS: 21 bytes * 100 = 2100 B/s（raw）
- IMU_STATUS: 23 bytes * 100 = 2300 B/s（raw）
- 合計 ≒ **4400 B/s**（raw）

COBS + delimiter を雑に 2 bytes/packet 上乗せしても：

- 2 packets * 2 bytes * 100 = 400 B/s 程度
- 合計 ≒ **~4.8 KB/s**

これは 92160 B/s の **5% 程度**で、十分余裕がある。
ログや他 IPC（LiDAR summary 等）が同居しても、直ちにボトルネックにはならない。

## 3. 変更内容（firmware）

### 3.1 タイマ周期を 10ms に変更

現状：

- `static mc::PeriodicTimer statusTimer(50);`
- `cfg::STATUS_INTERVAL_MS = 50`

変更：

- `static mc::PeriodicTimer statusTimer(cfg::STATUS_INTERVAL_MS);`
- `cfg::STATUS_INTERVAL_MS = 10;`  // 100Hz

送信部は既に

- `if (statusTimer.due(now_ms)) { sendStatus_(); sendImuStatus_(); }`

なので、ここはそのままで周期だけ上げればよい。

> 注意：IMU のサンプルは `IMU_READ_INTERVAL_MS = 5`（200Hz）で更新される。
> 100Hz 送信時は “直近の推定値” を 10ms ごとに送るだけで OK。

### 3.2 RPi 側の受信ループ（seriald/racerd）の前提確認

100Hz 化で問題になりがちな点：

- 受信バッファ溢れ（読み取りが遅い）
- フレーム復号（COBS）処理が重い
- ログ出力が同期的で詰まる

対策（推奨）：

- 受信は **ノンブロッキング + バッファリング**（既存方針を維持）
- “毎フレーム println” のような同期ログを避ける
- 解析/保存は別スレッド・別プロセスへ（可能なら）

## 4. “帯域 9割” を使い切るのはどのくらいのレートか（参考）

実効 92160 B/s の 90% は：

- 92160 * 0.9 = **82944 B/s**

STATUS+IMU_STATUS を 1セットあたり raw 44 bytes とみなすと：

- 82944 / 44 ≒ **1885 Hz**

COBS 等の上乗せを見ても、**~1kHz オーダ**まで行かない限り「帯域 9割」は踏まない。
つまり 100Hz は極めて安全圏。

## 5. テスト手順（v2）

### 5.1 ESP32 単体

- `status_seq` / `imu_seq` が 100Hz 相当で増えること
- `age_ms` が想定どおり（送信遅延が増えてない）こと
- `Serial2` の `setTxBufferSize(4096)` でも詰まらないこと

### 5.2 RPi 側

- 10秒間連続で受信して drop が無いこと
- `seq` 欠落率が一定以下（0%が理想）
- CPU 使用率が問題ないこと（ログを抑制して確認）
