## 目的

スタック内で `speed_mm_s` / `speed` の意味が「段階値(0..255)」と「物理(mm/s)」を行き来している問題を解消し、**planner（RPi）の出力を mm/s（物理）に統一**する。

## 背景（現状のズレ）

- RPi `Process::proc()` の `ProcResult.speed` が **段階値（実質 speed_input）** を返している。
- `Sender::send()` が段階値を `DrivePayload.speed_mm_s` に載せるために **mm/sへ換算**している。
- firmware 側は `DrivePayload.speed_mm_s` を **物理目標 mm/s** として扱い始めており、ここで意味が揺れる。
- さらに RPi 側のテレメトリ/UIも「speed_mm_s を想定した換算」を入れており混乱が増幅している。

## 方針

- **RPi planner 出力（ProcResult / TelemetrySample の cmd速度）を最初から mm/s にする**
- UI入力や“段階速度”は **別レイヤ**で `speed_input ⇄ mm/s` を変換する（境界を明確化）
- `Sender::send()` は **換算しない**（RPi→ESP32 の `DrivePayload.speed_mm_s` は mm/s を直に載せる）

## 作業項目

### 実装（RPi）

- [ ] `rpi/src/config/Config.h` の `FTG_SPEED_MIN/MAX/WARN_CAP` を mm/s 系へ変更
- [ ] `rpi/src/Process.cpp` の速度計算を mm/s に統一（距離/舵角連動）
- [ ] `rpi/src/Process.cpp` の brake cap（IMU由来）比較を mm/s で行う
- [ ] `rpi/src/Sender.cpp` の段階値→mm/s換算を削除し、mm/s をそのまま送る
- [ ] `rpi/src/Telemetry.cpp` の UI 表示から段階値→mm/s換算を削除する

### 互換/掃除

- [ ] 段階値（旧 speed_input）を必要とする UI/入力は別レイヤに移動（今回は最小差分で後回し）
- [ ] docs（命名/テレメトリ/調整）を mm/s 前提に合わせる

## 完了条件

- RPi 内部の「cmd速度」は mm/s で一貫してログ/テレメトリに出る
- `DrivePayload.speed_mm_s` は常に mm/s（物理）で送受信される
- 「speed_mm_s なのに 0..255 段階」の経路が主要経路から消える

