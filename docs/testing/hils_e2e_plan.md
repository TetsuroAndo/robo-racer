# 机上E2E 統合テスト計画（seriald + sim_esp32d + racerd）

## 目的
- 物理機体なしで「RPi↔ESP32契約（v1）」の最小E2Eを検証する
- `DRIVE` → `STATUS` の往復と、TTL/KILL/AUTOの安全挙動を机上で再現する

## 構成（想定）
```
[racerd] --(UDS seqpacket / mcproto)--> [seriald] --(PTY/UART)--> [sim_esp32d]
                                 ^                             |
                                 +----(UDS seqpacket)----------+
```

## 前提
- `seriald` が UDS (SOCK_SEQPACKET) でIPC中継できる
- `sim_esp32d` が UART で mcproto を受信/返信できる
- PTYを用いてUARTを模擬できる

## テスト環境
- OS: Linux（AF_UNIX SOCK_SEQPACKETが必要）
- 生成物:
  - `rpi/build/apps/seriald/seriald`
  - `rpi/build/apps/sim_esp32d/sim_esp32d`
- 通信:
  - PTY master/slave
  - UDS socket: `/tmp/seriald.sock`

## テストケース（最小）

### TC1: MODE_SET + DRIVE → STATUS
- 手順
  1. PTYを作成し、serialdのUARTをslaveへ接続
  2. sim_esp32dを同じslaveへ接続
  3. IPCクライアントから MODE_SET(AUTO) を送信
  4. 続けて DRIVE を送信
  5. STATUS が IPC で受信できること
- 期待結果
  - STATUS の `auto_active=1`
  - `speed_now_mm_s` / `steer_now_cdeg` が DRIVE に一致

### TC2: TTL失効 → STOP
- 手順
  1. AUTOでDRIVE送信
  2. TTLの2倍以上待機
  3. STATUS を確認
- 期待結果
  - `faults` に TTL_EXPIRED
  - `speed_now_mm_s=0`, `steer_now_cdeg=0`

### TC3: KILL → STOP維持
- 手順
  1. AUTOでDRIVE送信
  2. KILL送信
  3. STATUS を確認
- 期待結果
  - `faults` に KILL_LATCHED
  - 以降の DRIVE でも停止維持

### TC4: MANUAL時のDRIVE受理のみ
- 手順
  1. MODE_SET(MANUAL)
  2. DRIVE送信
  3. STATUS を確認
- 期待結果
  - `auto_active=0`
  - `faults` に AUTO_INACTIVE
  - 出力は停止

### TC5: PING → ACK
- 手順
  1. PING送信
  2. ACKを確認
- 期待結果
  - ACK の `seq` が一致

## 実装方針（最初の一歩）
- 既存 `test/rpi/seriald/test_seriald_integration.py` を拡張
  - PTYを共有し、sim_esp32dを subprocess で起動
  - IPCからMODE_SET/DRIVEを送信し、STATUS/ACKを検証
- 追加テスト: `test/rpi/hils/test_hils_e2e.py`
  - `racerd_stub`（テスト専用）で MODE_SET/DRIVE を送信
