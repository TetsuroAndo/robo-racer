# HILS v1（Interface Freeze）

## 目的
- 物理機体が無くても「DRIVE→STATUS」往復と安全挙動を検証できること
- TTL/KILL/モード切替の**安全条件**を机上で再現すること

## 正本
- `docs/firmware/hils_guide.md`（実機HILSの運用）
- `docs/proto/overview.md`（UARTプロトコル）

## v1要件（sim_esp32d）
- `DRIVE` を受理すると `STATUS` を返す
- `MODE_SET` に従い AUTO時のみ DRIVE を適用する
- `KILL` はラッチして停止維持する
- `ttl_ms` を超過したら停止する
- `PING` は常に `ACK` を返す

## v1受入条件（机上E2E）
- seriald + sim_esp32d + racerd の統合テストが通る
- 以下のシナリオを再現できる
  - DRIVE未受信が一定時間続く → STOP
  - KILL受信 → STOP維持
  - MANUAL時のDRIVEは受理のみ（適用しない）

## 既存HILS（実機）との関係
- 実機HILSは **出力観測経路**（Shadow/Hilsモード）として維持
- sim_esp32d は **机上の代替ESP32** として扱う

