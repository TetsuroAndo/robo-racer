# Policy Plugin v1（タスクC）

## 目的
- racerd に **FTG/AI の差し替え口**を用意する
- AI失敗時は **FTGへフェイルセーフ** する

## 前提/参照
- `docs/interfaces/ipc_payloads_v1.md`
- `docs/interfaces/ipc_topics_v1.md`
- `docs/proto/overview.md`

---

## モード
- `FTG`: ルールベース（既定）
- `AI`: 学習モデルの推論結果を使用
- `SHADOW`: 推論は行うが **出力は適用しない**（ログのみ）

---

## インターフェース（提案）

### 入力（PolicyInput）
- `ts_ms`
- `LidarSummaryPayload`（必須）
- `LidarScanChunkPayload`（任意、再構成済み）
- `VehicleStatusPayload`（任意）
- `ImuSamplePayload`（任意）

### 出力（PolicyOutput）
- `DriveCmdPayload`（必須）
  - `source` は以下を使用
    - FTG: `SOURCE_FTG`
    - AI: `SOURCE_AI`
    - Shadow: `SOURCE_SHADOW`
- `valid`（bool）: falseなら FTGへフォールバック

---

## 実行制約
- 周期: 20-50ms
- 推論処理は **周期内に完了**（過走行禁止）
- 期限超過時は `valid=false` を返す

---

## フェイルセーフ
- `valid=false` or 例外時は **FTGへ即時フォールバック**
- AUTOでなければ `DriveCmd` を出さない
- `fault` が立っている場合は **停止指令**を優先

---

## ログ要件
- モード切替（FTG/AI/SHADOW）を必ず記録
- AIの推論結果と実適用コマンドの差分を記録

---

## DoD
- 1つのポリシー差し替えで走行が変わる
- SHADOWで **ログのみ生成** できる
- AI失敗時に **FTGで安全に走れる**
