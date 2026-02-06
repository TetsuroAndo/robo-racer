# 後退禁止・確実停止の両立 改修計画書

**作成日**: 2026-02-07  
**ステータス**: 実装完了（ビルド要確認）

---

## 1. 概要

「**後退させない**」と「**十分に止める**」を両立するため、制御系を以下の方針で改修する。

### 1.1 現状の主因（衝突の構造バグ）

| 問題 | 説明 |
|------|------|
| brake_mode が ABS 由来のみ | `SafetySupervisor` は `stop_requested` を出しているが、`brake_mode` は ABS 由来だけで決まり、TSD20 の STOP/MARGIN がブレーキ要求に接続されていない |
| 距離が近いほど弱ブレーキ | `BrakeController` の `ratio = d/stop_dist` により、壁が近いほど `pwm_max_allowed` が小さくなり、止まるべき瞬間にブレーキが消える |
| 惰行で突っ込む | `Drive::tick` はブレーキ中に `Engine::outputBrake(duty)` を呼べるが、`brake_mode` が false のためブレーキ分岐に入らず、PWM=0 惰行になる |
| Decel が負PWMを混ぜる | `applyTargets_()` で `decel_out.pwm_cmd` をそのまま `setTargetPwm()` に混ぜており、減速＝負PWM＝連続後退の温床 |
| STALE でブレーキ解除 | Tsd20Limiter は STALE で `stop_requested=true` を出すが、BrakeController は STALE で return（ブレーキ無し）してしまう |

### 1.2 方針（最小で両立する形）

- **通常経路は「前進 PWM のみ」**（負 PWM を混ぜない）
- **減速は「ブレーキ duty」に変換して `outputBrake()` に寄せる**
- **TSD stop / margin / stale は必ずブレーキ有効**（距離で弱めない）
- それでも止まらない場合のみ、**短い逆転パルス（200ms 予算）**を非常時オプションで追加（連続後退しない）

---

## 2. 改修タスク一覧

| ID | タスク | ステータス | 備考 |
|----|--------|------------|------|
| B1 | SafetySupervisor: brake_mode に stop_requested を OR | 完了 | 最小修正・最優先 |
| B2 | BrakeController: 距離 ratio 削除・STALE 対応・stop_level ラッチ | 完了 | 近いほど弱い→近いほど強い |
| B3 | main.cpp: Decel 負 PWM → ブレーキ duty 変換 | 完了 | 通常経路で後退しない |
| B4 | main.cpp: 推進 PWM は前進のみ（負を禁止） | 完了 | pwm_cmd = max(0, ...) |
| B5 | main.cpp: 逆転パルス（非常用オプション） | 完了 | 総予算 200ms・ON/OFF で刻む |
| B6 | Drive.cpp: クールダウンで逆転パルスは通す | 完了 | 前進だけ抑制 |
| B7 | Config.h: 逆転パルス用パラメータ追加 | 完了 | BRAKE_REV_PULSE_* |

---

## 3. 詳細設計

### 3.1 推進と制動の分離

| チャネル | 内容 | 出力先 |
|----------|------|--------|
| 推進コマンド | 前進のみ `pwm_drive = max(0, …)` | `Engine::setTarget(pwm)` |
| 制動コマンド | `Engine::outputBrake(duty)`（短絡制動） | 逆転ではなくアクティブブレーキ |
| 逆転パルス | 非常時のみ・短パルス・連続しない | 総予算 200ms |

### 3.2 BrakeController の新ロジック

- **距離 ratio を削除**：`ratio = d/stop_dist` による「近いほど弱い」を廃止
- **stop_level 別の duty 上限**：
  - STOP: `BRAKE_PWM_MAX`
  - MARGIN: `BRAKE_PWM_MAX / 2`
  - STALE: `BRAKE_PWM_MAX`（安全側に倒す＝止める）
- **v_est による早期解除を廃止**：IMU 不調時でも「止めたい」ならブレーキを出す
- **stop_level ラッチ**：HOLD 中は `stop_requested` が落ちても直前の level を保持

### 3.3 逆転パルス（オプション）

- **起動条件**（全て満たした時のみ）：
  - `brake_active`
  - `stop_level == STOP` または `STALE`
  - `imu_calib`
  - `v_est > BRAKE_REV_PULSE_V_START_MM_S`
- **実行**：ON 25ms / OFF 25ms で刻む、総予算 200ms
- **連続逆転はしない**：on/off で必ず状態を戻し、IMU 推定を再評価

### 3.4 テスト観点

1. `StopLevel::STOP` / `MARGIN` / `STALE` それぞれで `applied_brake_duty > 0` になること
2. IMU 無効（`g_imu_valid=false`）でも STOP/MARGIN/STALE でブレーキが出ること
3. DecelController が active のときに `setTargetPwm()` が負にならないこと
4. 逆転パルス有効時：`BRAKE_COOLDOWN_MS` 中でも負 PWM が潰されないこと

---

## 4. シンプル化の完成形（将来）

- **推進**：SpeedController は最小の FF（線形 or テーブル）＋弱い P だけ
- **制動**：BrakeController 1 個に集約（STOP/MARGIN・距離・速度で duty を決める）
- **ABS/Decel**：残すなら「制動 duty の上限/下限を調整する係」に格下げ（推進 PWM に負符号で混ぜない）

---

## 5. 参照

- `docs/planning/decel_controller_status.md`：DecelController の現状
- `docs/planning/abs_backward_investigation.md`：後退問題の調査
