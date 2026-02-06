# 後退が発生する原因の調査

## 結論

**後退は ABS 由来ではなく、SpeedController の PID フィードバックによる「逆転ブレーキ」が原因です。**

ABS が `speed_mm_s = 0` を返すと、SpeedController が「目標 0・推定速度 > 0」として負の PWM を出し、モータが逆転（後退）して減速しようとします。この挙動が設計どおりに働いているため、後退が発生します。

---

## 1. 制御の流れ

```text
ABS active
  → safe.targets.speed_mm_s = 0
  → SpeedController.update(0, v_est, ...)
  → error = 0 - v_est = 負
  → pwm_cmd = pwm_ff + kp*error + _i = 負
  → drive.setTargetPwm(負)
  → Engine.outputSpeed(負)
  → LPWM 駆動 = 後退
```

---

## 2. 該当コード

### 2.1 SpeedController（負の PWM を出す）

```cpp:firmware/src/control/SpeedController.cpp
// v_target=0, v_est>0 のとき
out.error_mm_s = (float)v_target_mm_s - v_est_mm_s;  // = -v_est（負）
float pwm = (float)out.pwm_ff + kp * out.error_mm_s + _i;
// pwm_ff = 0, error < 0 → pwm = 負
out.pwm_cmd = (int16_t)lroundf(pwm_sat);  // 負の値
```

- `v_target = 0`（ABS が惰行を指示）
- `v_est > 0`（まだ前進している）
- `error = 0 - v_est < 0`
- `pwm = 0 + kp * (負) + _i` → 負の PWM

### 2.2 Engine（負の PWM → 後退）

```cpp:firmware/src/hardware/Engine.cpp
void Engine::outputSpeed(int cur) {
	if (cur > 0)
		applyPWM((uint8_t)cur, 0);   // RPWM = 前進
	else if (cur < 0)
		applyPWM(0, (uint8_t)(-cur)); // LPWM = 後退
	else
		applyPWM(0, 0);               // 停止
}
```

- `cur < 0` のとき LPWM を駆動 → 後退

### 2.3 main.cpp（ABS と SpeedController の接続）

```cpp:firmware/src/main.cpp
// ABS が speed_mm_s=0 を返す
const SafetyResult safe = safety.apply(...);
// SpeedController にそのまま渡す
const SpeedControlOutput out = speed_ctl.update(
    safe.targets.speed_mm_s, v_est, ...);
// 負の pwm_cmd をそのまま Engine へ
drive.setTargetPwm(out.pwm_cmd, now_ms);
```

---

## 3. 設計意図

`speed_command_v2.md` より:

> 4. ABS（減速補助）→ `v_target` をより小さく（または brake_mode 切替）
> その後、確定した `v_target` を SpeedController に入れて `pwm_cmd` を得る。

つまり:

- ABS は `v_target` を 0 にするだけ
- 実際の PWM は SpeedController が決定
- SpeedController は「目標 0・推定速度 > 0」のとき、減速のために負の PWM（逆転）を出す

したがって、**逆転ブレーキは仕様どおりの動作**です。

---

## 4. 後退し続ける理由

1. **オーバーシュート**  
   逆転ブレーキが強く、減速しすぎて後退に転じる。`v_est` が負になると `error` が正になり、理論上は前進側に戻るが、応答が遅いと後退が続く。

2. **brake_mode と減速レート**  
   `Drive::tick()` で `brake_mode` のとき `ENGINE_RATE_DOWN_BRAKE` を使うため、減速が速く、逆転まで一気に到達しやすい。

3. **IMU の遅れ**  
   `v_est` の更新が遅いと、実際には既に後退していても `v_est` がまだ正のままとなり、負の PWM が続く。

---

## 5. 対応案

### 案 A: brake_mode 時は負の PWM を出さない（惰行のみ）【採用済】

`SpeedController` で、`v_target >= 0` のときは負の PWM を出さない（惰行のみ）。逆転は ABS のみが指示する:

```cpp
// SpeedController.cpp
if (v_target_mm_s >= 0 && pwm_sat < 0.0f)
    pwm_sat = 0.0f;
```

### 案 B: SpeedController に brake_mode を渡す

`brake_mode` のときは負の PWM を出さないように SpeedController 側で制御する。

### 案 C: ABS 発動時は SpeedController をバイパス

ABS 発動時は `pwm_cmd = 0` を強制し、SpeedController の出力を使わない。

---

## 6. その他の逆転経路

| 箇所 | 内容 | 逆転の有無 |
|------|------|------------|
| **SpeedController** | v_target に応じて PWM 範囲を制限（改修済） | v_target >= 0 なら負を出さない |
| **ABS** | 現在は常に 0 を返す。負を返すロジックは削除済 | 逆転なし |
| **DriveHandler** | RPi からの DRIVE で `speed_mm_s < 0` を受信。`MC_ENABLE_MANUAL=0` のとき負を 0 にクランプ | 本番ビルドでは逆転なし。`MC_ENABLE_MANUAL=1` のときは負が通過 |
| **Tsd20Limiter** | `speed_mm_s <= 0` のときそのまま返す | 負は通過（上流次第） |
| **Engine** | `cur < 0` のとき LPWM で後退 | 下流の出力のみ |
| **RPi Sender** | `clamp_speed_input` は `[-SPEED_INPUT_LIMIT, +SPEED_INPUT_LIMIT]` を許容 | AI が負を出すと送信される |

**まとめ**: 逆転しうる経路は (1) SpeedController（改修済）、(2) RPi→DriveHandler（`MC_ENABLE_MANUAL=1` 時のみ）。本番では DriveHandler が負を 0 にクランプするため、逆転は ABS のみが指示する設計になっている。

---

## 前進しなくなった場合の確認事項

1. **TSD20**: `tsd.mm` が 500 以下だと Tsd20Limiter が 0 を返す。`tsd.valid` が false かつ `fail_count >= 5` でも 0。
2. **コマンド期限切れ**: DRIVE の TTL が切れると `cmd_pwm = 0` になる。
3. **低PWM**: `v_target` がデッドバンド内 (0–199) だと pwm が 7–20 程度になり、静止摩擦で動かない場合がある。→ **SPEED_PWM_MIN_FORWARD** で最小 30 を確保。

---

## 7. まとめ

| 項目 | 内容 |
|------|------|
| 後退の直接原因 | SpeedController の負の PWM（逆転ブレーキ） |
| トリガ | ABS が `speed_mm_s = 0` を返す |
| 設計上の位置づけ | 減速のための逆転ブレーキとして意図された動作 |
| 問題 | 強すぎる逆転・オーバーシュートで後退が続く |
| 対策 | **実装済** `v_target >= 0` のとき SpeedController は負の PWM を出さない。逆転は ABS のみが指示する。 |
