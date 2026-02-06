# 速度命令（speed_mm_s）を「物理速度目標」として成立させる設計書（v2）

## 0. 背景（いま何が起きているか）

現状 firmware は `speed_mm_s` を受け取っているが、実態は **線形スケールで PWM（-255..255）に変換しているだけ**。

- `Drive::speedMmSToPwm_()` が `pwm = speed_mm_s * 255 / SPEED_MAX_MM_S` を行い、その値を `Engine::setTarget()` に渡している。
- つまり `speed_mm_s` は「物理速度目標」ではなく、**“0..SPEED_MAX_MM_S を 0..255 に写像したコマンド”**でしかない（キャリブレーションなし）。

一方で firmware には IMU 推定が既にあり、`ImuEstimator` は

- 加速度（重力除去＋Fusion の線形加速度）から `a_long_mm_s2` を得て
- `v_est_mm_s += a_long_mm_s2 * dt` で速度推定 `v_est_mm_s` を持つ（ZUPT/リーク/上限クリップあり）

しかし現状この `v_est_mm_s` を **速度制御（スロットル制御）に使っていない**ため、「`speed_mm_s` を物理速度として一致させる」ことは原理的に無理。

## 1. 用語整理（メンバー混乱ポイントの解消）

このプロジェクトでは最低限、次の 3 つを明確に分離する。

### (A) speed_mm_s（物理速度目標）

- **RPi→ESP32 の DRIVE コマンドで送る値**
- 単位は mm/s
- 意味は「車体の縦方向の目標速度（物理量）」

### (B) speed_input（制御器の内部目標 or 制御入力）

- 安全（TSD20 clamp / ABS / kill / TTL）を通過した後の “最終的に実現したい速度目標” を指す内部値
- 単位は **mm/s のまま**にするのが自然（※PWMと混ぜない）

### (C) pwm（モータドライバへ出す最終指令）

- `Engine` が受け取る **-255..255（符号付き duty 相当）**
- 実際にモータドライバ（IBT-2/BTS7960）へ出力される量
- これは “制御結果” であり、物理速度とは一致しない（一致させるには制御器が必要）

> 結論：**speed_mm_s と pwm は別物**。
> “speed_mm_s を物理量として一致させたい” なら、IMU の `v_est_mm_s` をフィードバックにした速度制御ループを入れる。

## 2. ゴール（Done の定義）

- RPi が `speed_mm_s = X` を送ったとき、車体の推定速度 `v_est_mm_s` が **X に収束**する（過渡特性は許容範囲内）。
- 速度制御が入っても、既存の安全系（kill/TTL/TSD20 clamp/ABS）は優先される。
- キャリブレーション未実施でも「完全に破綻しない」フェイルセーフを持つ（例：最低限の feedforward + ゲイン抑制）。

## 3. 提案アーキテクチャ（v2）

### 3.1 速度制御を `Drive` から分離して “SpeedController” を導入

- 現状 `Drive` は `mm/s → pwm` を内包しているが、これは **open-loop** なので限界がある。
- v2 では `Drive` は「pwm をエンジンへ適用するだけ」にし、速度制御は別クラスに分離する。

例：

- `SpeedController`
  - input: `v_target_mm_s`, `v_est_mm_s`, `dt_s`, `calib_ok`, `killed/ttl`
  - output: `pwm_cmd`（-255..255）
- `Drive`
  - `setTargetPwm(int pwm)`（`Engine::setTarget()` 直結）

### 3.2 制御則（最低限の実装で成立させる）

車輪エンコーダ無し・IMU推定はドリフトするので、いきなり高度な同定は不要。まずは堅実に：

**(1) feedforward（速度→PWM の概算）**

- `pwm_ff = f(v_target_mm_s)`
- 初期案：現状と同じ線形写像を “暫定FF” として残す
  - `pwm_ff = clamp(v_target * 255 / SPEED_MAX_MM_S, -255..255)`

**(2) feedback（IMU 速度推定で補正）**

- 誤差 `e = v_target - v_est`
- `pwm = pwm_ff + Kp*e + Ki*∫e dt`
- 出力 `pwm` は -255..255 で saturate
- **アンチワインドアップ必須**（saturation 中は積分を止める/戻す）

**(3) 低速域の扱い**

IMU の `v_est` は低速で誤差が出やすいので、

- `v_target < V_DEADBAND`（例 200 mm/s）では **積分を弱める/停止**
- ZUPT 条件成立時は `v_est=0` へ寄せる（既存実装がある）

**(4) 逆方向（リバース）の扱い**

`v_est` は負方向を持たないため、逆方向時は **feedforward のみ**とし、フィードバックを無効化する。

### 3.3 キャリブレーション未実施（calib=false）時のフェイルセーフ

`ImuEstimate.calibrated == false` の間は推定が信用できないので、

- `Kp, Ki` を **強烈に下げる**、あるいは `Ki=0` で `pwm = pwm_ff + small_Kp*e`
- もしくは最初は `pwm = pwm_ff` のみ（open-loop）に落とす

### 3.4 既存の安全系との優先順位

最終的に `pwm_cmd` を出す前に、以下の順で “速度目標” を確定させる：

1. kill / TTL expiry → **速度目標 = 0**
2. 手動モード等で clamp を無効化するならポリシーに従う
3. TSD20 clamp（前方障害）→ `v_target` を上限でクリップ
4. ABS（減速補助）→ `v_target` をより小さく（または brake_mode 切替）

その後、確定した `v_target` を SpeedController に入れて `pwm_cmd` を得る。

## 4. 50km/h 目標とパラメータ（現状との整合）

- 50 km/h = 13.888… m/s = **13888.9 mm/s**
- 現在 `SPEED_MAX_MM_S = 5000` を上限にしているなら、物理目標として 50km/h を指すには **上限を 14000 以上**にする必要がある。

ただし注意：

- 上限を上げても **車体が出ない**なら、制御器は PWM=255 に張り付くだけ（飽和）で、それ以上は伸びない。
- よって v2 では「上限値」は
  - (a) 物理的に到達可能な最高速度（推奨）
  - (b) RPi が送る最大指令値（ソフト上の契約）
  を混同しないこと。

推奨：

- `mc_config::SPEED_MAX_MM_S` は「**契約上の最大指令**」として 14000 に上げてもよいが、
- 併せて “到達不能時の飽和” をメトリクス化してチューニング対象にする（例：pwm_saturated_ratio）。

## 5. 実装タスク（firmware）

### Task A: SpeedController の追加

- `firmware/src/control/SpeedController.h/.cpp`（新規）
- パラメータは `cfg::` に置く
  - `SPEED_KP`, `SPEED_KI`, `SPEED_KP_UNCAL`, `SPEED_KI_UNCAL`
  - `SPEED_I_CLAMP`, `SPEED_DEADBAND_MM_S`

### Task B: Drive の責務を整理

- `Drive::speedMmSToPwm_()` を廃止し、SpeedController の feedforward に移管
- `Drive` は `Engine::setTarget(pwm)` を受けるだけにする
- 新 API: `Drive::setTargetPwm(int16_t pwm, ...)`

### Task C: main loop の流れ変更

- 従来 `drive.setTargetMmS(speed_mm_s)` で直接PWM変換していた箇所を、
  - `v_target = clamp/abs/kill/ttl の結果`
  - `pwm_cmd = speed_ctl.update(v_target, imu_est.state().v_est_mm_s, dt, calib, ...)`
  - `drive.setTargetPwm(pwm_cmd)`
  に置き換える

### Task D: ログと可観測性

- 100〜200ms ごとのログに以下を追加：
  - `v_cmd_mm_s`, `v_est_mm_s`, `pwm_cmd`, `pwm_ff`, `err`, `integrator`, `saturated`
- RPi 側でも可視化できるとデバッグが速い（後述の IMU_STATUS 100Hz 化で十分）。
