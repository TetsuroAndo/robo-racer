# FTG Gap ベース改修・ステア追従改善 詳細改修計画書

## 1. 概要

### 1.1 目的

- **ロジック簡素化**: `Process::proc()` の softmax/コスト最適化を削除し、FTG 本来の「gap（連続空間）」ベースに置き換える
- **挙動改善**: 「奥が深く、範囲が広い方向」を明示的に狙う
- **ステア追従改善**: スピードにステアがついていけず刺さる問題を解消する

### 1.2 参照

- 正本: `docs/planning/ftg_v1.md`
- 実装: `rpi/src/Process.cpp`, `rpi/src/config/Config.h`
- 車両限界: `shared/config/include/mc_config/vehicle_limits.h`（`STEER_ANGLE_MAX_DEG=25`, `SPEED_MAX_MM_S=5000`）
- ステータス: `docs/planning/ftg_gap_steer_refactor_status.md`

---

## 2. 現状分析

### 2.1 制御フロー（proc 呼び出しの依存関係）

`proc()` は **LiDAR の新スキャン受信ごとに** 1 回だけ呼ばれる。

```41:55:rpi/src/lidar_to_esp.cpp
	while (!g_stop) {
		const auto &res = lidarReceiver.receive();
		sender.poll();
		MotionState motion{};
		const MotionState *motion_ptr =
			sender.motion(motion) ? &motion : nullptr;
		const auto procResult = process.proc(res, lastSteerAngle, tick, scan_id,
											 run_id, motion_ptr);
		sender.send(procResult.speed_mm_s, procResult.angle);
		lastSteerAngle = procResult.angle;
		++tick;
		++scan_id;
	}
```

- `lidarReceiver.receive()` がブロックし、新スキャン到着まで待つ
- 到着後、`process.proc(res, ...)` → `sender.send(...)` の順で実行
- 制御周期 = LiDAR スキャン周期（RPLIDAR C1 は約 10Hz）

### 2.2 corridor_min の算出（変更対象外）

`corridor_min[angle]` は各角度における「車幅+マージンを通過できる最小距離」として算出される。この部分は改修対象外。

**補足**: 「通れるのに壁距離が近いから停止」という症状は、根が `corridor_min` の作り方（左右対称の最小化）にあるケースが多い。gap 化だけでも改善することはあるが、**corridor_min が過小評価のままだと gap 自体が「無い」判定になり得る**。Phase 1 の後に必要なら Phase 1.5 で内外非対称 corridor 等を検討する。

```125:155:rpi/src/Process.cpp
	const int max_n = std::max(-cfg::FTG_ANGLE_MIN_DEG, cfg::FTG_ANGLE_MAX_DEG);
	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
		 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const float dist = smoothed[(size_t)idx];
		if (dist <= 0.0f)
			continue;
		float r_m = std::max(0.001f, dist / 1000.0f);
		// ... theta_req 計算 ...
		int min_mm = std::numeric_limits< int >::max();
		for (int a = angle - n; a <= angle + n; ++a) {
			// ...
			if (raw > 0 && raw < min_mm)
				min_mm = raw;
		}
		if (min_mm != std::numeric_limits< int >::max())
			corridor_min[(size_t)idx] = min_mm;
	}
```

### 2.3 proc() の現行ロジック（削除・置換対象）

`corridor_min` 算出後、**角度選択**は以下のコスト関数＋softmax で行っている。

#### 2.3.1 obs_cost（500mm 以上で 0 になる）

```165:171:rpi/src/Process.cpp
	auto obs_cost = [](int d_mm) -> float {
		if (d_mm >= cfg::FTG_COST_SAFE_MM)
			return 0.0f;
		const float x = (float)(cfg::FTG_COST_SAFE_MM - d_mm) /
						(float)cfg::FTG_COST_SAFE_MM;
		return x * x;
	};
```

`FTG_COST_SAFE_MM=500`（`Config.h` L108）のため、500mm 以上の角度はすべて `obs_cost=0` となり、距離の差が効かなくなる。

#### 2.3.2 jerk_weight（近距離で delta ペナルティを緩和）

```172:186:rpi/src/Process.cpp
	auto jerk_weight = [](int d_mm) -> float {
		float w = cfg::FTG_COST_W_DELTA;
		if (cfg::FTG_JERK_RELAX_MM > cfg::FTG_NEAR_OBSTACLE_MM) {
			if (d_mm < cfg::FTG_JERK_RELAX_MM) {
				float s =
					(float)(d_mm - cfg::FTG_NEAR_OBSTACLE_MM) /
					(float)(cfg::FTG_JERK_RELAX_MM - cfg::FTG_NEAR_OBSTACLE_MM);
				// ...
				w *= (s * s);
			}
		}
		return w;
	};
```

#### 2.3.3 コスト計算と softmax 平均

```187:216:rpi/src/Process.cpp
	for (int angle = cfg::FTG_ANGLE_MIN_DEG; angle <= cfg::FTG_ANGLE_MAX_DEG;
		 ++angle) {
		const int idx = angle - cfg::FTG_ANGLE_MIN_DEG;
		const int d_mm = corridor_min[(size_t)idx];
		if (d_mm <= 0)
			continue;
		has_data = true;
		// ...
		if (d_mm <= cfg::FTG_NEAR_OBSTACLE_MM)
			continue;
		const float a_norm =
			std::fabs((float)angle) / (float)mc_config::STEER_ANGLE_MAX_DEG;
		const float d_norm = std::fabs((float)angle - clamped_last) /
							 (float)mc_config::STEER_ANGLE_MAX_DEG;
		const float w_delta = jerk_weight(d_mm);
		const float j = cfg::FTG_COST_W_OBS * obs_cost(d_mm) +
						cfg::FTG_COST_W_TURN * a_norm +
						w_delta * (d_norm * d_norm);
		if (j < best_j) {
			best_j = j;
			best_angle = angle;
			best_dist = d_mm;
		}
		const float w = std::exp(-cfg::FTG_COST_BETA * j);
		z += w;
		angle_sum += w * (float)angle;
	}
```

- `j = w_obs*obs_cost(d) + w_turn*|angle| + w_delta*|angle-last|^2`
- `target_angle = angle_sum / z`（softmax 重み付き平均）

**結果としての挙動**:

| 現象 | 原因 |
|------|------|
| 500mm以上は全部同じ安全（obs_cost=0）→ 直進/前回角優先 | `FTG_COST_SAFE_MM=500` により遠方の差が消える |
| 前方が NEAR に落ちると、残った角度で「曲がり量が小さい所」に寄る | コスト最小化が差分・曲がり量を優先するため |

### 2.4 ステア slew 制限（遅さの主因）

`dt_s` は前回 `proc()` 呼び出しからの経過時間。`last_proc_ts_us_` は `mutable` で保持される（L49 で更新）。

```42:49:rpi/src/Process.cpp
	float dt_s = 0.1f;
	if (last_proc_ts_us_ > 0 && t0_us > last_proc_ts_us_) {
		dt_s = (float)(t0_us - last_proc_ts_us_) / 1000000.0f;
		if (dt_s < 0.001f)
			dt_s = 0.001f;
		else if (dt_s > 0.5f)
			dt_s = 0.5f;
	}
	last_proc_ts_us_ = t0_us;
```

10Hz なら `dt_s≈0.1s`。slew 制限は以下で適用される：

```230:236:rpi/src/Process.cpp
	const float max_delta = cfg::FTG_STEER_SLEW_DEG_PER_S * dt_s;
	float applied_angle_f = target_angle_f;
	const float delta = applied_angle_f - clamped_last;
	if (delta > max_delta)
		applied_angle_f = clamped_last + max_delta;
	else if (delta < -max_delta)
		applied_angle_f = clamped_last - max_delta;
```

```107:114:rpi/src/config/Config.h
// コスト関数（目的関数）
static constexpr int FTG_COST_SAFE_MM            = 500;   // ここから回避を開始
static constexpr int FTG_JERK_RELAX_MM           = 300;   // 近距離でジャーク抑制を緩める
static constexpr float FTG_COST_W_OBS            = 8.0f;
static constexpr float FTG_COST_W_TURN           = 0.01f;
static constexpr float FTG_COST_W_DELTA          = 0.3f;
static constexpr float FTG_COST_BETA             = 4.0f;  // soft-argminの鋭さ
static constexpr float FTG_STEER_SLEW_DEG_PER_S  = 120.0f;
```

`STEER_ANGLE_MAX_DEG=25`（`vehicle_limits.h`）のため：

- 120 deg/s × 0.1s = **12 deg/tick**
- 0→25deg に約 3tick ≒ **0.3 秒**
- 5m/s で 0.3 秒 → **約 1.5m** 進んでから舵が届く → 「曲がる前に刺さる」

**遅さの要因を分解すると**（レビューで「本当に slew だけが原因？」と言われたときの観測項目）:

| 要因 | 計測・確認方法 |
|------|----------------|
| LiDAR 待ち | `t_receive`（`receive()` ブロック時間） |
| Process 処理 | `t_proc`（`proc()` 内の処理時間） |
| poll / send | `t_poll`, `t_send` |
| UART 送信頻度 | Hz と ESP32 側の `STATUS age_ms`（適用遅延） |
| slew の効き | `out_angle` と `lastSteerAngle` の差分推移（slew で削られている証拠） |

「10Hz + slew 制限」が主因であることをログで示すと説得力が上がる。

### 2.5 速度決定（turn-cap 未実装）

現状の速度制限は以下のみ（L271-314）：

- `v_dist`: 距離に応じた指数飽和
- `v_steer`: 舵角に応じた cos 減速
- `warn` 時の `FTG_SPEED_WARN_CAP_MM_S` クランプ
- `has_brake_cap` 時のブレーキ距離ベース cap

**舵が追いつくまでの時間**を考慮した速度上限はない。

---

## 3. 改修設計

### 3.1 proc を gap + depth + width に置き換える

#### 3.1.1 通行可能判定

現行の NEAR 判定（L198-199）を流用：

```198:199:rpi/src/Process.cpp
		if (d_mm <= cfg::FTG_NEAR_OBSTACLE_MM)
			continue;
```

**注意**: `free = (corridor_min > NEAR)` だけだと、NEAR を超えた瞬間から gap に入るため、ノイズや一瞬の穴で **細かい gap が乱立** しやすく、蛇行の原因になる。

**推奨**: `FTG_GAP_FREE_MM` を追加し、`free[angle] = (corridor_min[angle] >= FTG_GAP_FREE_MM)` とする。

- 初期値: `FTG_WARN_OBSTACLE_MM` または `NEAR + 50mm` 程度が安定

#### 3.1.2 連続区間（gap）抽出

`free==true` が連続している区間を列挙（例: [-12..+8], [+20..+33]）。角度を `FTG_ANGLE_MIN_DEG` から `FTG_ANGLE_MAX_DEG` まで走査し、`free` の on/off 遷移で gap の開始・終了を検出する。

**幅フィルタ**: `FTG_GAP_MIN_WIDTH_DEG` を追加し、`width < FTG_GAP_MIN_WIDTH_DEG` の gap は無視する。例: 6deg 以上のみ有効。細かい gap による蛇行を抑える。

```cpp
bool in_gap = false;
int gap_s = 0;
for (int angle = FTG_ANGLE_MIN_DEG; angle <= FTG_ANGLE_MAX_DEG; ++angle) {
    const bool free = (corridor_min[idx] >= cfg::FTG_GAP_FREE_MM);
    if (free) {
        if (!in_gap) { in_gap = true; gap_s = angle; }
    } else {
        if (in_gap) { finalize_gap(gap_s, angle - 1); in_gap = false; }
    }
}
if (in_gap) finalize_gap(gap_s, FTG_ANGLE_MAX_DEG);
// finalize_gap 内で width < FTG_GAP_MIN_WIDTH_DEG ならスキップ
```

#### 3.1.3 gap の「深さ」と「広さ」を測る（finalize_gap 内）

`finalize_gap(s_deg, e_deg)` は各 gap に対して呼ばれる。処理の流れ：

1. **width** = `e_deg - s_deg + 1`
2. `width < FTG_GAP_MIN_WIDTH_DEG` なら **スキップ**（細かい gap を無視）
3. gap 内の `corridor_min` を `std::vector<int> ds` に収集
4. **depth_mm** = `ds` をソートし、`(ds.size-1) * FTG_GAP_DEPTH_Q` 位置の分位点
5. **peak_angle, peak_dist** = gap 内で最大距離の角度
6. **target** = 距離重み付き中心: `w = max(0, d - NEAR)^GAMMA`, `target = sum(w*angle)/sum(w)`
7. **depth_n** = `min(1, depth_mm / FTG_GAP_DEPTH_SAT_MM)`
8. **width_n** = `min(1, width / FTG_GAP_WIDTH_REF_DEG)`
9. **score** = `depth_n * (1 + FTG_GAP_WIDTH_WEIGHT * width_n) - pen`
10. 最高スコアなら `selected_target`, `best_angle`, `best_dist` を更新

| 指標 | 算出方法 | 狙い |
|------|----------|------|
| **width** | `width = gap_end_deg - gap_start_deg + 1` | 広い空間を優先 |
| **depth** | gap 内の `corridor_min` をソートし、分位点（20%）を採用 | 1 点だけ遠いノイズに引っ張られない |

#### 3.1.4 gap スコア

```
score = depth_n * (1 + FTG_GAP_WIDTH_WEIGHT * width_n) - pen
pen = FTG_GAP_TURN_PENALTY * |target|/STEER_MAX + FTG_GAP_DELTA_PENALTY * delta_relax(depth) * (|target-last|/STEER_MAX)^2
```

- `depth_n = min(1, depth_mm / FTG_GAP_DEPTH_SAT_MM)`
- `width_n = min(1, width / FTG_GAP_WIDTH_REF_DEG)`
- `delta_relax(depth)`: 近距離ほど舵の急変を許す（現行 `jerk_weight` と同様の考え方）

#### 3.1.5 狙う角度の決め方

gap 内の距離重み付き中心：

```
w[angle] = max(0, corridor_min[angle] - FTG_NEAR_OBSTACLE_MM)^FTG_GAP_WEIGHT_GAMMA
target = sum(w * angle) / sum(w)
```

最高スコアの gap を選び、その `target` を `target_angle_f` とする。

### 3.2 ステア追従改善

| 対策 | 変更箇所 | 内容 |
|------|----------|------|
| **slew 引き上げ** | `Config.h` L114 | `FTG_STEER_SLEW_DEG_PER_S` を 120 → 360。10Hz なら 36deg/tick → 25deg を 1tick で到達可能。**注**: 360 はサーボ物理性能を超えないことを保証する値ではなく、RPi 側 slew をボトルネックにしないための値 |
| **turn-cap** | `Process.cpp` 速度決定ブロック（L271 付近） | 舵角変更に必要な時間に対して、障害物までの距離から速度上限を作る |

### 3.3 速度側の turn-cap 式

`Process.cpp` の `out_speed` 算出後、`warn` チェック前に挿入：

```cpp
// turn-cap: 舵が追いつくまでの時間に対して、距離から速度上限を作る
const float slew = std::max(1.0f, cfg::FTG_STEER_SLEW_DEG_PER_S);
const float t_turn = std::fabs((float)out_angle - clamped_last) / slew;
const float avail_mm = (float)std::max(0, corridor_min_mm - cfg::FTG_NEAR_OBSTACLE_MM);
if (avail_mm > 0.0f && t_turn > 0.0f) {
    const float v_turn = avail_mm / (t_turn + cfg::FTG_TURN_CAP_LATENCY_S);
    int v_cap = (int)std::lround(std::max(0.0f, std::min(v_max, v_turn)));
    if (v_cap < out_speed)
        out_speed = v_cap;
}
```

- `t_turn = |delta_angle| / steer_slew`
- `v_turn = (corridor_min - NEAR) / (t_turn + latency)`

**将来の詰め**: `FTG_TURN_CAP_LATENCY_S` に、LiDAR データ age・UART 遅延・ESP32 適用遅延を加算して安全側に倒す余地がある。

---

## 4. パラメータ変更

### 4.1 Config.h の追加・変更

**変更箇所**: `rpi/src/config/Config.h` L107-119 付近

| 定数 | 現状 | 変更後 | 説明 |
|------|------|--------|------|
| `FTG_STEER_SLEW_DEG_PER_S` | 120.0（L114） | 360.0 | 10Hz でも 1tick で最大舵角到達可能に |
| `FTG_GAP_DEPTH_Q` | （新規） | 0.20 | gap 深さの分位点（20%） |
| `FTG_GAP_DEPTH_SAT_MM` | （新規） | 2500 | 深さ正規化の上限（2.5m） |
| `FTG_GAP_WIDTH_REF_DEG` | （新規） | 30 | 幅の正規化基準 |
| `FTG_GAP_WIDTH_WEIGHT` | （新規） | 0.80 | 幅の寄与 |
| `FTG_GAP_TURN_PENALTY` | （新規） | 0.12 | \|angle\| への軽い罰 |
| `FTG_GAP_DELTA_PENALTY` | （新規） | 0.18 | \|angle-last\| への軽い罰 |
| `FTG_GAP_WEIGHT_GAMMA` | （新規） | 2.0 | gap 内の角度重み w=(d-NEAR)^gamma |
| `FTG_GAP_FREE_MM` | （新規） | `FTG_WARN_OBSTACLE_MM` または `NEAR+50` | 通行可能判定の閾値。`free = corridor_min >= FTG_GAP_FREE_MM`。NEAR のみだと細かい gap が乱立し蛇行しやすい |
| `FTG_GAP_MIN_WIDTH_DEG` | （新規） | 6 | 幅がこれ未満の gap は無視。細かい gap による蛇行を抑える |
| `FTG_TURN_CAP_LATENCY_S` | （新規） | 0.08 | turn-cap 用の反応遅れ |

### 4.2 削除・未使用化するパラメータ

以下のコスト関数関連は gap 方式では使用しない（将来的に削除検討）:

- `FTG_COST_SAFE_MM`, `FTG_COST_W_OBS`, `FTG_COST_W_TURN`, `FTG_COST_W_DELTA`, `FTG_COST_BETA`（L108-113）

※ 初回パッチでは残置し、gap ロジックのみ置換。

### 4.3 テレメトリ互換（candidates スコア）

テレメトリの `candidates` と `heat_bins` は、現行では `score = 1/(1+j)`（L357）で算出している。

**重要**: gap 方式の `score = base - pen` は **負になり得る**。そのため `best_j = 1/best_score - 1` のような変換は危険である（best_score が 0 付近や負になると無限大・符号反転で破綻する）。

**推奨**: telemetry 用のスコアを **別定義** にする。

- `tele_score = clamp(depth_mm / FTG_GAP_DEPTH_SAT_MM, 0..1)` を candidates 用に使用
- `best_score` フィールドには gap スコアをそのまま入れる（互換不要なら）、または `tele_score` を入れる
- `1/(1+j)` 形式を維持したい場合は、gap スコアを必ず正にする設計（例: `score = exp(k*(base-pen))`）が必要だが、シンプルさが落ちる

```351:358:rpi/src/Process.cpp
		for (int angle = cfg::FTG_ANGLE_MIN_DEG;
			 angle <= cfg::FTG_ANGLE_MAX_DEG; ++angle) {
			// ...
			const float score = 1.0f / (1.0f + j);
			// ...
			candidates.push_back(CandidateScore{(float)angle, d_mm, score});
```

改修後は `candidates` 用に **telemetry 専用スコア** `tele_score = clamp(d_mm / FTG_GAP_DEPTH_SAT_MM, 0, 1)` を使用する。`best_score` は gap スコアのまま出力するか、選択した gap の `tele_score` を出力する。

---

## 5. 実装の進め方（最短ルート）

| Phase | 内容 | 優先度 |
|-------|------|--------|
| **1** | `proc()` の softmax/コスト選択部分を削除して gap 方式に置換（10Hz のまま） | 高 |
| **1.5** | （必要なら）corridor_min が過小評価のままだと gap が「無い」判定になり得る。Phase 1 の後に改善が足りない場合の保険として、**内外非対称 corridor**（内側厳格/外側緩和）や **内側 side hard-stop** を検討 | 低 |
| **2** | slew を 360deg/s に引き上げ | 高 |
| **3** | turn-cap を追加（舵が追いつくまで減速） | 高 |
| **4** | （将来）`Process` を stateful にして updateScan/tick 分離（制御 100Hz） | 中 |

### 5.1 置換対象コードブロック一覧

| 対象 | 行範囲 | 内容 |
|------|--------|------|
| 角度選択ロジック | L156-216 | `obs_cost`, `jerk_weight`, for ループ（コスト計算・softmax）を gap 抽出・`finalize_gap` に置換 |
| blocked/target 算出 | L224-227 | `z`, `angle_sum` を `found_gap`, `selected_target` に変更。**blocked 時**: gap が無いときは `min_corridor`/`min_angle` を best とする。意図は「最悪方向（最小距離）を観測として残す」ことで、テレメトリや後段の判断に有用 |
| telemetry スコア | （新規） | `tele_score = clamp(depth_mm/FTG_GAP_DEPTH_SAT_MM, 0, 1)` を candidates 用に使用。`best_j` 変換は行わない（gap スコアは負になり得るため危険） |
| ブロック時スナップ | L250-261 付近 | slew 途中で一瞬 NEAR に落ちた場合、gap が安全なら `best_angle` へスナップする分岐を追加 |
| 速度 turn-cap | L310 直後 | `out_speed` 算出後、`warn` チェック前に turn-cap ブロックを挿入 |
| テレメトリ candidates | L345-363 | `jerk_weight`, `obs_cost`, `j` の代わりに `score = d_mm/FTG_GAP_DEPTH_SAT_MM` で candidates を構築 |

### 5.2 依存ヘッダ

gap 方式で `std::sort` を使用するため、`Process.cpp` の `#include <algorithm>`（L5）が既に含まれていることを確認。追加の include は不要。

### 5.3 Process クラスの mutable メンバ

`proc()` は `const` メソッドだが、`last_proc_ts_us_` と `last_best_angle_` を更新するため `mutable` で宣言されている（`Process.h` L34-35）。gap 方式でもこの構造は維持する。

```31:36:rpi/src/Process.h
private:
	TelemetryEmitter *telemetry_;
	mutable bool has_last_best_ = false;
	mutable float last_best_angle_ = 0.0f;
	mutable uint64_t last_proc_ts_us_ = 0;
};
```

---

## 6. 期待される改善

| 項目 | 改善内容 |
|------|----------|
| 方向選択 | 「単に一番遠い 1 点」ではなく「広く深い空間（gap）」を狙うため、壁沿いでも挙動が安定 |
| ステア追従 | `FTG_STEER_SLEW_DEG_PER_S=360` で 10Hz 更新でも舵が追いつく |
| 刺さり抑制 | turn-cap により舵がまだ切れていないときに速度を落とし、刺さりを抑える |

---

## 7. 検証観点

### 7.1 ログ確認（process_telemetry.jsonl）

- `best_angle_deg` / `best_dist_mm` / `best_score`
- `steer_deg` と `raw_steer_deg` の差（slew でどれだけ削られているか）
- `limited_speed` が turn-cap で落ちているか

### 7.2 追加で根本対応（必要なら）

今回のパッチは「10Hz でも追従できるようにする」方向。さらに詰めるなら:

- `proc()` を **LiDAR 更新(10Hz)** と **制御 tick(100Hz)** に分離
- 舵と速度を 100Hz で出す構造にすると挙動が一段滑らかになる

---

## 8. パッチ適用対象ファイル

- `rpi/src/config/Config.h`
- `rpi/src/Process.cpp`
