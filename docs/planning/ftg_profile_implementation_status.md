# FTG プロファイル切り替え 実装ステータス

## 概要

実行時に 0..3 のプロファイル（守り→攻め）を切り替え、Telemetry UI/JSON に現在プロファイルを表示する機能。

## 状態

| 項目 | 状態 | 備考 |
|------|------|------|
| Profile.h 作成 | 完了 | rpi/src/config/Profile.h |
| CLI 解析（./robo-racer 1, --profile N） | 完了 | main.cpp |
| Process へのプロファイル渡し | 完了 | Process.h / Process.cpp |
| 運転パラメータのプロファイル上書き | 完了 | Process::proc() 内 |
| TelemetrySample への profile_id / speed_cap 追加 | 完了 | Telemetry.h |
| UI の PROF 表示 | 完了 | 3行目に色付きで別行表示（SAFE=緑, MID=シアン, FAST=黄, ATTACK=赤） |
| JSON への profile 出力 | 完了 | profile.id, profile.name |

## 使い方

```bash
# 守り
./robo-racer 0

# 現状（デフォルトと同じ）
./robo-racer 1

# 攻め寄り
./robo-racer 2

# 攻め
./robo-racer 3

# 明示指定（先頭 positional を汚したくない場合）
./robo-racer --profile 3
```

## プロファイルパラメータ

| プロファイル | 名前 | 特徴 |
|--------------|------|------|
| 0 | SAFE | 距離しきい値大、速度上限低、予測マージン大 |
| 1 | MID | 現状値と一致（デフォルト） |
| 2 | FAST | 攻め寄り、steer_speed_floor 0.2、gap ペナルティ減少 |
| 3 | ATTACK | 攻め、steer_speed_floor 0.3、最も積極的 |

## 変更ファイル一覧

- `rpi/src/config/Profile.h`（新規）
- `rpi/src/main.cpp`
- `rpi/src/Process.h`
- `rpi/src/Process.cpp`
- `rpi/src/Telemetry.h`
- `rpi/src/Telemetry.cpp`
