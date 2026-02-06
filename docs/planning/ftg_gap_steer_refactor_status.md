# FTG Gap ベース改修・ステア追従改善 ステータス

## 概要

- **対象**: `docs/planning/ftg_gap_steer_refactor_plan.md`
- **目的**: proc を gap ベースに置換し、ステア追従を改善する
- **Updated**: 2026-02-07

---

## 進捗サマリ

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | proc() を gap 方式に置換 | 完了 |
| 1.5 | （必要なら）内外非対称 corridor / 内側 side hard-stop | 保留 |
| 2 | slew を 360deg/s に引き上げ | 完了 |
| 3 | turn-cap を追加 | 完了 |
| 4 | updateScan/tick 分離（制御 100Hz） | 保留 |

---

## ステート

| 項目 | 状態 | 更新日 | メモ |
| --- | --- | --- | --- |
| 詳細改修計画書作成 | 完了 | 2026-02-07 | `ftg_gap_steer_refactor_plan.md` |
| 計画書フィードバック反映 | 完了 | 2026-02-07 | best_score危険性、FTG_GAP_FREE_MM/MIN_WIDTH、Phase1.5、slew意図、blocked意図等 |
| ステータスファイル作成 | 完了 | 2026-02-07 | 本ファイル |
| Phase 1: gap 方式実装 | 完了 | 2026-02-07 | softmax/コスト削除、gap 抽出・スコアリング |
| Phase 1.5: corridor 保険 | 保留 | - | 必要なら内外非対称 corridor / 内側 hard-stop |
| Phase 2: slew 引き上げ | 完了 | 2026-02-07 | Config.h `FTG_STEER_SLEW_DEG_PER_S=360` |
| Phase 3: turn-cap 実装 | 完了 | 2026-02-07 | 舵追従時間に応じた速度上限 |
| Phase 4: 制御分離 | 保留 | - | 必要に応じて検討 |
| ビルド・単体テスト | 要確認 | - | 環境により `make rpi` が失敗する場合あり |
| 実機検証 | 未着手 | - | テレメトリログ確認、刺さり/蛇行の観察 |

---

## 次のアクション

1. ~~計画書のパッチを `Config.h` および `Process.cpp` に適用~~ 完了
2. `make rpi` でビルド確認（Linux/RPi 環境推奨）
3. 実機または HILS でテレメトリログを取得し、`best_angle_deg` / `steer_deg` / `raw_steer_deg` / `limited_speed` を確認
