# 走行プロセス一新 ステート

## 目的
- LiDARビン配列から最遠ビンを選び、通過幅チェック＋平滑化で操舵角を決定する。
- 速度は一旦固定値でハードコードする。

## ステータス
- [完了] Process::proc のアルゴリズム置換（最遠ビン/幅/障害物チェック）
- [完了] 角度方向の平滑化（3〜5bin Moving Average）
- [完了] テレメトリ出力の整合更新
- [完了] 設定値を config/Config.h に集約
- [完了] 距離連動の速度決定
- [完了] 距離＋ステア連動の速度上限

## 影響範囲
- rpi/src/Process.cpp
- rpi/src/Process.h
- rpi/src/Telemetry.cpp
- rpi/src/config/Config.h

## メモ
- ビン分解能は 1deg（-90..90）。
- 距離0はスキャンエラー扱いで「通過可能」として扱う。
