# BIWAKO-8 ナビゲーション・サーボ制御 README

## リポジトリ構成

```
biwako8_navigation/
├── biwako8_fsm_navigation.py     # FSMナビゲーション（本番用）
├── servo_gui_server.py           # サーボ調整GUI（Webブラウザ）
├── servo_positions.csv           # サーボPWM設定値
├── waypoints.csv                 # ウェイポイント一覧
├── navigation_test.py            # ナビゲーションテスト
├── station_keeping_nav_test.py   # 定点維持ナビゲーションテスト
├── station_keeping_thruster_test.py      # スラスタ手動テスト（定点維持）
├── station_keeping_thruster_test_app.py  # スラスタ方向テスト（ブラウザUI付き）
├── straight_nav_test.py          # 直進移動ナビゲーションテスト（PD制御）
├── straight_thruster_test.py     # スラスタ手動テスト（直進移動）
├── open_close_test.py            # サーボOPEN/CLOSEテスト
├── servo_mode_selector.py        # サーボモード選択（OPEN/CLOSE）
├── .gitignore
├── fig/                          # FSM図など
└── csv/                          # 実験データログ（gitignore済み）
```

---

## 1. サーボ調整 GUI（`servo_gui_server.py`）

### 概要

ブラウザからサーボモータのPWM値をリアルタイムに調整し、`servo_positions.csv` に保存するWebアプリ。
BIWAKO-8のOPEN（定点維持モード）/CLOSE（直進移動モード）の形状切り替えに対応。

### 起動方法

```bash
# Flask が未インストールの場合
pip install flask

# サーボ調整GUIサーバーを起動
cd ~/biwako8_navigation
python3 servo_gui_server.py
```

### アクセス

ロボットのIPアドレスを確認：

```bash
ip addr show wlan0
```

ブラウザで以下にアクセス：

```
http://<ロボットのIPアドレス>:<PORT>/
```

> BlueOSのデフォルト（ポート80）とは別ポートなので競合しない。

### 操作方法

| 操作 | 方法 |
|------|------|
| PWM微調整（±1µs） | `‹` `›` ボタン |
| PWM調整（±10µs） | スライダー上でマウスホイール |
| PWM高速調整（±100µs） | Shift + マウスホイール |
| サーボに即時送信 | `SEND OPEN` / `SEND CLOSE` ボタン |
| 全サーボ一括プレビュー | `PREVIEW OPEN (All)` / `PREVIEW CLOSE (All)` |
| CSV保存 | `SAVE TO CSV` ボタン |
| CSVから再読み込み | `RELOAD CSV` ボタン |
| MAVLink再接続 | `RECONNECT` ボタン |

### MAVLink接続状態

- 🟢 緑：MAVLink接続済み（実機にPWM送信される）
- 🟠 橙：シミュレーションモード（ログ出力のみ）

### servo_positions.csv の形式

```csv
servo,open,close
9,1100,2000
10,850,2250
...
```

- `servo`：チャンネル番号（9〜16）
- `open`：定点維持モード時のPWM値 [µs]
- `close`：直進移動モード時のPWM値 [µs]
- PWM範囲：700〜2300 µs

> **注意**：`servo_positions.csv` は `git update-index --assume-unchanged` で変更追跡を無効化しているため、`git status` に表示されない。

---

## 2. ナビゲーションプログラム

### 2-1. FSMナビゲーション（`biwako8_fsm_navigation.py`）＜本番用＞

定点維持モードと直進移動モードを自動で切り替えながら、複数ウェイポイントを巡回する本番用プログラム。

#### 実験モードの設定

ファイル冒頭の `EXPERIMENT_MODE` を変更して実行する：

```python
EXPERIMENT_MODE = "HYBRID"      # 変形あり（定点維持 ↔ 直進移動を自動切替）
# EXPERIMENT_MODE = "TRAVEL_ONLY"  # 直進移動のみ（変形なし）
# EXPERIMENT_MODE = "KEEP_ONLY"    # 定点維持のみ（変形なし）
```

#### 主なパラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `DIST_TOLERANCE` | 1.5 m | ウェイポイント到達判定距離 |
| `KEEP_TIME` | 300.0 s | 定点維持時間（5分） |
| `MAX_LAPS` | 3 | 巡回周回数 |
| `Kp_yaw` | 1.2 | ヨー制御ゲイン |

#### 実行方法

```bash
python3 biwako8_fsm_navigation.py
```

#### ログファイル

実行ごとに自動生成：

```
robot_log_HYBRID_20251219070840.csv
```

ログ列：`timestamp, experiment_mode, lat, lon, wp_lat, wp_lon, distance, bearing_diff, action, roll, pitch, yaw, voltage, current, remaining, power, energy_Wh, state`

#### FSM状態遷移

```
Mission Start
    ↓
[TRAVEL] ウェイポイントへ向かって移動
    ↓ 距離 ≤ 1.5m
[KEEP] 定点維持（5分間）
    ↓ 5分経過 → 次のウェイポイントへ
    ↓ 3周完了
Mission End
```

---

### 2-2. テスト用プログラム

#### `navigation_test.py` — 基本ナビゲーションテスト

スラスタ制御なし。GPS・バッテリーのログ取得とウェイポイント到達判定のみ確認する。

```bash
python3 navigation_test.py
```

---

#### `station_keeping_nav_test.py` — 定点維持ナビゲーションテスト

定点維持モードのスラスタ制御（FORWARD/BACKWARD/LEFT/RIGHT）でウェイポイントを巡回。

```bash
python3 station_keeping_nav_test.py
```

---

#### `straight_nav_test.py` — 直進移動ナビゲーションテスト（PD制御）

PD制御によるSurge/Yaw制御でウェイポイントを巡回。初回GPS同期あり。

```bash
python3 straight_nav_test.py
```

主なPDゲイン設定：

```python
Kp_d  = 1.0   # 距離ゲイン
Kd_d  = 0.3   # 距離微分ゲイン
Kp_th = 1.5   # 方位ゲイン
Kd_th = 0.4   # 方位微分ゲイン
```

---

#### `station_keeping_thruster_test.py` — スラスタ手動テスト（定点維持モード）

キーボード入力でスラスタを手動制御する陸上確認用。

```bash
python3 station_keeping_thruster_test.py
```

コマンド：`1`=前進 / `2`=後退 / `3`=右回転 / `4`=左回転 / `0`=停止 / `q`=終了

---

#### `station_keeping_thruster_test_app.py` — スラスタ方向テスト（ブラウザUI付き）

ブラウザから移動方向・強度を指定してスラスタを動作させるWebアプリ。
rc4（Yaw）・rc5（Surge）・rc6（Sway）の3チャンネルを操作でき、前後左右・斜め4方向・旋回の計12方向をプリセットから選択できる。
斜め方向は合成推力が前後左右と等しくなるよう1/√2に正規化済み。

```bash
python3 station_keeping_thruster_test_app.py
```

ブラウザで以下にアクセス：

```
http://<ロボットのIPアドレス>:5002/
```

| 操作 | 方法 |
|------|------|
| 前後・横移動 | ジョイスティックをドラッグ |
| 旋回 | Yaw スライダー |
| 強度調整 | Intensity スライダー（0〜1.0） |
| 方向プリセット | 各方向ボタン（同じボタンを再度押すと停止） |
| 緊急停止 | STOP ボタン |

---

#### `straight_thruster_test.py` — スラスタ手動テスト（直進移動モード）

Surge/Yaw制御のテストとPDシミュレーションコマンドを持つ陸上確認用。

```bash
python3 straight_thruster_test.py
```

コマンド：`w/s/a/d`=前後左右 / `t1〜t4`=PDシミュレーション / `0`=停止 / `x`=終了

---

#### `open_close_test.py` — サーボOPEN/CLOSE繰り返しテスト

全サーボをOPEN→CLOSEと繰り返し動作させて変形動作を確認する。

```bash
python3 open_close_test.py
```

---

#### `servo_mode_selector.py` — サーボモード手動選択

OPEN（定点維持形状）またはCLOSE（直進移動形状）に手動で移行させる。

```bash
python3 servo_mode_selector.py
# → 0: OPEN, 1: CLOSE
```

---

## 3. ウェイポイントの設定（`waypoints.csv`）

```csv
lat,lon
34.xxxxxx,135.xxxxxx
34.oooooo,135.oooooo
...
```

Google マップ等で緯度・経度を取得して記入する。現在は4点×3周分（計12行）が設定済み。

---

## 4. よくある作業メモ

### サーボ値を調整したい

1. `python3 servo_gui_server.py` を起動
2. ブラウザで `http://<IP>:<PORT>/` にアクセス
3. スライダーやボタンでPWM値を調整 → `SEND` で確認
4. `SAVE TO CSV` で保存

### ナビゲーション実験を実行したい

1. `waypoints.csv` にウェイポイントを設定
2. `biwako8_fsm_navigation.py` の `EXPERIMENT_MODE` を設定
3. `python3 biwako8_fsm_navigation.py` を実行
4. ログは `robot_log_<MODE>_<timestamp>.csv` に自動保存