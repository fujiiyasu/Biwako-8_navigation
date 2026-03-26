#!/usr/bin/env python3
"""
BIWAKO-8 Thruster Direction Test Server
ブラウザから移動方向・強度を指定してスラスタを回転させるテスト用アプリ

対応チャンネル:
  rc4 = Yaw   (旋回)
  rc5 = Surge (前後)
  rc6 = Sway  (左右横移動)

起動方法:
  python3 thruster_test_server.py
  ブラウザで http://<robot-ip>:5002/ を開く
"""

import threading
import time
from flask import Flask, request, jsonify

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[WARNING] pymavlink not available. Running in simulation mode.")

app = Flask(__name__)

# ============================================================
# 設定
# ============================================================
MAVLINK_CONNECTION = "udp:127.0.0.1:14551"
PWM_NEU   = 1500
PWM_MIN   = 1100
PWM_MAX   = 1900
PWM_RANGE = 200      # ±200 → 1300〜1700
LOOP_HZ   = 20.0
DT        = 1.0 / LOOP_HZ

# ============================================================
# MAVLink
# ============================================================
master = None
mavlink_lock = threading.Lock()

def connect_mavlink():
    global master
    if not MAVLINK_AVAILABLE:
        return False
    try:
        master = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        master.wait_heartbeat(timeout=5)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        print(f"Connected: system {master.target_system}")
        return True
    except Exception as e:
        print(f"[MAVLink] Connection failed: {e}")
        master = None
        return False

# ============================================================
# RC 状態・送信スレッド
# ============================================================
rc_state = {"rc4": PWM_NEU, "rc5": PWM_NEU, "rc6": PWM_NEU}
send_running = True

def clamp(v):
    return max(PWM_MIN, min(PWM_MAX, int(v)))

def send_loop():
    """20Hz で rc_channels_override を送り続ける"""
    while send_running:
        if master:
            with mavlink_lock:
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    0, 0, 0,
                    rc_state["rc4"],   # yaw
                    rc_state["rc5"],   # surge
                    rc_state["rc6"],   # sway
                    0, 0
                )
        time.sleep(DT)

send_thread = threading.Thread(target=send_loop, daemon=True)
send_thread.start()

# ============================================================
# 制御関数
# ============================================================
def set_channels(surge=0.0, yaw=0.0, sway=0.0):
    """surge/yaw/sway ∈ [-1.0, +1.0]"""
    rc_state["rc5"] = clamp(PWM_NEU + surge * PWM_RANGE)
    rc_state["rc4"] = clamp(PWM_NEU + yaw   * PWM_RANGE)
    rc_state["rc6"] = clamp(PWM_NEU + sway  * PWM_RANGE)

def stop_all():
    rc_state["rc4"] = PWM_NEU
    rc_state["rc5"] = PWM_NEU
    rc_state["rc6"] = PWM_NEU

# ============================================================
# API
# ============================================================
@app.route("/api/status")
def api_status():
    return jsonify({
        "connected": master is not None,
        "rc4_yaw":   rc_state["rc4"],
        "rc5_surge": rc_state["rc5"],
        "rc6_sway":  rc_state["rc6"],
    })

@app.route("/api/move", methods=["POST"])
def api_move():
    """
    Body: { "surge": float, "yaw": float, "sway": float }
    各値 -1.0 〜 +1.0
    """
    d = request.json or {}
    surge = float(d.get("surge", 0.0))
    yaw   = float(d.get("yaw",   0.0))
    sway  = float(d.get("sway",  0.0))
    set_channels(surge=surge, yaw=yaw, sway=sway)
    return jsonify({
        "surge": surge, "yaw": yaw, "sway": sway,
        "rc4": rc_state["rc4"],
        "rc5": rc_state["rc5"],
        "rc6": rc_state["rc6"],
    })

@app.route("/api/stop", methods=["POST"])
def api_stop():
    stop_all()
    return jsonify({"status": "stopped"})

@app.route("/api/reconnect", methods=["POST"])
def api_reconnect():
    ok = connect_mavlink()
    return jsonify({"connected": ok})

# ============================================================
# フロントエンド
# ============================================================
@app.route("/")
def index():
    return r"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BIWAKO-8 Thruster Test</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  background: #0a0e14;
  color: #c8d8e8;
  font-family: 'Courier New', monospace;
  min-height: 100vh;
  padding: 20px;
}
h1 {
  font-size: 16px;
  letter-spacing: 3px;
  color: #00d4ff;
  margin-bottom: 20px;
  padding-bottom: 10px;
  border-bottom: 1px solid #1e3a5f;
}
.layout {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 20px;
  max-width: 900px;
  margin: 0 auto;
}
@media (max-width: 600px) { .layout { grid-template-columns: 1fr; } }

/* ---- パネル ---- */
.panel {
  background: #111720;
  border: 1px solid #1e3a5f;
  border-radius: 6px;
  padding: 16px;
}
.panel-title {
  font-size: 11px;
  letter-spacing: 2px;
  color: #4a6080;
  margin-bottom: 14px;
  text-transform: uppercase;
}

/* ---- ジョイスティック ---- */
.joystick-wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 10px;
}
.joystick-canvas {
  border: 1px solid #1e3a5f;
  border-radius: 50%;
  cursor: crosshair;
  touch-action: none;
  background: #0d1520;
}
.joy-label {
  font-size: 11px;
  color: #4a6080;
  letter-spacing: 1px;
}

/* ---- スライダー ---- */
.slider-row {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 12px;
}
.slider-row label {
  font-size: 11px;
  color: #4a6080;
  width: 60px;
  text-transform: uppercase;
  letter-spacing: 1px;
}
.slider-row input[type=range] {
  flex: 1;
  -webkit-appearance: none;
  height: 4px;
  background: #1e3a5f;
  border-radius: 2px;
  outline: none;
  cursor: pointer;
}
.slider-row input[type=range]::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 16px; height: 16px;
  border-radius: 50%;
  background: #0a0e14;
  border: 2px solid #00d4ff;
  cursor: pointer;
}
.slider-val {
  font-size: 13px;
  min-width: 48px;
  text-align: right;
  color: #00d4ff;
}

/* ---- PWM 表示 ---- */
.pwm-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  margin-top: 6px;
}
.pwm-box {
  background: #0d1520;
  border: 1px solid #1e3a5f;
  border-radius: 4px;
  padding: 10px 6px;
  text-align: center;
}
.pwm-box .ch  { font-size: 10px; color: #4a6080; letter-spacing: 1px; }
.pwm-box .val { font-size: 18px; color: #00d4ff; margin: 4px 0; }
.pwm-box .role{ font-size: 10px; color: #4a6080; }

/* ---- プリセットボタン ---- */
.preset-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 6px;
}
.btn {
  padding: 10px 4px;
  border: 1px solid #1e3a5f;
  border-radius: 4px;
  background: #0d1520;
  color: #c8d8e8;
  font-family: inherit;
  font-size: 11px;
  cursor: pointer;
  letter-spacing: 1px;
  transition: all 0.15s;
  text-align: center;
}
.btn:hover { border-color: #00d4ff; color: #00d4ff; }
.btn.stop  { border-color: #e24b4a; color: #e24b4a; grid-column: 1/-1; }
.btn.stop:hover { background: rgba(226,75,74,0.1); }
.btn.active { border-color: #00d4ff; background: rgba(0,212,255,0.1); color: #00d4ff; }

/* ---- ステータス ---- */
.status-line {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 11px;
  color: #4a6080;
  margin-bottom: 14px;
}
.dot {
  width: 8px; height: 8px;
  border-radius: 50%;
  background: #4a6080;
}
.dot.on  { background: #39ff14; box-shadow: 0 0 6px #39ff14; }
.dot.sim { background: #ff6b35; box-shadow: 0 0 6px #ff6b35; }

/* ---- ベクトル表示 ---- */
#vec-canvas {
  display: block;
  margin: 10px auto 0;
  border: 1px solid #1e3a5f;
  border-radius: 6px;
  background: #0d1520;
}
</style>
</head>
<body>
<h1>BIWAKO-8 / THRUSTER TEST</h1>

<div class="status-line">
  <span class="dot" id="dot"></span>
  <span id="status-txt">Connecting...</span>
  <button class="btn" style="padding:4px 10px;margin-left:auto" onclick="reconnect()">RECONNECT</button>
</div>

<div class="layout">

  <!-- 左: ジョイスティック (surge × sway) -->
  <div class="panel">
    <div class="panel-title">Surge / Sway  —  前後 × 横移動</div>
    <div class="joystick-wrap">
      <canvas id="joy-xy" class="joystick-canvas" width="200" height="200"></canvas>
      <span class="joy-label">横方向: Sway (rc6)  縦方向: Surge (rc5)</span>
    </div>
  </div>

  <!-- 右: Yaw スライダー + プリセット -->
  <div class="panel">
    <div class="panel-title">Yaw  —  旋回 (rc4)</div>
    <div class="slider-row">
      <label>YAW</label>
      <input type="range" id="yaw-slider" min="-100" max="100" value="0"
             oninput="onYawSlider(this.value)">
      <span class="slider-val" id="yaw-val">0.00</span>
    </div>

    <div class="panel-title" style="margin-top:16px">Intensity  —  強度</div>
    <div class="slider-row">
      <label>強度</label>
      <input type="range" id="int-slider" min="0" max="100" value="70"
             oninput="onIntSlider(this.value)">
      <span class="slider-val" id="int-val">0.70</span>
    </div>

    <div class="panel-title" style="margin-top:16px">プリセット</div>
    <div class="preset-grid" id="preset-grid">
      <button class="btn" onclick="preset('fwd')">前進</button>
      <button class="btn" onclick="preset('fwd_r')">前進+右旋回</button>
      <button class="btn" onclick="preset('fwd_l')">前進+左旋回</button>
      <button class="btn" onclick="preset('diag_fr')">斜め右前</button>
      <button class="btn" onclick="preset('diag_fl')">斜め左前</button>
      <button class="btn" onclick="preset('diag_br')">斜め右後</button>
      <button class="btn" onclick="preset('diag_bl')">斜め左後</button>
      <button class="btn" onclick="preset('bwd')">後退</button>
      <button class="btn" onclick="preset('right')">右横移動</button>
      <button class="btn" onclick="preset('left')">左横移動</button>
      <button class="btn" onclick="preset('yaw_r')">右旋回</button>
      <button class="btn" onclick="preset('yaw_l')">左旋回</button>
      <button class="btn stop" onclick="sendStop()">STOP</button>
    </div>
  </div>

  <!-- PWM 表示 -->
  <div class="panel">
    <div class="panel-title">PWM 出力値</div>
    <div class="pwm-grid">
      <div class="pwm-box">
        <div class="ch">RC4</div>
        <div class="val" id="pwm-rc4">1500</div>
        <div class="role">YAW</div>
      </div>
      <div class="pwm-box">
        <div class="ch">RC5</div>
        <div class="val" id="pwm-rc5">1500</div>
        <div class="role">SURGE</div>
      </div>
      <div class="pwm-box">
        <div class="ch">RC6</div>
        <div class="val" id="pwm-rc6">1500</div>
        <div class="role">SWAY</div>
      </div>
    </div>
  </div>

  <!-- ベクトル可視化 -->
  <div class="panel">
    <div class="panel-title">移動ベクトル</div>
    <canvas id="vec-canvas" width="200" height="200"></canvas>
  </div>

</div>

<script>
// ============================================================
// 状態
// ============================================================
let surge = 0, yaw = 0, sway = 0;
let intensity = 0.70;
let joyActive = false;

// ============================================================
// ジョイスティック（surge × sway）
// ============================================================
const joyCanvas = document.getElementById('joy-xy');
const joyCtx    = joyCanvas.getContext('2d');
const joyR = 90, joyCX = 100, joyCY = 100;
let joyX = 0, joyY = 0;

function drawJoy() {
  joyCtx.clearRect(0, 0, 200, 200);
  joyCtx.strokeStyle = '#1e3a5f';
  joyCtx.lineWidth = 0.5;
  // 外円
  joyCtx.beginPath();
  joyCtx.arc(joyCX, joyCY, joyR, 0, Math.PI*2);
  joyCtx.stroke();
  // 十字線
  joyCtx.beginPath();
  joyCtx.moveTo(joyCX - joyR, joyCY); joyCtx.lineTo(joyCX + joyR, joyCY);
  joyCtx.moveTo(joyCX, joyCY - joyR); joyCtx.lineTo(joyCX, joyCY + joyR);
  joyCtx.stroke();
  // スティック位置
  const px = joyCX + joyX * joyR;
  const py = joyCY - joyY * joyR;
  joyCtx.fillStyle = '#00d4ff';
  joyCtx.beginPath();
  joyCtx.arc(px, py, 10, 0, Math.PI*2);
  joyCtx.fill();
  // 中心点
  joyCtx.fillStyle = '#1e3a5f';
  joyCtx.beginPath();
  joyCtx.arc(joyCX, joyCY, 3, 0, Math.PI*2);
  joyCtx.fill();
}

function joyFromEvent(e) {
  const rect = joyCanvas.getBoundingClientRect();
  const cx = rect.left + rect.width/2;
  const cy = rect.top  + rect.height/2;
  const clientX = e.touches ? e.touches[0].clientX : e.clientX;
  const clientY = e.touches ? e.touches[0].clientY : e.clientY;
  const scale = joyCanvas.width / rect.width;
  let nx = (clientX - cx) / (rect.width/2);
  let ny = (clientY - cy) / (rect.height/2);
  const len = Math.sqrt(nx*nx + ny*ny);
  if (len > 1) { nx /= len; ny /= len; }
  joyX = nx;
  joyY = -ny;  // canvas Y軸反転
  sway  =  joyX * intensity;
  surge =  joyY * intensity;
  drawJoy();
  sendMove();
}

function joyEnd() {
  joyActive = false;
  joyX = 0; joyY = 0;
  sway = 0; surge = 0;
  drawJoy();
  sendMove();
}

joyCanvas.addEventListener('mousedown',  e => { joyActive=true; joyFromEvent(e); });
joyCanvas.addEventListener('mousemove',  e => { if(joyActive) joyFromEvent(e); });
joyCanvas.addEventListener('mouseup',    joyEnd);
joyCanvas.addEventListener('mouseleave', joyEnd);
joyCanvas.addEventListener('touchstart', e => { e.preventDefault(); joyActive=true; joyFromEvent(e); }, {passive:false});
joyCanvas.addEventListener('touchmove',  e => { e.preventDefault(); if(joyActive) joyFromEvent(e); }, {passive:false});
joyCanvas.addEventListener('touchend',   joyEnd);
drawJoy();

// ============================================================
// Yaw スライダー
// ============================================================
function onYawSlider(v) {
  yaw = parseFloat(v) / 100 * intensity;
  document.getElementById('yaw-val').textContent = yaw.toFixed(2);
  sendMove();
}

function onIntSlider(v) {
  intensity = parseFloat(v) / 100;
  document.getElementById('int-val').textContent = intensity.toFixed(2);
}

// ============================================================
// プリセット
// ============================================================
const S2 = 0.707;  // 1/√2: 斜め方向の正規化係数
const PRESETS = {
  fwd:     { surge:+1,  yaw:0,    sway:0   },
  bwd:     { surge:-1,  yaw:0,    sway:0   },
  right:   { surge:0,   yaw:0,    sway:+1  },
  left:    { surge:0,   yaw:0,    sway:-1  },
  yaw_r:   { surge:0,   yaw:+1,   sway:0   },
  yaw_l:   { surge:0,   yaw:-1,   sway:0   },
  fwd_r:   { surge:+1,  yaw:+0.5, sway:0   },
  fwd_l:   { surge:+1,  yaw:-0.5, sway:0   },
  diag_fr: { surge:+S2, yaw:0,    sway:+S2 },
  diag_fl: { surge:+S2, yaw:0,    sway:-S2 },
  diag_br: { surge:-S2, yaw:0,    sway:+S2 },
  diag_bl: { surge:-S2, yaw:0,    sway:-S2 },
};

let activePreset = null;

function preset(key) {
  const p = PRESETS[key];
  surge = p.surge * intensity;
  yaw   = p.yaw   * intensity;
  sway  = p.sway  * intensity;
  joyX  = p.sway;
  joyY  = p.surge;
  document.getElementById('yaw-val').textContent = yaw.toFixed(2);
  document.getElementById('yaw-slider').value = Math.round(p.yaw * 100);
  drawJoy();
  sendMove();

  // ボタンハイライト
  document.querySelectorAll('.preset-grid .btn').forEach(b => b.classList.remove('active'));
  if (activePreset !== key) {
    event.target.classList.add('active');
    activePreset = key;
  } else {
    activePreset = null;
    sendStop();
  }
}

// ============================================================
// API 送信
// ============================================================
async function sendMove() {
  try {
    const r = await fetch('/api/move', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ surge, yaw, sway })
    });
    const d = await r.json();
    updatePWM(d.rc4, d.rc5, d.rc6);
    drawVector(d.rc5, d.rc6, d.rc4);
  } catch(e) {}
}

async function sendStop() {
  surge = yaw = sway = 0;
  joyX  = joyY = 0;
  drawJoy();
  document.getElementById('yaw-slider').value = 0;
  document.getElementById('yaw-val').textContent = '0.00';
  document.querySelectorAll('.preset-grid .btn').forEach(b => b.classList.remove('active'));
  activePreset = null;
  try {
    const r = await fetch('/api/stop', {method:'POST'});
    const d = await r.json();
  } catch(e) {}
  updatePWM(1500, 1500, 1500);
  drawVector(1500, 1500, 1500);
}

// ============================================================
// PWM 表示更新
// ============================================================
function updatePWM(rc4, rc5, rc6) {
  document.getElementById('pwm-rc4').textContent = rc4;
  document.getElementById('pwm-rc5').textContent = rc5;
  document.getElementById('pwm-rc6').textContent = rc6;
}

// ============================================================
// ベクトル表示
// ============================================================
const vecCanvas = document.getElementById('vec-canvas');
const vecCtx    = vecCanvas.getContext('2d');

function drawVector(rc5, rc6, rc4) {
  const W = 200, H = 200, cx = 100, cy = 100, R = 80;
  vecCtx.clearRect(0, 0, W, H);

  // 背景円
  vecCtx.strokeStyle = '#1e3a5f';
  vecCtx.lineWidth = 0.5;
  vecCtx.beginPath();
  vecCtx.arc(cx, cy, R, 0, Math.PI*2);
  vecCtx.stroke();
  vecCtx.beginPath();
  vecCtx.moveTo(cx-R, cy); vecCtx.lineTo(cx+R, cy);
  vecCtx.moveTo(cx, cy-R); vecCtx.lineTo(cx, cy+R);
  vecCtx.stroke();

  // surge/sway ベクトル（並進）
  const s_norm = (rc5 - 1500) / 200;
  const sw_norm = (rc6 - 1500) / 200;
  const vx = sw_norm * R;
  const vy = -s_norm * R;

  if (Math.abs(vx) > 1 || Math.abs(vy) > 1) {
    vecCtx.strokeStyle = '#00d4ff';
    vecCtx.lineWidth = 2.5;
    vecCtx.beginPath();
    vecCtx.moveTo(cx, cy);
    vecCtx.lineTo(cx + vx, cy + vy);
    vecCtx.stroke();
    // 矢印頭
    const angle = Math.atan2(vy, vx);
    vecCtx.fillStyle = '#00d4ff';
    vecCtx.beginPath();
    vecCtx.moveTo(cx+vx, cy+vy);
    vecCtx.lineTo(cx+vx - 10*Math.cos(angle-0.4), cy+vy - 10*Math.sin(angle-0.4));
    vecCtx.lineTo(cx+vx - 10*Math.cos(angle+0.4), cy+vy - 10*Math.sin(angle+0.4));
    vecCtx.closePath();
    vecCtx.fill();
  }

  // yaw 弧（旋回）
  const y_norm = (rc4 - 1500) / 200;
  if (Math.abs(y_norm) > 0.02) {
    vecCtx.strokeStyle = '#ff6b35';
    vecCtx.lineWidth = 2;
    const startAngle = -Math.PI/2;
    const endAngle   = startAngle + y_norm * Math.PI;
    vecCtx.beginPath();
    vecCtx.arc(cx, cy, R * 0.55, startAngle,
               endAngle, y_norm < 0);
    vecCtx.stroke();
    // 旋回方向ラベル
    vecCtx.fillStyle = '#ff6b35';
    vecCtx.font = '11px Courier New';
    vecCtx.textAlign = 'center';
    vecCtx.fillText(y_norm > 0 ? 'YAW >' : '< YAW', cx, cy + R*0.55 + 16);
  }

  // ラベル
  vecCtx.fillStyle = '#4a6080';
  vecCtx.font = '10px Courier New';
  vecCtx.textAlign = 'center';
  vecCtx.fillText('FWD', cx, cy - R - 6);
  vecCtx.fillText('BWD', cx, cy + R + 14);
  vecCtx.textAlign = 'left';
  vecCtx.fillText('R', cx + R + 4, cy + 4);
  vecCtx.textAlign = 'right';
  vecCtx.fillText('L', cx - R - 4, cy + 4);
}

// ============================================================
// ステータス確認
// ============================================================
async function checkStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    const dot = document.getElementById('dot');
    const txt = document.getElementById('status-txt');
    if (d.connected) {
      dot.className = 'dot on';
      txt.textContent = 'MAVLink connected';
    } else {
      dot.className = 'dot sim';
      txt.textContent = 'Simulation mode';
    }
    updatePWM(d.rc4_yaw, d.rc5_surge, d.rc6_sway);
    drawVector(d.rc5_surge, d.rc6_sway, d.rc4_yaw);
  } catch(e) {
    document.getElementById('status-txt').textContent = 'Server error';
  }
}

async function reconnect() {
  await fetch('/api/reconnect', {method:'POST'});
  checkStatus();
}

checkStatus();
setInterval(checkStatus, 5000);
drawVector(1500, 1500, 1500);
</script>
</body>
</html>"""

# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("BIWAKO-8 Thruster Test Server")
    print("=" * 50)
    connected = connect_mavlink()
    if not connected:
        print("[INFO] Starting in simulation mode")
    print("\nAccess: http://<robot-ip>:5002/")
    print("Press Ctrl+C to stop\n")
    try:
        app.run(host="0.0.0.0", port=5002, debug=False)
    finally:
        send_running = False
        stop_all()