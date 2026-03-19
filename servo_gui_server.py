#!/usr/bin/env python3
"""
BIWAKO-8 Servo Adjustment GUI Server
WebブラウザからサーボモータのPWM値を調整し、servo_positions.csvに保存するFlaskサーバー
"""

from flask import Flask, request, jsonify, send_file
import csv
import os
import io
import threading
import time

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[WARNING] pymavlink not available. Running in simulation mode.")

app = Flask(__name__)

# === 設定 ===
CSV_PATH = os.path.join(os.path.dirname(__file__), "servo_positions.csv")
MAVLink_CONNECTION = 'udp:127.0.0.1:14551'
VALID_CHANNELS = list(range(9, 17))
PWM_MIN = 700
PWM_MAX = 2300

# === MAVLink接続 ===
master = None
mavlink_lock = threading.Lock()

def connect_mavlink():
    global master
    if not MAVLINK_AVAILABLE:
        return False
    try:
        print(f"Connecting to MAVLink: {MAVLink_CONNECTION}")
        master = mavutil.mavlink_connection(MAVLink_CONNECTION)
        master.wait_heartbeat(timeout=5)
        print(f"Connected: system {master.target_system}, component {master.target_component}")
        return True
    except Exception as e:
        print(f"[MAVLink] Connection failed: {e}")
        master = None
        return False

def set_servo_pwm(channel: int, pwm: int, repeat: int = 3):
    """PWMコマンドを送信"""
    if master is None:
        print(f"[SIM] Servo {channel} → {pwm} µs")
        return True
    try:
        with mavlink_lock:
            for _ in range(repeat):
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    channel,
                    pwm,
                    0, 0, 0, 0, 0
                )
                time.sleep(0.05)
        return True
    except Exception as e:
        print(f"[MAVLink] Send error: {e}")
        return False

# === CSVの読み書き ===
def load_csv():
    """servo_positions.csvを読み込む"""
    servos = {}
    if not os.path.exists(CSV_PATH):
        # デフォルト値（ニュートラル）
        for ch in VALID_CHANNELS:
            servos[ch] = {"open": 1500, "close": 1500}
        return servos
    with open(CSV_PATH, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ch = int(row["servo"])
            servos[ch] = {
                "open": int(row["open"]),
                "close": int(row["close"])
            }
    return servos

def save_csv(servos: dict):
    """servo_positions.csvに書き込む"""
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["servo", "open", "close"])
        writer.writeheader()
        for ch in sorted(servos.keys()):
            writer.writerow({
                "servo": ch,
                "open": servos[ch]["open"],
                "close": servos[ch]["close"]
            })

# === API エンドポイント ===
@app.route("/api/status")
def api_status():
    return jsonify({
        "mavlink_connected": master is not None,
        "simulation_mode": master is None,
        "csv_path": CSV_PATH
    })

@app.route("/api/servos")
def api_get_servos():
    servos = load_csv()
    result = []
    for ch in sorted(servos.keys()):
        result.append({
            "channel": ch,
            "open": servos[ch]["open"],
            "close": servos[ch]["close"]
        })
    return jsonify(result)

@app.route("/api/servo/set", methods=["POST"])
def api_set_servo():
    data = request.json
    channel = data.get("channel")
    pwm = data.get("pwm")

    if channel not in VALID_CHANNELS:
        return jsonify({"success": False, "error": f"Invalid channel: {channel}"}), 400
    if not (PWM_MIN <= pwm <= PWM_MAX):
        return jsonify({"success": False, "error": f"PWM out of range: {pwm} (allowed: {PWM_MIN}–{PWM_MAX})"}), 400

    success = set_servo_pwm(channel, pwm)
    return jsonify({"success": success, "channel": channel, "pwm": pwm})

@app.route("/api/servo/save", methods=["POST"])
def api_save():
    data = request.json  # [{channel, open, close}, ...]
    servos = {}
    for item in data:
        ch = int(item["channel"])
        op = int(item["open"])
        cl = int(item["close"])
        if ch not in VALID_CHANNELS:
            return jsonify({"success": False, "error": f"Invalid channel {ch}"}), 400
        if not (PWM_MIN <= op <= PWM_MAX and PWM_MIN <= cl <= PWM_MAX):
            return jsonify({"success": False, "error": f"PWM out of range for channel {ch}"}), 400
        servos[ch] = {"open": op, "close": cl}
    save_csv(servos)
    return jsonify({"success": True, "saved": len(servos)})

@app.route("/api/servo/preview", methods=["POST"])
def api_preview():
    """全サーボをOpen/Closeに動かしてプレビュー"""
    data = request.json
    mode = data.get("mode", "open")  # "open" or "close"
    servos_data = data.get("servos", [])

    for item in servos_data:
        ch = int(item["channel"])
        pwm = int(item["open"] if mode == "open" else item["close"])
        if ch in VALID_CHANNELS and PWM_MIN <= pwm <= PWM_MAX:
            set_servo_pwm(ch, pwm)
            time.sleep(0.1)

    return jsonify({"success": True, "mode": mode})

@app.route("/api/reconnect", methods=["POST"])
def api_reconnect():
    success = connect_mavlink()
    return jsonify({"success": success, "connected": master is not None})

# === フロントエンド ===
@app.route("/")
def index():
    html = r"""<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BIWAKO-8 Servo Adjuster</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;400;600;700&display=swap');

  :root {
    --bg: #0a0e14;
    --panel: #111720;
    --border: #1e3a5f;
    --accent: #00d4ff;
    --accent2: #ff6b35;
    --accent3: #39ff14;
    --text: #c8d8e8;
    --muted: #4a6080;
    --open-color: #39ff14;
    --close-color: #ff4444;
    --pwm-min: 700;
    --pwm-max: 2300;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Exo 2', sans-serif;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Grid background */
  body::before {
    content: '';
    position: fixed;
    inset: 0;
    background-image:
      linear-gradient(rgba(0,212,255,0.03) 1px, transparent 1px),
      linear-gradient(90deg, rgba(0,212,255,0.03) 1px, transparent 1px);
    background-size: 40px 40px;
    pointer-events: none;
    z-index: 0;
  }

  .container {
    position: relative;
    z-index: 1;
    max-width: 1200px;
    margin: 0 auto;
    padding: 20px;
  }

  /* Header */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 16px 24px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    margin-bottom: 20px;
    position: relative;
    overflow: hidden;
  }
  header::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    animation: scanline 3s linear infinite;
  }
  @keyframes scanline {
    0% { transform: translateX(-100%); }
    100% { transform: translateX(100%); }
  }

  .logo {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .logo-icon {
    width: 36px; height: 36px;
    border: 2px solid var(--accent);
    border-radius: 50%;
    display: flex; align-items: center; justify-content: center;
    font-size: 16px;
    color: var(--accent);
    animation: pulse 2s ease-in-out infinite;
  }
  @keyframes pulse {
    0%, 100% { box-shadow: 0 0 0 0 rgba(0,212,255,0.4); }
    50% { box-shadow: 0 0 0 8px rgba(0,212,255,0); }
  }
  .logo h1 {
    font-size: 18px;
    font-weight: 700;
    letter-spacing: 2px;
    color: var(--accent);
  }
  .logo span {
    font-size: 11px;
    color: var(--muted);
    letter-spacing: 1px;
    display: block;
    font-family: 'Share Tech Mono', monospace;
  }

  /* Status bar */
  .status-bar {
    display: flex;
    align-items: center;
    gap: 16px;
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
  }
  .status-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--muted);
    display: inline-block;
    margin-right: 6px;
    transition: background 0.3s;
  }
  .status-dot.connected { background: var(--accent3); box-shadow: 0 0 6px var(--accent3); }
  .status-dot.simmode { background: var(--accent2); box-shadow: 0 0 6px var(--accent2); }

  /* Action buttons */
  .top-actions {
    display: flex;
    gap: 10px;
    margin-bottom: 20px;
    flex-wrap: wrap;
  }

  .btn {
    padding: 10px 20px;
    border: 1px solid var(--border);
    border-radius: 3px;
    background: var(--panel);
    color: var(--text);
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    letter-spacing: 1px;
    cursor: pointer;
    transition: all 0.2s;
    text-transform: uppercase;
  }
  .btn:hover { border-color: var(--accent); color: var(--accent); }
  .btn.primary {
    border-color: var(--accent);
    color: var(--accent);
    background: rgba(0,212,255,0.05);
  }
  .btn.primary:hover { background: rgba(0,212,255,0.15); }
  .btn.success {
    border-color: var(--accent3);
    color: var(--accent3);
    background: rgba(57,255,20,0.05);
  }
  .btn.success:hover { background: rgba(57,255,20,0.15); }
  .btn.danger {
    border-color: var(--close-color);
    color: var(--close-color);
    background: rgba(255,68,68,0.05);
  }
  .btn.danger:hover { background: rgba(255,68,68,0.15); }
  .btn.warn {
    border-color: var(--accent2);
    color: var(--accent2);
    background: rgba(255,107,53,0.05);
  }
  .btn.warn:hover { background: rgba(255,107,53,0.15); }

  /* Servo grid */
  .servo-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(520px, 1fr));
    gap: 16px;
  }

  .servo-card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 20px;
    transition: border-color 0.2s;
    position: relative;
    overflow: hidden;
  }
  .servo-card.active {
    border-color: var(--accent);
  }
  .servo-card::before {
    content: '';
    position: absolute;
    left: 0; top: 0; bottom: 0;
    width: 3px;
    background: var(--border);
    transition: background 0.2s;
  }
  .servo-card.active::before { background: var(--accent); }

  .servo-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 16px;
  }
  .servo-title {
    font-family: 'Share Tech Mono', monospace;
    font-size: 14px;
    color: var(--accent);
    letter-spacing: 1px;
  }
  .servo-ch-badge {
    font-size: 11px;
    color: var(--muted);
    font-family: 'Share Tech Mono', monospace;
  }

  /* PWM Row */
  .pwm-row {
    margin-bottom: 14px;
  }
  .pwm-label {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
    font-size: 12px;
  }
  .pwm-label-text {
    font-family: 'Share Tech Mono', monospace;
    letter-spacing: 1px;
    color: var(--muted);
    text-transform: uppercase;
  }
  .pwm-label-text.open { color: var(--open-color); }
  .pwm-label-text.close { color: var(--close-color); }

  .pwm-value-display {
    font-family: 'Share Tech Mono', monospace;
    font-size: 16px;
    font-weight: 700;
    min-width: 60px;
    text-align: right;
    transition: color 0.2s;
  }
  .pwm-value-display.open { color: var(--open-color); }
  .pwm-value-display.close { color: var(--close-color); }

  /* Slider with track visualization */
  .slider-container {
    position: relative;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .step-btn {
    width: 28px; height: 28px;
    border: 1px solid var(--border);
    background: var(--bg);
    color: var(--text);
    border-radius: 2px;
    cursor: pointer;
    font-size: 14px;
    font-weight: bold;
    display: flex; align-items: center; justify-content: center;
    transition: all 0.15s;
    flex-shrink: 0;
    user-select: none;
  }
  .step-btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,212,255,0.1); }
  .step-btn:active { transform: scale(0.9); }

  input[type=range] {
    -webkit-appearance: none;
    flex: 1;
    height: 4px;
    border-radius: 2px;
    outline: none;
    cursor: pointer;
  }
  input[type=range].open-range {
    background: linear-gradient(to right, var(--open-color) 0%, var(--open-color) var(--pct, 50%), #1e3a2f var(--pct, 50%), #1e3a2f 100%);
  }
  input[type=range].close-range {
    background: linear-gradient(to right, var(--close-color) 0%, var(--close-color) var(--pct, 50%), #3a1e1e var(--pct, 50%), #3a1e1e 100%);
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 18px; height: 18px;
    border-radius: 50%;
    border: 2px solid;
    cursor: pointer;
    transition: transform 0.1s;
  }
  input[type=range].open-range::-webkit-slider-thumb {
    background: var(--bg);
    border-color: var(--open-color);
    box-shadow: 0 0 6px var(--open-color);
  }
  input[type=range].close-range::-webkit-slider-thumb {
    background: var(--bg);
    border-color: var(--close-color);
    box-shadow: 0 0 6px var(--close-color);
  }
  input[type=range]::-webkit-slider-thumb:hover { transform: scale(1.2); }

  /* Send button row */
  .send-row {
    display: flex;
    gap: 8px;
    margin-top: 12px;
  }
  .send-btn {
    flex: 1;
    padding: 8px;
    border: 1px solid;
    border-radius: 2px;
    background: transparent;
    cursor: pointer;
    font-family: 'Share Tech Mono', monospace;
    font-size: 11px;
    letter-spacing: 1px;
    text-transform: uppercase;
    transition: all 0.15s;
  }
  .send-btn.open {
    border-color: var(--open-color);
    color: var(--open-color);
  }
  .send-btn.open:hover { background: rgba(57,255,20,0.1); }
  .send-btn.close {
    border-color: var(--close-color);
    color: var(--close-color);
  }
  .send-btn.close:hover { background: rgba(255,68,68,0.1); }

  /* Toast */
  .toast-container {
    position: fixed;
    bottom: 24px; right: 24px;
    z-index: 1000;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .toast {
    padding: 12px 20px;
    background: var(--panel);
    border-left: 3px solid var(--accent);
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
    border-radius: 2px;
    animation: slideIn 0.3s ease, fadeOut 0.3s ease 2.7s forwards;
    max-width: 320px;
  }
  .toast.error { border-color: var(--close-color); color: var(--close-color); }
  .toast.success { border-color: var(--accent3); color: var(--accent3); }
  .toast.info { border-color: var(--accent); color: var(--accent); }
  @keyframes slideIn { from { transform: translateX(100px); opacity: 0; } to { transform: translateX(0); opacity: 1; } }
  @keyframes fadeOut { from { opacity: 1; } to { opacity: 0; pointer-events: none; } }

  /* CSV preview */
  .csv-preview {
    margin-top: 20px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 16px;
  }
  .csv-preview h3 {
    font-size: 12px;
    font-family: 'Share Tech Mono', monospace;
    color: var(--muted);
    letter-spacing: 2px;
    margin-bottom: 12px;
    text-transform: uppercase;
  }
  .csv-table {
    width: 100%;
    border-collapse: collapse;
    font-family: 'Share Tech Mono', monospace;
    font-size: 12px;
  }
  .csv-table th {
    text-align: left;
    color: var(--muted);
    padding: 4px 12px;
    border-bottom: 1px solid var(--border);
    font-weight: 400;
    letter-spacing: 1px;
  }
  .csv-table td {
    padding: 6px 12px;
    border-bottom: 1px solid rgba(30,58,95,0.3);
    transition: background 0.2s;
  }
  .csv-table tr:hover td { background: rgba(0,212,255,0.03); }
  .csv-table .val-open { color: var(--open-color); }
  .csv-table .val-close { color: var(--close-color); }

  /* Scroll hint */
  .scroll-hint {
    text-align: center;
    color: var(--muted);
    font-size: 11px;
    font-family: 'Share Tech Mono', monospace;
    letter-spacing: 1px;
    margin-top: 8px;
  }
</style>
</head>
<body>
<div class="container">
  <!-- Header -->
  <header>
    <div class="logo">
      <div class="logo-icon">⚙</div>
      <div>
        <h1>BIWAKO-8</h1>
        <span>SERVO ADJUSTMENT INTERFACE</span>
      </div>
    </div>
    <div class="status-bar">
      <div id="status-indicator">
        <span class="status-dot" id="status-dot"></span>
        <span id="status-text">Connecting...</span>
      </div>
      <button class="btn" onclick="reconnect()">RECONNECT</button>
    </div>
  </header>

  <!-- Top Actions -->
  <div class="top-actions">
    <button class="btn success" onclick="previewAll('open')">▶ PREVIEW OPEN (All)</button>
    <button class="btn danger" onclick="previewAll('close')">▶ PREVIEW CLOSE (All)</button>
    <button class="btn primary" onclick="saveAll()">💾 SAVE TO CSV</button>
    <button class="btn" onclick="loadFromServer()">↺ RELOAD CSV</button>
  </div>

  <!-- Servo Cards -->
  <div class="servo-grid" id="servo-grid">
    <!-- Populated by JS -->
  </div>

  <!-- CSV Preview -->
  <div class="csv-preview">
    <h3>Current Values → servo_positions.csv</h3>
    <table class="csv-table">
      <thead>
        <tr>
          <th>SERVO (CH)</th>
          <th>OPEN (µs)</th>
          <th>CLOSE (µs)</th>
        </tr>
      </thead>
      <tbody id="csv-preview-body"></tbody>
    </table>
  </div>

  <div class="scroll-hint">[ マウスホイール or ボタンでPWM微調整 | クリックで即送信 ]</div>
</div>

<!-- Toast -->
<div class="toast-container" id="toast-container"></div>

<script>
const PWM_MIN = 700, PWM_MAX = 2300;
let servoData = []; // [{channel, open, close}, ...]

// ====== Toast ======
function toast(msg, type='info') {
  const el = document.createElement('div');
  el.className = `toast ${type}`;
  el.textContent = msg;
  document.getElementById('toast-container').appendChild(el);
  setTimeout(() => el.remove(), 3100);
}

// ====== Status ======
async function checkStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    const dot = document.getElementById('status-dot');
    const txt = document.getElementById('status-text');
    if (d.mavlink_connected) {
      dot.className = 'status-dot connected';
      txt.textContent = 'MAVLink Connected';
    } else {
      dot.className = 'status-dot simmode';
      txt.textContent = 'Simulation Mode';
    }
  } catch(e) {
    document.getElementById('status-text').textContent = 'Server Error';
  }
}

async function reconnect() {
  toast('Reconnecting...', 'info');
  const r = await fetch('/api/reconnect', {method:'POST'});
  const d = await r.json();
  checkStatus();
  toast(d.connected ? 'MAVLink connected!' : 'Running in simulation mode', d.connected ? 'success' : 'info');
}

// ====== Load Servos ======
async function loadFromServer() {
  const r = await fetch('/api/servos');
  servoData = await r.json();
  renderCards();
  updateCsvPreview();
  toast('Loaded from server', 'info');
}

// ====== Render Cards ======
function renderCards() {
  const grid = document.getElementById('servo-grid');
  grid.innerHTML = '';

  servoData.forEach(s => {
    const card = document.createElement('div');
    card.className = 'servo-card';
    card.id = `card-${s.channel}`;

    const openPct = pct(s.open);
    const closePct = pct(s.close);

    card.innerHTML = `
      <div class="servo-header">
        <span class="servo-title">SERVO ${s.channel - 8}</span>
        <span class="servo-ch-badge">CH ${s.channel} &nbsp;|&nbsp; PWM ${PWM_MIN}–${PWM_MAX} µs</span>
      </div>

      <!-- OPEN -->
      <div class="pwm-row">
        <div class="pwm-label">
          <span class="pwm-label-text open">OPEN</span>
          <span class="pwm-value-display open" id="open-val-${s.channel}">${s.open} µs</span>
        </div>
        <div class="slider-container">
          <button class="step-btn" onclick="step(${s.channel},'open',-1)" title="-10µs">‹</button>
          <input type="range" class="open-range" id="open-slider-${s.channel}"
            min="${PWM_MIN}" max="${PWM_MAX}" value="${s.open}"
            style="--pct: ${openPct}%"
            oninput="onSlider(${s.channel},'open',this.value)"
            onwheel="onWheel(event,${s.channel},'open')">
          <button class="step-btn" onclick="step(${s.channel},'open',+1)" title="+10µs">›</button>
        </div>
      </div>

      <!-- CLOSE -->
      <div class="pwm-row">
        <div class="pwm-label">
          <span class="pwm-label-text close">CLOSE</span>
          <span class="pwm-value-display close" id="close-val-${s.channel}">${s.close} µs</span>
        </div>
        <div class="slider-container">
          <button class="step-btn" onclick="step(${s.channel},'close',-1)" title="-10µs">‹</button>
          <input type="range" class="close-range" id="close-slider-${s.channel}"
            min="${PWM_MIN}" max="${PWM_MAX}" value="${s.close}"
            style="--pct: ${closePct}%"
            oninput="onSlider(${s.channel},'close',this.value)"
            onwheel="onWheel(event,${s.channel},'close')">
          <button class="step-btn" onclick="step(${s.channel},'close',+1)" title="+10µs">›</button>
        </div>
      </div>

      <!-- Send Buttons -->
      <div class="send-row">
        <button class="send-btn open" onclick="sendPwm(${s.channel},'open')">▶ SEND OPEN</button>
        <button class="send-btn close" onclick="sendPwm(${s.channel},'close')">▶ SEND CLOSE</button>
      </div>
    `;
    grid.appendChild(card);

    // Wheel event on sliders (passive: false to allow preventDefault)
    ['open','close'].forEach(mode => {
      document.getElementById(`${mode}-slider-${s.channel}`)
        .addEventListener('wheel', (e) => onWheel(e, s.channel, mode), {passive: false});
    });
  });
}

function pct(val) {
  return ((val - PWM_MIN) / (PWM_MAX - PWM_MIN) * 100).toFixed(1);
}

// ====== Slider / Wheel / Step ======
function onSlider(ch, mode, rawVal) {
  const val = parseInt(rawVal);
  updateValue(ch, mode, val, false);
}

function onWheel(e, ch, mode) {
  e.preventDefault();
  const delta = e.deltaY < 0 ? +10 : -10;
  const shiftDelta = e.shiftKey ? delta * 10 : delta; // Shift = ±100
  step(ch, mode, shiftDelta);
}

function step(ch, mode, delta) {
  const current = getServoValue(ch, mode);
  const newVal = Math.max(PWM_MIN, Math.min(PWM_MAX, current + delta));
  updateValue(ch, mode, newVal, false);
}

function getServoValue(ch, mode) {
  const s = servoData.find(x => x.channel === ch);
  return s ? s[mode] : 1500;
}

function updateValue(ch, mode, val, send=false) {
  // Update data
  const s = servoData.find(x => x.channel === ch);
  if (!s) return;
  s[mode] = val;

  // Update UI
  document.getElementById(`${mode}-val-${ch}`).textContent = `${val} µs`;
  const slider = document.getElementById(`${mode}-slider-${ch}`);
  slider.value = val;
  slider.style.setProperty('--pct', pct(val) + '%');

  // Activate card
  document.getElementById(`card-${ch}`).classList.add('active');

  // Update CSV preview
  updateCsvPreview();

  if (send) sendPwm(ch, mode);
}

// ====== Send PWM ======
async function sendPwm(ch, mode) {
  const s = servoData.find(x => x.channel === ch);
  if (!s) return;
  const pwm = s[mode];
  try {
    const r = await fetch('/api/servo/set', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({channel: ch, pwm})
    });
    const d = await r.json();
    if (d.success) {
      toast(`CH${ch} → ${pwm}µs (${mode.toUpperCase()})`, 'success');
    } else {
      toast(`Error: ${d.error}`, 'error');
    }
  } catch(e) {
    toast('Network error', 'error');
  }
}

// ====== Preview All ======
async function previewAll(mode) {
  try {
    const r = await fetch('/api/servo/preview', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({mode, servos: servoData})
    });
    const d = await r.json();
    toast(`All servos → ${mode.toUpperCase()}`, d.success ? 'success' : 'error');
  } catch(e) {
    toast('Network error', 'error');
  }
}

// ====== Save All ======
async function saveAll() {
  try {
    const r = await fetch('/api/servo/save', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(servoData)
    });
    const d = await r.json();
    if (d.success) {
      toast(`Saved ${d.saved} servos to servo_positions.csv ✓`, 'success');
    } else {
      toast(`Save error: ${d.error}`, 'error');
    }
  } catch(e) {
    toast('Network error', 'error');
  }
}

// ====== CSV Preview Table ======
function updateCsvPreview() {
  const tbody = document.getElementById('csv-preview-body');
  tbody.innerHTML = servoData.map(s => `
    <tr>
      <td>CH ${s.channel}</td>
      <td class="val-open">${s.open}</td>
      <td class="val-close">${s.close}</td>
    </tr>
  `).join('');
}

// ====== Init ======
checkStatus();
loadFromServer();
setInterval(checkStatus, 10000);
</script>
</body>
</html>"""
    return html

if __name__ == "__main__":
    print("="*50)
    print("BIWAKO-8 Servo Adjustment GUI Server")
    print("="*50)
    print(f"CSV path: {CSV_PATH}")
    
    # MAVLink接続試行（失敗してもサーバーは起動）
    connected = connect_mavlink()
    if not connected:
        print("[INFO] Starting in simulation mode (no MAVLink)")
    
    print("\nAccess: http://<robot-ip>:5001/")
    print("Press Ctrl+C to stop\n")
    
    app.run(host="0.0.0.0", port=5001, debug=False)