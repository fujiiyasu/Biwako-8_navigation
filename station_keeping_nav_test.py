import math
import time
import csv
import argparse
from datetime import datetime
from pymavlink import mavutil

# ==== 実行時オプション ====
# 使用例:
#   python3 station_keeping_nav_test.py --mode 4dir   # 4方向（デフォルト）
#   python3 station_keeping_nav_test.py --mode 8dir   # 8方向
#   python3 station_keeping_nav_test.py --mode yaw    # 旋回（straight-travelと同制御則）
parser = argparse.ArgumentParser(description="Station-keeping navigation test")
parser.add_argument(
    "--mode",
    choices=["4dir", "8dir", "yaw"],
    default="4dir",
    help="Control mode: 4dir (default), 8dir, yaw"
)
args = parser.parse_args()
CONTROL_MODE = args.mode

# ==== 設定 ====
WAYPOINT_FILE = "waypoints.csv"
DIST_THRESHOLD = 1.5
LOOP_HZ = 10.0
DT = 1.0 / LOOP_HZ

timestamp_str = datetime.now().strftime("%Y%m%d%H%M%S")
LOG_FILE = f"robot_log_{CONTROL_MODE}_{timestamp_str}.csv"

PWM_MIN, PWM_MAX = 1100, 1900
NEU = 1500
MOVE_PWM     = 1700
MOVE_PWM_REV = 3000 - MOVE_PWM          # 1300
MOVE_PWM_DIAG     = int(NEU + (MOVE_PWM - NEU) * 0.707)      # 1641
MOVE_PWM_DIAG_REV = int(NEU - (MOVE_PWM - NEU) * 0.707)      # 1359

# yawモード用PDゲイン（straight_nav_test.pyと同じ値）
PWM_RANGE   = 200
Kp_d  = 1.0
Kd_d  = 0.3
Kp_th = 1.5
Kd_th = 0.4
DIST_NORM_SCALE = 1.0
YAW_NORM_SCALE  = 90.0

RAMP_US_PER_SEC = 300.0
RAMP_US = RAMP_US_PER_SEC * DT
EMA_ALPHA = 0.5

# ==== MAVLink 接続 ====
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")
print(f"Control mode: {CONTROL_MODE}")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 147, 0, 0, 0, 0, 0, 0        # BATTERY_STATUS 最速
)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0    # GLOBAL_POSITION_INT 10Hz
)

# ==== RC制御用変数 ====
curr   = {f'rc{i}': NEU for i in range(1, 9)}
target = curr.copy()

def clamp(v):
    return max(PWM_MIN, min(PWM_MAX, v))

def update_channels():
    for k in curr:
        diff = target[k] - curr[k]
        if diff > RAMP_US:    curr[k] += RAMP_US
        elif diff < -RAMP_US: curr[k] -= RAMP_US
        else:                 curr[k] = target[k]
        curr[k] = EMA_ALPHA * curr[k] + (1 - EMA_ALPHA) * curr[k]
        curr[k] = clamp(int(curr[k]))
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        curr['rc1'], curr['rc2'], curr['rc3'], curr['rc4'],
        curr['rc5'], curr['rc6'], curr['rc7'], curr['rc8']
    )

# 4dir / 8dir 用（rc5=surge, rc6=sway）
def set_target(rc5=NEU, rc6=NEU):
    target['rc5'] = rc5
    target['rc6'] = rc6

# yaw モード用（rc5=surge, rc4=yaw）
def set_surge_yaw(surge, yaw):
    target['rc5'] = clamp(int(NEU + surge * PWM_RANGE))
    target['rc4'] = clamp(int(NEU + yaw   * PWM_RANGE))

def stop():
    target['rc4'] = NEU
    target['rc5'] = NEU
    target['rc6'] = NEU

# ==== バッテリキャッシュ ====
last_batt = {"V": None, "I": None, "Rem": None, "P": None, "t": None}

def harvest_messages(max_reads=20):
    for _ in range(max_reads):
        m = master.recv_match(blocking=False)
        if not m:
            break
        if m.get_type() == 'BATTERY_STATUS':
            voltages = [v / 1000.0 for v in m.voltages if v != 65535]
            V   = sum(voltages) if voltages else None
            I   = m.current_battery / 100.0 if m.current_battery != -1 else None
            Rem = m.battery_remaining if m.battery_remaining != -1 else None
            P   = V * I if (V and I) else None
            last_batt.update({"V": V, "I": I, "Rem": Rem, "P": P, "t": time.time()})

# ==== 位置・方位 ====
def get_position_heading_nonblocking(last_pos):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat     = msg.lat / 1e7
        lon     = msg.lon / 1e7
        heading = msg.hdg / 100.0 if msg.hdg != 65535 else last_pos[2]
        return lat, lon, heading
    return last_pos

# ==== ログ ====
def log_data(timestamp, pos, waypoint, distance, diff, action,
             voltage, current, remaining, power, energy_Wh):
    with open(LOG_FILE, "a", newline="") as f:
        csv.writer(f).writerow([
            timestamp, pos[0], pos[1],
            waypoint[0], waypoint[1],
            round(distance, 2), round(diff, 2),
            action,
            round(voltage,  2) if voltage  else None,
            round(current,  2) if current  else None,
            remaining,
            round(power,    2) if power    else None,
            round(energy_Wh, 4)
        ])

# ==== 補助関数 ====
def load_waypoints(filename):
    waypoints = []
    with open(filename, newline="") as f:
        for row in csv.DictReader(f):
            waypoints.append((float(row["lat"]), float(row["lon"])))
    return waypoints

def calc_distance_and_bearing(pos1, pos2):
    dx = (pos2[1] - pos1[1]) * 111000 * math.cos(math.radians(pos1[0]))
    dy = (pos2[0] - pos1[0]) * 111000
    distance = math.sqrt(dx**2 + dy**2)
    bearing  = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    return distance, bearing

def bearing_diff(current_heading, target_bearing):
    return (target_bearing - current_heading + 180) % 360 - 180

# ============================================================
# 制御則（変更箇所 1/2）
# ============================================================
def decide_action(diff):
    """bearing_diff [deg] からアクション文字列を返す"""

    if CONTROL_MODE == "4dir":
        if   -45  <= diff <  45:  return "FORWARD"
        elif  45  <= diff < 135:  return "RIGHT"
        elif -135 <= diff < -45:  return "LEFT"
        else:                     return "BACKWARD"

    elif CONTROL_MODE == "8dir":
        if   -22.5 <= diff <  22.5: return "FORWARD"
        elif  22.5 <= diff <  67.5: return "FWD_RIGHT"
        elif  67.5 <= diff < 112.5: return "RIGHT"
        elif 112.5 <= diff < 157.5: return "BWD_RIGHT"
        elif diff >= 157.5 or diff < -157.5: return "BACKWARD"
        elif -157.5 <= diff < -112.5: return "BWD_LEFT"
        elif -112.5 <= diff <  -67.5: return "LEFT"
        else:                         return "FWD_LEFT"   # -67.5 ~ -22.5

    else:  # yaw: アクションはPD出力値の文字列で返す（後述）
        return None  # apply_control() 内で直接計算

# yaw モード用PD状態
_last_dist_norm = 0.0
_last_theta_norm = 0.0

def apply_control(distance, diff):
    """
    モードに応じてスラスタ指令を送り、ログ用アクション文字列を返す。
    """
    global _last_dist_norm, _last_theta_norm

    # ---- 4dir ----
    if CONTROL_MODE == "4dir":
        action = decide_action(diff)
        if   action == "FORWARD":  set_target(rc5=MOVE_PWM,     rc6=NEU)
        elif action == "BACKWARD": set_target(rc5=MOVE_PWM_REV, rc6=NEU)
        elif action == "RIGHT":    set_target(rc5=NEU,           rc6=MOVE_PWM)
        elif action == "LEFT":     set_target(rc5=NEU,           rc6=MOVE_PWM_REV)
        else:                      set_target()
        return action

    # ---- 8dir ----
    elif CONTROL_MODE == "8dir":
        action = decide_action(diff)
        if   action == "FORWARD":    set_target(rc5=MOVE_PWM,      rc6=NEU)
        elif action == "BACKWARD":   set_target(rc5=MOVE_PWM_REV,  rc6=NEU)
        elif action == "RIGHT":      set_target(rc5=NEU,            rc6=MOVE_PWM)
        elif action == "LEFT":       set_target(rc5=NEU,            rc6=MOVE_PWM_REV)
        elif action == "FWD_RIGHT":  set_target(rc5=MOVE_PWM_DIAG, rc6=MOVE_PWM_DIAG)
        elif action == "FWD_LEFT":   set_target(rc5=MOVE_PWM_DIAG, rc6=MOVE_PWM_DIAG_REV)
        elif action == "BWD_RIGHT":  set_target(rc5=MOVE_PWM_DIAG_REV, rc6=MOVE_PWM_DIAG)
        elif action == "BWD_LEFT":   set_target(rc5=MOVE_PWM_DIAG_REV, rc6=MOVE_PWM_DIAG_REV)
        else:                        set_target()
        return action

    # ---- yaw（straight-travelと同制御則） ----
    else:
        dist_norm  = min(distance / DIST_NORM_SCALE, 1.0)
        theta_norm = max(-1.0, min(1.0, diff / YAW_NORM_SCALE))

        P_surge = Kp_d  * dist_norm
        P_yaw   = Kp_th * theta_norm
        D_surge = Kd_d  * (dist_norm  - _last_dist_norm)  / DT
        D_yaw   = Kd_th * (theta_norm - _last_theta_norm) / DT

        surge = max(-1.0, min(1.0, P_surge + D_surge))
        yaw   = max(-1.0, min(1.0, P_yaw   + D_yaw))

        _last_dist_norm  = dist_norm
        _last_theta_norm = theta_norm

        set_surge_yaw(surge, yaw)
        return f"surge={surge:.3f},yaw={yaw:.3f}"

# ==== メインループ ====
def main():
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow([
            "timestamp", "lat", "lon", "wp_lat", "wp_lon",
            "distance", "bearing_diff", "action",
            "voltage_V", "current_A", "remaining_%", "power_W", "energy_Wh"
        ])

    waypoints    = load_waypoints(WAYPOINT_FILE)
    wp_index     = 0
    energy_Wh    = 0.0
    last_energy_t = time.time()
    last_pos     = (0.0, 0.0, 0.0)

    print(f"=== Start station-keeping nav test  mode={CONTROL_MODE} ===")

    while wp_index < len(waypoints):
        loop_start = time.time()

        # === 位置・方位 ===
        lat, lon, heading = get_position_heading_nonblocking(last_pos)
        last_pos    = (lat, lon, heading)
        current_pos = [lat, lon]

        target_wp = waypoints[wp_index]
        distance, target_bearing = calc_distance_and_bearing(current_pos, target_wp)
        diff = bearing_diff(heading, target_bearing) if heading is not None else 0.0

        # === 制御（変更箇所 2/2） ===
        action = apply_control(distance, diff)

        # === スラスタ更新 ===
        update_channels()

        # === バッテリ更新 ===
        harvest_messages()
        V, I, Rem, P, _ = (last_batt[k] for k in ("V", "I", "Rem", "P", "t"))

        # === エネルギー積分 ===
        now = time.time()
        if P is not None:
            energy_Wh += P * ((now - last_energy_t) / 3600.0)
        last_energy_t = now

        # === ログ ===
        log_data(datetime.now().isoformat(), current_pos, target_wp,
                 distance, diff, action, V, I, Rem, P, energy_Wh)

        print(f"[{wp_index}] dist={distance:.2f}m diff={diff:.1f}° "
              f"pos=({lat:.6f},{lon:.6f}) hdg={heading:.1f}° "
              f"action={action} V={V} I={I} Rem={Rem} E={energy_Wh:.4f}Wh")

        # === 到達判定 ===
        if distance < DIST_THRESHOLD:
            print(f"Waypoint {wp_index} reached.")
            wp_index += 1
            stop()

        # === 周期維持 ===
        elapsed = time.time() - loop_start
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)

    print("=== All waypoints reached ===")
    stop()
    for _ in range(int(1 / DT)):
        update_channels()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Emergency stop")
        stop()
        for _ in range(int(1 / DT)):
            update_channels()
        master.close()