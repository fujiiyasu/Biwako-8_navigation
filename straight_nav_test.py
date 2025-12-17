############################################################
# 直進移動モード（PD制御 + 初回GPS同期 + 充実ログ機能）完全版
############################################################

import math
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# ============================================================
# ハイパーパラメータ
# ============================================================
DIST_NORM_SCALE = 1.0
YAW_NORM_SCALE = 90.0

Kp_d = 1.0
Kd_d = 0.3
Kp_th = 1.5
Kd_th = 0.4

# PWM設定
PWM_MIN, PWM_MAX = 1100, 1900
NEU = 1500
PWM_RANGE = 200

# smoothing
LOOP_HZ = 10.0
DT = 1.0 / LOOP_HZ
RAMP_US_PER_SEC = 300.0
RAMP_US = RAMP_US_PER_SEC * DT
EMA_ALPHA = 0.5

# waypoint 到達判定
DIST_THRESHOLD = 1.5

# ログファイル
timestamp_str = datetime.now().strftime("%Y%m%d%H%M%S")
LOG_FILE = f"robot_log_{timestamp_str}.csv"

WAYPOINT_FILE = "waypoints.csv"


############################################################
# MAVLINK 接続
############################################################
master = mavutil.mavlink_connection("udp:127.0.0.1:14551")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}")

# ARM
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1,0,0,0,0,0,0
)

# GPS 10Hz
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0,0,0,0,0
)

# BATTERY_STATUS (最速)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 147, 0, 0,0,0,0,0
)


############################################################
# 初回GPS同期
############################################################
def wait_first_gps():
    print("Waiting for first GPS fix...")
    while True:
        m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if not m:
            continue

        lat = m.lat / 1e7
        lon = m.lon / 1e7
        heading = m.hdg / 100.0 if m.hdg != 65535 else 0.0

        if abs(lat) < 0.0001 and abs(lon) < 0.0001:
            continue

        print(f"GPS FIX → lat={lat}, lon={lon}, heading={heading}")
        return (lat, lon, heading)


############################################################
# RC スラスタ制御
############################################################
curr = {f'rc{i}': NEU for i in range(1, 9)}
target = curr.copy()

def clamp(v):
    return max(PWM_MIN, min(PWM_MAX, v))

def update_channels():
    for k in curr:
        diff = target[k] - curr[k]
        if diff > RAMP_US: curr[k] += RAMP_US
        elif diff < -RAMP_US: curr[k] -= RAMP_US
        else: curr[k] = target[k]

        curr[k] = EMA_ALPHA * curr[k] + (1 - EMA_ALPHA) * curr[k]
        curr[k] = clamp(int(curr[k]))

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        curr['rc1'], curr['rc2'], curr['rc3'], curr['rc4'],
        curr['rc5'], curr['rc6'], curr['rc7'], curr['rc8']
    )


def set_surge_yaw(surge, yaw):
    rc5 = int(NEU + surge * PWM_RANGE)
    rc4 = int(NEU + yaw   * PWM_RANGE)
    target['rc5'] = clamp(rc5)
    target['rc4'] = clamp(rc4)


############################################################
# Battery キャッシュ
############################################################
last_batt = {"V": None, "I": None, "Rem": None, "P": None}

def harvest_messages(max_reads=20):
    for _ in range(max_reads):
        m = master.recv_match(blocking=False)
        if not m:
            break

        if m.get_type() == "BATTERY_STATUS":
            voltages = [v/1000.0 for v in m.voltages if v != 65535]
            V = sum(voltages) if voltages else None
            I = m.current_battery/100.0 if m.current_battery != -1 else None
            Rem = m.battery_remaining if m.battery_remaining != -1 else None
            P = V*I if (V and I) else None
            last_batt.update({"V":V, "I":I, "Rem":Rem, "P":P})


############################################################
# GPS Non-blocking
############################################################
def get_position_heading_nonblocking(last_pos):
    m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if m:
        lat = m.lat / 1e7
        lon = m.lon / 1e7
        heading = m.hdg / 100.0 if m.hdg != 65535 else last_pos[2]

        if abs(lat) < 0.0001 and abs(lon) < 0.0001:
            return last_pos

        return (lat, lon, heading)
    return last_pos


############################################################
# Waypoint utilities
############################################################
def load_waypoints(filename):
    wp = []
    with open(filename, newline="") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            wp.append((float(row["lat"]), float(row["lon"])))
    return wp

def calc_distance_and_bearing(p1, p2):
    dx = (p2[1] - p1[1]) * 111000 * math.cos(math.radians(p1[0]))
    dy = (p2[0] - p1[0]) * 111000
    dist = math.sqrt(dx*dx + dy*dy)
    bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    return dist, bearing

def bearing_diff(hdg, tgt):
    return (tgt - hdg + 180) % 360 - 180


############################################################
# PD Control
############################################################
last_distance_norm = 0.0
last_theta_norm = 0.0

def pd_control(distance, diff):

    global last_distance_norm, last_theta_norm

    distance_norm = min(distance / DIST_NORM_SCALE, 1.0)
    theta_norm = max(-1.0, min(+1.0, diff / YAW_NORM_SCALE))

    P_surge = Kp_d * distance_norm
    P_yaw   = Kp_th * theta_norm

    d_distance = (distance_norm - last_distance_norm) / DT
    d_theta    = (theta_norm    - last_theta_norm) / DT

    D_surge = Kd_d * d_distance
    D_yaw   = Kd_th * d_theta

    surge = max(-1, min(+1, P_surge + D_surge))
    yaw   = max(-1, min(+1, P_yaw   + D_yaw))

    last_distance_norm = distance_norm
    last_theta_norm = theta_norm

    set_surge_yaw(surge, yaw)

    return surge, yaw


############################################################
# ログ書き込み
############################################################
def log_data(timestamp, pos, waypoint, distance, diff, surge, yaw,
             voltage, current, remaining, power, energy_Wh):

    action_str = f"surge={surge:.3f}, yaw={yaw:.3f}"

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            timestamp,
            pos[0], pos[1],
            waypoint[0], waypoint[1],
            round(distance,2),
            round(diff,2),
            action_str,
            round(voltage,2) if voltage else None,
            round(current,2) if current else None,
            remaining,
            round(power,2) if power else None,
            round(energy_Wh,4)
        ])


############################################################
# MAIN
############################################################
def main():

    # ログヘッダ
    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp","lat","lon","wp_lat","wp_lon",
            "distance","bearing_diff","action",
            "voltage_V","current_A","remaining_%","power_W","energy_Wh"
        ])

    last_pos = wait_first_gps()
    wp_list = load_waypoints(WAYPOINT_FILE)
    wp_index = 0

    energy_Wh = 0.0
    last_energy_t = time.time()

    print("=== Start PD Autopilot (Straight Mode + Logging) ===")

    while wp_index < len(wp_list):

        loop_start = time.time()

        # --- GPS ---
        lat, lon, heading = get_position_heading_nonblocking(last_pos)
        last_pos = (lat, lon, heading)
        current_pos = [lat, lon]

        # --- 目標 ---
        wp = wp_list[wp_index]
        distance, tgt_bearing = calc_distance_and_bearing(current_pos, wp)
        diff = bearing_diff(heading, tgt_bearing)

        # --- PD制御 ---
        surge, yaw = pd_control(distance, diff)
        update_channels()

        # --- Battery 更新 ---
        harvest_messages()
        V, I, Rem, P = (last_batt[k] for k in ("V","I","Rem","P"))

        # --- Energy 積算 ---
        now = time.time()
        dt_e = now - last_energy_t
        if P is not None:
            energy_Wh += P * (dt_e / 3600.0)
        last_energy_t = now

        # --- ログ ---
        timestamp = datetime.now().isoformat()
        log_data(timestamp, current_pos, wp, distance, diff,
                 surge, yaw, V, I, Rem, P, energy_Wh)

        print(f"[{wp_index}] dist={distance:.2f} diff={diff:.1f}° "
              f"surge={surge:.2f} yaw={yaw:.2f} "
              f"V={V} I={I} Rem={Rem} energy={energy_Wh:.4f}Wh")

        # --- Waypoint到達 ---
        if distance < DIST_THRESHOLD:
            print(f"Waypoint {wp_index} reached!")
            wp_index += 1
            set_surge_yaw(0,0)

        # --- 周期調整 ---
        dt = time.time() - loop_start
        if DT - dt > 0:
            time.sleep(DT - dt)

    print("=== Mission Complete ===")
    set_surge_yaw(0,0)
    for _ in range(int(1/DT)):
        update_channels()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Emergency Stop!")
        set_surge_yaw(0,0)
        for _ in range(int(1/DT)):
            update_channels()
        master.close()
