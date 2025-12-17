import math
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# ==== 設定 ====
WAYPOINT_FILE = "waypoints.csv"
DIST_THRESHOLD = 1.5
LOOP_HZ = 10.0
DT = 1.0 / LOOP_HZ

# ログファイル名を日時付きで生成
timestamp_str = datetime.now().strftime("%Y%m%d%H%M%S")
LOG_FILE = f"robot_log_{timestamp_str}.csv"

PWM_MIN, PWM_MAX = 1100, 1900
NEU = 1500
MOVE_PWM = 1700
MOVE_PWM_REV = 3000 - MOVE_PWM

RAMP_US_PER_SEC = 300.0
RAMP_US = RAMP_US_PER_SEC * DT
EMA_ALPHA = 0.5

# ==== MAVLink 接続 ====
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# Arm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

# BATTERY_STATUS 要求
BATTERY_STATUS_MSG_ID = 147
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, BATTERY_STATUS_MSG_ID, 0,  # 0 = 最速
    0, 0, 0, 0, 0
)

# GLOBAL_POSITION_INT を 10Hz で要求
GLOBAL_POSITION_INT_MSG_ID = 33
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, GLOBAL_POSITION_INT_MSG_ID, 100000,  # 100000us = 0.1s = 10Hz
    0, 0, 0, 0, 0
)

# ==== RC制御用変数 ====
curr = {f'rc{i}': NEU for i in range(1, 9)}
target = curr.copy()

def clamp(v): return max(PWM_MIN, min(PWM_MAX, v))

def update_channels():
    for k in curr:
        diff = target[k] - curr[k]
        if diff > RAMP_US:   curr[k] += RAMP_US
        elif diff < -RAMP_US: curr[k] -= RAMP_US
        else:                curr[k] = target[k]

        curr[k] = EMA_ALPHA * curr[k] + (1 - EMA_ALPHA) * curr[k]
        curr[k] = clamp(int(curr[k]))

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        curr['rc1'], curr['rc2'], curr['rc3'], curr['rc4'],
        curr['rc5'], curr['rc6'], curr['rc7'], curr['rc8']
    )

def set_target(rc5=NEU, rc6=NEU):
    target['rc5'] = rc5
    target['rc6'] = rc6

# ==== バッテリキャッシュ ====
last_batt = {"V": None, "I": None, "Rem": None, "P": None, "t": None}

def harvest_messages(max_reads=20):
    """ 非ブロッキングで受信し、BATTERY_STATUSをキャッシュ """
    for _ in range(max_reads):
        m = master.recv_match(blocking=False)
        if not m:
            break
        if m.get_type() == 'BATTERY_STATUS':
            voltages = [v/1000.0 for v in m.voltages if v != 65535]
            V = sum(voltages) if voltages else None
            I = m.current_battery/100.0 if m.current_battery != -1 else None
            Rem = m.battery_remaining if m.battery_remaining != -1 else None
            P = V*I if (V and I) else None
            last_batt.update({"V": V, "I": I, "Rem": Rem, "P": P, "t": time.time()})

# ==== 位置・方位 ====
def get_position_heading_nonblocking(last_pos):
    """ GPSをノンブロッキングで取得，新しいデータがなければ前回値を返す """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        heading = msg.hdg / 100.0 if msg.hdg != 65535 else last_pos[2]
        return lat, lon, heading
    else:
        return last_pos

# ==== ログ ====
def log_data(timestamp, pos, waypoint, distance, diff, action,
             voltage, current, remaining, power, energy_Wh):
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, pos[0], pos[1],
                         waypoint[0], waypoint[1],
                         round(distance, 2), round(diff, 2),
                         action,
                         round(voltage, 2) if voltage else None,
                         round(current, 2) if current else None,
                         remaining,
                         round(power, 2) if power else None,
                         round(energy_Wh, 4)])

# ==== 補助関数 ====
def load_waypoints(filename):
    waypoints = []
    with open(filename, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append((float(row["lat"]), float(row["lon"])))
    return waypoints

def calc_distance_and_bearing(pos1, pos2):
    dx = (pos2[1] - pos1[1]) * 111000 * math.cos(math.radians(pos1[0]))
    dy = (pos2[0] - pos1[0]) * 111000
    distance = math.sqrt(dx**2 + dy**2)
    bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    return distance, bearing

def bearing_diff(current_heading, target_bearing):
    return (target_bearing - current_heading + 180) % 360 - 180

def decide_thruster_command(diff):
    if -45 <= diff < 45:
        return "FORWARD"
    elif 45 <= diff < 135:
        return "RIGHT"
    elif -135 <= diff < -45:
        return "LEFT"
    else:
        return "BACKWARD"

# ==== メインループ ====
def main():
    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "lat", "lon", "wp_lat", "wp_lon",
                         "distance", "bearing_diff", "action",
                         "voltage_V", "current_A", "remaining_%", "power_W", "energy_Wh"])

    waypoints = load_waypoints(WAYPOINT_FILE)
    wp_index = 0
    energy_Wh = 0.0
    last_energy_t = time.time()

    # 初期位置 (Noneの場合は0にする)
    last_pos = (0.0, 0.0, 0.0)

    print("=== Start test (station-keeping mode with thrusters) ===")

    while wp_index < len(waypoints):
        loop_start = time.time()

        # === 位置・方位 ===
        lat, lon, heading = get_position_heading_nonblocking(last_pos)
        last_pos = (lat, lon, heading)
        current_pos = [lat, lon]

        target_wp = waypoints[wp_index]
        distance, target_bearing = calc_distance_and_bearing(current_pos, target_wp)
        diff = bearing_diff(heading, target_bearing) if heading is not None else 0.0

        # === アクション決定 ===
        action = decide_thruster_command(diff)
        if action == "FORWARD":
            set_target(rc5=MOVE_PWM, rc6=NEU)
        elif action == "BACKWARD":
            set_target(rc5=MOVE_PWM_REV, rc6=NEU)
        elif action == "RIGHT":
            set_target(rc5=NEU, rc6=MOVE_PWM)
        elif action == "LEFT":
            set_target(rc5=NEU, rc6=MOVE_PWM_REV)
        else:
            set_target(rc5=NEU, rc6=NEU)

        # === スラスタ更新 ===
        update_channels()

        # === バッテリ更新 ===
        harvest_messages()
        V, I, Rem, P, t_batt = (last_batt[k] for k in ("V","I","Rem","P","t"))

        # === エネルギー積分 ===
        now = time.time()
        dt_e = now - last_energy_t
        if P is not None:
            energy_Wh += P * (dt_e / 3600.0)
        last_energy_t = now

        # === ログ ===
        timestamp = datetime.now().isoformat()
        log_data(timestamp, current_pos, target_wp, distance, diff, action,
                 V, I, Rem, P, energy_Wh)

        print(f"[{wp_index}] dist={distance:.2f}m diff={diff:.1f}° "
              f"pos=({lat:.6f}, {lon:.6f}) heading={heading:.1f}° "
              f"action={action} V={V} I={I} Rem={Rem} energy={energy_Wh:.4f}Wh")

        # 到達判定
        if distance < DIST_THRESHOLD:
            print(f"Waypoint {wp_index} reached.")
            wp_index += 1
            set_target(rc5=NEU, rc6=NEU)

        # 周期維持
        elapsed = time.time() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("=== All waypoints reached ===")
    set_target(rc5=NEU, rc6=NEU)
    for _ in range(int(1 / DT)):
        update_channels()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Emergency stop")
        set_target(rc5=NEU, rc6=NEU)
        for _ in range(int(1 / DT)):
            update_channels()
        master.close()
