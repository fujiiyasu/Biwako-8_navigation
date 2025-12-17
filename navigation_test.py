import math
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# ==== 設定 ====
WAYPOINT_FILE = "waypoints.csv"
DIST_THRESHOLD = 1.5   # [m]
LOOP_HZ = 10.0         # [Hz]
DT = 1.0 / LOOP_HZ
GPS_UPDATE_HZ = 1.0    # [Hz]
GPS_UPDATE_INTERVAL = 1.0 / GPS_UPDATE_HZ
LOG_FILE = "robot_log.csv"

# ==== MAVLink 接続 ====
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# BATTERY_STATUS を 10Hz に設定
BATTERY_STATUS_MSG_ID = 147
interval_us = int(1e6 / LOOP_HZ)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    BATTERY_STATUS_MSG_ID,
    interval_us,
    0, 0, 0, 0, 0
)
print(f"Requested BATTERY_STATUS at {LOOP_HZ} Hz")


# ==== 関数群 ====

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

def get_battery_data():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=False)
    if not msg:
        return None, None, None, None
    voltages = [v/1000.0 for v in msg.voltages if v != 65535]
    total_voltage = sum(voltages) if voltages else None
    current = msg.current_battery / 100.0
    remaining = msg.battery_remaining
    power = None
    if total_voltage is not None and current is not None:
        power = total_voltage * current
    return total_voltage, current, remaining, power

def get_gps_data():
    """1回だけGPSメッセージを読んで返す"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if not msg:
        return None, None, None
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    heading = msg.hdg / 100.0 if msg.hdg != 65535 else None
    return lat, lon, heading

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


# ==== メインループ ====

def main():
    # ログ初期化
    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "lat", "lon", "wp_lat", "wp_lon",
                         "distance", "bearing_diff", "action",
                         "voltage_V", "current_A", "remaining_%", "power_W", "energy_Wh"])

    waypoints = load_waypoints(WAYPOINT_FILE)
    wp_index = 0

    energy_Wh = 0.0

    # GPSキャッシュ用
    last_gps_time = 0
    last_lat, last_lon, last_heading = None, None, None

    print("=== Start test (loop=10Hz, GPS=1Hz) ===")

    while wp_index < len(waypoints):
        loop_start = time.time()

        # === GPSを1Hzで更新 ===
        if time.time() - last_gps_time >= GPS_UPDATE_INTERVAL:
            lat, lon, heading = get_gps_data()
            if lat is not None and lon is not None:
                last_lat, last_lon, last_heading = lat, lon, heading
                last_gps_time = time.time()

        if last_lat is None or last_lon is None:
            print("⚠ GPSデータ未取得")
            continue

        current_pos = [last_lat, last_lon]
        heading = last_heading if last_heading is not None else 0.0

        target_wp = waypoints[wp_index]
        distance, target_bearing = calc_distance_and_bearing(current_pos, target_wp)
        diff = bearing_diff(heading, target_bearing)

        action = decide_thruster_command(diff)

        # バッテリーデータ
        voltage, current, remaining, power = get_battery_data()
        if power is not None:
            energy_Wh += power * (DT / 3600.0)

        # ログ
        timestamp = datetime.now().isoformat()
        log_data(timestamp, current_pos, target_wp, distance, diff, action,
                 voltage, current, remaining, power, energy_Wh)

        # コンソール出力
        print(f"[{wp_index}] dist={distance:.2f}m diff={diff:.1f}° "
              f"pos=({last_lat:.6f}, {last_lon:.6f}) heading={heading:.1f}° "
              f"action={action} energy={energy_Wh:.4f}Wh")

        # 到達判定
        if distance < DIST_THRESHOLD:
            print(f"Waypoint {wp_index} reached.")
            wp_index += 1

        # ループ周期維持 (10Hz)
        elapsed = time.time() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("=== All waypoints reached ===")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped by user.")
        master.close()
