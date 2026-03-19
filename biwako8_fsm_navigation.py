############################################################
# FSM Navigation Experiment Program
# Supports:
#  - HYBRID       : Travel + Station Keeping (with deformation)
#  - TRAVEL_ONLY : Travel mode only (no deformation)
#  - KEEP_ONLY   : Station keeping mode only (no deformation)
############################################################

import math
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# ============================================================
# Experiment Mode
# ============================================================
EXPERIMENT_MODE = "HYBRID"
# "HYBRID", "TRAVEL_ONLY", "KEEP_ONLY"

# ============================================================
# Experiment Parameters
# ============================================================
WAYPOINT_FILE = "waypoints.csv"
SERVO_CSV = "servo_positions.csv"

DIST_TOLERANCE = 1.5     # [m]
DIST_RECOVER   = 0.5     # [m]
KEEP_TIME = 300.0        # [s]
MAX_LAPS = 3

TRAVEL_SURGE  = 1.0
RECOVER_SURGE = 1.0
Kp_yaw = 1.2

SERVO_STEP_DELAY = 0.2
DEFORM_WAIT = 3.0

PWM_MIN, PWM_MAX = 1100, 1900
NEU = 1500
PWM_RANGE = 200

LOOP_HZ = 10.0
DT = 1.0 / LOOP_HZ
RAMP_US = 300.0 * DT

# ============================================================
# Logging
# ============================================================
timestamp_str = datetime.now().strftime("%Y%m%d%H%M%S")
LOG_FILE = f"robot_log_{EXPERIMENT_MODE}_{timestamp_str}.csv"

# ============================================================
# MAVLink
# ============================================================
master = mavutil.mavlink_connection("udp:127.0.0.1:14551")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

def set_msg_interval(msg_id, hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0
    )

set_msg_interval(33, 10)   # position
set_msg_interval(30, 10)   # attitude
set_msg_interval(147, 0)  # battery

# ============================================================
# Deformation Control
# ============================================================
servo_states = []
current_mode = None

def load_servo_states():
    with open(SERVO_CSV, newline="") as f:
        for r in csv.DictReader(f):
            servo_states.append(
                (int(r["servo"]), int(r["open"]), int(r["close"]))
            )

def set_servo_pwm(ch, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, ch, pwm, 0, 0, 0, 0, 0
    )

def deform_to(mode):
    global current_mode
    if mode == current_mode:
        return

    if mode == "OPEN":
        for ch, open_pwm, _ in servo_states:
            set_servo_pwm(ch, open_pwm)
            time.sleep(SERVO_STEP_DELAY)

    elif mode == "CLOSE":
        for ch, _, close_pwm in servo_states:
            set_servo_pwm(ch, close_pwm)
            time.sleep(SERVO_STEP_DELAY)

    time.sleep(DEFORM_WAIT)
    current_mode = mode

# ============================================================
# RC Control
# ============================================================
curr = {f'rc{i}': NEU for i in range(1, 9)}
target = curr.copy()

def clamp(v):
    return max(PWM_MIN, min(PWM_MAX, int(v)))

def update_channels():
    for k in curr:
        diff = target[k] - curr[k]
        if diff > RAMP_US:
            curr[k] += RAMP_US
        elif diff < -RAMP_US:
            curr[k] -= RAMP_US
        else:
            curr[k] = target[k]
        curr[k] = clamp(curr[k])

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        curr['rc1'], curr['rc2'], curr['rc3'], curr['rc4'],
        curr['rc5'], curr['rc6'], curr['rc7'], curr['rc8']
    )

def set_surge_yaw(surge, yaw):
    yaw = max(-1.0, min(1.0, yaw))
    target['rc5'] = clamp(NEU + surge * PWM_RANGE)
    target['rc4'] = clamp(NEU + yaw   * PWM_RANGE)

def stop_thrusters():
    set_surge_yaw(0.0, 0.0)

# ============================================================
# State Holders
# ============================================================
last_pos = (0.0, 0.0, 0.0)
last_att = {"roll": None, "pitch": None, "yaw": None}
last_batt = {"V": None, "I": None, "Rem": None, "P": None}

# ============================================================
# MAVLink Harvest
# ============================================================
def harvest_messages(max_reads=30):
    global last_pos
    for _ in range(max_reads):
        m = master.recv_match(blocking=False)
        if not m:
            break

        if m.get_type() == "GLOBAL_POSITION_INT":
            lat = m.lat / 1e7
            lon = m.lon / 1e7
            hdg = m.hdg / 100.0 if m.hdg != 65535 else last_pos[2]
            last_pos = (lat, lon, hdg)

        elif m.get_type() == "ATTITUDE":
            last_att["roll"]  = math.degrees(m.roll)
            last_att["pitch"] = math.degrees(m.pitch)
            last_att["yaw"]   = math.degrees(m.yaw)

        elif m.get_type() == "BATTERY_STATUS":
            volts = [v/1000.0 for v in m.voltages if v != 65535]
            V = sum(volts) if volts else None
            I = m.current_battery/100.0 if m.current_battery != -1 else None
            Rem = m.battery_remaining if m.battery_remaining != -1 else None
            P = V * I if (V and I) else None
            last_batt.update({"V": V, "I": I, "Rem": Rem, "P": P})

# ============================================================
# Geometry
# ============================================================
def calc_distance_bearing(p1, p2):
    dx = (p2[1] - p1[1]) * 111000 * math.cos(math.radians(p1[0]))
    dy = (p2[0] - p1[0]) * 111000
    dist = math.hypot(dx, dy)
    brg = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    return dist, brg

def bearing_diff(h, t):
    return (t - h + 180) % 360 - 180

# ============================================================
# Logging
# ============================================================
def log_data(ts, pos, wp, dist, diff, surge, yaw, energy, state):
    with open(LOG_FILE, "a", newline="") as f:
        csv.writer(f).writerow([
            ts, EXPERIMENT_MODE,
            pos[0], pos[1],
            wp[0], wp[1],
            round(dist, 2),
            round(diff, 2),
            f"surge={surge:.2f}, yaw={yaw:.2f}",
            last_att["roll"], last_att["pitch"], last_att["yaw"],
            last_batt["V"], last_batt["I"], last_batt["Rem"],
            last_batt["P"], round(energy, 4),
            state
        ])

# ============================================================
# MAIN FSM
# ============================================================
def main():

    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow([
            "timestamp","experiment_mode",
            "lat","lon","wp_lat","wp_lon",
            "distance","bearing_diff","action",
            "roll","pitch","yaw",
            "voltage","current","remaining","power","energy_Wh",
            "state"
        ])

    load_servo_states()

    waypoints = []
    with open(WAYPOINT_FILE, newline="") as f:
        for r in csv.DictReader(f):
            waypoints.append((float(r["lat"]), float(r["lon"])))

    wp_id, lap_id = 0, 1
    state = "TRAVEL"
    recovering = False
    keep_start = None

    energy_Wh = 0.0
    last_energy_t = time.time()

    # Initial deformation
    if EXPERIMENT_MODE == "KEEP_ONLY":
        deform_to("OPEN")
    else:
        deform_to("CLOSE")

    print("=== Mission Start ===")

    while lap_id <= MAX_LAPS:

        loop_t0 = time.time()
        harvest_messages()

        pos = last_pos
        wp = waypoints[wp_id]
        dist, brg = calc_distance_bearing(pos[:2], wp)
        diff = bearing_diff(pos[2], brg)

        surge = yaw = 0.0

        if state == "TRAVEL":

            surge = TRAVEL_SURGE
            yaw = Kp_yaw * diff
            set_surge_yaw(surge, yaw)

            if dist <= DIST_TOLERANCE:
                stop_thrusters()
                if EXPERIMENT_MODE == "HYBRID":
                    deform_to("OPEN")
                state = "KEEP"
                keep_start = time.time()
                recovering = False

        elif state == "KEEP":

            elapsed = time.time() - keep_start

            if not recovering:
                if dist > DIST_TOLERANCE:
                    recovering = True
                else:
                    stop_thrusters()
            else:
                if dist <= DIST_RECOVER:
                    recovering = False
                    stop_thrusters()
                else:
                    surge = RECOVER_SURGE
                    yaw = Kp_yaw * diff
                    set_surge_yaw(surge, yaw)

            if elapsed >= KEEP_TIME:
                wp_id += 1
                if wp_id >= len(waypoints):
                    wp_id = 0
                    lap_id += 1
                if EXPERIMENT_MODE == "HYBRID":
                    deform_to("CLOSE")
                state = "TRAVEL"

        update_channels()

        now = time.time()
        if last_batt["P"] is not None:
            energy_Wh += last_batt["P"] * ((now - last_energy_t) / 3600.0)
        last_energy_t = now

        log_data(datetime.now().isoformat(),
                 pos, wp, dist, diff,
                 surge, yaw, energy_Wh, state)

        sleep_t = DT - (time.time() - loop_t0)
        if sleep_t > 0:
            time.sleep(sleep_t)

    print("=== Mission Complete ===")
    stop_thrusters()

# ============================================================
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_thrusters()
        master.close()
