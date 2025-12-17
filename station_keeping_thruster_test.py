# ============================================================
# Thruster Manual Test Code (for on-land verification)
# Base code: station_keeping_nav_test.py
# ============================================================

import time
from pymavlink import mavutil

# ==== PWM Settings ====
PWM_NEU = 1500
PWM_FWD = 1600
PWM_REV = 1400   # = 3000 - 1600 (if needed)
PWM_TURN_R = 1600
PWM_TURN_L = 1400

RAMP_US_PER_SEC = 300.0
LOOP_HZ = 20.0
DT = 1.0 / LOOP_HZ
RAMP_US = RAMP_US_PER_SEC * DT

# ==== MAVLink 接続 ====
master = mavutil.mavlink_connection("udp:127.0.0.1:14551")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected.")

# Arm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

# ==== RC State ====
curr = {f"rc{i}": PWM_NEU for i in range(1, 9)}
target = curr.copy()

def clamp(v, lo=1100, hi=1900):
    return max(lo, min(hi, v))

def update_channels():
    """target に向かって徐々に curr を更新し、送信"""
    for k in curr:
        diff = target[k] - curr[k]

        if diff > RAMP_US:
            curr[k] += RAMP_US
        elif diff < -RAMP_US:
            curr[k] -= RAMP_US
        else:
            curr[k] = target[k]

        curr[k] = clamp(int(curr[k]))

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        curr['rc1'], curr['rc2'], curr['rc3'], curr['rc4'],
        curr['rc5'], curr['rc6'], curr['rc7'], curr['rc8']
    )

def set_thruster(rc5=PWM_NEU, rc6=PWM_NEU):
    """スラスタ PWM 値セット"""
    target["rc5"] = rc5
    target["rc6"] = rc6

# ============================================================
# Main Manual Test Loop
# ============================================================
print("\n=== Thruster Manual Test Mode ===")
print("1: Forward")
print("2: Backward")
print("3: Turn Right")
print("4: Turn Left")
print("0: Stop")
print("q: Quit")
print("==============================\n")

try:
    while True:
        cmd = input("Input command (0/1/2/3/4/q): ").strip()

        if cmd == "1":
            print(">>> FORWARD")
            set_thruster(rc5=PWM_FWD, rc6=PWM_NEU)

        elif cmd == "2":
            print(">>> BACKWARD")
            set_thruster(rc5=PWM_REV, rc6=PWM_NEU)

        elif cmd == "3":
            print(">>> TURN RIGHT")
            set_thruster(rc5=PWM_NEU, rc6=PWM_TURN_R)

        elif cmd == "4":
            print(">>> TURN LEFT")
            set_thruster(rc5=PWM_NEU, rc6=PWM_TURN_L)

        elif cmd == "0":
            print(">>> STOP")
            set_thruster(rc5=PWM_NEU, rc6=PWM_NEU)

        elif cmd == "q":
            print("Exiting...")
            break

        else:
            print("Invalid command.")
            continue

        # 操作中も定期的に送信
        for _ in range(10):
            update_channels()
            time.sleep(DT)

finally:
    print(">>> Emergency STOP")
    set_thruster(PWM_NEU, PWM_NEU)
    for _ in range(20):
        update_channels()
        time.sleep(DT)

    master.close()
