# ============================================================
# RC5/RC4 Surge+Yaw Control + PD Simulation Commands
# ============================================================

import math
import time
import threading
from pymavlink import mavutil

# ===== SETTINGS =====
PWM_NEU = 1500
PWM_RANGE = 200        # ★最大変動幅を200に変更（1500±200）
LOOP_HZ = 20.0
DT = 1.0 / LOOP_HZ

# PDゲイン（必要に応じて調整可能）
Kp_d = 1.0      # 距離ゲイン
Kp_th = 1.5     # 方位ゲイン

# ============================================================
# MAVLINK CONNECT
# ============================================================

master = mavutil.mavlink_connection("udp:127.0.0.1:14551")
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected.")

# ARM
print("[ARM] Sending arm request...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# ============================================================
# RC STATE
# ============================================================

target = {"rc4": PWM_NEU, "rc5": PWM_NEU}
running = True
last_cmd = "0"

def clamp(v):
    return max(1100, min(1900, v))

# ============================================================
# Continuous RC Override Thread
# ============================================================

def update_channels_continuous():
    while running:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, 0, target["rc4"],
            target["rc5"],
            0, 0, 0
        )
        time.sleep(DT)

thread = threading.Thread(target=update_channels_continuous, daemon=True)
thread.start()

# ============================================================
# Surge + Yaw Control (manual or PD)
# ============================================================

def move(surge, yaw):
    """ surge, yaw ∈ [-1, 1] """
    rc5 = int(PWM_NEU + surge * PWM_RANGE)  # Surge
    rc4 = int(PWM_NEU + yaw   * PWM_RANGE)  # Yaw

    target["rc5"] = clamp(rc5)
    target["rc4"] = clamp(rc4)

def stop_thrusters():
    target["rc5"] = PWM_NEU
    target["rc4"] = PWM_NEU

# ============================================================
# PD Simulation Function
# ============================================================

def pd_sim(distance_norm, theta_norm):
    """
    distance_norm: 距離誤差（0〜1）
    theta_norm   : 方位誤差（0〜1）符号つき
    """
    surge = Kp_d * distance_norm
    yaw = Kp_th * theta_norm

    # 正規化 (1.0 を超えた分はクリップ)
    surge = max(-1.0, min(+1.0, surge))
    yaw = max(-1.0, min(+1.0, yaw))

    print(f"[PD SIM] distance={distance_norm}, theta={theta_norm}")
    print(f"   → surge={surge:.2f}, yaw={yaw:.2f}")
    
    move(surge, yaw)

# ============================================================
# Input Thread
# ============================================================

def input_thread():
    global last_cmd, running
    print("\n=== Surge (RC5) + Yaw (RC4) Control ===")
    print("w/s/a/d/q/e/z/c : manual control")
    print("0 : stop")
    print("x : exit (disarm)")
    print("--- PD Simulation Commands ---")
    print("t1 : Far × Large error")
    print("t2 : Far × Small error")
    print("t3 : Near × Large error")
    print("t4 : Near × Small error")
    print("========================================\n")

    while running:
        cmd = input("Command: ").strip()
        last_cmd = cmd

        if cmd == "x":
            running = False
            return

threading.Thread(target=input_thread, daemon=True).start()

# ============================================================
# MAIN LOOP
# ============================================================

try:
    while running:

        # ==== Manual Control ====
        if last_cmd == "w":
            move(+1, 0)
        elif last_cmd == "s":
            move(-1, 0)
        elif last_cmd == "a":
            move(0, +1)
        elif last_cmd == "d":
            move(0, -1)
        elif last_cmd == "q":
            move(+0.7, +0.7)
        elif last_cmd == "e":
            move(+0.7, -0.7)
        elif last_cmd == "z":
            move(-0.7, +0.7)
        elif last_cmd == "c":
            move(-0.7, -0.7)
        elif last_cmd == "0":
            stop_thrusters()

        # ==== PD Simulation ====
        elif last_cmd == "t1":
            # 遠い × 方位誤差大
            pd_sim(distance_norm=1.0, theta_norm=0.5)
        elif last_cmd == "t2":
            # 遠い × 方位誤差小
            pd_sim(distance_norm=1.0, theta_norm=0.1)
        elif last_cmd == "t3":
            # 近い × 方位誤差大
            pd_sim(distance_norm=0.2, theta_norm=0.5)
        elif last_cmd == "t4":
            # 近い × 方位誤差小
            pd_sim(distance_norm=0.2, theta_norm=0.1)

        time.sleep(DT)

finally:
    running = False
    thread.join(timeout=1.0)

    print("[DISARM] Sending disarm request...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    master.close()
    print("Closed.")
