import csv
import time
from pymavlink import mavutil

# === 接続設定 ===
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# === サーボ制御関数 ===
def set_servo_pwm(channel, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,   # サーボ番号 (1–16)
        pwm,       # PWM値 [µs]
        0, 0, 0, 0, 0
    )

# === CSVからサーボ設定を読み込む ===
servo_states = []  # [(ch, open_pwm, close_pwm), ...]
with open("servo_positions.csv", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        ch = int(row["servo"])
        open_pwm = int(row["open"])
        close_pwm = int(row["close"])

        # 範囲チェック (700〜2300)
        if not (700 <= open_pwm <= 2300 and 700 <= close_pwm <= 2300):
            raise ValueError(f"Servo {ch}: 値が範囲外です (700〜2300)")

        servo_states.append((ch, open_pwm, close_pwm))

print("Loaded servo states:")
for ch, open_pwm, close_pwm in servo_states:
    print(f" Servo {ch}: OPEN={open_pwm}, CLOSE={close_pwm}")

# === ユーザにモードを選択させる ===
choice = input("モードを選択してください (0=OPEN, 1=CLOSE): ").strip()

if choice == "0":
    print("Moving to OPEN state...")
    for ch, open_pwm, _ in servo_states:
        set_servo_pwm(ch, open_pwm)
        time.sleep(0.2)
elif choice == "1":
    print("Moving to CLOSE state...")
    for ch, _, close_pwm in servo_states:
        set_servo_pwm(ch, close_pwm)
        time.sleep(0.2)
else:
    print("不正な入力です。0 または 1 を入力してください。")

# 終了時に接続を閉じる
master.close()
print("Program finished.")
