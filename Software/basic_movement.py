# Should be improved, but this is untested!!!
# Refer to previous version if this doesn't work

import time
import math
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# ----------------------
# Motor setup
# ----------------------
motorcount = 4
i2c_addresses = [25, 26, 27, 28]  # FL, FR, BR, BL
i2c = busio.I2C(board.SCL, board.SDA)
motor = [PowerfulBLDCDriver(i2c, addr) for addr in i2c_addresses]

# ----------------------
# Apply hardcoded calibration & set speed mode
# ----------------------
calibration_data = [
    (1439128576, 1252),  # Motor 0 (FL)
    (1793410560, 1236),  # Motor 1 (FR)
    (1320240896, 1251),  # Motor 2 (BR)
    (1345708544, 1258)   # Motor 3 (BL)
]

for i in range(motorcount):
    motor[i].set_current_limit_foc(65536)
    motor[i].set_id_pid_constants(1500, 200)
    motor[i].set_iq_pid_constants(1500, 200)
    motor[i].set_speed_pid_constants(4e-2, 4e-4, 3e-2)
    motor[i].set_position_pid_constants(275, 0, 0)
    motor[i].set_position_region_boundary(250000)
    motor[i].set_speed_limit(10000000)
    motor[i].configure_operating_mode_and_sensor(15, 1)
    motor[i].configure_command_mode(15)

    # Apply calibration
    motor[i].set_calibration_ELECANGLEOFFSET(calibration_data[i][0])
    motor[i].set_calibration_SINCOSCENTRE(calibration_data[i][1])

    # Switch to speed mode
    motor[i].configure_operating_mode_and_sensor(3, 1)
    motor[i].configure_command_mode(12)

print("✅ Motors ready with hardcoded calibration.")

# ----------------------
# Helper to set motor speeds
# ----------------------
def setMotorSpeeds(speeds):
    for i in range(motorcount):
        motor[i].set_speed(int(speeds[i]))

# ----------------------
# Move robot in heading (0-360 degrees) at % of max speed
# ----------------------
def move_heading(angle_deg, speed_percent):
    # Ensure speed percent is 1–10
    speed_percent = max(1, min(speed_percent, 10))
    speed_value = int((speed_percent / 10) * 5000000)  # max speed scaled

    angle_rad = math.radians(angle_deg)

    # Omniwheel X arrangement (FL, FR, BR, BL)
    fl = -math.sin(angle_rad + math.pi/4)  # Front left
    fr =  math.cos(angle_rad + math.pi/4)  # Front right
    br =  math.sin(angle_rad + math.pi/4)  # Back right
    bl = -math.cos(angle_rad + math.pi/4)  # Back left

    speeds = [fl * speed_value, fr * speed_value, br * speed_value, bl * speed_value]
    setMotorSpeeds(speeds)

# ----------------------
# Main loop with live direction + speed changes
# ----------------------
current_heading = None
current_speed_percent = 5  # default 50% speed
moving = False

print("Enter: heading(0-360),speed(1-10) — example: 0,1 for forward at 10% speed.")
print("Or just enter heading to keep last speed. Type 'stop' to halt.")

while True:
    if moving and current_heading is not None:
        move_heading(current_heading, current_speed_percent)  # Keep moving
        time.sleep(0.05)

    cmd = input("> ").strip().lower()
    if cmd == "stop":
        setMotorSpeeds([0, 0, 0, 0])
        moving = False
        print("⏹ Stopped")
    else:
        try:
            parts = cmd.split(",")
            heading = float(parts[0])
            if 0 <= heading < 360:
                current_heading = heading
                if len(parts) > 1:
                    speed_input = int(parts[1])
                    if 1 <= speed_input <= 10:
                        current_speed_percent = speed_input
                    else:
                        print("⚠ Speed must be between 1 and 10. Keeping last speed.")
                moving = True
                print(f"▶ Heading {current_heading}°, Speed {current_speed_percent*10}%")
            else:
                print("❌ Heading must be 0–359.")
        except ValueError:
            print("❌ Invalid input. Format: heading,speed or heading only.")
