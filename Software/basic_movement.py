# Calibrates all motors using example code and then drives in direction specified by input for 2 seconds
# Need to update to not require calibration step and also fix the movement direction.
# Currently with 0 all the wheels spin clockwise, giving an anticlockwise spin instead of forwards movement.

import time
import sys
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# ----------------------
# Motor setup
# ----------------------
motor = [None] * 8
motormode = [0] * 8
motorcount = 4  # we have 4 motors
i2c_addresses = [25, 26, 27, 28]  # the addresses/pins from your calibration

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Create motor objects
for i in range(motorcount):
    motor[i] = PowerfulBLDCDriver(i2c, i2c_addresses[i])

# ----------------------
# Calibration
# ----------------------
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
    motor[i].set_calibration_options(300, 2097152, 50000, 500000)
    motor[i].start_calibration()
    print(f"Calibrating motor {i}...", end="")
    while not motor[i].is_calibration_finished():
        print(".", end="")
        sys.stdout.flush()
        time.sleep(0.5)
    print(" Done")
    print(f"ELECANGLEOFFSET: {motor[i].get_calibration_ELECANGLEOFFSET()}")
    print(f"SINCOSCENTRE: {motor[i].get_calibration_SINCOSCENTRE()}")

    motor[i].configure_operating_mode_and_sensor(3, 1)
    motor[i].configure_command_mode(12)
    motormode[i] = 12

print("✅ All motors calibrated and in speed mode.")

# ----------------------
# Helper to set motor speeds
# ----------------------
def setMotorSpeeds(speeds):
    for i in range(motorcount):
        motor[i].set_speed(int(speeds[i]))

# ----------------------
# Move robot in heading (0-360 degrees)
# ----------------------
def move_heading(angle_deg, speed=5000000, duration=2):
    import math
    angle_rad = math.radians(angle_deg)

    # Mecanum drive formula
    fl = speed * (math.sin(angle_rad + math.pi/4))
    fr = speed * (math.cos(angle_rad + math.pi/4))
    rl = speed * (math.cos(angle_rad + math.pi/4))
    rr = speed * (math.sin(angle_rad + math.pi/4))

    speeds = [fl, fr, rl, rr]
    print(f"▶ Moving {angle_deg}° for {duration}s with speeds {speeds}")
    setMotorSpeeds(speeds)

    time.sleep(duration)
    setMotorSpeeds([0, 0, 0, 0])
    print("⏹ Stopped")

# ----------------------
# Main loop
# ----------------------
while True:
    try:
        heading = float(input("Enter heading (0-360°, or -1 to quit): "))
        if heading == -1:
            break
        move_heading(heading)
    except ValueError:
        print("❌ Please enter a number.")
