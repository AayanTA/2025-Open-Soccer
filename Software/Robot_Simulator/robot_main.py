# robot_main.py
import time
import sys
import select
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
import controller   # shared controller module
import math

# I2C addresses for your motors (FL, FR, BR, BL)
ADDRS = [25, 26, 27, 28]

# calibration values (hardcoded from your logs)
CAL_ELEC = [1439128576, 1793410560, 1320240896, 1345708544]
CAL_SINCOS = [1252, 1236, 1251, 1258]

# motor objects
i2c = busio.I2C(board.SCL, board.SDA)
motors = [PowerfulBLDCDriver(i2c, a) for a in ADDRS]

MAX_MOTOR_SPEED = 60000000

def init_motors():
    for i, m in enumerate(motors):
        m.set_current_limit_foc(65536 * 2)
        m.set_id_pid_constants(1500, 200)
        m.set_iq_pid_constants(1500, 200)
        m.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
        m.set_position_pid_constants(275, 0, 0)
        m.set_position_region_boundary(250000)
        m.set_speed_limit(10000000)

        # set calibration values (skip calibration run)
        m.set_ELECANGLEOFFSET(CAL_ELEC[i])
        m.set_SINCOSCENTRE(CAL_SINCOS[i])

        m.configure_operating_mode_and_sensor(3, 1)
        m.configure_command_mode(12)

    time.sleep(0.05)

def set_motors_from_wheels(wheels):
    """ wheels: [fl, fr, br, bl] normalized -1..1 """
    for idx, w in enumerate(wheels):
        speed = int(clamp(w, -1.0, 1.0) * MAX_MOTOR_SPEED)
        motors[idx].set_speed(speed)

def clamp(v, lo, hi): return max(lo, min(hi, v))

def read_stdin_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def main():
    init_motors()
    print("Robot controller started. Type 'angle,speed' (e.g. '10,5') or 'stop' to stop motors.")
    current_angle = None
    current_speed = 0
    imu_yaw = None  # if you later add BNO08x on Pi, set imu_yaw from IMU readings here

    try:
        while True:
            cmd = read_stdin_nonblocking()
            if cmd:
                if cmd.lower().strip() == "stop":
                    set_motors_from_wheels([0,0,0,0])
                    current_angle = None
                    current_speed = 0
                    print("Stopped")
                else:
                    try:
                        a_str, s_str = cmd.split(",")
                        a = float(a_str) % 360.0
                        s = int(s_str)
                        s = max(1, min(10, s))
                        current_angle = a
                        current_speed = s
                        print(f"Commanded angle {a} speed {s}")
                    except Exception as e:
                        print("Bad command. Use angle,speed or stop.", e)

            if current_angle is not None:
                aim, t_speed, rot_cmd, wheels = controller.compute_motion(
                    ball_angle_deg=current_angle,
                    distance_estimate=None,
                    speed_level=current_speed,
                    imu_yaw=imu_yaw
                )
                set_motors_from_wheels(wheels)
            else:
                # idle
                time.sleep(0.02)
            time.sleep(0.01)
    except KeyboardInterrupt:
        set_motors_from_wheels([0,0,0,0])
        print("Exiting and motors stopped.")

if __name__ == "__main__":
    main()
