#!/usr/bin/env python3
"""
Integrated ball-tracker + motor control + optional BNO08x IMU
- Uses Picamera2 + OpenCV for ball detection (adapted from your working script)
- Uses steelbar_powerful_bldc_driver.PowerfulBLDCDriver for motors
- Uses Adafruit BNO08x CircuitPython driver if available
Controls:
  start         -> start tracking/moving
  stop          -> stop motors and pause
  dir <0-359>   -> set direction manually
  speed <1-10>  -> set speed (1 -> 10%)
  quit / exit   -> stop program
"""

import time
import math
import sys
import select
import numpy as np
import cv2

# hardware libs
import board
import busio
from picamera2 import Picamera2

# motor driver lib (your steelbar library)
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# Try to import BNO08x library (optional)
try:
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_ROTATION_VECTOR,
    )
    HAVE_IMU = True
except Exception:
    # If adafruit libs are not installed or IMU not connected, we fallback
    HAVE_IMU = False

# ---------------------------
# Configuration & constants
# ---------------------------
# Motor I2C addresses (your mapping)
MOTOR_ADDRS = [25, 26, 27, 28]  # Motor1 FL, Motor2 FR, Motor3 BR, Motor4 BL

# Hardcoded calibration values (per motor 0..3)
CAL_ELECANGLEOFFSET = [1439128576, 1793410560, 1320240896, 1345708544]
CAL_SINCOSCENTRE    = [1252, 1236, 1251, 1258]

# Motor speed scaling
MAX_MOTOR_SPEED = 60000000  # as used previously

# Heading control
HEADING_KP = 1.0  # proportional gain; tune as needed

# Camera parameters (as per your working script)
BALL_RGB = np.array([42, 99, 224])
COLOR_THRESHOLD = 65
PERSIST_TIME = 2.0  # seconds to remember last detection
FRAME_SIZE = (640, 480)
FRAME_CENTER = (FRAME_SIZE[0] // 2, FRAME_SIZE[1] // 2)

# Startup values
running = False
desired_direction = None   # if user sets direction manually
speed_setting = 5          # 1..10 integer (percent *10)
speed_normalized = speed_setting / 10.0

# ---------------------------
# Initialize hardware
# ---------------------------
# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Motors: create objects for addresses and apply calibration + FOC settings
motors = []
for idx, addr in enumerate(MOTOR_ADDRS):
    drv = PowerfulBLDCDriver(i2c, addr)
    motors.append(drv)

# Configure motors (no runtime calibration; use stored values)
for i, m in enumerate(motors):
    # minimal setup (taken from example)
    m.set_current_limit_foc(65536*2)   # safety: slightly larger limit
    m.set_id_pid_constants(1500, 200)
    m.set_iq_pid_constants(1500, 200)
    m.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
    m.set_position_pid_constants(275, 0, 0)
    m.set_position_region_boundary(250000)
    m.set_speed_limit(10000000)
    # set hard-coded calibration values
    m.set_ELECANGLEOFFSET(int(CAL_ELECANGLEOFFSET[i]))
    m.set_SINCOSCENTRE(int(CAL_SINCOSCENTRE[i]))
    # configure FOC & speed command mode
    m.configure_operating_mode_and_sensor(3, 1)
    m.configure_command_mode(12)

# IMU (optional)
if HAVE_IMU:
    try:
        bno = BNO08X_I2C(i2c, address=0x74)
        # enable game rotation vector (fast, gyro+acc fused, not necessarily magnetic-north referenced)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        imu_ok = True
        print("BNO08x IMU enabled (game rotation vector).")
    except Exception as e:
        imu_ok = False
        print("Failed to init BNO08x IMU:", e)
else:
    imu_ok = False

initial_yaw = 0.0
initial_yaw_set = False

# set up camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": FRAME_SIZE}))
picam2.start()
prev_gray = None

# helper: non-blocking stdin read
def read_stdin_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

# quaternion -> euler helper (degrees)
def quaternion_to_euler_deg(w, x, y, z):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def normalize_angle_180(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle

# Movement mixing for X omni (matches your sign convention):
# For translationAngleDeg = 0 (forward) expect: m1 (FL) negative, m2 (FR) positive, m3 (BR) positive, m4 (BL) negative
def set_motor_speeds(translation_angle_deg, translation_speed_normalized, rotation):
    # compute drive/strafe
    rad = math.radians(translation_angle_deg)
    drive  = translation_speed_normalized * math.cos(rad)
    strafe = translation_speed_normalized * math.sin(rad)
    m1 = -(drive + strafe + rotation)  # Front Left  (should be negative for forward)
    m2 =  (drive - strafe - rotation)  # Front Right
    m3 =  (drive + strafe - rotation)  # Back Right
    m4 = -(drive - strafe + rotation)  # Back Left
    arr = [m1, m2, m3, m4]
    # normalize
    maxv = max(abs(x) for x in arr)
    if maxv > 1.0:
        arr = [x / maxv for x in arr]
    # scale to speed
    scaled = [int(x * MAX_MOTOR_SPEED) for x in arr]
    # send commands
    for idx, s in enumerate(scaled):
        motors[idx].set_speed(int(s))
    # debug print
    print(f"[motors] dir={int(translation_angle_deg)} speed={translation_speed_normalized:.2f} rot={rotation:.3f} -> {scaled}")

def stop_motors():
    for m in motors:
        m.set_speed(0)
    print("[motors] stopped")

# Main loop
try:
    last_position = None
    last_angle = None
    last_radius = 0
    last_seen_time = 0.0

    print("Ready. Commands: start | stop | dir <0-359> | speed <1-10> | quit")
    while True:
        # handle stdin commands
        cmd = read_stdin_nonblocking()
        if cmd:
            parts = cmd.split()
            if parts[0].lower() in ("start", "s"):
                running = True
                print("▶️ Running")
            elif parts[0].lower() in ("stop",):
                running = False
                stop_motors()
                print("⏸️ Stopped")
            elif parts[0].lower() in ("dir",):
                try:
                    a = float(parts[1])
                    desired_direction = a % 360.0
                    print(f"Manual direction set to {desired_direction:.1f}°")
                except Exception:
                    print("Usage: dir <0-359>")
            elif parts[0].lower() in ("speed",):
                try:
                    v = int(parts[1])
                    if 1 <= v <= 10:
                        speed_setting = v
                        speed_normalized = v / 10.0
                        print(f"Speed set to {v} (normalized {speed_normalized:.2f})")
                    else:
                        print("speed must be 1..10")
                except Exception:
                    print("Usage: speed <1-10>")
            elif parts[0].lower() in ("quit", "exit"):
                print("Exiting...")
                break
            else:
                print("Unknown command:", cmd)

        if not running:
            time.sleep(0.05)
            continue

        # Capture frame & do detection
        frame_rgb = picam2.capture_array()
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        motion_mask = None
        if prev_gray is not None:
            frame_diff = cv2.absdiff(gray, prev_gray)
            _, motion_mask = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)
            motion_mask = cv2.dilate(motion_mask, None, iterations=2)
        prev_gray = gray

        diff = np.linalg.norm(frame_rgb - BALL_RGB, axis=2)
        mask = (diff < COLOR_THRESHOLD).astype(np.uint8) * 255
        if motion_mask is not None:
            mask = cv2.bitwise_and(mask, motion_mask)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_score = float('inf')
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 15:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center = (int(x + w / 2), int(y + h / 2))
            edge_weight = 1.2 if (x <= 5 or y <= 5 or x + w >= FRAME_SIZE[0]-5 or y + h >= FRAME_SIZE[1]-5) else 1.0
            mask_roi = np.zeros_like(mask)
            cv2.drawContours(mask_roi, [contour], -1, 255, -1)
            mean_color = np.array(cv2.mean(frame_rgb, mask=mask_roi)[:3])
            color_dist = np.linalg.norm(mean_color - BALL_RGB) * edge_weight
            if color_dist < best_score:
                best_score = color_dist
                best_contour = contour

        detected_angle = None
        detected_radius = None
        if best_contour is not None and best_score < COLOR_THRESHOLD:
            (x, y), radius = cv2.minEnclosingCircle(best_contour)
            center = (int(x), int(y))
            last_position = center
            last_radius = int(radius)
            last_seen_time = time.time()
            dx = center[0] - FRAME_CENTER[0]
            dy = FRAME_CENTER[1] - center[1]
            angle = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
            last_angle = angle
            detected_angle = angle
            detected_radius = radius
            print(f"Ball pos {center} angle {int(angle)} radius {int(radius)}")
        elif last_position and time.time() - last_seen_time < PERSIST_TIME:
            detected_angle = last_angle
            detected_radius = last_radius
            print(f"Using last seen angle {int(last_angle)} (persist)")
        else:
            # No ball found -> search behaviour: rotate slowly
            print("No ball detected -> rotating to search")
            # simple rotate: set motors to spin pattern (positive values chosen to give rotation)
            # pattern from earlier spinAround: m1 & m2 forward, m3 & m4 reverse
            rot_speed = int(0.12 * MAX_MOTOR_SPEED)  # slow spin
            motors[0].set_speed(rot_speed)
            motors[1].set_speed(rot_speed)
            motors[2].set_speed(-rot_speed)
            motors[3].set_speed(-rot_speed)
            time.sleep(0.05)
            continue

        # IMU heading correction (if available)
        rotation = 0.0
        if imu_ok:
            try:
                # game_quaternion property returns (i, j, k, real)
                quat = bno.game_quaternion  # returns tuple or None
                if quat:
                    i, j, k, real = quat
                    _, _, yaw = quaternion_to_euler_deg(real, i, j, k)
                    if not initial_yaw_set:
                        initial_yaw = yaw
                        initial_yaw_set = True
                    rel = normalize_angle_180(yaw - initial_yaw)
                    rotation = HEADING_KP * (rel / 180.0)
                    rotation = max(min(rotation, 1.0), -1.0)
                else:
                    rotation = 0.0
            except Exception as e:
                print("IMU read error:", e)
                rotation = 0.0

        # Use manual desired_direction override if supplied, else use camera reading
        if desired_direction is not None:
            dir_to_move = desired_direction
        else:
            dir_to_move = detected_angle

        # Scale speed from speed_normalized
        speed_norm = speed_normalized

        # Command motors
        set_motor_speeds(dir_to_move, speed_norm, rotation)

        # small sleep to avoid hogging CPU
        time.sleep(0.02)

except KeyboardInterrupt:
    print("KeyboardInterrupt — cleaning up")

finally:
    stop_motors()
    picam2.stop()
    cv2.destroyAllWindows()
    print("Program end.")
