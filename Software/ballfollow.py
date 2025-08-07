# Hopefully will work, needs adjustment

import cv2
import numpy as np
import time
import math
from picamera2 import Picamera2
from gpiozero import Button
import board
import busio
from adafruit_mpu6050 import MPU6050
from pwmio import PWMOut
from steelbar_powerful_bldc_driver import Motor

# ─────────────────────────── GPIO SETUP ───────────────────────────
button = Button(2)
running = False
was_running = False

def toggle_run():
    global running
    running = not running
    print(f"🔘 Button pressed: {'▶️ Running' if running else '⏸️ Paused'}")

button.when_pressed = toggle_run

# ─────────────────────────── CAMERA SETUP ───────────────────────────
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# ─────────────────────────── IMU SETUP ───────────────────────────
i2c = busio.I2C(board.SCL, board.SDA)
imu = MPU6050(i2c)
initial_angle = None

# ─────────────────────────── MOTOR SETUP ───────────────────────────
left_motor = Motor(PWMOut(board.D5), PWMOut(board.D6))
right_motor = Motor(PWMOut(board.D9), PWMOut(board.D10))

def stop_motors():
    left_motor.stop()
    right_motor.stop()

def set_motor_speeds(left_speed, right_speed):
    left_motor.set_speed(left_speed)
    right_motor.set_speed(right_speed)

# ─────────────────────────── BALL TRACKING SETUP ───────────────────────────
ball_rgb = np.array([42, 99, 224])
color_threshold = 65
frame_center = (320, 240)
prev_gray = None
last_position = None
last_angle = None
last_seen_time = 0
persist_time = 2.0

# ─────────────────────────── MAIN LOOP ───────────────────────────
print("🟡 Waiting for button press to start...")

while True:
    if not running:
        if was_running:
            print("⏸️ Paused")
            stop_motors()
            was_running = False
        time.sleep(0.1)
        continue

    if not was_running:
        print("▶️ Started tracking")
        initial_angle = imu.gyro[2]  # Get Z axis (yaw)
        was_running = True

    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Motion mask
    motion_mask = None
    if prev_gray is not None:
        diff = cv2.absdiff(gray, prev_gray)
        _, motion_mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        motion_mask = cv2.dilate(motion_mask, None, iterations=2)
    prev_gray = gray

    # Color mask
    diff = np.linalg.norm(frame - ball_rgb, axis=2)
    mask = (diff < color_threshold).astype(np.uint8) * 255
    if motion_mask is not None:
        mask = cv2.bitwise_and(mask, motion_mask)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score = float('inf')

    for contour in contours:
        if cv2.contourArea(contour) < 15:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        mask_roi = np.zeros_like(mask)
        cv2.drawContours(mask_roi, [contour], -1, 255, -1)
        mean_color = np.array(cv2.mean(frame, mask=mask_roi)[:3])
        color_dist = np.linalg.norm(mean_color - ball_rgb)
        if color_dist < best_score:
            best_score = color_dist
            best_contour = contour

    # Ball logic
    ball_detected = False
    if best_contour is not None and best_score < color_threshold:
        (x, y), radius = cv2.minEnclosingCircle(best_contour)
        center = (int(x), int(y))
        dx = center[0] - frame_center[0]
        dy = frame_center[1] - center[1]
        angle = (math.degrees(math.atan2(dx, dy)) + 360) % 360
        last_position = center
        last_angle = angle
        last_seen_time = time.time()
        ball_detected = True
    elif last_position and time.time() - last_seen_time < persist_time:
        angle = last_angle
        ball_detected = True
    else:
        stop_motors()
        continue

    # Heading Correction
    current_gyro = imu.gyro[2]  # Z-axis (yaw)
    heading_error = current_gyro - initial_angle
    correction = -heading_error * 0.8  # proportional gain

    # Motor decision
    if ball_detected:
        deviation = angle - 180
        base_speed = 0.5  # You can tune this
        turn = (deviation / 180) * base_speed
        left_speed = max(min(base_speed - turn + correction, 1), -1)
        right_speed = max(min(base_speed + turn - correction, 1), -1)
        set_motor_speeds(left_speed, right_speed)

    # Optional: Save debug images
    cv2.imwrite("/home/versioncontrol/Desktop/frame.jpg", frame)
    cv2.imwrite("/home/versioncontrol/Desktop/mask.jpg", mask)
