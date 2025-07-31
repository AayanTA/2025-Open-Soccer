import cv2
import numpy as np
import time
from picamera2 import Picamera2
import math
import time

# Initialize Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# Ball RGB sample (mean from samples)
ball_rgb = np.array([42, 99, 224])
color_threshold = 65  # Tighter threshold to avoid wires

# Time setup
start_time = time.time()
run_duration = 60

# Detection memory setup
last_position = None
last_angle = None
last_radius = 0
last_seen_time = 0
persist_time = 2.0  # seconds to remember last seen ball

# Image center
frame_center = (320, 240)

# Motion detection setup
prev_gray = None

while True:
    if time.time() - start_time > run_duration:
        break

    frame_rgb = picam2.capture_array()

    # Convert to grayscale for motion
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    motion_mask = None
    if prev_gray is not None:
        frame_diff = cv2.absdiff(gray, prev_gray)
        _, motion_mask = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)
        motion_mask = cv2.dilate(motion_mask, None, iterations=2)
    prev_gray = gray

    # Color similarity mask
    diff = np.linalg.norm(frame_rgb - ball_rgb, axis=2)
    mask = (diff < color_threshold).astype(np.uint8) * 255

    # Combine with motion if available
    if motion_mask is not None:
        mask = cv2.bitwise_and(mask, motion_mask)

    # Morphological cleaning
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score = float('inf')

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 15:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        center = (int(x + w / 2), int(y + h / 2))

        # Skip objects fully on the edge (optional tightening)
        if x <= 5 or y <= 5 or x + w >= 635 or y + h >= 475:
            edge_weight = 1.2  # boost edge-score to help with partial visibility
        else:
            edge_weight = 1.0

        # Evaluate color match in ROI
        mask_roi = np.zeros_like(mask)
        cv2.drawContours(mask_roi, [contour], -1, 255, -1)
        mean_color = np.array(cv2.mean(frame_rgb, mask=mask_roi)[:3])
        color_dist = np.linalg.norm(mean_color - ball_rgb) * edge_weight

        if color_dist < best_score:
            best_score = color_dist
            best_contour = contour

    # Ball detection and memory
    if best_contour is not None and best_score < color_threshold:
        (x, y), radius = cv2.minEnclosingCircle(best_contour)
        center = (int(x), int(y))
        last_position = center
        last_radius = int(radius)
        last_seen_time = time.time()

        dx = center[0] - frame_center[0]
        dy = frame_center[1] - center[1]
        angle = (math.degrees(math.atan2(dx, dy)) + 360) % 360
        last_angle = angle

        # Draw detection
        cv2.circle(frame_rgb, center, last_radius, (0, 255, 0), 2)
        cv2.circle(frame_rgb, center, 5, (255, 0, 0), -1)
        cv2.putText(frame_rgb, f"Angle: {int(angle)} deg", (center[0] + 10, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        print(f"Ball Position: {center}, Angle: {int(angle)}°")

    elif last_position and time.time() - last_seen_time < persist_time:
        # Draw last known location
        cv2.circle(frame_rgb, last_position, last_radius, (0, 255, 255), 2)
        cv2.putText(frame_rgb, f"Last Angle: {int(last_angle)} deg", (last_position[0] + 10, last_position[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        print(f"Last Seen Ball @ {last_position}, Angle: {int(last_angle)}°")
    '''
    # Display
    cv2.imshow("Ball Tracking", frame_rgb)
    cv2.imshow("Color + Motion Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    '''
    # Save the frames to view manually instead of displaying
    cv2.imwrite("/home/versioncontrol/Desktop/frame.jpg", frame_rgb)
    cv2.imwrite("/home/versioncontrol/Desktop/mask.jpg", mask)

cv2.destroyAllWindows()
picam2.stop()
