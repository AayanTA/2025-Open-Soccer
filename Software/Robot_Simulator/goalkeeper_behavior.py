# goalkeeper_behavior.py — stay in front of own goal, track ball's Y

from common import (WIDTH, HEIGHT, BLUE, GOAL_OPENING, angle_to_target_deg,
                    normalize_angle_deg, clamp)
import controller

def step(robot, ball, dt):
    defend_left = (robot.color == BLUE)
    guard_x = 60 if defend_left else WIDTH - 60

    # Track ball's Y but clamp to goal opening (with a small margin so we stay centered)
    margin = 18
    gy0 = HEIGHT//2 - GOAL_OPENING//2 + margin
    gy1 = HEIGHT//2 + GOAL_OPENING//2 - margin
    target_y = clamp(ball.y, gy0, gy1)
    target_x = guard_x

    # Use controller pursuit policy toward the guard point (reusing same code as robot)
    target_angle_world = angle_to_target_deg(robot.x, robot.y, target_x, target_y)
    target_angle_rel = (target_angle_world - robot.initial_heading) % 360.0
    imu_yaw_rel = normalize_angle_deg(robot.heading - robot.initial_heading)

    return controller.compute_motion(
        ball_angle_deg=target_angle_rel,
        imu_yaw=imu_yaw_rel,
        last_translation_angle=robot.last_translation_angle_deg,
        dribbler_active=False
    )
