# striker_behaviour.py — capture ball, then drive to opponent goal using camera-style angles

from common import (WIDTH, HEIGHT, BLUE, angle_to_target_deg, normalize_angle_deg)
import controller

def step(robot, ball, dt):
    """
    The simulator sets robot.has_ball based on dribbler zone.
    Here we convert *world* geometry to the same 0..360° relative angles your camera uses,
    then call the shared controller.
    """
    imu_yaw_rel = normalize_angle_deg(robot.heading - robot.initial_heading)

    if robot.has_ball:
        # --- Take the ball to the opponent goal ---
        goal_x = WIDTH - 30 if robot.color == BLUE else 30
        goal_y = HEIGHT // 2
        goal_angle_world = angle_to_target_deg(robot.x, robot.y, goal_x, goal_y)
        goal_angle_rel = (goal_angle_world - robot.initial_heading) % 360.0

        return controller.compute_motion(
            ball_angle_deg=0,  # ignored in scoring mode
            imu_yaw=imu_yaw_rel,
            last_translation_angle=robot.last_translation_angle_deg,
            dribbler_active=True,
            goal_angle_deg=goal_angle_rel
        )

    # --- Otherwise: chase the ball ---
    world_ball_angle = angle_to_target_deg(robot.x, robot.y, ball.x, ball.y)
    ball_angle_rel = (world_ball_angle - robot.initial_heading) % 360.0

    return controller.compute_motion(
        ball_angle_deg=ball_angle_rel,
        imu_yaw=imu_yaw_rel,
        last_translation_angle=robot.last_translation_angle_deg,
        dribbler_active=False
    )
