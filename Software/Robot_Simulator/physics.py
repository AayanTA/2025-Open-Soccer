# physics.py — unchanged core you had, with keep_in_bounds using real velocity

import math

def circle_circle_collision_detection(x1, y1, r1, x2, y2, r2):
    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)
    min_d = r1 + r2
    if dist < min_d:
        if dist < 1e-9:
            nx, ny = 1.0, 0.0
        else:
            nx, ny = dx/dist, dy/dist
        return True, dist, nx, ny
    return False, dist, 0.0, 0.0

def resolve_robot_ball_collision(robot_x, robot_y, robot_vx, robot_vy, robot_radius,
                                 ball_x, ball_y, ball_vx, ball_vy, ball_radius,
                                 restitution=0.6, damping=0.97, separate=True):
    colliding, dist, nx, ny = circle_circle_collision_detection(
        robot_x, robot_y, robot_radius, ball_x, ball_y, ball_radius
    )
    if not colliding:
        return ball_vx, ball_vy

    # Push ball out of the bumper (positional correction)
    if dist < 1e-6:
        dist = 1e-6
    overlap = (robot_radius + ball_radius) - dist
    if separate and overlap > 0:
        ball_x += nx * (overlap + 1e-4)
        ball_y += ny * (overlap + 1e-4)

    # Relative velocity
    rvx = ball_vx - robot_vx
    rvy = ball_vy - robot_vy
    rel_norm = rvx * nx + rvy * ny
    if rel_norm > 0:
        # already separating
        return ball_vx, ball_vy

    # Impulse only to ball (robot mass >> ball)
    j = -(1 + restitution) * rel_norm
    bx = ball_vx + j * nx
    by = ball_vy + j * ny

    # Light damping to kill jitter
    bx *= damping
    by *= damping
    return bx, by

def apply_friction(vx, vy, friction_coeff, dt):
    speed = math.hypot(vx, vy)
    if speed <= 1e-9:
        return 0.0, 0.0
    decel = friction_coeff * speed * dt
    if decel >= speed:
        return 0.0, 0.0
    s = (speed - decel) / speed
    return vx * s, vy * s

def keep_in_bounds(x, y, vx, vy, radius, width, height, restitution=0.8):
    # Left
    if x - radius < 0:
        x = radius
        vx = abs(vx) * restitution
    # Right
    if x + radius > width:
        x = width - radius
        vx = -abs(vx) * restitution
    # Top
    if y - radius < 0:
        y = radius
        vy = abs(vy) * restitution
    # Bottom
    if y + radius > height:
        y = height - radius
        vy = -abs(vy) * restitution
    return x, y, vx, vy
