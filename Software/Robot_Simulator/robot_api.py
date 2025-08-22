# robot_api.py
import math
from dataclasses import dataclass

@dataclass
class RobotState:
    id: int
    x: float   # mm
    y: float   # mm
    theta: float  # deg, 0 = facing +y (field forward)
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

@dataclass
class BallState:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0

def xdrive_wheel_speeds_from_cmd(translation_angle_deg, translation_speed_norm, rotation_norm):
    """
    Returns wheel speeds normalized -1..1 for front-left, front-right, back-right, back-left
    using the X-drive mixing you used in your robot code.
    """
    rad = math.radians(translation_angle_deg % 360.0)
    drive = translation_speed_norm * math.cos(rad)
    strafe = translation_speed_norm * math.sin(rad)
    m1 = -(drive + strafe + rotation_norm)  # FL
    m2 =  (drive - strafe - rotation_norm)  # FR
    m3 =  (drive + strafe - rotation_norm)  # BR
    m4 = -(drive - strafe + rotation_norm)  # BL
    arr = [m1, m2, m3, m4]
    maxv = max(abs(x) for x in arr)
    if maxv > 1.0:
        arr = [x / maxv for x in arr]
    return arr

def xdrive_body_vel_from_wheels(wheel_speeds, Vmax_mm_s=500.0, Omega_max_deg_s=180.0):
    """Approximate conversion from normalized wheel speeds to robot body velocities."""
    m1, m2, m3, m4 = wheel_speeds
    drive = 0.5 * (m2 + m3)
    strafe = 0.5 * (m3 - m2)
    rotation = -0.25 * (m1 - m2 + m3 - m4)
    vx = drive * Vmax_mm_s
    vy = strafe * Vmax_mm_s
    omega = rotation * Omega_max_deg_s
    return vx, vy, omega
