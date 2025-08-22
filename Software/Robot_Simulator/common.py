# common.py — shared constants and helpers

import math

# ----- Field geometry -----
WIDTH, HEIGHT = 900, 600
GOAL_OPENING = 160

# ----- Colours -----
BLUE  = (60, 140, 240)
RED   = (240, 80, 80)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# ----- Helpers -----
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize_angle_deg(a):
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a

def angle_to_target_deg(src_x, src_y, target_x, target_y):
    dx = target_x - src_x
    dy = target_y - src_y
    return (math.degrees(math.atan2(dx, -dy)) % 360.0)
