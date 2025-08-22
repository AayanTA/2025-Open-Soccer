# controller.py — pursuit + goal carry + tweakable zones/directions (keeps prior API)

import math

# ---------------- Angle helpers ----------------
def normalize_angle_deg(a):
    while a >= 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a

def wrap360(a):
    a %= 360.0
    if a < 0:
        a += 360.0
    return a

def smooth_angle_transition(current_angle, target_angle, k=0.15):
    """
    Critically damp large jumps in commanded translation angle so the robot doesn't snap.
    Returns angle in [0,360).
    """
    cur = normalize_angle_deg(current_angle)
    tgt = normalize_angle_deg(target_angle)
    diff = normalize_angle_deg(tgt - cur)
    return wrap360(cur + diff * k)

# ---------------- Movement config (EDIT HERE) ----------------
def create_movement_config(
    *,
    # Movement directions (edit these to change where the robot *moves* for a given zone)
    angles=None,
    # Angle-zone boundaries (edit these to change *which* angles map to zones)
    zones=None,
    # Speed multipliers by zone type
    speed_multipliers=None,
):
    """
    Returns a dict with:
      - 'angles': mapping from labels -> commanded translation angles (deg)
      - 'zones': list of dicts: { 'range': (lo_deg, hi_deg), 'label': <angles key>, 'speed_group': 'forward'|'side'|'back' }
                 ranges are inclusive of lo, exclusive of hi; if lo > hi, the range wraps across 0°
      - 'speed_multipliers': dict for speed scaling by group
    """

    default_angles = {
        # These defaults match your previously working behaviour
        # (Feel free to change e.g. 'side_right': 170.0 etc.)
        'forward':        0.0,
        'forward_right':  80.0,
        'side_right':     160.0,   # <-- example: set to 170.0 to make more rear-biased right-move
        'back_right':     180.0,
        'avoid_own_goal': 135.0,
        'back_left':      180.0,
        'side_left':      200.0,   # kept as in your previous file
        'forward_left':   280.0,
    }
    if angles:
        default_angles.update(angles)

    default_zones = [
        # Directly ahead:
        {'range': (345,  15),  'label': 'forward',        'speed_group': 'forward'},  # wraps across 0
        {'range': ( 15,  75),  'label': 'forward_right',  'speed_group': 'forward'},
        {'range': ( 75,  90),  'label': 'side_right',     'speed_group': 'side'},
        {'range': ( 90, 165),  'label': 'back_right',     'speed_group': 'back'},
        {'range': (165, 195),  'label': 'avoid_own_goal', 'speed_group': 'back'},     # straight behind
        {'range': (195, 270),  'label': 'back_left',      'speed_group': 'back'},
        {'range': (270, 285),  'label': 'side_left',      'speed_group': 'side'},
        {'range': (285, 345),  'label': 'forward_left',   'speed_group': 'forward'},
    ]
    if zones:
        default_zones = zones

    default_speeds = {'forward': 1.0, 'side': 0.8, 'back': 1.0}
    if speed_multipliers:
        default_speeds.update(speed_multipliers)

    return {
        'angles': default_angles,
        'zones': default_zones,
        'speed_multipliers': default_speeds,
    }

# One place to tweak defaults; behaviours will use this unless they pass a custom config
DEFAULT_MOVEMENT_CONFIG = create_movement_config()

# ---------------- Kinematics mix ----------------
def compute_wheel_mix(translation_angle_deg, translation_speed_norm, rotation_norm):
    """
    X-drive mixer (front-left, front-right, back-right, back-left), normalized -1..1.
    translation_angle_deg uses your convention: 0=forward, 90=right, 180=back, 270=left.
    """
    ang = math.radians(translation_angle_deg)
    drive = translation_speed_norm * math.cos(ang)
    strafe = translation_speed_norm * math.sin(ang)

    # X-drive mixing (FL, FR, BR, BL)
    t_fl = -(drive + strafe)
    t_fr =  (drive - strafe)
    t_br =  (drive + strafe)
    t_bl = -(drive - strafe)

    r_fl =  rotation_norm
    r_fr =  rotation_norm
    r_br = -rotation_norm
    r_bl = -rotation_norm

    wheels = [t_fl + r_fl, t_fr + r_fr, t_br + r_br, t_bl + r_bl]
    m = max(1.0, max(abs(w) for w in wheels))
    return [w/m for w in wheels]

# ---------------- Pursuit policy ----------------
def _select_zone(ball_angle_deg, zones):
    """
    Returns the matched zone dict from config['zones'] for the given ball_angle_deg in [0,360).
    """
    a = wrap360(ball_angle_deg)
    for z in zones:
        lo, hi = z['range']
        if lo <= hi:
            if lo <= a < hi:
                return z
        else:
            # wrapped interval (e.g., 345..15)
            if a >= lo or a < hi:
                return z
    # Fallback: first zone
    return zones[0]

def _pursuit_policy(ball_angle_deg, translation_speed, last_translation_angle, config):
    """
    Translate camera ball angle -> commanded translation angle via zones/config.
    """
    angles = config['angles']
    zones = config['zones']
    spd_groups = config['speed_multipliers']

    z = _select_zone(ball_angle_deg, zones)
    tgt = angles[z['label']]
    spd = translation_speed * spd_groups[z['speed_group']]

    if last_translation_angle is not None:
        out_ang = smooth_angle_transition(last_translation_angle, tgt, 0.15)
    else:
        out_ang = tgt
    return out_ang, spd

# ---------------- Public API (kept) ----------------
def compute_motion(ball_angle_deg, distance_estimate=None, speed_level=10, imu_yaw=None,
                   bias_direction=90.0, last_translation_angle=None, movement_config=None,
                   dribbler_active=False, goal_angle_deg=None):
    """
    If dribbler_active=True and goal_angle_deg is given: drive straight to goal at max speed.
    Otherwise: use pursuit policy (ball-chasing).
    All angles are degrees (0..360), same convention as your camera & drive code.
    """
    config = movement_config if movement_config is not None else DEFAULT_MOVEMENT_CONFIG

    # --- Scoring / carry-to-goal mode ---
    if dribbler_active and goal_angle_deg is not None:
        target_angle = wrap360(goal_angle_deg)
        translation_speed = 1.0  # full send to goal
        if last_translation_angle is not None:
            translation_angle = smooth_angle_transition(last_translation_angle, target_angle, 0.15)
        else:
            translation_angle = target_angle
        rotation_cmd = 0.0  # keep it straight while carrying
        wheels = compute_wheel_mix(translation_angle, translation_speed, rotation_cmd)
        return translation_angle, translation_speed, rotation_cmd, wheels

    # --- Ball pursuit ---
    translation_speed = 1.0  # max (you can scale by speed_level if desired)
    translation_angle, translation_speed = _pursuit_policy(
        ball_angle_deg, translation_speed, last_translation_angle, config
    )

    # Gentle yaw centering (relative IMU yaw)
    if imu_yaw is not None:
        yaw_err = normalize_angle_deg(imu_yaw)
        rot_gain = 0.8
        rotation_cmd = -(yaw_err / 180.0) * rot_gain
        a = wrap360(ball_angle_deg)
        # reduce rotation when ball is directly behind (avoid oversteer)
        is_behind = (165 <= a <= 195)
        if is_behind:
            rotation_cmd *= 0.4
        rotation_cmd = max(-1.0, min(1.0, rotation_cmd))
    else:
        rotation_cmd = 0.0

    wheels = compute_wheel_mix(translation_angle, translation_speed, rotation_cmd)
    return translation_angle, translation_speed, rotation_cmd, wheels
