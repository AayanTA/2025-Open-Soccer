# simulator.py (updated)
import pygame, sys, time, math, importlib, os, runpy

# Ensure local imports work when running from any working directory
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

import argparse

# Support both package-relative import (when run with -m) and script execution
try:
    from .robot_api import RobotState, BallState, xdrive_body_vel_from_wheels, xdrive_wheel_speeds_from_cmd  # type: ignore
    from .sim_hardware import bind_address_to_robot, SimAPI, PowerfulBLDCDriver, BNO08X_I2C, bind_IMU_to_robot  # type: ignore
except Exception:
    # Fallback: load sibling modules directly by path to avoid import resolution issues
    _robot_api_path = os.path.join(_THIS_DIR, "robot_api.py")
    _sim_hw_path = os.path.join(_THIS_DIR, "sim_hardware.py")
    _ra_spec = importlib.util.spec_from_file_location("robot_api", _robot_api_path)
    robot_api = importlib.util.module_from_spec(_ra_spec)
    assert _ra_spec and _ra_spec.loader
    _ra_spec.loader.exec_module(robot_api)
    _sh_spec = importlib.util.spec_from_file_location("sim_hardware", _sim_hw_path)
    sim_hardware = importlib.util.module_from_spec(_sh_spec)
    assert _sh_spec and _sh_spec.loader
    _sh_spec.loader.exec_module(sim_hardware)
    RobotState = robot_api.RobotState
    BallState = robot_api.BallState
    xdrive_body_vel_from_wheels = robot_api.xdrive_body_vel_from_wheels
    xdrive_wheel_speeds_from_cmd = robot_api.xdrive_wheel_speeds_from_cmd
    bind_address_to_robot = sim_hardware.bind_address_to_robot
    SimAPI = sim_hardware.SimAPI
    PowerfulBLDCDriver = sim_hardware.PowerfulBLDCDriver
    BNO08X_I2C = sim_hardware.BNO08X_I2C
    bind_IMU_to_robot = sim_hardware.bind_IMU_to_robot

# Field geometry (scaled)
FIELD_MM_W = 1580.0
FIELD_MM_H = 2190.0
WINDOW_W = 1000
WINDOW_H = int(WINDOW_W * (FIELD_MM_W / FIELD_MM_H))
FPS = 60

def f2px(pos):
    x_mm, y_mm = pos
    px = int((x_mm / FIELD_MM_H) * WINDOW_W)
    py = int((1.0 - (y_mm / FIELD_MM_W)) * WINDOW_H)
    return px, py

class SimRobot:
    def __init__(self, id, x_mm, y_mm, theta_deg, addr_list):
        self.id = id
        self.state = RobotState(id, x_mm, y_mm, theta_deg)
        self.wheel_norms = [0.0,0.0,0.0,0.0]  # FL,FR,BR,BL normalized -1..1
        self.addr_list = addr_list  # list of i2c addresses mapped to wheels in order FL,FR,BR,BL
        # bind addresses to this sim robot so simulated drivers can connect
        for a in addr_list:
            bind_address_to_robot(a, self)
        # create a simulated BNO object and bind
        self.bno = BNO08X_I2C(None, address=0x74 + id)
        bind_IMU_to_robot(self.bno, self)

        # store initial yaw (used by IMU-relative returns)
        self.initial_theta = self.state.theta

    def set_wheel_from_address(self, address, normalized):
        # map address to wheel index
        if address not in self.addr_list:
            return
        idx = self.addr_list.index(address)
        # clamp normalized
        n = max(-1.0, min(1.0, normalized))
        self.wheel_norms[idx] = n

    def apply_wheel_norms(self, arr):
        # ensure we copy and clamp
        arrc = [max(-1.0, min(1.0, float(x))) for x in arr]
        # if arr length !=4, ignore
        if len(arrc) == 4:
            self.wheel_norms = arrc.copy()

    def cmd_to_wheel_norms(self, angle_deg, speed_norm, rotation_norm):
        # clamp inputs
        speed_norm = max(0.0, min(1.0, speed_norm))
        rotation_norm = max(-1.0, min(1.0, rotation_norm))
        return xdrive_wheel_speeds_from_cmd(angle_deg, speed_norm, rotation_norm)

    def get_relative_yaw(self):
        # return yaw relative to initial_theta in [-180,180)
        rel = self.state.theta - self.initial_theta
        while rel >= 180.0: rel -= 360.0
        while rel < -180.0: rel += 360.0
        return rel

    def apply(self, dt):
        # convert wheel norms into body velocities
        # clamp wheel norms again defensively
        norms = [max(-1.0, min(1.0, w)) for w in self.wheel_norms]

        # Max speeds (tune here)
        Vmax_mm_s = 700.0   # forward linear velocity when wheel norm is 1
        Omega_max_deg_s = 120.0  # rotation when rotation term is 1

        vx, vy, omega = xdrive_body_vel_from_wheels(norms, Vmax_mm_s=Vmax_mm_s, Omega_max_deg_s=Omega_max_deg_s)

        # safety clamp velocities (prevent explosion)
        MAX_V = 1200.0  # mm/s
        MAX_OMEGA = 360.0  # deg/s
        vx = max(-MAX_V, min(MAX_V, vx))
        vy = max(-MAX_V, min(MAX_V, vy))
        omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))

        # transform body velocities to world coordinates.
        theta_rad = math.radians(self.state.theta)
        dx_world = vx * math.sin(theta_rad) + vy * math.cos(theta_rad)
        dy_world = vx * math.cos(theta_rad) - vy * math.sin(theta_rad)

        # integrate
        self.state.x += dx_world * dt
        self.state.y += dy_world * dt
        self.state.theta = (self.state.theta + omega * dt) % 360.0

        # keep robot inside field bounds (clamp)
        margin = 30.0
        self.state.x = max(margin, min(FIELD_MM_H - margin, self.state.x))
        self.state.y = max(margin, min(FIELD_MM_W - margin, self.state.y))

def load_controller_as_plugin(module_path, robot_id=0):
    spec = importlib.import_module(module_path)
    # Prefer class Controller
    if hasattr(spec, 'Controller'):
        try:
            return spec.Controller(robot_id)
        except TypeError:
            return spec.Controller()
    # fallback to function update
    if hasattr(spec, 'update'):
        class _FWrapper:
            def __init__(self): pass
            def update(self, dt, robot_state, ball_state, sim_hw):
                return spec.update(dt, robot_state, ball_state, sim_hw)
        return _FWrapper()
    raise RuntimeError("Controller module must define Controller class or update function")

def run_sim(controller_a="controllers.soccer_controller", controller_b="controllers.soccer_controller"):
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("RCJ Open Simulator (2 robots)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)

    # initial placements: robots on opposite halves, facing specified headings
    center_x = FIELD_MM_H / 2.0
    center_y = FIELD_MM_W / 2.0
    offset = 250.0

    # Robot 0 (blue) start facing 90 degrees (user requested)
    r0 = SimRobot(0, center_x - offset, FIELD_MM_W*0.25, 90.0, addr_list=[25,26,27,28])

    # Robot 1 (red) start facing 270 degrees (user requested)
    # avoid using the same i2c addresses as robot0 in sim; they are arbitrary here
    r1 = SimRobot(1, center_x + offset, FIELD_MM_W*0.75, 270.0, addr_list=[35,36,37,38])

    # store initial thetas explicitly (so IMU rel yaw is meaningful)
    r0.initial_theta = r0.state.theta
    r1.initial_theta = r1.state.theta

    # Bind a SimAPI for each robot (this will be given to plugin controllers)
    sim_api_r0 = SimAPI(r0)
    sim_api_r1 = SimAPI(r1)

    # Load controllers
    c0 = load_controller_as_plugin(controller_a, robot_id=0)
    c1 = load_controller_as_plugin(controller_b, robot_id=1)

    ball = BallState(center_x, center_y, 0.0, 0.0)

    running = False
    mouse_drag = None

    while True:
        dt = clock.tick(FPS)/1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); return
            if ev.type == pygame.MOUSEBUTTONDOWN:
                mx,my = ev.pos
                bx,by = f2px((ball.x, ball.y))
                if (mx-bx)**2+(my-by)**2 <= 15**2:
                    mouse_drag = ('ball',)
                else:
                    for rid,r in enumerate([r0,r1]):
                        rx,ry = f2px((r.state.x, r.state.y))
                        if (mx-rx)**2+(my-ry)**2 <= 24**2:
                            mouse_drag = ('robot', rid)
                            break
            if ev.type == pygame.MOUSEBUTTONUP:
                mouse_drag = None
            if ev.type == pygame.MOUSEMOTION and mouse_drag:
                mx,my = ev.pos
                x_mm = (mx / WINDOW_W) * FIELD_MM_H
                y_mm = (1.0 - (my / WINDOW_H)) * FIELD_MM_W
                if mouse_drag[0] == 'ball':
                    ball.x = x_mm; ball.y = y_mm; ball.vx=0; ball.vy=0
                else:
                    idx = mouse_drag[1]
                    if idx==0:
                        r0.state.x = x_mm; r0.state.y = y_mm
                    else:
                        r1.state.x = x_mm; r1.state.y = y_mm
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    running = not running
                if ev.key == pygame.K_r:
                    # reset positions
                    r0.state.x, r0.state.y = center_x - offset, FIELD_MM_W*0.25
                    r1.state.x, r1.state.y = center_x + offset, FIELD_MM_W*0.75
                    # reset facing as requested
                    r0.state.theta = 90.0
                    r1.state.theta = 270.0
                    r0.initial_theta = r0.state.theta
                    r1.initial_theta = r1.state.theta
                    ball.x, ball.y = center_x, center_y; ball.vx=0; ball.vy=0

        if running:
            # plugin controllers: they return dict commands or call sim_api methods
            cmd0 = c0.update(dt, r0.state, ball, sim_api_r0)
            if cmd0 is None:
                pass
            elif cmd0.get('type') == 'cmd':
                arr = xdrive_wheel_speeds_from_cmd(cmd0['translation_angle_deg'], cmd0['translation_speed'], cmd0.get('rotation',0.0))
                r0.apply_wheel_norms(arr)
            elif cmd0.get('type') == 'wheels':
                r0.apply_wheel_norms(cmd0['wheel_speeds'])

            cmd1 = c1.update(dt, r1.state, ball, sim_api_r1)
            if cmd1 is None:
                pass
            elif cmd1.get('type') == 'cmd':
                arr = xdrive_wheel_speeds_from_cmd(cmd1['translation_angle_deg'], cmd1['translation_speed'], cmd1.get('rotation',0.0))
                r1.apply_wheel_norms(arr)
            elif cmd1.get('type') == 'wheels':
                r1.apply_wheel_norms(cmd1['wheel_speeds'])

            # physics step
            r0.apply(dt)
            r1.apply(dt)
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            ball.vx *= 0.995; ball.vy *= 0.995

            # ball collisions: gentle pushes
            for r in (r0,r1):
                dx = ball.x - r.state.x; dy = ball.y - r.state.y
                d = math.hypot(dx,dy)
                if d < (90 + 25): # ball radius 25mm, robot radius ~90
                    # simple gentle push proportional to overlap only
                    overlap = (90 + 25) - d
                    if d < 1e-6:
                        nx,ny = 0,0
                    else:
                        nx,ny = dx/d, dy/d
                    force = 120.0 * min(1.0, overlap/50.0)  # reduce impulsive pushes
                    ball.vx += nx * force * dt * 60.0
                    ball.vy += ny * force * dt * 60.0

        # drawing
        screen.fill((20,110,20))
        pygame.draw.rect(screen, (255,255,255), (0,0,WINDOW_W,WINDOW_H), 4)
        # draw goals (centered on short sides)
        gy = WINDOW_H//2
        gw = 8
        gh = int(WINDOW_H*0.25)
        pygame.draw.rect(screen, (200,200,255), (0, gy-gh//2, gw, gh))
        pygame.draw.rect(screen, (200,200,255), (WINDOW_W-gw, gy-gh//2, gw, gh))

        # draw ball
        bx,by = f2px((ball.x, ball.y))
        pygame.draw.circle(screen, (255,165,0),(bx,by), 10)

        # draw robots
        for idx, r in enumerate([r0,r1]):
            rx,ry = f2px((r.state.x, r.state.y))
            color = (40,120,255) if idx==0 else (255,70,70)
            pygame.draw.circle(screen, color, (rx,ry), 24)
            th = math.radians(r.state.theta)
            hx = rx + int(math.sin(th)*30); hy = ry - int(math.cos(th)*30)
            pygame.draw.line(screen, (255,255,255),(rx,ry),(hx,hy),3)
            s = f"R{r.id} θ:{int(r.state.theta)}"
            surf = font.render(s, True, (255,255,255))
            screen.blit(surf, (rx+30, ry-8))

        status = "RUN" if running else "PAUSE"
        screen.blit(font.render(f"{status} - Space to start/pause - R reset - Drag to reposition",True,(255,255,255)), (8,8))
        pygame.display.flip()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--a", help="controller module for robot A (dotted module path)", default="controllers.example_controller")
    parser.add_argument("--b", help="controller module for robot B (dotted module path)", default="controllers.example_controller")
    args = parser.parse_args()
    run_sim(controller_a=args.a, controller_b=args.b)
